#include <ros/ros.h>
#include <boost/thread.hpp>
#include <actionlib/server/action_server.h>
#include <actionlib_msgs/GoalStatus.h>
#include <unistd.h>
#include <fcntl.h>
#include <queue>                  
#include <wpa_supplicant_node/ScanAction.h>
#include <wpa_supplicant_node/AssociateAction.h>
#include <wpa_supplicant_node/AddNetwork.h>
#include <wpa_supplicant_node/RemoveNetwork.h>
#include <wpa_supplicant_node/SetNetworkState.h>
#include <wpa_supplicant_node/SetNetworkParameters.h>
#include <wpa_supplicant_node/NetworkList.h>
#include <wpa_supplicant_node/FrequencyList.h>
 
extern "C" {
#include <includes.h>
#include <common.h>
#include <wpa_supplicant_i.h>
#include <eloop.h>
#include <drivers/driver.h>
#include <scan.h>
#include <bss.h>
#include <config.h>
#include <common/wpa_common.h>
#include <common/ieee802_11_defs.h>
#include "wpa_supplicant_node.h"
}

typedef boost::function<void ()> WorkFunction;

static class RosApi {
  //int eloop_pid_;
  bool initialized_;
  std::queue<WorkFunction> work_queue_;
  boost::condition_variable main_thread_cv_;
  enum main_thread_states { MAIN_THREAD_NOT_WAITING, MAIN_THREAD_WAITING, MAIN_THREAD_PAUSED };
  volatile main_thread_states main_thread_state_;
  int pipefd[2];
  boost::shared_ptr<boost::thread> ros_spin_loop_;
  volatile bool shutting_down_;

public:
  boost::mutex mutex_;
  
  RosApi()
  {
    shutting_down_ = false;
    main_thread_state_ = MAIN_THREAD_NOT_WAITING;
  }

  void doWork()
  {
    char buffer[16]; // Any non-zero size is fine here.

    ROS_INFO("doWork()");
    
    {
      boost::mutex::scoped_lock lock(mutex_);
      mainThreadWait(lock);
    }

    int bytes;
    do {
      bytes = read(pipefd[0], buffer, sizeof(buffer));
      ROS_INFO("doWork read %i bytes from the dummy fifo.", bytes);
    } while (bytes == sizeof(buffer));

    while (1)
    { 
      WorkFunction work;
      {
        boost::mutex::scoped_lock lock(mutex_);
        if (work_queue_.empty())
          break;
        work = work_queue_.front();
        work_queue_.pop();
      }
      work();
    }
  }

  void addWork(const WorkFunction &f)
  {
    ROS_INFO("addWork()");
    
    boost::mutex::scoped_lock lock(mutex_);

    work_queue_.push(f);
    triggerWork();
  }     

  void triggerWork()
  {
    if (write(pipefd[1], pipefd, 1) != 1) // pipefd is used as a dummy buffer.
      ROS_ERROR("addWork Failed to write to wakeup pipe.");
  }

  int init(int *argc, char ***argv)
  {
    ROS_INFO("ros_init");

    // Register the socket that will be used to wake up the event loop.
    if (pipe2(pipefd, O_NONBLOCK | O_CLOEXEC))
    {
      ROS_FATAL("pipe2 failed: %s (%i)", strerror(errno), errno);
      return -1;
    }
    ROS_INFO("pipefds: %i <- %i", pipefd[0], pipefd[1]);
    
    ros::init(*argc, *argv, "wpa_supplicant", ros::init_options::NoSigintHandler);
    ros_spin_loop_.reset(new boost::thread(boost::bind(&ros::spin)));

    return 0;
  }
 
private:
  void waitForMainThreadState(boost::mutex::scoped_lock &lock, main_thread_states target)
  {
    while (main_thread_state_ != target && !shutting_down_)
    {
      main_thread_cv_.notify_one();
      main_thread_cv_.wait(lock);
    }
  }

  void mainThreadWait(boost::mutex::scoped_lock &lock)
  { // If somebody waiting for us, let them make progress.
    if (main_thread_state_ == MAIN_THREAD_WAITING)
    {
      main_thread_state_ = MAIN_THREAD_PAUSED;
      waitForMainThreadState(lock, MAIN_THREAD_PAUSED);
    }
  }

public:
  bool waitForMainThread(boost::mutex::scoped_lock &lock)
  { // Call with mutex_ held. Returns with main thread waiting.
    main_thread_state_ = MAIN_THREAD_WAITING;
    waitForMainThreadState(lock, MAIN_THREAD_PAUSED);
    main_thread_state_ = MAIN_THREAD_NOT_WAITING;
    return !shutting_down_;
  }
  
  void init2()
  {
    ROS_INFO("ros_init");
    eloop_register_read_sock(pipefd[0], &ros_do_work, NULL, NULL);
  }

  void uninit()
  {
    ROS_INFO("ros_deinit");
  
    boost::mutex::scoped_lock lock(mutex_);
    shutting_down_ = true;
    mainThreadWait(lock);

    ros::shutdown(); 
    // FIXME Need to wait for shutdown to complete?
  
    if (ros_spin_loop_ && !ros_spin_loop_->timed_join((boost::posix_time::milliseconds) 2000))
      ROS_ERROR("ROS thread did not die after two seconds. Exiting anyways. This is probably a bad sign.");

    eloop_unregister_read_sock(pipefd[0]);
    close(pipefd[0]);
    close(pipefd[1]);
  }
} g_ros_api;

typedef actionlib::ActionServer<wpa_supplicant_node::ScanAction> ScanActionServer;
typedef actionlib::ActionServer<wpa_supplicant_node::AssociateAction> AssociateActionServer;
const ScanActionServer     ::GoalHandle null_scan_goal_handle_;
const AssociateActionServer::GoalHandle null_associate_goal_handle_;

struct ros_interface
{
private:
  wpa_supplicant *wpa_s_;
  
  ros::NodeHandle nh_;

//Scan Action
  boost::recursive_mutex scan_mutex_;
  ScanActionServer sas_;
  std::queue<ScanActionServer::GoalHandle> scan_queue_;
  ScanActionServer::GoalHandle current_scan_;
  std::vector<int> current_scan_frequencies_;

// Associate Action
  boost::recursive_mutex associate_mutex_;
  AssociateActionServer aas_;
  AssociateActionServer::GoalHandle active_association_;
  bool associate_work_requested_;
  std::queue<AssociateActionServer::GoalHandle> associate_goal_queue_;
  std::queue<AssociateActionServer::GoalHandle> associate_cancel_queue_;

// Services
  ros::ServiceServer add_network_service_;
  ros::ServiceServer remove_network_service_;
  ros::ServiceServer set_network_state_service_;
  ros::ServiceServer set_network_parameter_service_;

// Topics  
  ros::Publisher frequency_list_publisher_;
  ros::Publisher network_list_publisher_;
  ros::Publisher association_publisher_;
  ros::Publisher scan_publisher_;

public:
  ros_interface(const ros::NodeHandle &nh, wpa_supplicant *wpa_s) :
    wpa_s_(wpa_s),
    nh_(ros::NodeHandle(nh, wpa_s->ifname)),
    sas_(nh_, "scan",      boost::bind(&ros_interface::scanGoalCallback,      this, _1), boost::bind(&ros_interface::scanCancelCallback,      this, _1), true),
    aas_(nh_, "associate", boost::bind(&ros_interface::associateGoalCallback, this, _1), boost::bind(&ros_interface::associateCancelCallback, this, _1), true)
  {
    add_network_service_ = nh_.advertiseService("add_network", &ros_interface::addNetwork, this);
    remove_network_service_ = nh_.advertiseService("remove_network", &ros_interface::removeNetwork, this);
    set_network_state_service_ = nh_.advertiseService("set_network_state", &ros_interface::setNetworkState, this);
    set_network_parameter_service_ = nh_.advertiseService("set_network_parameter", &ros_interface::setNetworkParameter, this);

    network_list_publisher_ = nh_.advertise<wpa_supplicant_node::NetworkList>("network_list", 1, true);
    frequency_list_publisher_ = nh_.advertise<wpa_supplicant_node::FrequencyList>("frequency_list", 1, true);
    association_publisher_ = nh_.advertise<wpa_supplicant_node::AssociateFeedback>("association_state", 1, true);
    scan_publisher_ = nh_.advertise<wpa_supplicant_node::ScanResult>("scan_results", 1, true);

    publishFrequencyList();
    publishNetworkList();
    publishUnassociated();
  }

  void publishUnassociated()
  {
    wpa_supplicant_node::AssociateFeedback fbk;
    association_publisher_.publish(fbk);
  }

  void ifaceIdle()
  {
  }

  void scanCompleted(wpa_scan_results *scan_res)
  {
    ROS_INFO("scanCompleted");

    boost::recursive_mutex::scoped_lock lock(scan_mutex_);
    
    eloop_cancel_timeout(scanTimeoutHandler, wpa_s_, NULL);
    
    wpa_supplicant_node::ScanResult rslt;
    if (scan_res)
    {
      fillRosResp(rslt, *scan_res);
      scan_publisher_.publish(rslt);
    }

    if (current_scan_ == null_scan_goal_handle_)
      ROS_ERROR("scanCompleted with current_scan_ not set.");
    else
    {
      if (scan_res)
      {
        // TODO copy output to response.
        ROS_INFO("Scan completed successfully.");
        current_scan_.setSucceeded(rslt);
      }
      else
      {
        ROS_INFO("Scan failed.");
        current_scan_.setAborted(rslt);
      }
      current_scan_ = null_scan_goal_handle_;
      lockedScanTryActivate();
    }
  }

  void assocFailed(const u8 *bssid, const char *s)
  {
    ROS_INFO("assocFailed");
    boost::recursive_mutex::scoped_lock lock(associate_mutex_);
    
    eloop_cancel_timeout(associateTimeoutHandler, wpa_s_, NULL);
    
    if (active_association_ == null_associate_goal_handle_)
    {
      ROS_ERROR("Got disassociation with no active association on BSSID: " MACSTR " (%s)", MAC2STR(bssid), s);
    }
    else
    {
      if (os_memcmp(bssid, &active_association_.getGoal()->bssid, ETH_ALEN) &&
          !is_zero_ether_addr(bssid))
      {
        ROS_ERROR("Got disassociation on unexpected BSSID: " MACSTR " (%s)", MAC2STR(bssid), s);
      }
    
      active_association_.setAborted();
      active_association_ = null_associate_goal_handle_;
    }

    publishUnassociated();
    requestAssociateWork();
  }
  
  void assocSucceeded()
  {
    ROS_INFO("assocSucceeded");
    boost::recursive_mutex::scoped_lock lock(associate_mutex_);
    
    eloop_cancel_timeout(associateTimeoutHandler, wpa_s_, NULL);

    wpa_supplicant_node::AssociateFeedback fbk;
    fbk.associated = true;
    fillRosBss(fbk.bss, *wpa_s_->current_bss);
    // FIXME Set BSS.
    // FIXME Flag a problem if called twice.
    active_association_.publishFeedback(fbk);
    association_publisher_.publish(fbk);
  }

private:       
  void publishNetworkList()
  {
    wpa_supplicant_node::NetworkList netlist;                          

    for (wpa_ssid *ssid = wpa_s_->conf->ssid; ssid; ssid = ssid->next)
    { 
      wpa_supplicant_node::Network net;
      net.network_id = ssid->id;
      net.enabled = !ssid->disabled;

      char **parameters = wpa_config_get_all(ssid, 0);
      for (char **cur_parameter = parameters; *cur_parameter; cur_parameter += 2)
      {
        wpa_supplicant_node::NetworkParameter param;
        param.key = cur_parameter[0];
        param.value = cur_parameter[1];
        net.parameters.push_back(param);
        free(cur_parameter[0]);
        free(cur_parameter[1]);
      }
      free(parameters);

      netlist.networks.push_back(net);
    }

    network_list_publisher_.publish(netlist);
  }

  void publishFrequencyList()
  {
    // FIXME This should be determined in a more general way.

    wpa_supplicant_node::FrequencyList f;
    for (int i = 2412; i <= 2462; i += 5)
      f.frequencies.push_back(i);
    for (int i = 5180; i <= 5320; i += 20)
      f.frequencies.push_back(i);
    for (int i = 5500; i <= 5580; i += 20)
      f.frequencies.push_back(i);
    for (int i = 5500; i <= 5580; i += 20)
      f.frequencies.push_back(i);
    for (int i = 5680; i <= 5700; i += 20)
      f.frequencies.push_back(i);
    for (int i = 5745; i <= 5825; i += 20)
      f.frequencies.push_back(i);

    frequency_list_publisher_.publish(f);
  }

  bool addNetwork(wpa_supplicant_node::AddNetwork::Request &req, wpa_supplicant_node::AddNetwork::Response &rsp)
  {
    ROS_INFO("addNetwork");
    
    boost::mutex::scoped_lock lock(g_ros_api.mutex_);
    if (!g_ros_api.waitForMainThread(lock))
      return false;
  

    return true;
  }

  bool removeNetwork(wpa_supplicant_node::RemoveNetwork::Request &req, wpa_supplicant_node::RemoveNetwork::Response &rsp)
  {
    ROS_INFO("removeNetwork");
    
    boost::mutex::scoped_lock lock(g_ros_api.mutex_);
    if (!g_ros_api.waitForMainThread(lock))
      return false;
    
    
    return true;
  }

  bool setNetworkState(wpa_supplicant_node::SetNetworkState::Request &req, wpa_supplicant_node::SetNetworkState::Response &rsp)
  {
    ROS_INFO("setNetworkState");
    
    boost::mutex::scoped_lock lock(g_ros_api.mutex_);
    if (!g_ros_api.waitForMainThread(lock))
      return false;
    

    return true;
  }

  bool setNetworkParameter(wpa_supplicant_node::SetNetworkParameters::Request &req, wpa_supplicant_node::SetNetworkParameters::Response &rsp)
  {
    ROS_INFO("setNetworkParameter");
    
    boost::mutex::scoped_lock lock(g_ros_api.mutex_);
    if (!g_ros_api.waitForMainThread(lock))
      return false;
    

    return true;
  }
  void requestAssociateWork()
  {
      associate_work_requested_ = true;
      g_ros_api.addWork(boost::bind(&ros_interface::associateWork, this));
  }

  void stopActiveAssociation()
  {
    ROS_INFO("stopActiveAssociation()");
    if (active_association_ == null_associate_goal_handle_)
      ROS_ERROR("stopActiveAssociation called with no active association.");
    
    wpa_supplicant_disassociate(wpa_s_, WLAN_REASON_DEAUTH_LEAVING);
  }

  void startActiveAssociation(AssociateActionServer::GoalHandle &gh)
  {
    ROS_INFO("startActiveAssociation()");
    if (active_association_ != null_associate_goal_handle_)
      ROS_ERROR("startActiveAssociation called with no active association.");

    wpa_bss *bss;
    boost::shared_ptr<const wpa_supplicant_node::AssociateGoal> goal = gh.getGoal();
    const u8 *bssid = &goal->bssid[0];
    std::string ssid = goal->ssid;

    bss = wpa_bss_get(wpa_s_, bssid, (u8 *) ssid.c_str(), ssid.length());
    
    wpa_ssid *net;
    for (net = wpa_s_->conf->ssid; net != NULL; net = net->next)
      if (ssid.length() == net->ssid_len && !os_memcmp(ssid.c_str(), net->ssid, net->ssid_len))
          break;

    if (bss && net)
    {
      gh.setAccepted();
      active_association_ = gh;
      wpa_supplicant_node::AssociateFeedback fbk;
      fbk.associated = false;
      active_association_.publishFeedback(fbk);

      eloop_register_timeout(5, 0, associateTimeoutHandler, wpa_s_, NULL);
      
      // Note, the following line may involve a call to assocFailed, so
      // everything needs to be in a consistent state before we call it.
      wpa_supplicant_associate(wpa_s_, bss, net);
    }
    else
    {
      if (!bss)
        ROS_ERROR("startActiveAssociation could not find requested bss.");
      if (!net)
        ROS_ERROR("startActiveAssociation could not find requested ssid.");
      gh.setRejected();
    }
    ROS_INFO("startActiveAssociation() done");
  }

  void associateWork()
  {
    boost::recursive_mutex::scoped_lock lock(associate_mutex_);
    associate_work_requested_ = false;

    while (!associate_cancel_queue_.empty())
    {
      AssociateActionServer::GoalHandle gh = associate_cancel_queue_.front();
      associate_cancel_queue_.pop();
      if (gh == active_association_)
      {
        stopActiveAssociation();
        return; // We will get called when the disassociate completes.
      }
      else
        gh.setCanceled();
    }

    while (associate_goal_queue_.size() > 1)
    {
      AssociateActionServer::GoalHandle gh = associate_goal_queue_.front();
      associate_goal_queue_.pop();
      gh.setRejected();
    }
    
    if (!associate_goal_queue_.empty())
    {
      AssociateActionServer::GoalHandle gh = associate_goal_queue_.front();
      if (active_association_ != null_associate_goal_handle_ && active_association_.getGoal()->ssid != gh.getGoal()->ssid)
      {
        ROS_INFO("Unassociating prior to ESSID switch.");
        stopActiveAssociation();
        //// Don't call stopActiveAssociation here so that reassociate can
        //// happen. FIXME What happens if reassoc fails? Are we back on the
        //// original AP? That doesn't sound great.
        //active_association_.setAborted();
        //active_association_ = null_associate_goal_handle_; // Avoids error message in startActiveAssociation.
      }
      else
      {
        if (active_association_ != null_associate_goal_handle_)
        {
          active_association_.setAborted();
          active_association_ = null_associate_goal_handle_; // Avoids error message in startActiveAssociation.
        }

        associate_goal_queue_.pop();
      
        startActiveAssociation(gh);      
      }
    }
  }

  void associateGoalCallback(AssociateActionServer::GoalHandle &gh)
  {
    boost::recursive_mutex::scoped_lock lock(associate_mutex_);
    
    associate_goal_queue_.push(gh);
    requestAssociateWork();
  }

  void associateCancelCallback(AssociateActionServer::GoalHandle &gh)
  {
    boost::recursive_mutex::scoped_lock lock(associate_mutex_);
    
    associate_cancel_queue_.push(gh);
    requestAssociateWork();
  }
  
  void fillRosBss(wpa_supplicant_node::Bss &ros_bss, wpa_bss &bss)
  {
    ros_bss.stamp = ros::Time(bss.last_update.sec + 1e-6 * bss.last_update.usec);
    ros_bss.ssid.assign((const char *) bss.ssid, bss.ssid_len);
    memcpy(&ros_bss.bssid[0], bss.bssid, sizeof(bss.bssid));
    ros_bss.frequency = bss.freq;
    ros_bss.beacon_interval = bss.beacon_int;
    ros_bss.capabilities = bss.caps;
    ros_bss.quality = bss.qual;
    ros_bss.level = bss.level;
    ros_bss.noise = bss.noise;
  }
  
  void fillRosResp(wpa_supplicant_node::ScanResult &rslt, wpa_scan_results &scan_res)
  {
    rslt.success = true;
    rslt.bss.clear();
    for (size_t i = 0; i < scan_res.num; i++)
    {
      wpa_scan_res &cur = *scan_res.res[i];
      wpa_supplicant_node::Bss bss;
      const u8* ssid_ie = wpa_scan_get_ie(&cur, WLAN_EID_SSID);
      int ssid_len = ssid_ie ? ssid_ie[1] : 0;
      const char *ssid = ssid_ie ? (const char *) ssid_ie + 2 : "";
      bss.ssid.assign(ssid, ssid_len);
      memcpy(&bss.bssid[0], cur.bssid, sizeof(bss.bssid));
      bss.noise = cur.noise;
      bss.quality = cur.qual;
      bss.level = cur.level;
      bss.capabilities = cur.caps;
      bss.beacon_interval = cur.beacon_int;
      bss.frequency = cur.freq;
      bss.stamp = ros::Time::now() - ros::Duration(cur.age / 1000.0);
      rslt.bss.push_back(bss);
    }
  }

  void scanGoalCallback(ScanActionServer::GoalHandle &gh)
  {
    boost::recursive_mutex::scoped_lock lock(scan_mutex_);
    
    ROS_INFO("scanGoalCallback()");

    scan_queue_.push(gh);

    if (current_scan_ == null_scan_goal_handle_)
      g_ros_api.addWork(boost::bind(&ros_interface::scanTryActivate, this));
  }

  void scanCancelCallback(ScanActionServer::GoalHandle &gh)
  {
    boost::recursive_mutex::scoped_lock lock(scan_mutex_);

    unsigned int status = gh.getGoalStatus().status;

    if (current_scan_ == gh)
    {
      g_ros_api.addWork(boost::bind(&ros_interface::scanCancel, this, gh));
    }
    else
    {
      if (status != actionlib_msgs::GoalStatus::PREEMPTING)
        ROS_ERROR("scanCancelCallback called with unexpected goal status %i", status);
      gh.setCanceled();
    }      
  }

  void scanCancel(ScanActionServer::GoalHandle &gh)
  {
    ROS_INFO("scanCancel()");
    boost::recursive_mutex::scoped_lock lock(scan_mutex_);
    
    // Are we still active (we may have succeeded before getting here)?
    if (current_scan_ == gh)
    {
      // FIXME Anything I can do here to actually cancel the scan?
      gh.setCanceled();
      current_scan_ = null_scan_goal_handle_;
      scanTryActivate();
    }
  }

  void lockedScanTryActivate()
  {
    // THIS VERSION SHOULD BE CALLED WITH THE LOCK HELD!
    ROS_INFO("scanTryActivate()");
    
    // Do we have a scan to activate?
    while (!scan_queue_.empty() && current_scan_ == null_scan_goal_handle_)
    {
      current_scan_ = scan_queue_.front();
      scan_queue_.pop();

      // Is this scan still pending? (Otherwise it has already been
      // canceled elsewhere.)
      if (current_scan_.getGoalStatus().status != actionlib_msgs::GoalStatus::PENDING)
      {
        ROS_INFO("Skipping canceled scan.");
        current_scan_ = null_scan_goal_handle_;
        continue;
      }
      
      boost::shared_ptr<const wpa_supplicant_node::ScanGoal> goal = current_scan_.getGoal();
      struct wpa_driver_scan_params wpa_req;
      bzero(&wpa_req, sizeof(wpa_req));
      
      std::string err = fillWpaReq(goal, wpa_req);

      if (err.empty())
      {
        current_scan_.setAccepted();
                                
        ROS_INFO("Starting scan.");
        // The following timeout should never get used, as there is a
        // nearly identical one in the scan code. It is nevertheless here
        // just in case.
        int timeout;
        if (wpa_req.freqs == 0)
          timeout = 10000;
        else
          timeout = goal->frequencies.size() * 550;
        eloop_register_timeout(timeout / 1000, 1000 * (timeout % 1000), scanTimeoutHandler, wpa_s_, NULL);
        wpa_supplicant_trigger_scan(wpa_s_, &wpa_req);
      }
      else
      {
        current_scan_.setRejected(wpa_supplicant_node::ScanResult(), err);
        current_scan_ = null_scan_goal_handle_;
        continue;
      }
    }
      
    ROS_INFO("Leaving scanTryActivate");
    if (scan_queue_.empty())
      ROS_INFO("scan_queue_ is empty.");
    if (current_scan_ != null_scan_goal_handle_)
      ROS_INFO("A scan is active.");
  }

  static void scanTimeoutHandler(void *wpa_s, void *unused)
  {
    ROS_INFO("Scan timeout!");
    ((wpa_supplicant *) wpa_s)->ros_api->scanCompleted(NULL);
  }
 
  static void associateTimeoutHandler(void *wpa_s, void *unused)
  {
    ROS_INFO("Associate timeout!");
    static const u8 zeroMAC[] = "\0\0\0\0\0\0";
    ((wpa_supplicant *) wpa_s)->ros_api->assocFailed(zeroMAC, "Associatiot timed out.");
  }
 
  std::string fillWpaReq(boost::shared_ptr<const wpa_supplicant_node::ScanGoal> &g, struct wpa_driver_scan_params &wpa_req)
  {
    wpa_req.num_ssids = g->ssids.size();

#define QUOTEME(x) #x
    if (wpa_req.num_ssids > WPAS_MAX_SCAN_SSIDS)
      return "Too many ESSIDs in scan request. Maximum number is "QUOTEME(WPAS_MAX_SCAN_SSIDS)".";

    for (unsigned int i = 0; i < wpa_req.num_ssids; i++)
    {
      wpa_req.ssids[i].ssid = (const u8 *) g->ssids[i].c_str();
      wpa_req.ssids[i].ssid_len = g->ssids[i].size();

      if (wpa_req.ssids[i].ssid_len > WPA_MAX_SSID_LEN)
        return "Ssid is too long. Maximum length is "QUOTEME(WPA_MAX_SSID_LEN)" characters.";
    }
#undef QUOTEME
                                    
    current_scan_frequencies_.clear();
    int num_frequencies = g->frequencies.size();
    
    if (num_frequencies > 0)
    {
      for (int i = 0; i < num_frequencies; i++)
        current_scan_frequencies_.push_back(g->frequencies[i]);
      current_scan_frequencies_.push_back(0);
      wpa_req.freqs = &current_scan_frequencies_[0]; // Use wpa_req before the goal dies!
    }
  
    return "";
  }

  void scanTryActivate()
  {
    boost::recursive_mutex::scoped_lock lock(scan_mutex_);
    lockedScanTryActivate();
  }
};

extern "C" {

int ros_init(int *argc, char ***argv)
{
  return g_ros_api.init(argc, argv);
}

void ros_init2()
{
  return g_ros_api.init2();
}

void ros_deinit()
{
  g_ros_api.uninit();
}

void ros_add_iface(wpa_supplicant *wpa_s)
{
  wpa_s->ros_api = new ros_interface(ros::NodeHandle("wifi"), wpa_s);
}

void ros_remove_iface(wpa_supplicant *wpa_s)
{
  delete wpa_s->ros_api;
  wpa_s->ros_api = NULL;
}

void ros_iface_idle(wpa_supplicant *wpa_s)
{
  if (wpa_s->ros_api)
    wpa_s->ros_api->ifaceIdle();
}
  
void ros_scan_completed(wpa_supplicant *wpa_s, wpa_scan_results *scan_res)
{
  if (wpa_s->ros_api)
    wpa_s->ros_api->scanCompleted(scan_res);
}

void ros_do_work(int, void *, void *)
{
  g_ros_api.doWork();
}

void ros_assoc_failed(wpa_supplicant *wpa_s, const u8 *bssid, const char *reason)
{
  if (wpa_s->ros_api)
    wpa_s->ros_api->assocFailed(bssid, reason);
}

void ros_assoc_success(wpa_supplicant *wpa_s)
{
  if (wpa_s->ros_api)
    wpa_s->ros_api->assocSucceeded();
}

} // extern "C"
