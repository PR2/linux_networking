#include <ros/ros.h>
#include <boost/thread.hpp>
#include <actionlib/server/action_server.h>
#include <actionlib_msgs/GoalStatus.h>
#include <unistd.h>
#include <fcntl.h>
#include <queue>                  
#include <wpa_supplicant_node/ScanAction.h>
#include <wpa_supplicant_node/AssociateAction.h>

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

static class {
  //int eloop_pid_;
  bool initialized_;
  std::queue<WorkFunction> work_queue_;
  boost::mutex mutex_;
  int pipefd[2];
  boost::shared_ptr<boost::thread> ros_spin_loop_;

public:
  void doWork()
  {
    char buffer[16]; // Any non-zero size is fine here.

    ROS_INFO("doWork()");

    boost::mutex::scoped_lock(mutex_);

    int bytes;
    do {
      bytes = read(pipefd[0], buffer, sizeof(buffer));
      ROS_INFO("doWork read %i bytes from the dummy fifo.", bytes);
    } while (bytes == sizeof(buffer));

    while (!work_queue_.empty())
    {
      work_queue_.front()();
      work_queue_.pop();
    }
  }

  void addWork(const WorkFunction &f)
  {
    char dummy[1];

    ROS_INFO("addWork()");
    
    boost::mutex::scoped_lock(mutex_);

    work_queue_.push(f);
    if (write(pipefd[1], dummy, 1) != 1)
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
  
  void init2()
  {
    ROS_INFO("ros_init");
    eloop_register_read_sock(pipefd[0], &ros_do_work, NULL, NULL);
  }

  void uninit()
  {
    ROS_INFO("ros_deinit");
  
    ros::shutdown(); 
    // FIXME Need to wait for shutdown to complete?
  
    if (ros_spin_loop_ && !ros_spin_loop_->timed_join((boost::posix_time::milliseconds) 2000))
      ROS_ERROR("ROS thread did not die after two seconds. Exiting anyways. This is probably a bad sign.");

    eloop_unregister_read_sock(pipefd[0]);
    close(pipefd[0]);
    close(pipefd[1]);
  }
} ros_global;

typedef actionlib::ActionServer<wpa_supplicant_node::ScanAction> ScanActionServer;
typedef actionlib::ActionServer<wpa_supplicant_node::AssociateAction> AssociateActionServer;
const ScanActionServer     ::GoalHandle null_scan_goal_handle_;
const AssociateActionServer::GoalHandle null_associate_goal_handle_;

struct ros_api
{
private:
  wpa_supplicant *wpa_s_;

//Scan Action
  boost::mutex scan_mutex_;
  ScanActionServer sas_;
  std::queue<ScanActionServer::GoalHandle> scan_queue_;
  ScanActionServer::GoalHandle current_scan_;
  std::vector<int> current_scan_frequencies_;

// Associate Action
  boost::mutex associate_mutex_;
  AssociateActionServer aas_;
  AssociateActionServer::GoalHandle active_association_;
  bool associate_work_requested_;
  std::queue<AssociateActionServer::GoalHandle> associate_goal_queue_;
  std::queue<AssociateActionServer::GoalHandle> associate_cancel_queue_;
  
public:
  ros_api(const ros::NodeHandle &nh, wpa_supplicant *wpa_s) :
    wpa_s_(wpa_s),
    sas_(nh, std::string(wpa_s->ifname) + "/scan",      boost::bind(&ros_api::scanGoalCallback,      this, _1), boost::bind(&ros_api::scanCancelCallback,      this, _1), true),
    aas_(nh, std::string(wpa_s->ifname) + "/associate", boost::bind(&ros_api::associateGoalCallback, this, _1), boost::bind(&ros_api::associateCancelCallback, this, _1), true)
  {
  }

  void ifaceIdle()
  {
  }

  void scanCompleted(wpa_scan_results *scan_res)
  {
    ROS_INFO("scanCompleted");

    boost::mutex::scoped_lock(scan_mutex_);
    
    if (current_scan_ == null_scan_goal_handle_)
      ROS_ERROR("scanCmopleted with current_scan_ not set.");
    else
    {
      if (scan_res)
      {
        // TODO copy output to response.
        ROS_INFO("Scan completed successfully.");
        wpa_supplicant_node::ScanResult rslt;
        fillRosResp(rslt, *scan_res);
        current_scan_.setSucceeded(rslt);
      }
      else
      {
        ROS_INFO("Scan failed.");
        current_scan_.setAborted();
      }
      current_scan_ = null_scan_goal_handle_;
      lockedScanTryActivate();
    }
  }

  void assocFailed(const char *s)
  {
    ROS_INFO("assocFailed");
    boost::mutex::scoped_lock(associate_mutex_);
    active_association_.setAborted();
    stopActiveAssociation(false);
  }
  
  void assocSucceeded()
  {
    ROS_INFO("assocSucceeded");
    boost::mutex::scoped_lock(associate_mutex_);

    wpa_supplicant_node::AssociateFeedback fbk;
    fbk.associated = true;
    // FIXME Set BSS.
    // FIXME Flag a problem if called twice.
    active_association_.publishFeedback(fbk);
  }

private:       
  void requestAssociateWork()
  {
      associate_work_requested_ = true;
      ros_global.addWork(boost::bind(&ros_api::associateWork, this));
  }

  void stopActiveAssociation(bool disassociate = true)
  {
    ROS_INFO("stopActiveAssociation()");
    if (active_association_ == null_associate_goal_handle_)
      ROS_ERROR("stopActiveAssociation called with no active association.");
    
    if (disassociate)
      wpa_supplicant_disassociate(wpa_s_, WLAN_REASON_DEAUTH_LEAVING);
    active_association_ = null_associate_goal_handle_;
  }

  void startActiveAssociation(AssociateActionServer::GoalHandle &gh)
  {
    ROS_INFO("startActiveAssociation()");
    if (active_association_ != null_associate_goal_handle_)
      ROS_ERROR("startActiveAssociation called with no active association.");

    wpa_bss *bss;
    boost::shared_ptr<const wpa_supplicant_node::AssociateGoal> goal = gh.getGoal();
    bss = wpa_bss_get(wpa_s_, &goal->bss.bssid[0], (u8 *) goal->bss.ssid.c_str(), goal->bss.ssid.length());
    
    wpa_ssid *ssid;
    for (ssid = wpa_s_->conf->ssid; ssid != NULL; ssid = ssid->next)
      if (goal->bss.ssid.length() == ssid->ssid_len && !os_memcmp(goal->bss.ssid.c_str(), ssid->ssid, ssid->ssid_len))
          break;

    if (bss && ssid && goal->bss.frequency == bss->freq)
    {
      ROS_INFO("wpa_s in startActiveAssociation: %p", wpa_s_);
      ROS_INFO("wpa_s.ros in startActiveAssociation: %p", wpa_s_->ros_api);
      ROS_INFO("&wpa_s.ros_api in startActiveAssociation: %p", &wpa_s_->ros_api);
      ROS_INFO("sizeof(wpa_s) in startActiveAssociation: %zi", sizeof(*wpa_s_));
      gh.setAccepted();
      active_association_ = gh;
      wpa_supplicant_associate(wpa_s_, bss, ssid);
      wpa_supplicant_node::AssociateFeedback fbk;
      fbk.associated = false;
      active_association_.publishFeedback(fbk);
    }
    else
    {
      ROS_ERROR("startActiveAssociation could not find requested bss, ssid or frequency.");
      gh.setRejected();
    }
  }

  void associateWork()
  {
    boost::mutex::scoped_lock(associate_mutex_);

    associate_work_requested_ = false;

    while (!associate_cancel_queue_.empty())
    {
      AssociateActionServer::GoalHandle gh = associate_cancel_queue_.front();
      associate_cancel_queue_.pop();
      if (gh == active_association_)
        stopActiveAssociation();
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
      if (active_association_ != null_associate_goal_handle_)
      {
        // Don't call stopActiveAssociation here so that reassociate can
        // happen. FIXME What happens if reassoc fails? Are we back on the
        // original AP? That doesn't sound great.
        active_association_.setAborted();
        active_association_ = null_associate_goal_handle_; // Avoids error message in startActiveAssociation.
      }

      AssociateActionServer::GoalHandle gh = associate_goal_queue_.front();
      associate_goal_queue_.pop();
      
      startActiveAssociation(gh);      
    }
  }

  void associateGoalCallback(AssociateActionServer::GoalHandle &gh)
  {
    boost::mutex::scoped_lock(associate_mutex_);
    
    associate_goal_queue_.push(gh);
    requestAssociateWork();
  }

  void associateCancelCallback(AssociateActionServer::GoalHandle &gh)
  {
    boost::mutex::scoped_lock(associate_mutex_);
    
    associate_cancel_queue_.push(gh);
    requestAssociateWork();
  }
  
  void fillRosResp(wpa_supplicant_node::ScanResult &rslt, wpa_scan_results &scan_res)
  {
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
      bss.age = cur.age / 1000.0;
      rslt.bss.push_back(bss);
    }
  }

  void scanGoalCallback(ScanActionServer::GoalHandle &gh)
  {
    boost::mutex::scoped_lock(scan_mutex_);
    
    ROS_INFO("scanGoalCallback()");

    scan_queue_.push(gh);

    if (current_scan_ == null_scan_goal_handle_)
      ros_global.addWork(boost::bind(&ros_api::scanTryActivate, this));
  }

  void scanCancelCallback(ScanActionServer::GoalHandle &gh)
  {
    boost::mutex::scoped_lock(scan_mutex_);

    unsigned int status = gh.getGoalStatus().status;

    if (current_scan_ == gh)
    {
      ros_global.addWork(boost::bind(&ros_api::scanCancel, this, gh));
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
    boost::mutex::scoped_lock(scan_mutex_);
    
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
                                
        // FIXME Copy set of ESSIDs and freqs to scan.
        
        ROS_INFO("Starting scan.");
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
      wpa_req.freqs = &current_scan_frequencies_[0];
    }
  
    return "";
  }

  void scanTryActivate()
  {
    boost::mutex::scoped_lock(scan_mutex_);
    lockedScanTryActivate();
  }
};

extern "C" {

int ros_init(int *argc, char ***argv)
{
  return ros_global.init(argc, argv);
}

void ros_init2()
{
  return ros_global.init2();
}

void ros_deinit()
{
  ros_global.uninit();
}

void ros_add_iface(wpa_global *global, wpa_supplicant *wpa_s)
{
  ROS_INFO("ros_add_iface");
  wpa_s->ros_api = new ros_api(ros::NodeHandle(), wpa_s);
  ROS_INFO("wpa_s in ros_add_iface: %p", wpa_s);
  ROS_INFO("wpa_s.ros in ros_add_iface: %p", wpa_s->ros_api);
}

void ros_remove_iface(wpa_global *global, wpa_supplicant *wpa_s)
{
  ROS_INFO("ros_remove_iface");
  delete wpa_s->ros_api;
}

void ros_iface_idle(wpa_supplicant *wpa_s)
{
  wpa_s->ros_api->ifaceIdle();
}
  
void ros_scan_completed(wpa_supplicant *wpa_s, wpa_scan_results *scan_res)
{
  ROS_INFO("wpa_s in ros_scan_completed: %p", wpa_s);
  ROS_INFO("wpa_s.ros in ros_scan_completed: %p", wpa_s->ros_api);
  wpa_s->ros_api->scanCompleted(scan_res);
}

void ros_do_work(int, void *, void *)
{
  ros_global.doWork();
}

void ros_assoc_failed(wpa_supplicant *wpa_s, const char *reason)
{
  wpa_s->ros_api->assocFailed(reason);
}

void ros_assoc_success(wpa_supplicant *wpa_s)
{
  wpa_s->ros_api->assocSucceeded();
}

} // extern "C"
