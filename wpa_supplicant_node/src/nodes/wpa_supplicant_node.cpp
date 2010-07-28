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
#include "includes.h"
#include "common.h"
#include "../../wpa_supplicant/wpa_supplicant/wpa_supplicant_i.h"
#include "wpa_supplicant_node.h"
#include "eloop.h"
#include "drivers/driver.h"
#include "../../wpa_supplicant/wpa_supplicant/scan.h"
#include "common/wpa_common.h"
#include "common/ieee802_11_defs.h"
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
    // FIXME Notify the event loop kill(eloop_pid_, SIGALARM);
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

struct ros_api
{
private:
  wpa_supplicant &wpa_s_;

//Scan Action
  boost::mutex scan_mutex_;
  ScanActionServer sas_;
  std::queue<ScanActionServer::GoalHandle> scan_queue_;
  ScanActionServer::GoalHandle current_scan_;
  std::vector<int> current_scan_frequencies_;

// Associate Action
  boost::mutex associate_mutex_;
//  AssociateActionServer aas_;
  
public:
  ros_api(const ros::NodeHandle &nh, wpa_supplicant *wpa_s) :
    wpa_s_(*wpa_s),
    sas_(nh, wpa_s->ifname, boost::bind(&ros_api::scanGoalCallback, this, _1), boost::bind(&ros_api::scanCancelCallback, this, _1), true)
//    aas_(nh, wpa_s->ifname, boost::bind(&ros_api::associateGoalCallback, this, _1), boost::bind(&ros_api::associateCancelCallback, this, _1), true)
  {
  }

  void ifaceIdle()
  {
  }

  void scanCompleted(wpa_scan_results *scan_res)
  {
    ROS_INFO("scanCompleted");

    boost::mutex::scoped_lock(scan_mutex_);
    
    if (current_scan_ == ScanActionServer::GoalHandle())
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
      current_scan_ = ScanActionServer::GoalHandle();
      lockedScanTryActivate();
    }
  }

private:                            
  void fillRosResp(wpa_supplicant_node::ScanResult &rslt, wpa_scan_results &scan_res)
  {
    rslt.access_points.clear();
    for (size_t i = 0; i < scan_res.num; i++)
    {
      wpa_scan_res &cur = *scan_res.res[i];
      wpa_supplicant_node::AccessPoint ap;
      const u8* ssid_ie = wpa_scan_get_ie(&cur, WLAN_EID_SSID);
      int ssid_len = ssid_ie ? ssid_ie[1] : 0;
      const char *ssid = ssid_ie ? (const char *) ssid_ie + 2 : "";
      ap.ssid.assign(ssid, ssid_len);
      memcpy(&ap.bssid[0], cur.bssid, sizeof(ap.bssid));
      ap.noise = cur.noise;
      ap.quality = cur.qual;
      ap.level = cur.level;
      ap.capabilities = cur.caps;
      ap.beacon_interval = cur.beacon_int;
      ap.frequency = cur.freq;
      ap.age = cur.age / 1000.0;
      rslt.access_points.push_back(ap);
    }
  }

  void scanGoalCallback(ScanActionServer::GoalHandle &gh)
  {
    boost::mutex::scoped_lock(scan_mutex_);
    
    ROS_INFO("scanGoalCallback()");

    scan_queue_.push(gh);

    if (current_scan_ == ScanActionServer::GoalHandle())
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
      current_scan_ = ScanActionServer::GoalHandle();
      scanTryActivate();
    }
  }

  void lockedScanTryActivate()
  {
    // THIS VERSION SHOULD BE CALLED WITH THE LOCK HELD!
    ROS_INFO("scanTryActivate()");
    
    // Do we have a scan to activate?
    while (!scan_queue_.empty() && current_scan_ == ScanActionServer::GoalHandle())
    {
      current_scan_ = scan_queue_.front();
      scan_queue_.pop();

      // Is this scan still pending? (Otherwise it has already been
      // canceled elsewhere.)
      if (current_scan_.getGoalStatus().status != actionlib_msgs::GoalStatus::PENDING)
      {
        ROS_INFO("Skipping canceled scan.");
        current_scan_ = ScanActionServer::GoalHandle();
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
        wpa_supplicant_trigger_scan(&wpa_s_, &wpa_req);
      }
      else
      {
        current_scan_.setRejected(wpa_supplicant_node::ScanResult(), err);
        current_scan_ = ScanActionServer::GoalHandle();
        continue;
      }
    }
      
    ROS_INFO("Leaving scanTryActivate");
    if (scan_queue_.empty())
      ROS_INFO("scan_queue_ is empty.");
    if (current_scan_ != ScanActionServer::GoalHandle())
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
      wpa_req.ssids[i].ssid = (const u8 *) &g->ssids[i][0];
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
  wpa_s->ros_api->scanCompleted(scan_res);
}

void ros_do_work(int, void *, void *)
{
  ros_global.doWork();
}

} // extern "C"
