#include <ros/ros.h>
#include <boost/thread.hpp>
#include <actionlib/server/action_server.h>
#include <actionlib_msgs/GoalStatus.h>
#include <unistd.h>
#include <fcntl.h>
#include <queue>                  
#include <wpa_supplicant_node/ScanAction.h>

extern "C" {
#include "includes.h"
#include "common.h"
#include "../../wpa_supplicant/wpa_supplicant/wpa_supplicant_i.h"
#include "wpa_supplicant_node.h"
#include "eloop.h"
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
    ROS_INFO("doWork()");

    boost::mutex::scoped_lock(mutex_);

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

struct ros_api
{
  wpa_supplicant &wpa_s_;
  ScanActionServer sas_;
  std::queue<ScanActionServer::GoalHandle> scan_queue_;
  boost::mutex mutex_;
  ScanActionServer::GoalHandle current_scan_;
  
public:
  ros_api(const ros::NodeHandle &nh, wpa_supplicant *wpa_s) :
    wpa_s_(*wpa_s),
    sas_(nh, wpa_s->ifname, boost::bind(&ros_api::scanGoalCallback, this, _1), boost::bind(&ros_api::scanCancelCallback, this, _1), true)
  {
  }

  void ifaceIdle()
  {
  }

  void scanCompleted(wpa_scan_results *scan_res)
  {
    ROS_INFO("scanCompleted");

    boost::mutex::scoped_lock(mutex_);
    
    if (current_scan_ == ScanActionServer::GoalHandle())
      ROS_ERROR("scanCmopleted with current_scan_ not set.");
    else
    {
      current_scan_.setSucceeded();
    }
  }

private:
  void scanGoalCallback(ScanActionServer::GoalHandle &gh)
  {
    boost::mutex::scoped_lock(mutex_);
    
    ROS_INFO("scanGoalCallback()");

    scan_queue_.push(gh);

    if (current_scan_ == ScanActionServer::GoalHandle())
      ros_global.addWork(boost::bind(&ros_api::scanTryActivate, this));
  }

  void scanCancelCallback(ScanActionServer::GoalHandle &gh)
  {
    boost::mutex::scoped_lock(mutex_);

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
  }

  void scanTryActivate()
  {
    ROS_INFO("scanTryActivate()");
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
