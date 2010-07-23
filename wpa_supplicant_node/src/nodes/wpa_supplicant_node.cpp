#include <ros/ros.h>
#include <boost/thread.hpp>
#include <actionlib/server/action_server.h>
#include <actionlib_msgs/GoalStatus.h>
#include <queue>
#include <wpa_supplicant_node/ScanAction.h>

extern "C" {
#include "includes.h"
#include "common.h"
#include "../../wpa_supplicant/wpa_supplicant/wpa_supplicant_i.h"
}

typedef const boost::function<void ()> WorkFunction;

static class {
  int eloop_pid_;
  bool initialized_;
  std::queue<WorkFunction> work_queue_;
  boost::mutex mutex_;
public:
  void doWork()
  {
    boost::mutex::scoped_lock(mutex_);

    while (!work_queue.empty())
    {
      work_queue.pop()();
    }
  }

  void addWork(WorkFunction &f)
  {
    boost::mutex::scoped_lock(mutex_);

    work_queue.enqueue(f);
    kill(eloop_pid_, SIGALARM);
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

private:
  void scanGoalCallback(ScanActionServer::GoalHandle &gh)
  {
    boost::mutex::scoped_lock(mutex_);

    scan_queue_.enqueue(gh);

    if (!current_scan_)
      ros_global.addWork(boost::bind(&ros_api::scanTryActivate, this));
  }

  void scanCancelCallback(ScanActionServer::GoalHandle &gh)
  {
    boost::mutex::scoped_lock(mutex_);

    unsigned int status = gh.getGoalStatus().status;

    if (current_scan_ == gh)
    {
      ros_global.addWork(boost::bind(&ros_api::cancelScan, this, gh));
    }
    else
    {
      if (status != actionlib_msgs::GoalStatus::PREEMPTING)
        ROS_ERROR("scanCancelCallback called with unexpected goal status %i", status);
      gh.setCanceled();
    }      
  }

  void scanCompleted(wpa_scan_results *scan_res)
  {
  }
};

extern "C" {

void ros_init(int *argc, char ***argv)
{
  ros::init(*argc, *argv, "wpa_supplicant");
  ROS_INFO("ros_init");
  ros_global.eloop_pid = getpid();
  ros_global.initialized = true;
}

void ros_deinit()
{
  ROS_INFO("ros_deinit");
  ros::shutdown(); 
  // FIXME Need to wait for shutdown to complete?
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

void ros_do_work()
{
  ros_global->doWork();
}
