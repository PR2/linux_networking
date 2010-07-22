#include <ros/ros.h>
#include <boost/thread.hpp>
#include <actionlib/server/action_server.h>
#include <queue>
#include <wpa_supplicant_node/ScanAction.h>

extern "C" {
#include "includes.h"
#include "common.h"
#include "../../wpa_supplicant/wpa_supplicant/wpa_supplicant_i.h"
}

static struct {
  int eloop_pid;
  bool initialized;
} ros_global;

typedef actionlib::ActionServer<wpa_supplicant_node::ScanAction> ScanActionServer;

struct ros_api
{
  wpa_global &global_;
  wpa_supplicant &wpa_s_;
  ScanActionServer sas_;
  std::queue<ScanActionServer::GoalHandle> scan_queue_;
  boost::mutex lock_;
  
public:
  ros_api(wpa_global *global, wpa_supplicant *wpa_s) :
    global_(*global),
    wpa_s_(*wpa_s),
    sas_(nh, boost::bind(&ros_api::scanGoalCallback, this, _1), boost::bind(&ros_api::scanCancelCallback, this, _1))
  {
  }

  void ifaceIdle()
  {
  }

private:
  void scanGoalCallback(ScanActionServer::GoalHandle &gh)
  {
    boost::mutex::scoped_lock(mutex_);
  }

  void scanCancelCallback(ScanActionServer::GoalHandle &gh)
  {
    boost::mutex::scoped_lock(mutex_);

    unsigned int status = gh.getGoalStatus();

    switch (status)
    {
      actionlib_msgs::GoalStatus::RECALLING:
        gh.setCancelled();
        break;

      actionlib_msgs::GoalStatus::PREEMPTING:
        // FIXME Need to do something here.
        break;

      default:
        ROS_ERROR("scanCancelCallback called with unexpected goal status %i", gh.getGoalStatus());
        break;
    }
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
  wpa_s->ros_api = new ros_api(global, wpa_s);
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

}
