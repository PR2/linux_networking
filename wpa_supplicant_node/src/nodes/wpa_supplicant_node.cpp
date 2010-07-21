#include <ros/ros.h>

extern "C" {
#include "wpa_supplicant_node.h"
}



extern "C" {

void ros_init(int *argc, char ***argv)
{
  ros::init(*argc, *argv, "wpa_supplicant");
}

void ros_deinit()
{
  ros::shutdown(); 
  // FIXME Need to wait for shutdown to complete?
}

void ros_add_iface(struct wpa_global *global, struct wpa_supplicant *wpa_s)
{
}

void ros_remove_iface(struct wpa_global *global, struct wpa_supplicant *wpa_s)
{
}

}
