#ifndef __WPA_SUPPLICANT__ROS_H__
#define __WPA_SUPPLICANT__ROS_H__

#include "../../wpa_supplicant/wpa_supplicant/wpa_supplicant_i.h"

void ros_init(int *argc, char ***argv);
void ros_deinit();
void ros_add_iface(struct wpa_global *global, struct wpa_supplicant *wpa_s);
void ros_remove_iface(struct wpa_global *global, struct wpa_supplicant *wpa_s);

#endif
