#ifndef __WPA_SUPPLICANT__ROS_H__
#define __WPA_SUPPLICANT__ROS_H__

#include "../../wpa_supplicant/wpa_supplicant/wpa_supplicant_i.h"

int ros_init(int *argc, char ***argv);
void ros_init2();
void ros_deinit();
void ros_add_iface(struct wpa_global *global, struct wpa_supplicant *wpa_s);
void ros_remove_iface(struct wpa_global *global, struct wpa_supplicant *wpa_s);
void ros_do_work(int, void *, void *);
void ros_scan_completed(struct wpa_supplicant *wpa_s, struct wpa_scan_results *scan_res);
void ros_assoc_success(struct wpa_supplicant *wpa_s);
void ros_assoc_failed(struct wpa_supplicant *wpa_s, const char *reason);

#endif
