// Copy to secrets.h and fill in. secrets.h is never committed to Git.
#pragma once
#define WIFI_SSID      "your-wifi-or-hotspot"
#define WIFI_PASS      "wifi-password"
// Optional backup network (e.g. your phone hotspot):
// #define WIFI_SSID2   "hotspot-name"
// #define WIFI_PASS2   "hotspot-password"
#define MQTT_HOST      "xxxxxxxx.s1.eu.hivemq.cloud"
#define MQTT_PORT      8883
#define MQTT_USER      "probe1"
#define MQTT_PASS      "probe1-password"
#define DEVICE_ID      "B9"   // loom number      // topics: loom/<DEVICE_ID>/...
#define PULSES_PER_REV 1   // counted with 40 ms min gap (pulses arrive as a close pair)
