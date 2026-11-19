# Loom probe firmware (ESP32) — ChipToByte Technologies

Field probe for loom signal monitoring. Publishes filtered loom events (run / stop with reason
weft·warp·manual, picks, RPM) over MQTT/TLS and updates itself over the air.

**No secrets in this repo.** Wi-Fi and MQTT credentials are written once over USB from a local
`include/secrets.h` (see `secrets.example.h`) into the ESP32's flash storage; OTA images read them from there.

## Updating
- **Remote (GitHub):** bump `VERSION` on main → Actions builds and publishes `firmware.bin` to the release →
  send `ota latest` on MQTT topic `loom/<id>/cmd`.
- **On the LAN:** open `http://<device-ip>/` (user `admin`), upload `firmware.bin`.
- A new image must reach MQTT within 5 min (and not crash-loop), otherwise the device boots back into the previous version.

## MQTT commands (`loom/<id>/cmd`)
`ping` · `info` (shows flash backlog) · `reboot` · `ota latest` · `ota <url>` · `wifi2 <ssid>|<pass>` · `wifi_swap` · `webpass <new, 8+ chars>`

## Store-and-forward (v2.2)
Events are kept in flash (LittleFS on the `spiffs` partition, ~512 KB ≈ 3,000 events) whenever the device is
offline, and survive power cuts and reboots. While offline a summary is also stored every 2 min (marked
`"buffered":true`). On reconnect the backlog is sent oldest-first with original timestamps; the server
ignores duplicates. Credentials live in NVS (separate partition) and are never touched.
