#pragma once

// Copy to wifi.h (git-ignored) and set the network credentials before flashing. At boot the camera joins the first of these networks in range; with none, or none joined within 30 s, it joins nothing and keeps its radio for FPV (ota.h). The drone's K A request (tools/ota_direct.py camera-ap) starts its own access point (apSsid, a password of at least 8 characters, or none without one).
constexpr WifiNetwork wifiNetworks[] = {{"", ""}};
constexpr char apSsid[] = "";
constexpr char apPassword[] = "";
