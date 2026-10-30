#pragma once

// Copy to wifi.h (git-ignored) and set the network credentials before flashing. At boot the camera joins the first of these networks in range; with none, or none joined within 30 s, it starts its own access point (apSsid, a password of at least 8 characters, or none without one).
constexpr WifiNetwork wifiNetworks[] = {{"", ""}};
constexpr char apSsid[] = "";
constexpr char apPassword[] = "";
