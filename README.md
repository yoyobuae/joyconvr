## Joycon-OpenVR-Driver

Extremely barebones Joycon SteamVR driver.

* No tracking at all from the Joycons themselves
* Tries to emulate Vive wand controllers
* Hardcoded button mappings:
  * L/R buttons -> left/right trigger buttons
  * ZL/ZR buttons -> left/right grip buttons
  * +/- buttons -> left/right system buttons
  * Capture/Home buttons -> left/right app buttons
  * Left/right joysticks direction/press -> left/right touchpad direction/press

# Build

No external build dependencies. From root of repo:
  * `cmake -B build`
  * `cmake --build build`

# Instalation

* Do one of these:
  * Copy `build/joyconvr` folder (the entire folder) to `~/.steam/steam/steamapps/common/SteamVR/drivers/`
  * Run `~/.steam/steam/steamapps/common/SteamVR/bin/vrpathreg.sh adddriver <path_to_folder_containing_joyconvr>`
* Double check that `joyconvr` driver shows up and is enabled in `Steam VR settings > Startup/shutdown > Manage Add-ons`
* Install https://github.com/nicman23/dkms-hid-nintendo
* Install https://github.com/DanielOgorchock/joycond
* Manually start `joycond` service (or set it up to start automatically)
* Install whatever needed to make bluetooth work on your PC
* Press and hold the SYNC button on Joycon and connect to it from Bluetooth applet on PC (or command line equivalent)
* Once the left and right Joycons are connected, press L and R buttons at the same time to put them into combined mode
* When you run SteamVR, two controllers should show up and should respond to inputs (remember, no Joycon tracking is implemented)

# Caveats

Nintendo Joycon SYNC/SR/SL buttons are on a cable that's extremely fragile. Even being super careful, occasional use of the SYNC button will eventually cause the cable to break. So be ready to acquire a replacement cables and the tools to open the Joycons, since without SYNC button it will no longer be possible to connect Joycons back to PC after they are paired with the Switch.

`joyconvr` SteamVR driver will need to be disabled when not in use, because it generates errors on SteamVR when Joycons are not connected.

Since no tracking is implemented, you will need to use some other source of tracking (like https://github.com/ju1ce/April-Tag-VR-FullBody-Tracker) and then use SteamVR TrackingOverride feature: https://github.com/ValveSoftware/openvr/wiki/TrackingOverrides

SteamVR configuration file should be at `~/.steam/steam/config/steamvr.vrsettings`. The device paths for trackers seem to work a bit weird for me. You can try either something like this:
```
   "TrackingOverrides" : {
      "/devices/joyconvr/ApriltagTracker2" : "/user/hand/right",
      "/devices/joyconvr/ApriltagTracker3" : "/user/head",
      "/devices/joyconvr/ApriltagTracker1" : "/user/hand/left"
   },
```
Or (assuming you are using apriltagtrackers driver for tracking):
```
   "TrackingOverrides" : {
      "/devices/apriltagtrackers/ApriltagTracker2" : "/user/hand/right",
      "/devices/apriltagtrackers/ApriltagTracker3" : "/user/head",
      "/devices/apriltagtrackers/ApriltagTracker1" : "/user/hand/left"
   },
```

