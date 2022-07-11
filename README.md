# Open-Blimp Workshop Code
Welcome to the Github Repository for the Open-Blimp Workshop Series

To view documentation and tutorials on running the demos, visit [https://open-blimp.notion.site/](https://open-blimp.notion.site/)

Note: This branch is for an upgraded hardware version of the Open-Blimp using the Raspberry Pi Zero 2 W and the VL53L1X Distance Sensor and also incorporates a multicast form of blimp motor control, in which all agents share the same frequency and are indexed by a unique ID

Software changes:
- Parallel streaming from RPi02w using multiprocessing
- Parallel angle control from base using multiprocessing
- Using BNO055 library https://github.com/adafruit/Adafruit_Python_BNO055 instead due to integration issues with updated RPi02w
- data is now requested from base station without sshing into pi
- option to install startup script to automatically stream on boot


Blimp Orientation: facing from directly behind the camera with the altitude sensor pointing down
- positive x: towards the camera
- positive y: towards the left perpendicular of the camera
- positive z: towards the sky

