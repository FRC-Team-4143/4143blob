4143pclpyramid
==============

Uses Point Cloud Library and a Microsoft Kinect to detect and determine orientation and distance to the 2013 FRC pyramid game piece.

Currently it detects one plane as the ground and colors those points green.  Then it detects up to 12 bars (lines) of the pyramid and colors those.  It also pipes out line coefficients for found lines and ground plane.

![ScreenShot](https://raw.github.com/FIRST4143/4143pclpyramid/master/screenshots/screenshot-1369589041.png)

* to load from pcd file instead of Kinect specify -file option
* Press 'q' to quit
* Press 's' to save .pcd cloud file
* Press 't' to toggle between original cloud and pyramid cloud
* Press 'j' to save .png
* Press 'h' for help

Requires PCL and a Kinect sensor.

to compile:
```
cd build
cmake ..
make -j 4
```

Next steps are to verify the ground plane is actually a reasonable plane. Determine which of the lines are reasonable verticle bars of the pyramid.  Are they 60 degrees to the ground plane?  Determine how far the camera is from the point where the verticle bar intersects the ground plane.  Determine which bars are horizontal bars.  Determine the angle of the camera to the pyramid.

Links:

* http://www.pointclouds.org
* http://www.usfirst.org/roboticsprograms/frc

