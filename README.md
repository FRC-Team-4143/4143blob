4143pclpyramid
==============

Uses Point Cloud Library and a Microsoft Kinect to detect and determine orientation and distance to the 2013 FRC pyramid game piece.

Currently it detects one plane as the ground and colors those points green.
Then it detects up to 10 bars (lines) of the pyramid and colors those red.
It also pipes out line coefficients for found lines.

Press 'q' to quit.

Requires PCL and a Kinect sensor.

Next steps are to verify the ground plane is actually a reasonable plane. Determine which of the lines are reasonable verticle bars of the pyramid.  Are they 60 degrees to the ground plane?  Determine how far the camera is from the point where the verticle bar intersects the ground plane.  Determine which bars are horizontal bars.  Determine the angle of the camera to the pyramid.

[http://www.pointclouds.org]
[http://www.usfirst.org/roboticsprograms/frc]4143pclpyramid
