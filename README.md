# Quaternion Heading Issue
In this repository, I uploaded the code I used for quaternion calculations. To compute the quaternion in the code, I used the Madgwick algorithm with a magnetometer. I believe the calculations are quite successful because they produce rational values consistent with the theory. Furthermore, the quaternion values can be visualized using a 3D Cube, which can be found on GitHub. The sensor's rotation aligns with the visualization, provided that the magnetometer's X-axis points north when the code is first run; otherwise, the sensor's rotation will not match the 3D Cube visualization.

I assume that the issue requiring the magnetometer's X-axis to face north at the first run is because the sensor cannot detect the north direction in this quaternion calculation. This assumption is based on experiments where I ran the program multiple times with the magnetometer's X-axis pointing in different directions. However, the quaternion value always remains at 1,0,0,0 (not absolute but close to that value).

The sensor calibration I performed was only on the gyroscope and magnetometer. For the gyroscope, I used bias correction calibration, and for the magnetometer, I used soft iron and hard iron calibration. I did not calibrate the accelerometer because I believe its values are already accurate.

To better understand this issue, I uploaded four videos illustrating how I ran my program, how I calibrated the sensors, the initial position of the sensor, and the 3D Cube simulation using Python.

## Demo Video

There are two groups of videos below. The first group consists of simulation videos where the magnetometer's X-axis faces north, and the second group consists of simulation videos where the magnetometer's X-axis faces east. I uploaded two videos for each group due to equipment limitations.

The zoom view videos offer high quality for observing the visuals clearly, but there is a delay between the sensor and the visualization due to internet speed. Meanwhile, the camera view videos are better for comparing the sensor simulation with the visualization without any delay.

#### video groups 1
[X axis magnetometer pointed to North zoom view](https://youtu.be/pWliJR0dxj4)  
[X axis magnetometer pointed to North camera view](https://youtu.be/Q5Pou4o74G4)  
From the videos above where the magnetometer's X-axis faces north, the initial quaternion values correspond to the sensor's position, namely 1,0,0,0. The simulation using the 3D Cube also aligns with the movements in Python.

#### video groups 2
[X axis magnetometer pointed to East zoom view](https://youtu.be/GrCHB4ByquI)  
[X axis magnetometer pointed to East camera view](https://youtu.be/uOq0CAsV6eo)  
From the videos above where the magnetometer's X-axis faces north, the initial quaternion values when the sensor is first powered on are 1,0,0,0. Similarly, the quaternion values are also 1,0,0,0 when the magnetometer's X-axis faces east. This is the issue I am questioning—why does this happen? I believe the magnetometer values should differ when the X-axis faces north and east, so the quaternion values should also be different.
