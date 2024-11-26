In attempting to gain further insight into into calibration techniques for IMU decided to do a bit of a dive into the calibration techniques from accelerometers, magnetometers and gyroscopes.   So here goes.

INSERT LINK to paper


## Magnetometers

From my readings there are 2 primary types of calibrations that can be performed for magnetometers as well as accelerometers but first lets talk about magnetometers:

1.	Elipsoid to Sphere fitting and
2.	6-point tumble calibration.
3.	Magnetic Declination Correction

Using Microsoft CoPilot:  Calibrating a magnetometer is essential to ensure accurate readings by correcting for various distortions. Here are some common calibration techniques:

1.	**Hard-Iron Calibration:**\
o	Cause: Hard-iron distortions are caused by permanent magnetic objects near the sensor, creating a constant offset in the magnetic field.\
o	Correction: This involves finding the offset values for the X, Y, and Z axes by rotating the magnetometer in all directions and calculating the midpoint between the minimum and maximum values for each axis. These offsets are then subtracted from the raw readings1.

2.	**Soft-Iron Calibration:**\
o	Cause: Soft-iron distortions are due to ferromagnetic materials near the sensor that distort the magnetic field, causing the data to form an ellipsoid instead of a sphere.\
o	Correction: This involves fitting an ellipsoid to the data points and then transforming it into a sphere. This process adjusts the scale and alignment of the data to correct for the distortions1.

3.	**Magnetic Declination:**\
o	Cause: The difference between magnetic north and true north varies depending on your location on Earth.\
o	Correction: This involves adjusting the magnetometer readings to account for the local magnetic declination, which can be obtained from geomagnetic models or online calculators1.

4.	**Ellipsoid Fitting:**\
o	Cause: Combined hard-iron and soft-iron distortions.\
o	Correction: This technique involves fitting an ellipsoid to the collected data points and then transforming it into a sphere. This method corrects for both types of distortions simultaneously.

5.	**Online Calibration:**\
o	Cause: Dynamic changes in the environment or sensor placement.\
o	Correction: This involves continuously updating the calibration parameters in real-time as the sensor operates, ensuring accurate readings even in changing conditions.

Ellipsoid Fitting addresses issues identified in Hard Iron and Soft Iron distortions.

Recommended readings:

1.	Learn more about magnetometer models and HSI calibration · VectorNav
2.	Tutorial: How to calibrate a compass (and accelerometer) with Arduino: https://thecavepearlproject.org/2015/05/22/calibrating-any-compass-or-accelerometer-for-arduino/
3.	[Calibrating an eCompass in the Presence of Hard- and Soft-Iron Interference – NXP AN4246](https://www.nxp.com/docs/en/application-note/AN4246.pdf)

Other than Ellipsoid/Sphere fitting another method is available:

The **6-point tumble calibration** is a method used to calibrate magnetometers (and other 3-axis sensors like accelerometers) by measuring the sensor’s output in six different orientations. This technique helps to determine and correct for offsets, gains, and cross-axis sensitivities. Here’s a step-by-step overview:

1.	Positioning: The sensor is placed in six different orientations:\
o	+X axis up\
o	-X axis up\
o	+Y axis up\
o	-Y axis up\
o	+Z axis up\
o	-Z axis up

2.	Data Collection: In each position, the sensor’s output is recorded. Ideally, these measurements should reflect the true magnetic field vector in each orientation.

3.	Calculations:\
o	Offsets: The average of the positive and negative measurements for each axis gives the offset. For example, the offset for the X-axis is calculated as ((X_{+} + X_{-}) / 2).
o	Gains: The difference between the positive and negative measurements for each axis gives the gain. For example, the gain for the X-axis is calculated as ((X_{+} - X_{-}) / 2).\
o	Cross-axis Sensitivities: These are calculated to correct for any misalignment between the sensor axes.

4.	Correction: The calculated offsets, gains, and cross-axis sensitivities are then used to correct the raw sensor data, ensuring accurate readings.

This method is particularly useful because it doesn’t require any special equipment or reference fields, just the ability to accurately position the sensor in the six specified orientations12.


 
