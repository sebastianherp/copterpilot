DO NOT USE THIS CODE YET
========================

I warned you ... this is my repository for code
for a small Arduino based quadrocopter I am building.

I don't expect this thing to fly very soon (some parts missing), but
that doesn't stop me from playing with my Arduino and some sensors.

Goal
----

Building a working quadrocopter for not too much money with the following features:
*   gyros and accelerometers to fly the thing
*   magnetometer to compensate drift in yaw axis
*   barometer for holding position (height)
*   LED show (8 RGB channels, 2 for each rim)
*   Buzzer
*   Voltage divider to check the battery voltage

Future features:
*  GPS for position hold
*  ultrasonic/distance sensors for automatic landing and calibrating the barometer
*  camera for FPV-flights
*  GPS and camera possibly via strapped on Nexus S (enables sending commands via the network)

Content
-------
pilot: the Arduino sketch  
libraries/BMP085: code for the barometer  
libraries/IMU3000: code for the imu3000 (gyro & accelerometer)  
libraries/LSM303DLH: code fot the magnetometer (compass)  
libraries/FreeIMU: code for the IMU-stuff ... not really working  

everything is forked from somewhere, I'll add the sources ... promised