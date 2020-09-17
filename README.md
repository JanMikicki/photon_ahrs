# photon_drone

Code for quadcopter control with Particle Photon as flight controlewr and MPU 9255 10-DoF sensor

It uses SparkFun library for MPU 9255 mag, gyro and accel integration (madgwick) and then uses PIDs to keep attitude.

Particle Cloud is utilized for troubleshooting, trimming and some preliminary joystick input. 
