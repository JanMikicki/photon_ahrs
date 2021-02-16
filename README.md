# photon_drone

(https://www.youtube.com/watch?v=fUP6soGiDcI)

Code for quadcopter control with Particle Photon as flight controller and MPU 9255 10-DoF sensor board.

It uses SparkFun library for MPU 9255 mag, gyro and accel integration (Madgwick algorithm) and then uses PIDs to keep attitude.

You can use joystick input via UDP/TCP.

Particle Cloud is utilized for troubleshooting, trimming etc.
