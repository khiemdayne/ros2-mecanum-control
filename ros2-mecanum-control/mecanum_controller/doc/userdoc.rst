mecanum_controller
=====================

Controller for mobile robots with mecanum drive (4 wheels at the moment).

As input it takes velocity commands for the robot body, which are translated to wheel commands for the mecanum drive base.

Odometry is computed from hardware feedback or from the command input and can be published.


Other features
--------------

   + Realtime-safe implementation.
   + Odometry publishing
   + Task-space velocity, acceleration and jerk limits
   + Automatic stop after command time-out
