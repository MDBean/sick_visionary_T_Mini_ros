^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package sick_visionary
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
0.2.2 (2020-12-02)
------------------
* support for Visionary-T Mini
* Fix bug for Visionary-S pointcloud data
* Contributors: Andreas Richert, Timo Rautenberg

0.2.1 (2019-12-23)
------------------
* adapted VisionayControl module to control also through CoLa 2 besides CoLa B
* support for control of Visionary-T VGA
* Contributors: Andreas Richert

0.2.0 (2019-11-03)
------------------
* support for Visionary-T VGA
* Contributors: Andreas Richert

0.1.1 (2017-12-06)
------------------
* increased performance
* support cartesian data e.g. for Visionary-T DT or AG
* support polar data
* Contributors: Andreas Richert

0.1.0 (2017-09-01)
------------------
* rename to visionary
* usage of Visionary_Common library
* support for Visionary-S
* Contributors: Andreas Richert

0.0.2 (2016-10-07)
------------------
* rename to visionary_t
* Contributors: Florian Weisshardt

0.0.1 (2016-08-02)
------------------
* fixed warning
* added parameter:prevent_frame_skipping
  f true: prevents skipping of frames and publish everything, otherwise use newest data to publish to ROS world
* using pointer instead of copy constructor (optimization)
* warning instead of debug msg
* fixed receive queue
* added functions to check header + size before parsing
* debug messaging
* check data header
* detach publishing data from network thread
* Renamed launch file to match the driver name
* Updated formatting and adding parameter comment in launch file.
* initial version of sick_visionary_t_driver ROS driver
* Contributors: Florian Weisshardt, Joshua Hampp, Marco Dierschke
