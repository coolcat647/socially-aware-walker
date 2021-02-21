DynPick ROS driver
==================================

|img_model_wdf_6m200_3|

ROS driver for `Wacoh-tech force sensor <http://www.wacoh-tech.com/en/products/dynpick/>`_.

Prerequisite
==============

 * (Only tested on) Ubuntu 14.04 64bit.
 * ROS Indigo needs to be installed.

Install
==========

 1. Create `/etc/udev/rules.d/80-wakoh.rules` with the following line

  ::

    ## see https://github.com/tork-a/dynpick_driver
    ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE="0666"

 2. Run `$ sudo service udev restart`

 3. Plug the sensor USB to the Ubuntu computer. Also make sure that the following device was detected.

  ::

    $ lsusb
    :
    Bus 003 Device 004: ID 10c4:ea60 Cygnal Integrated Products, Inc. CP210x Composite Device

 4. Obtain `dynpick_driver` package.

 4-a. Via DEB (RECOMMENDED). Run `sudo apt-get install ros-indigo-dynpick-driver`.

 4-b. (Source build. Not recommended) clone this repository and build.::

  $ cd %YOUR_CATKING_WORKSPACE%/src
  $ git clone https://github.com/tork-a/dynpick_driver.git
  $ cd %YOUR_CATKING_WORKSPACE%
  $ catkin_make

Operation
==========

You can view the force and torque vectors in rviz by launching run.launch::

  $ roslaunch dynpick_driver sample.launch

 You can configure some things by passing arguments::

  $ roslaunch dynpick_driver sample.launch device:=/dev/ttyUSB0 rate:=500 sensor_frame_id:=/sensor/force rvizconfig:=`rospack find dynpick_driver`/launch/sample.rviz

 where:

 * `device`: port name of the connected sensor device
 * `rate`: refresh rate
 * `sensor_frame_id`: This name will be used as a `tf <http://wiki.ros.org/tf>`_ frame of the connected sensor device
 * `rvizconfig`: name of the `RViz configuration file <http://wiki.ros.org/rviz/UserGuide#Configurations>`_ that can be used for storing your own `RViz` setting
 * `topic`: name of the published topic (WrenchStamped). Default is '/force'

Video is posted on `ROS wiki <http://wiki.ros.org/action/subscribe/dynpick_driver>`_

Document
========

ROS API
--------

Published Topics
~~~~~~~~~~~~~~~~~~

`/force` (`geometry_msgs/WrenchStamped <http://docs.ros.org/api/geometry_msgs/html/msg/WrenchStamped.html>`_)

Services
~~~~~~~~~~~~~~~~~~

`/dynpick_driver_node/get_loggers` (`roscpp/GetLoggers <http://wiki.ros.org/roscpp>`_)

`/dynpick_driver_node/set_logger_level` (`roscpp/SetLoggerLevel <http://wiki.ros.org/roscpp>`_)

Parameters
~~~~~~~~~~~~~~~~~~

`/dynpick_driver_node/device`

  Device file location. e.g. `/dev/ttyUSB0`

`/dynpick_driver_node/frame_id`

  Sensor's `tf` frame id. Default: `/sensor`

`/dynpick_driver_node/rate`

  Publish rate. Default: 1000 hz

Documentation
--------

`API doc is available <http://docs.ros.org/indigo/api/dynpick_driver/html/>`_ also on `ros.org.<http://wiki.ros.org/dynpick_driver>`_

DEB build status
================

+--------------+-----------------------------------+------------------------------------+--------------------------------+--------------------------------------+--------------------------------------+-------------------------------+
| ROS Distro   | Development Branch (travis)       | Development branch (ros.org)       | Source deb                     | binarydeb Trusty i386                | binarydeb Trusty AMD64               | Documentation (ros.org)       |
+==============+===================================+====================================+================================+======================================+======================================+===============================+
| Indigo       | |indigo_buildstatus_devel_travis| | |indigo_buildstatus_devel_ros.org| | |indigo_buildstatus_sourcedeb| | |indigo_buildstatus_binarydeb_amd64| | |indigo_buildstatus_binarydeb_amd64| | |indigo_buildstatus_doc|      |
+--------------+-----------------------------------+------------------------------------+--------------------------------+--------------------------------------+--------------------------------------+-------------------------------+

`Devel Test Status <http://wiki.ros.org/regression_tests#Development_Tests>`_
-------------------------------------------------------------------------------------

ROS Indigo
|indigo_job_devel-indigo-dynpick_driver|


.. |img_model_wdf_6m200_3| image:: http://www.wacoh-tech.com/img/img_model_wdf_6m200_3.jpg
.. |indigo_buildstatus_devel_travis| image:: https://travis-ci.org/tork-a/dynpick_driver.png?branch=master
.. |indigo_buildstatus_devel_ros.org| image:: http://build.ros.org/job/Idev__dynpick_driver__ubuntu_trusty_amd64/badge/icon
.. |indigo_buildstatus_sourcedeb| image:: http://build.ros.org/job/Isrc_uT__dynpick_driver__ubuntu_trusty__source/badge/icon
.. |indigo_buildstatus_binarydeb_i386| image:: http://build.ros.org/job/Ibin_uT64__dynpick_driver__ubuntu_trusty_i386__binary/badge/icon
.. |indigo_buildstatus_binarydeb_amd64| image:: http://build.ros.org/job/Ibin_uT64__dynpick_driver__ubuntu_trusty_amd64__binary/badge/icon
.. |indigo_buildstatus_doc| image:: http://build.ros.org/job/Idoc__dynpick_driver__ubuntu_trusty_amd64/badge/icon

