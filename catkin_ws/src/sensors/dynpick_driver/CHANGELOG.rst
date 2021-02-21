^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package dynpick_driver
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.2.0 (2017-09-22)
------------------
* Fix ROS buildfarm error

  * revert `#27 <https://github.com/tork-a/dynpick_driver/issues/27>`_  (`#40 <https://github.com/tork-a/dynpick_driver/issues/40>`_ )
  * add header file and use `fputs` instaed of `fprintf` to avoid compile error
     * warning: implicit declaration of function
     * warning: format not a string literal and no format arguments

* [sample.launch] args improvement (`#38 <https://github.com/tork-a/dynpick_driver/issues/38>`_)

  * [sample.launch] Choose running RViz or not.
  * [sample.launch] pass wrench topic name.

* Adding more build-in features by sensor (`#39 <https://github.com/tork-a/dynpick_driver/issues/39>`_)

  * Fix reading problems with A6200 version
    the reply seems to differ for some reason, however this fix it by cleaning up the socket after sensitivity request and filter request
  * Set built-in filter at startup
  * Add automatical adjustment of LSB/N

    * generalized "readFromSocket" function
    * flushing socket at beginning (to avoid byte offset)
    * receiving calibration (LSB/N and LSB/Nm) from sensor and multiplying it to data
    * tabs vs. spaces cleanup

* Contributors: Isaac I.Y. Saito, Kei Okada, Lorenz Halt

0.1.1 (2016-12-20)
------------------
* [fix] Add a missing folder to be installed `#37 <https://github.com/tork-a/dynpick_driver/issues/37>`_
* [capability] Add a tested product model. `#35 <https://github.com/tork-a/dynpick_driver/issues/35>`_
* Contributors: Isaac I.Y. Saito

0.1.0 (2016-09-15)
------------------
* ADD urdf model `#32 <https://github.com/tork-a/dynpick_driver/issues/32>`_
  * Changed sample launch to show model instead of static tf-frame
* Add tare service `#30 <https://github.com/tork-a/dynpick_driver/issues/30>`_
  * uses std::srvs::Trigger
  * ordered dependencies alphabetically
  * Need to send offset_reset command several times (3 at my tests) to work
* Contributors: Lorenz Halt

0.0.11 (2016-06-02)
-------------------
* Workaround for ROS buildfarm error. `#27 <https://github.com/tork-a/dynpick_driver/issues/27>`_
* Contributors: Isaac I.Y. Saito

0.0.10 (2016-06-02)
-------------------
* [feat] Add test binary for measuring device throughput on Ubuntu (tested against QNX). `#25 <https://github.com/tork-a/dynpick_driver/issues/25>`_
* [doc] Add a tested product ID. `#26 <https://github.com/tork-a/dynpick_driver/issues/26>`_
* Contributors: Isaac I.Y. Saito

0.0.9 (2016-03-23)
------------------
* [fix] run.launch not publishing Wrench topic issue.
* [feat] Rename run.launch --> driver.launch
* [sys] add test
* [sys] add travis.yml
* [doc] Switched to sphinx; this doxygen setting seems not working, and the doc we want by rosdoc is not about api docs but rst files. (`#20 <https://github.com/tork-a/dynpick_driver/issues/20>`_)
* Contributors: Kei Okada, Isaac I.Y. Saito

0.0.8 (2015-08-14)
------------------
* [doc] Utilize rosdoc_lite.
* Contributors: Isaac I.Y. Saito

0.0.7 (2015-08-12)
------------------
* [feat] Whether or not running RViz
* Contributors: Isaac I.Y. Saito

0.0.6 (2015-08-11)
------------------
* [fix] Missing launch file variables
* [doc] Add more doc
* Contributors: Ron Tajima, Isaac I.Y. Saito

0.0.5 (2015-04-01)
------------------
* (Feature) Do while to read enough data (fix for `#1 <https://github.com/tork-a/dynpick_driver/issues/1>`_)
* (Doc) Add to readme new launch usage.
* Contributors: Kei Okada, Isaac I.Y. Saito

0.0.4 (2015-03-23)
------------------
* Reusable launch files.
* Contributors: Isaac I.Y. Saito

0.0.3 (2014-12-29)
------------------
* Fix/Typo, this may break downstream packages `#7 <https://github.com/tork-a/dynpick_driver/issues/7>`_ from k-okada/fix_typo
* Fix/Install missing resource. `#6 <https://github.com/tork-a/dynpick_driver/issues/6>`_ from 130s/fix/install_launch
* Contributors: Kei Okada, Isaac I.Y. Saito

0.0.2 (2014-12-24)
------------------
* Initial commit
* Add doc
* Contributors: Kei Okada, Isaac I.Y. Saito
