^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package velodyne_gazebo_plugins
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.9 (2019-03-08)
------------------
* Added min_intensity parameter to support cliping of low intensity returns
* Contributors: Jonathan Wheare, Kevin Hallenbeck

1.0.8 (2018-09-08)
------------------
* Changed iteration order to more closely represent the live velodyne driver
* Contributors: Kevin Hallenbeck

1.0.7 (2018-07-03)
------------------
* Added GPU support
* Added support for Gazebo 9
* Improved behavior of max range calculation
* Removed trailing slashes in robot namespace
* Fixed resolution of 1 not supported
* Fixed issue with only 1 vert or horiz ray
* Fixed cmake exports and warning
* Contributors: Kevin Hallenbeck, Jacob Seibert, Naoki Mizuno

1.0.6 (2017-10-17)
------------------
* Use robotNamespace as prefix for PointCloud2 topic frame_id by default
* Use Gazebo LaserScan message instead of direct LaserShape access, fixes timestamp issue
* Contributors: Kevin Hallenbeck, Max Schwarz, Micho Radovnikovich

1.0.5 (2017-09-05)
------------------
* Fixed ground plane projection by removing interpolation
* Contributors: Kevin Hallenbeck, Micho Radovnikovich

1.0.4 (2017-04-24)
------------------
* Updated package.xml format to version 2
* Removed gazebo_plugins dependency
* Contributors: Kevin Hallenbeck

1.0.3 (2016-08-13)
------------------
* Gazebo7 integration
* Contributors: Kevin Hallenbeck, Konstantin Sorokin

1.0.2 (2016-02-03)
------------------
* Display laser count when loading gazebo plugin
* Don't reverse ring for newer gazebo versions
* Changed to PointCloud2. Handle min and max range. Noise. General cleanup.
* Start from block laser
* Contributors: Kevin Hallenbeck
