^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package trimble_gnss_driver_ros2
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.1 (2024-06-07)
------------------
* change QOS
* Merge pull request `#5 <https://github.com/LCAS/trimble_gnss_driver_ros2/issues/5>`_ from ibrahimhroob/patch-1
  Update ros-ci.yml
* Update ros-ci.yml
  update branch name to ros2
* Merge pull request `#4 <https://github.com/LCAS/trimble_gnss_driver_ros2/issues/4>`_ from ibrahimhroob/ros2
  Added github workflow + refactor
* Merge pull request `#1 <https://github.com/LCAS/trimble_gnss_driver_ros2/issues/1>`_ from ibrahimhroob/dev_container
  Dev container
* remove dev container
* Merge branch 'GPrathap:ros2' into dev_container
* Create README.md
* use dev container template ros2 package
* Merge pull request `#3 <https://github.com/LCAS/trimble_gnss_driver_ros2/issues/3>`_ from ibrahimhroob/ros2
  gps_base_link 👯
* Update trimble_gnss_driver.launch.py
* Update trimble_gnss_driver.launch.py with heading offset param
* add heading offset to gsof_driver.py
* Merge pull request `#1 <https://github.com/LCAS/trimble_gnss_driver_ros2/issues/1>`_ from ibrahimhroob/patch-1
  add params for antenna pose
* add params for antenna pose
  getting the transformation from the tf transformation did not work, we where getting error that it could not find the transformation between the two links! however when we were trying to get the transformation from the terminal using this command: "ros2 run tf2_ros tf2_echo back_antenna_link front_antenna_link" we were able to get it! so as a fix for "now" we hardcoded those params!!!!!
* update tf buffer
* updating sim_time
* fixing quaternion estimation
* corrected rtk port
* init commit
* Contributors: GPrathap, Geesara Prathap Kulathunga, IH, Ibrahim Hroob
