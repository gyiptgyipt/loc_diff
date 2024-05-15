# The package name means (localization + differential wheeled robot) 

# navigation2_gps_waypoints_follower_tutorials
Tutorial code referenced from https://navigation.ros.org/

This custom robot integrated with gps,imu,depth_camera plugins.(You can find the plugin integration in description/urdf/black_donut.xacro)

-Add [the Sonoma_Raceway model zip file](https://github.com/Htet-Wai-Yan-HWY/loc_diff/blob/main/Sonoma%20Raceway.zip) to your ~/.gazebo/model/ and extrect it to load in the gazeo world.

-Run gazebo default(empty_world) and find Sonoma_Raceway in insert tab in the top-left corner of gazebo.
-And Save that world in (your_workspace)/gazebo/world/ .

### Build the package in your workspace.

-go to your workspace

```
colcon build --symlink-install
```

```
source install/setup.bash
```

### Now you can launch the sensors integrated robot simulation 

```
ros2 launch rlbd_gazebo black_donut_sim_ros2_control.launch.py
```

```
ros2 launch rlbd_gazebo mapviz.launch
```

```
ros2 launch rlnav2 gps_waypoint_follower.launch.py
```

[YOUTUBE_VIDEO](https://www.youtube.com/watch?v=lSZ8QRvmXd8&t=50s)

..........<to_continue>........

