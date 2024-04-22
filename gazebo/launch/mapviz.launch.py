import launch
import launch.actions
import launch.substitutions
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="swri_transform",
            # args: x y z yaw=-1.13 pitch roll
            arguments=["0", "0", "0", "-1.13", "0", "0", "/wgs84", "map"],
            parameters=[
                {"use_sim_time": True}
            ]
        ),
        launch_ros.actions.Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="swri_transform",
            # args: x y z yaw=-1.13 pitch roll
            arguments=["0", "0", "0", "-1.13", "0", "0", "map", "world"],
            parameters=[
                {"use_sim_time": True}
            ]
        ),
        launch_ros.actions.Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="odom",
            # args: x y z yaw=-1.13 pitch roll
            arguments=["0", "0", "0", "0", "0", "0", "world", "odom"],
            parameters=[
                {"use_sim_time": True}
            ]
        ),
        launch_ros.actions.Node(
            package="swri_transform_util",
            executable="initialize_origin.py",
            name="initialize_origin",
            parameters=[
                {"local_xy_frame": "/wgs84"},
                {"local_xy_origin" : "auto"},
                {"local_xy_navsatfix_topic" : "/gps/fix"},
                # {"local_xy_navsatfix_topic" : "/fix"}
                {"use_sim_time": True}


            ],
        ),
        launch_ros.actions.Node(
            package="mapviz",
            executable="mapviz",
            name="mapviz",
        )

    ])


#https://robotics.stackexchange.com/questions/73919/mapviz-no-transform-between-wgs84-to-odom