from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    fiesta_node = Node(
        package="asr_sdm_esdf_map",
        executable="test_fiesta",
        name="asr_sdm_esdf_map",
        output="screen",
        emulate_tty=True,
        parameters=[
            {
                "resolution": 0.1,
                "update_esdf_every_n_sec": 0.1,
                "reserved_size": 1000000,
                "lx": -20.0,
                "ly": -20.0,
                "lz": -1.6,
                "rx": 20.0,
                "ry": 20.0,
                "rz": 2.0,
                "min_ray_length": 0.5,
                "max_ray_length": 5.0,
                "ray_cast_num_thread": 0,
                "center_x": 3.2247735741936430e+02,
                "center_y": 2.3707634648111778e+02,
                "focal_x": 3.8445808939187248e+02,
                "focal_y": 3.8398275569654390e+02,
                "p_hit": 0.70,
                "p_miss": 0.35,
                "p_min": 0.12,
                "p_max": 0.97,
                "p_occ": 0.80,
                "global_map": False,
                "global_update": False,
                "global_vis": False,
                "radius_x": 3.0,
                "radius_y": 3.0,
                "radius_z": 1.5,
                "use_depth_filter": True,
                "depth_filter_tolerance": 0.1,
                "depth_filter_max_dist": 10.0,
                "depth_filter_min_dist": 0.1,
                "depth_filter_margin": 0,
                "visualize_every_n_updates": 10,
                "slice_vis_max_dist": 2.0,
                "slice_vis_level": 1.6,
                "vis_lower_bound": 0.0,
                "vis_upper_bound": 10.0,
            }
        ],
        remappings=[
            ("depth", "/camera/depth/image_rect_raw"),
            ("transform", "/vins_estimator/camera_pose"),
        ],
    )

    return LaunchDescription([fiesta_node])
