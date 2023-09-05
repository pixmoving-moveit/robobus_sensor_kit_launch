# Copyright 2023 Pixmoving, Inc. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


import launch
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.actions import SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode


def launch_setup(context, *args, **kwargs):
    # set scan ground filter as a component
    ground_segmentation_1 = ComposableNode(
        package="ground_segmentation",
        plugin="ground_segmentation::ScanGroundFilterComponent",
        name="ground_segmentation_1",
        remappings=[
                    ("input", "/sensing/lidar/front_left/ouster/points"),
                    ("output", "/sensing/lidar/front_left/ouster/points_no_ground"),
                ],
        parameters=[
                    {
                        "global_slope_max_angle_deg": 10.0,
                        "local_slope_max_angle_deg": 13.0, # recommended 30.0 for non elevation_grid_mode
                        "split_points_distance_tolerance": 0.2,
                        "use_virtual_ground_point": True,
                        "split_height_distance": 0.2,
                        "non_ground_height_threshold": 0.20,
                        "grid_size_m": 0.1,
                        "grid_mode_switch_radius": 20.0,
                        "gnd_grid_buffer_size": 4,
                        "detection_range_z_max": 2.5,
                        "elevation_grid_mode": True,
                        "use_recheck_ground_cluster": True,
                    }
                ],
        extra_arguments=[
            {"use_intra_process_comms": LaunchConfiguration("use_intra_process")}
        ],
    )
    # set scan ground filter as a component
    ground_segmentation_2 = ComposableNode(
        package="ground_segmentation",
        plugin="ground_segmentation::ScanGroundFilterComponent",
        name="ground_segmentation_2",
        remappings=[
                    ("input", "/sensing/lidar/front_right/ouster/points"),
                    ("output", "/sensing/lidar/front_right/ouster/points_no_ground"),
                ],
        parameters=[
                    {
                        "global_slope_max_angle_deg": 10.0,
                        "local_slope_max_angle_deg": 13.0, # recommended 30.0 for non elevation_grid_mode
                        "split_points_distance_tolerance": 0.2,
                        "use_virtual_ground_point": True,
                        "split_height_distance": 0.2,
                        "non_ground_height_threshold": 0.20,
                        "grid_size_m": 0.1,
                        "grid_mode_switch_radius": 20.0,
                        "gnd_grid_buffer_size": 4,
                        "detection_range_z_max": 2.5,
                        "elevation_grid_mode": True,
                        "use_recheck_ground_cluster": True,
                    }
                ],
        extra_arguments=[
            {"use_intra_process_comms": LaunchConfiguration("use_intra_process")}
        ],
    )
    # set scan ground filter as a component
    ground_segmentation_3 = ComposableNode(
        package="ground_segmentation",
        plugin="ground_segmentation::ScanGroundFilterComponent",
        name="ground_segmentation_3",
        remappings=[
                    ("input", "/sensing/lidar/rear_left/ouster/points"),
                    ("output", "/sensing/lidar/rear_left/ouster/points_no_ground"),
                ],
        parameters=[
                    {
                        "global_slope_max_angle_deg": 10.0,
                        "local_slope_max_angle_deg": 13.0, # recommended 30.0 for non elevation_grid_mode
                        "split_points_distance_tolerance": 0.2,
                        "use_virtual_ground_point": True,
                        "split_height_distance": 0.2,
                        "non_ground_height_threshold": 0.20,
                        "grid_size_m": 0.1,
                        "grid_mode_switch_radius": 20.0,
                        "gnd_grid_buffer_size": 4,
                        "detection_range_z_max": 2.5,
                        "elevation_grid_mode": True,
                        "use_recheck_ground_cluster": True,
                    }
                ],
        extra_arguments=[
            {"use_intra_process_comms": LaunchConfiguration("use_intra_process")}
        ],
    )
    # set scan ground filter as a component
    ground_segmentation_4 = ComposableNode(
        package="ground_segmentation",
        plugin="ground_segmentation::ScanGroundFilterComponent",
        name="ground_segmentation_4",
        remappings=[
                    ("input", "/sensing/lidar/rear_right/ouster/points"),
                    ("output", "/sensing/lidar/rear_right/ouster/points_no_ground"),
                ],
        parameters=[
                    {
                        "global_slope_max_angle_deg": 10.0,
                        "local_slope_max_angle_deg": 13.0, # recommended 30.0 for non elevation_grid_mode
                        "split_points_distance_tolerance": 0.2,
                        "use_virtual_ground_point": True,
                        "split_height_distance": 0.2,
                        "non_ground_height_threshold": 0.20,
                        "grid_size_m": 0.1,
                        "grid_mode_switch_radius": 20.0,
                        "gnd_grid_buffer_size": 4,
                        "detection_range_z_max": 2.5,
                        "elevation_grid_mode": True,
                        "use_recheck_ground_cluster": True,
                    }
                ],
        extra_arguments=[
            {"use_intra_process_comms": LaunchConfiguration("use_intra_process")}
        ],
    )
    # set concat filter as a component
    concat_component = ComposableNode(
        package="pointcloud_preprocessor",
        plugin="pointcloud_preprocessor::PointCloudConcatenateDataSynchronizerComponent",
        name="concatenate_data",
        remappings=[("output", "concatenated/pointcloud_unfilter")],
        parameters=[
            {
                "input_topics": [
                    "/sensing/lidar/front_left/ouster/points_no_ground",
                    "/sensing/lidar/rear_left/ouster/points_no_ground",
                    "/sensing/lidar/front_right/ouster/points_no_ground",
                    "/sensing/lidar/rear_right/ouster/points_no_ground",
                    "/sensing/ultra_sonic_radar/front_rear/pointcloud",
                    "/sensing/ultra_sonic_radar/left_right/pointcloud"
                ],
                "output_frame": LaunchConfiguration("base_frame"),
            }
        ],
        extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
    )
    # set crop box filter as a component
    cropbox_component = ComposableNode(
        package="pointcloud_preprocessor",
        plugin="pointcloud_preprocessor::CropBoxFilterComponent",
        name="crop_box_filter",
        remappings=[
            ("input", "concatenated/pointcloud_unfilter"),
            ("output", "concatenated/pointcloud_1"),
        ],
        parameters=[
            {
                "input_frame": LaunchConfiguration("base_frame"),
                "output_frame": LaunchConfiguration("base_frame"),
                "min_x": -2.05,
                "max_x": 2.05,
                "min_y": -1.2,
                "max_y": 1.2,
                "min_z": -0.5,
                "max_z": 2.5,
                "negative": True,
            }
        ],
    )
    cropbox_component_1 = ComposableNode(
        package="pointcloud_preprocessor",
        plugin="pointcloud_preprocessor::CropBoxFilterComponent",
        name="crop_box_filter_1",
        remappings=[
            ("input", "concatenated/pointcloud_1"),
            ("output", "concatenated/pointcloud"),
        ],
        parameters=[
            {
                "input_frame": LaunchConfiguration("base_frame"),
                "output_frame": LaunchConfiguration("base_frame"),
                "min_x": -100.0,
                "max_x": 100.0,
                "min_y": -50.0,
                "max_y": 50.0,
                "min_z": -0.4,
                "max_z": 3.0,
                "negative": False,
            }
        ],
    )
    # set container to run all required components in the same process
    container = ComposableNodeContainer(
        name=LaunchConfiguration("container_name"),
        namespace="",
        package="rclcpp_components",
        executable=LaunchConfiguration("container_executable"),
        composable_node_descriptions=[],
        condition=UnlessCondition(LaunchConfiguration("use_pointcloud_container")),
        output="screen",
    )

    target_container = (
        container
        if UnlessCondition(LaunchConfiguration("use_pointcloud_container")).evaluate(context)
        else LaunchConfiguration("container_name")
    )

    # load concat or passthrough filter
    concat_loader = LoadComposableNodes(
        # composable_node_descriptions=[concat_component, cropbox_component, cropbox_component_1],
        composable_node_descriptions=[ground_segmentation_1, ground_segmentation_2, ground_segmentation_3, ground_segmentation_4, concat_component, cropbox_component, cropbox_component_1],
        target_container=target_container,
        condition=IfCondition(LaunchConfiguration("use_concat_filter")),
    )

    return [container, concat_loader]


def generate_launch_description():
    launch_arguments = []

    def add_launch_arg(name: str, default_value=None):
        launch_arguments.append(DeclareLaunchArgument(name, default_value=default_value))

    add_launch_arg("base_frame", "base_link")
    add_launch_arg("use_multithread", "False")
    add_launch_arg("use_intra_process", "False")
    add_launch_arg("use_pointcloud_container", "True")
    add_launch_arg("container_name", "pointcloud_preprocessor_container")

    set_container_executable = SetLaunchConfiguration(
        "container_executable",
        "component_container",
        condition=UnlessCondition(LaunchConfiguration("use_multithread")),
    )

    set_container_mt_executable = SetLaunchConfiguration(
        "container_executable",
        "component_container_mt",
        condition=IfCondition(LaunchConfiguration("use_multithread")),
    )

    return launch.LaunchDescription(
        launch_arguments
        + [set_container_executable, set_container_mt_executable]
        + [OpaqueFunction(function=launch_setup)]
    )
