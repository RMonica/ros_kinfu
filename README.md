2015-01-03

Author: Riccardo Monica <rmonica[at]ce.unipr.it>  
  RIMLab, Department of Information Engineering, University of Parma, Italy  
  <http://www.rimlab.ce.unipr.it/>

Introduction
------------

This repository contains a KinFu Large Scale wrapper for ROS (Robot Operating System, www.ros.org).

KinFu is the KinectFusion implementation by PCL (Point Cloud Library, <www.pointclouds.org>).

The original version of this wrapper was developed by Michael Korn &lt;michael.korn(at)uni-due.de&gt; and published at <http://fsstud.is.uni-due.de/svn/ros/is/kinfu/>. That version only published the tracked reference frame to TF and the current synthetic depth map.

During my research work, I have added several features. These include:

- World model download as ROS message
- Known voxels download as ROS message
- Commands to start, stop, run only once, reset
- Use external tracking, from ROS messages or TF frames
- Clear part of the world model, defined by a sphere, a bounding box or a cylinder
- Request for synthetic depth maps, projected from arbitrary locations
- Evaluation of synthetic depth maps (Next Best View)
- Detection of unknown borders and frontiers
- Integration of empty (missed) measurements

As of 2016, PCL KinFu Large Scale source code was integrated into the repository, as PCL support for KinFu is unknown.

As of 2018-03-12, the integrated source code diverged significantly from original KinFu, due to many off-by-one errors which were discovered and fixed in the rolling buffer code.
The code is not backward compatible with original KinFu.

In this repository, there are four ROS packages, explained in the following sections.

kinfu\_msgs
-----------

Message definitions for the other packages.

kinfu
-----

The core package. This package runs KinFu and the extensions. Since PCL dropped support for KinFu, the source code has been copied in the package. The simple installation procedure is detailed in `kinfu/INSTALL.txt`.

By default, depth images should be sent to `/camera/depth/image_raw`, and the corresponding camera info is `/camera/depth/camera_info`. Image width and height must also be set with parameters `depth_width` and `depth_height`.

Negative depth values in the depth image are converted into positive empty (miss) measurements, where the space is set to empty up to the measurement, but no occupied voxels are generated at the end.

Two message types may be sent to the `kinfu` node to access special functionalities: requests and commands.

Commands are used to change the KinFu behavior at runtime and are executed by sending a `kinfu_msgs/KinfuCommand` message to the topic `/kinfu_command_topic`. After its execution, a `std_msgs/String` ack message is sent to `/kinfu_command_ack_topic`, reporting the command_id as specified in the command, concatenated with string ":OK" if it succeeded or ":NO" otherwise.

A pose hint for the KinFu tracking may be added to a command. This can be combined with some commands to guarantee that they are executed and simultaneously the KinFu tracking is set to that position. For example, sending a `COMMAND_TYPE_RESUME` with a forced hint allows to resume the KinFu from a specific pose. Command `COMMAND_TYPE_TRIGGER` executes KinFu for exactly one iteration, with the pose provided.

It is possible to force the tracked pose to stick to a TF frame, thus disabling the internal ICP tracking. This may be done by setting the parameters `forced_tf_position` to true and `first_frame_reference_name` and `current_frame_reference_name` respectively to the reference frame and the Kinect frame. The same result may also be achieved at runtime with `COMMAND_TYPE_SET_FORCED_TF_FRAMES`.

Requests ask kinfu to publish parts of the internal representation, processed in a few ways.
See `kinfu_msgs/KinfuTsdfRequest.msg` for the message type.

For requests, the kinfu node offers two interfaces:
###message-based interface
Requests are sent to the kinfu node, through the topic defined by the parameter request_topic (default: `/kinfu_request_topic`).
Responses are published by the kinfu node into the topic specified by the request_source_name field in the request.

**Note**: The message-based interface is maintained mostly for backward compatibility, even if it has some niche use cases. Currently, the action-based interface is preferred. It was originally created to solve issues which are no longer existing (see *kinfu_output* below).

**Note**: Since ROS is unable to guarantee the delivery of a message sent by a just-created publisher, kinfu creates a latched publisher and keeps it alive until a subscriber is detected on the topic. If no subscriber is detected, the publisher is discarded after 30 seconds.

###action-based interface
This interface wraps the request/response mechanism of kinfu inside an action (actionlib).
The kinfu node creates an action server (by default, `/kinfu_output/actions/request`), of type `kinfu_msgs/Request.action`.
Multiple actions may be active at the same time and are executed concurrently, if possible (access to KinFu is always exclusive).

Sometimes, the response from KinFu may be too large for ROS, whose messages are limited to 1 GB. When this happens, the output is saved to a temporary file, and the response is empty except for the `file_name` field.

Parameters and their default values are listed in `kinfu/src/parameters.h`.

kinfu\_examples
---------------

**save_mesh**: a sample node which uses the action-based interface to request the current mesh 3D reconstruction and saves it into a PLY file.
**save_voxelgrid**: a sample node which requests the voxelgrid in a bounding box, shows how to access its elements, and saves it to a file.
**save_tsdf_volume**: a sample node that saves the TSDF volume as a point cloud.
**load_tsdf_volume**: a sample node that reloads the TSDF volume and the voxelgrid into a running instance of KinFu, overwriting the 3D reconstruction in a boundind box. Useful to save and reload the internal state.
**shift_checker_service**: a sample node showing how to trigger the shifting mechanism of KinFu from an external node. Also see `launch/kinfu_check_for_shift.launch`.

kinfu\_tf\_feeder
-----------------

The `kinfu_tf_feeder node` is a simple utility node that feeds the hints from TF by sending commands to the kinfu node. This allows for a greater flexibility than using the `forced_tf_position` parameter. The node may send the hint only if the TF frame is recent enough. In addition, it can use `COMMAND_TYPE_TRIGGER`, so a suspended kinfu node may be executed only when fresh TF data is available.

kinfu\_voxelgrid\_conversions
-----------------------------

This node can convert the `std_msgs/Float32MultiArray` message from `REQUEST_TYPE_GET_VOXELGRID` into:

- `sensor_msgs/PointCloud2`
- `arm_navigation_msgs/CollisionMap` (for OpenRAVE planner)
- `moveit_msgs/PlanningScene` (for MoveIt! planner)

The collision maps are built as a set of cubes. A few basic compression algorithms are available.

kinfu\_output
-------------

When the kinfu package was first created, in 2013, PCL and ROS were not fully independent yet. ROS would include (parts of) its own version of PCL, even when another version was compiled specifically to enable KinFu. This caused all sorts of compatibility problems. The `kinfu_output` node was used to solve some of these problems.

As of 2016, the source of KinFu is included in the package, and separation between ROS and PCL is complete on both Ubuntu 14.04 and Ubuntu 16.04. The `kinfu_output` node is not needed anymore.

Related publications
--------------------

1. Riccardo Monica, Jacopo Aleotti, Davide Piccinini, *Humanoid Robot Next Best View Planning Under Occlusions Using Body Movement Primitives*, IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), 2019
2. Riccardo Monica, Jacopo Aleotti, *A 3D Robot Self Filter for Next Best View Planning*, IEEE International Conference on Robotic Computing (IRC), 2019
3. R. Monica, J. Aleotti, *Contour-based next-best view planning from point cloud segmentation of unknown objects*, Autonomous Robots, Volume 42, Issue 2, February 2018, Pages 443-458
4. Riccardo Monica, Jacopo Aleotti, Stefano Caselli, *A KinFu based approach for robot spatial attention and view planning*, Robotics and Autonomous Systems, Volume 75, Part B, 2016

2021-06-21
