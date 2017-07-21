/*
 * Copyright (c) 2013-2015, Riccardo Monica
 *   RIMLab, Department of Information Engineering, University of Parma, Italy
 *   http://www.rimlab.ce.unipr.it/
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted
 * provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this list of conditions
 * and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list of
 * conditions and the following disclaimer in the documentation and/or other materials provided with
 * the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors may be used to
 * endorse or promote products derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
 * FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
 * IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef PARAMETERS_H
#define PARAMETERS_H

// Input ROS topics
#define PARAM_NAME_PREFIX_TOPIC           "prefix_topic"
#define PARAM_DEFAULT_PREFIX_TOPIC        "/camera"

#define PARAM_NAME_DEPTH_IMAGE_TOPIC      "depth_image_topic"
#define PARAM_DEFAULT_DEPTH_IMAGE_TOPIC   "/depth/image_raw"

#define PARAM_NAME_CAMERA_INFO_TOPIC      "camera_info_topic"
#define PARAM_DEFAULT_CAMERA_INFO_TOPIC   "/depth/camera_info"

#define PARAM_NAME_IMAGE_TOPIC            "image_topic"
#define PARAM_DEFAULT_IMAGE_TOPIC         "/rgb/image_color"

#define PARAM_NAME_IN_RESET_TOPIC        "require_reset_topic"
#define PARAM_DEFAULT_IN_RESET_TOPIC     "/kinfu_require_reset_topic"

#define PARAM_NAME_REQUEST_TOPIC         "request_topic"
#define PARAM_DEFAULT_REQUEST_TOPIC      "/kinfu_request_topic"

#define PARAM_NAME_REQUEST_ACTION_NAME    "request_action_name"
#define PARAM_DEFAULT_REQUEST_ACTION_NAME "/kinfu_output/actions/request"

#define PARAM_NAME_COMMAND_TOPIC         "command_topic"
#define PARAM_DEFAULT_COMMAND_TOPIC      "/kinfu_command_topic"

// only when PARAM_NAME_FORCED_TF_POSITION is enabled (by param or by command)
#define PARAM_NAME_FORCED_REFERENCE_FRAME "forced_tf_reference_frame"
#define PARAM_DEFAULT_FORCED_REFERENCE_FRAME ""

#define PARAM_NAME_FORCED_CURRENT_FRAME   "forced_tf_current_frame"
#define PARAM_DEFAULT_FORCED_CURRENT_FRAME ""

#define PARAM_NAME_REQUEST_ACTION_MAGIC   "kinfu_output_request_action_magic"
#define PARAM_DEFAULT_REQUEST_ACTION_MAGIC "KINFU_OUTPUT_ACTION_MAGIC_SOURCE_NAME"

// kinfu parameters
#define PARAM_NAME_VOLUME_SIZE            "volume_size"
#define PARAM_SNAME_VOLUME_SIZE           "vs"
#define PARAM_DEFAULT_VOLUME_SIZE         (double(3.0))

#define PARAM_NAME_SHIFT_DISTANCE         "shift_distance"
#define PARAM_SNAME_SHIFT_DISTANCE        "sd"
#define PARAM_DEFAULT_SHIFT_DISTANCE      (double(1.5))

#define PARAM_NAME_CUDA_DEVICE_ID         "cuda_device_id"
#define PARAM_DEFAULT_CUDA_DEVICE_ID      (int(0))

#define PARAM_NAME_SNAPSHOT_RATE          "snapshot_rate"
#define PARAM_SNAME_SNAPSHOT_RATE         "sr"
#define PARAM_DEFAULT_SNAPSHOT_RATE       (int(45))

#define PARAM_NAME_EXTRACT_TEXTURES       "extract_textures"
#define PARAM_SNAME_EXTRACT_TEXTURES      "et"
#define PARAM_DEFAULT_EXTRACT_TEXTURES    (bool(false))  // NOT IMPLEMENTED

#define PARAM_NAME_DEPTH_WIDTH            "depth_width"
#define PARAM_DEFAULT_DEPTH_WIDTH         640

#define PARAM_NAME_DEPTH_HEIGHT           "depth_height"
#define PARAM_DEFAULT_DEPTH_HEIGHT        480

#define PARAM_NAME_MARCHING_CUBE_SIZE     "marching_cubes_volume_size"
#define PARAM_DEFAULT_MARCHING_CUBE_SIZE  0 // if 0, VOLUME_X (512) will be used

// if true, the kinfu engine will start automatically
// otherwise, it will wait for a COMMAND_TYPE_RESUME
#define PARAM_NAME_AUTOSTART              "autostart"
#define PARAM_DEFAULT_AUTOSTART           (bool(true))

#define PARAM_NAME_EXTRACT_KNOWN_POINTS    "extract_known_points"
#define PARAM_DEFAULT_EXTRACT_KNOWN_POINTS (bool(false))

#define PARAM_NAME_EXTRACT_BORDER_POINTS    "extract_incomplete_border_points"
#define PARAM_DEFAULT_EXTRACT_BORDER_POINTS (bool(false))

#define PARAM_NAME_EXTRACT_FRONTIER_POINTS    "extract_incomplete_frontier_points"
#define PARAM_DEFAULT_EXTRACT_FRONTIER_POINTS (bool(false))

#define PARAM_NAME_ENABLE_MIN_MOTION      "enable_minimum_motion"
#define PARAM_DEFAULT_ENABLE_MIN_MOTION   (bool(true))

// if true, the kinect position will be read as forced hint from tf
// (as specified: PARAM_NAME_FORCED_REFERENCE_FRAME and PARAM_NAME_FORCED_CURRENT_FRAME)
#define PARAM_NAME_FORCED_TF_POSITION     "forced_tf_position"
#define PARAM_DEFAULT_FORCED_TF_POSITION  (bool(false))

// Output ROS topics
#define PARAM_NAME_TF_REFERENCE_FRAME     "first_frame_reference_name"
#define PARAM_DEFAULT_TF_REFERENCE_FRAME  "/kinfu_first_frame"

#define PARAM_NAME_TF_CURRENT_FRAME       "current_frame_reference_name"
#define PARAM_DEFAULT_TF_CURRENT_FRAME    "/kinfu_current_frame"

#define PARAM_NAME_CURRENT_VIEW_TOPIC     "current_view_topic"
#define PARAM_DEFAULT_CURRENT_VIEW_TOPIC  "/kinfu_current_view"

#define PARAM_NAME_ICP_LOST_TOPIC         "icp_lost_topic"
#define PARAM_DEFAULT_ICP_LOST_TOPIC      "/kinfu_icp_lost_topic"

#define PARAM_NAME_RESPONSE_TOPIC         "response_topic"
#define PARAM_DEFAULT_RESPONSE_TOPIC      "/kinfu_response_topic"

#define PARAM_NAME_COMMAND_ACK_TOPIC      "command_ack_topic"
#define PARAM_DEFAULT_COMMAND_ACK_TOPIC   "/kinfu_command_ack_topic"

#endif // PARAMETERS_H
