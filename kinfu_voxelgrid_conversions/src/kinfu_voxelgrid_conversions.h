/*
 * Copyright (c) 2015, Riccardo Monica
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

#ifndef KINFU_VOXELGRID_CONVERSIONS_H
#define KINFU_VOXELGRID_CONVERSIONS_H

#define PARAM_NAME_INPUT_TOPIC                       "input_topic"
#define PARAM_DEFAULT_INPUT_TOPIC                    "/kinfu_voxelgrid_to_collision_map/input"

#define PARAM_NAME_OUTPUT_TOPIC                      "output_topic"
#define PARAM_DEFAULT_OUTPUT_TOPIC                   "/kinfu_voxelgrid_to_collision_map/output"

#define PARAM_NAME_OUTPUT_TYPE                       "output_type"
#define PARAM_VALUE_OUTPUT_TYPE_ARM_NAVIGATION_MSGS  "arm_navigation_msgs/CollisionMap"
#define PARAM_VALUE_OUTPUT_TYPE_MOVEIT_MSGS          "moveit_msgs/PlanningScene"
#define PARAM_VALUE_OUTPUT_TYPE_POINTCLOUD           "sensor_msgs/PointCloud2"
#define PARAM_DEFAULT_OUTPUT_TYPE                    PARAM_VALUE_OUTPUT_TYPE_ARM_NAVIGATION_MSGS

#define PARAM_NAME_CENTER                            "center"
#define PARAM_DEFAULT_CENTER                         "0.0 0.0 0.0"

#define PARAM_NAME_INPUT_SCALE                       "input_scale"
#define PARAM_DEFAULT_INPUT_SCALE                    (double(0.005859375))

#define PARAM_NAME_OUTPUT_SCALE                      "output_scale"
#define PARAM_DEFAULT_OUTPUT_SCALE                   (double(0.005859375))

#define PARAM_NAME_FRAME_ID                          "frame_id"
#define PARAM_DEFAULT_FRAME_ID                       "/world"

#define PARAM_NAME_COMPRESS                          "compress"
#define PARAM_DEFAULT_COMPRESS                       (bool(true))

// warning: carving may reduce effectiveness of compression
#define PARAM_NAME_CARVE                             "carve"
#define PARAM_DEFAULT_CARVE                          (bool(false))

// use for PARAM_VALUE_OUTPUT_TYPE_POINTCLOUD to get proper intensity
#define PARAM_NAME_KEEP_VOXEL_TYPE                   "keep_voxel_type"
#define PARAM_DEFAULT_KEEP_VOXEL_TYPE                (bool(false))

#define PARAM_NAME_COLOR                             "color"
#define PARAM_DEFAULT_COLOR                          "1.0 0.5 0.0 0.8"

#define PARAM_NAME_MOVEIT_NAMESPACE                  "moveit_namespace"
#define PARAM_DEFAULT_MOVEIT_NAMESPACE               "my_collision_map"

#define PARAM_NAME_COLLISION_VALUES                  "collision_values"
#define PARAM_VALUE_COLLISION_UNKNOWN                'u'
#define PARAM_VALUE_COLLISION_OCCUPIED               'o'
#define PARAM_VALUE_COLLISION_EMPTY                  'e'
#define PARAM_DEFAULT_COLLISION_VALUES               "uo"

#endif // KINFU_VOXELGRID_CONVERSIONS_H
