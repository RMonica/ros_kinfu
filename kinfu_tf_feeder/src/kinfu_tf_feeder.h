/*
 * Copyright (c) 2014-2015, Riccardo Monica
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

#ifndef KINFU_TF_FEEDER_H
#define KINFU_TF_FEEDER_H

// input
#define PARAM_NAME_REFERENCE_FRAME     "reference_frame_id"
#define PARAM_DEFAULT_REFERENCE_FRAME  "/kinfu_tf_feeder/reference_frame"

#define PARAM_NAME_POSE_FRAME          "pose_frame_id"
#define PARAM_DEFAULT_POSE_FRAME       "/kinfu_tf_feeder/pose_frame"

// output
#define PARAM_NAME_KINFU_COMMAND_TOPIC "kinfu_command_topic"
#define PARAM_DEFAULT_KINFU_COMMAND_TOPIC "/kinfu_command_topic"

// other configurations
#define PARAM_NAME_POSE_TIMEOUT        "pose_timeout"
#define PARAM_DEFAULT_POSE_TIMEOUT     0.033

#define PARAM_NAME_START_KINFU         "start_kinfu"
#define PARAM_DEFAULT_START_KINFU      true

#define PARAM_NAME_RATE                "rate"
#define PARAM_DEFAULT_RATE             10

#define PARAM_NAME_FORCED              "forced"
#define PARAM_DEFAULT_FORCED           true

#define PARAM_NAME_TRIGGERED           "triggered"
#define PARAM_DEFAULT_TRIGGERED        false

#endif // KINFU_TF_FEEDER_H
