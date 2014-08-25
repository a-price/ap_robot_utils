/**
 * \file test_dh_transforms.cpp
 * \brief
 *
 * \author Andrew Price
 * \date 8 6, 2014
 *
 * \copyright
 *
 * Copyright (c) 2014, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * This file is provided under the following "BSD-style" License:
 * Redistribution and use in source and binary forms, with or
 * without modification, are permitted provided that the following
 * conditions are met:
 * * Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above
 *   copyright notice, this list of conditions and the following
 *   disclaimer in the documentation and/or other materials provided
 *   with the distribution.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 * USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <ap_robot_utils/DHParams.h>
#include <ap_robot_utils/pose_conversions.h>

void createTransforms(const std::vector<DHParams>& params, const std::vector<float>& angles, std::vector<tf::StampedTransform>& tfs)
{
	assert(params.size() == angles.size());
	tfs.clear();
	for (int i = 0; i < params.size(); ++i)
	{
		tf::StampedTransform stf;
		Eigen::Matrix4f transform = params[i].toRotationMatrix(angles[i]);
		stf.setData(ap::toTF(transform));
		stf.frame_id_ = std::to_string(i);
		stf.child_frame_id_ = std::to_string(i+1);
		stf.stamp_ = ros::Time::now();
		tfs.push_back(stf);
	}
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "dh_test");
	ros::NodeHandle nh;

	tf::TransformBroadcaster tb;

	std::vector<DHParams> params;
	//params.push_back(DHParams(0, 0, 0, 0));
	params.push_back(DHParams(0.335, -M_PI/2.0f, 0.075, 0));
	params.push_back(DHParams(0, 0, 0.270, -M_PI/2.0f));
	params.push_back(DHParams(0, M_PI/2.0f, -0.090, M_PI));
	params.push_back(DHParams(0.295, -M_PI/2.0f, 0, 0));
	params.push_back(DHParams(0, M_PI/2.0f, 0, 0));
	params.push_back(DHParams(0, M_PI/2.0f, 0, 0));

	std::vector<tf::StampedTransform> tfs;

	ros::Rate r(50);
	for (int i = 0; i < params.size(); ++i)
	{
		std::vector<float> angles(params.size(), 0.0f);
		for (int t = 0; t < 360; ++t)
		{
			angles[i] = t * M_PI/180.0f;
			createTransforms(params, angles, tfs);
			tb.sendTransform(tfs);
			ros::spinOnce();
			if (!ros::ok()) { break; }
			r.sleep();
		}
		if (!ros::ok()) { break; }
	}

	return 0;
}
