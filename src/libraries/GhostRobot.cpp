/**
 * \file GhostRobot.cpp
 * \brief
 *
 * \author Andrew Price
 * \date 2014-11-21
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

#include "ap_robot_utils/GhostRobot.h"
#include "ap_robot_utils/pose_conversions.h"

GhostRobot::GhostRobot(const ap::shared_ptr<urdf::Model>& robotModel, const std_msgs::ColorRGBA& color)
{
	mColor = color;
	mColor.a = 0.5;
	RobotKin::Robot rkRobot;
	RobotKin::URDF::loadURDFModel(rkRobot, robotModel);
	mLinkage = rkRobot.linkage("base_link");
	if (mLinkage.name() == "invalid")
	{
		if (rkRobot.linkages().size() < 1)
		{
			ROS_FATAL("No linkages in robot model.");
			return;
		}
		else
		{
			mLinkage = rkRobot.linkage(0);
		}
	}

	for (std::pair<std::string, boost::shared_ptr<urdf::Link> > pair : robotModel->links_)
	{
		boost::shared_ptr<urdf::Link>& link = pair.second;
		if (link->visual->geometry->type == urdf::Geometry::MESH)
		{
			boost::shared_ptr<urdf::Mesh> mesh = boost::static_pointer_cast<urdf::Mesh>(link->visual->geometry);
			mMeshResources[pair.first] = mesh->filename;
		}
	}


}

visualization_msgs::MarkerArray GhostRobot::visualizeJoints(const Eigen::VectorX& q)
{
	mLinkage.setValues(q);

	const int nJoints = mLinkage.nJoints();
	visualization_msgs::MarkerArray markers;

	for (int j = 0; j < nJoints; ++j)
	{
		RobotKin::Joint& joint = mLinkage.joint(j);
		const Eigen::Isometry3& pose = joint.respectToLinkage();


		visualization_msgs::Marker m;
		m.action = visualization_msgs::Marker::ADD;
		m.type = visualization_msgs::Marker::MESH_RESOURCE;
		m.scale.x = 1; m.scale.y = 1; m.scale.z = 1;
		m.mesh_resource = mMeshResources[joint.name()];
		m.color = mColor;
		m.header.frame_id = mLinkage.name();
		m.header.stamp = ros::Time::now();
		m.pose = ap::toPose(pose);

		markers.markers.push_back(m);
	}

	return markers;
}
