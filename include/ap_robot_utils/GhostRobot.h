/**
 * \file GhostRobot.h
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

#ifndef GHOSTROBOT_H
#define GHOSTROBOT_H

#include <RobotKin/Robot.h>
#include <RobotKin/urdf_parsing.h>

#include <ros/ros.h>
#include <urdf/model.h>
#include <resource_retriever/retriever.h>
#include <visualization_msgs/MarkerArray.h>

#include "ap_robot_utils/eigen_definitions.h"
#include "ap_robot_utils/shared_ptr.h"

class GhostRobot
{
public:
	GhostRobot(const ap::shared_ptr<urdf::Model>& robotModel, const std_msgs::ColorRGBA& color);

	visualization_msgs::MarkerArray visualizeJoints(const Eigen::VectorX& q);
protected:
	RobotKin::Linkage mLinkage;
	std_msgs::ColorRGBA mColor;
	std::map<std::string, std::string> mMeshResources;
};

#endif // GHOSTROBOT_H
