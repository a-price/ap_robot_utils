/**
 * \file ros_utils.cpp
 * \brief
 *
 * \author Andrew Price
 * \date 9 9, 2014
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

#include "ap_robot_utils/ros_utils.h"
#include <cstring>
#include <ros/package.h>
#include <urdf/model.h>

#include <RobotKin/Robot.h>
#include <RobotKin/Robots/Triage.h>
#include <RobotKin/urdf_parsing.h>

#include <fstream>

namespace ap
{

const static std::string PACKAGE_PREFIX = "package://";

std::string parsePackageURL(const std::string& url)
{
	size_t is_package = url.find(PACKAGE_PREFIX);
	if (std::string::npos == is_package)
	{
		// Not a package path
		return url;
	}

	std::string filename = url;
	filename.erase(0, PACKAGE_PREFIX.length());
	size_t pos = filename.find("/");
	if (pos != std::string::npos)
	{
		std::string package = filename.substr(0, pos);
		filename.erase(0, pos);
		std::string package_path = ros::package::getPath(package);
		filename = package_path + filename;
	}

	return filename;
}

std::string packagePathToContents(const std::string& filename)
{
	std::string path;
	path = ap::parsePackageURL(filename);
	std::ifstream ifs(path);
	std::stringstream buffer;
	buffer << ifs.rdbuf();
	ifs.close();
	return buffer.str();
}

ap::shared_ptr<RobotKin::Robot> loadRKRobot(const ros::NodeHandle& nh)
{
	ap::shared_ptr<urdf::Model> robotModel(new urdf::Model);

	std::string robotDescription;
	if (!nh.getParam("/robot_description", robotDescription))
	{
		throw std::runtime_error("Parameter for robot description not provided");
	}

	if (!robotModel->initString(robotDescription))
	{
		throw std::runtime_error("Unable to parse URDF model.");
	}
	else
	{
		ROS_INFO_STREAM("Loaded URDF model '" << robotModel->getName() << "'.");
	}

	ap::shared_ptr<RobotKin::Robot> rkRobot(new RobotKin::Robot);
	RobotKin::URDF::loadURDFModel(*rkRobot, robotModel);

	RobotKin::Linkage* pLinkage = &rkRobot->linkage("base_link"); // Default to "base_link"
	if ("invalid" == pLinkage->name())
	{
		if (rkRobot->linkages().size() < 1)
		{
			throw std::runtime_error("No linkages in robot model.");
		}
		else
		{
			pLinkage = rkRobot->linkages()[0];
		}
	}

	std::map<std::string, RobotKin::AnalyticalIK>::const_iterator iter = RobotKin::KNOWN_AIK_SOLVERS.find(robotModel->getName());
	if (RobotKin::KNOWN_AIK_SOLVERS.end() != iter)
	{
		pLinkage->analyticalIK = iter->second;
		ROS_INFO_STREAM("Loaded AIK solver for '" << pLinkage->name() << "'.");
	}
	else
	{
		ROS_ERROR_STREAM("No solver found for '" << robotModel->getName() << "'.");
	}

	return rkRobot;
}

ap::shared_ptr<RobotKin::Robot> loadRKRobot(const std::string& filename)
{
	std::string expandedFilename = parsePackageURL(filename);

	ap::shared_ptr<urdf::Model> robotModel(new urdf::Model);

	std::ifstream ifs(expandedFilename);
	std::string robotDescription((std::istreambuf_iterator<char>(ifs)), std::istreambuf_iterator<char>());

	if (!robotModel->initString(robotDescription))
	{
		throw std::runtime_error("Unable to parse URDF model.");
	}
	else
	{
//		ROS_INFO_STREAM("Loaded URDF model '" << robotModel->getName() << "'.");
	}

	ap::shared_ptr<RobotKin::Robot> rkRobot(new RobotKin::Robot);
	RobotKin::URDF::loadURDFModel(*rkRobot, robotModel);

	RobotKin::Linkage* pLinkage = &rkRobot->linkage("base_link"); // Default to "base_link"
	if ("invalid" == pLinkage->name())
	{
		if (rkRobot->linkages().size() < 1)
		{
			throw std::runtime_error("No linkages in robot model.");
		}
		else
		{
			pLinkage = rkRobot->linkages()[0];
		}
	}

	std::map<std::string, RobotKin::AnalyticalIK>::const_iterator iter = RobotKin::KNOWN_AIK_SOLVERS.find(robotModel->getName());
	if (RobotKin::KNOWN_AIK_SOLVERS.end() != iter)
	{
		pLinkage->analyticalIK = iter->second;
//		ROS_INFO_STREAM("Loaded AIK solver for '" << pLinkage->name() << "'.");
	}
	else
	{
//		ROS_ERROR_STREAM("No solver found for '" << robotModel->getName() << "'.");
	}

	return rkRobot;
}

std::string timeStamp()
{
	std::time_t now = std::time(NULL);
	std::tm * ptm = std::localtime(&now);
	char timeStr[32];
	std::strftime(timeStr, 32, "%Y-%m-%d %H:%M:%S", ptm);
	return std::string(timeStr);
}

} // namespace ap
