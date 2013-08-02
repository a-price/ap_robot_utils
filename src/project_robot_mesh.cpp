/**
 * \file project_robot_mesh.cpp
 * \brief
 *
 * \author Andrew Price
 * \date August 1, 2013
 *
 * \copyright
 *
 * Copyright (c) 2013, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Humanoid Robotics Lab Georgia Institute of Technology
 * Director: Mike Stilman http://www.golems.org
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
#include <image_transport/image_transport.h>
#include <opencv/cv.h>
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <tf/transform_listener.h>
#include <boost/foreach.hpp>
#include <sensor_msgs/image_encodings.h>
#include <urdf/model.h>

#include <assimp/mesh.h>
#include <assimp/scene.h>
#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>

#include <ap_robot_utils/geometry_utils.h>

class MeshProjector
{
public:
	MeshProjector(urdf::Model& robotModel, const std::string camera_frame_id)
		: mRobotModel(robotModel)
	{
		this->mCameraFrameID = camera_frame_id;
		//this->mCameraModel.cameraInfo().header.frame_id = camera_frame_id;

		for (std::pair<std::string, boost::shared_ptr<urdf::Link> >  linkPair : mRobotModel.links_)
		{
			boost::shared_ptr<urdf::Link> link = linkPair.second;
			mMeshFrameIDs.push_back(link->name);
		}
	}

	MeshProjector(urdf::Model& robotModel, const std::string camera_frame_id, const std::vector<std::string>& frame_ids)
		: mRobotModel(robotModel),
		  mMeshFrameIDs(frame_ids)
	{
		this->mCameraFrameID = camera_frame_id;
		//this->mCameraModel.cameraInfo().header.frame_id = camera_frame_id;
	}

	void init()
	{
		for (int frame = 0; frame = mMeshFrameIDs.size(); frame++)
		{
			boost::shared_ptr<const urdf::Link> link = mRobotModel.getLink(mMeshFrameIDs[frame]);

			if (!link->visual->geometry->type == urdf::Geometry::MESH)
			{
				continue;
			}

			boost::shared_ptr<urdf::Mesh>& mesh = (boost::shared_ptr<urdf::Mesh>&)link->visual->geometry;
			const aiScene *scene = importer.ReadFile(mesh->filename,
													 aiProcessPreset_TargetRealtime_Fast);

			meshes.push_back(*scene->mMeshes[0]);
		}
	}

	cv::Mat projectWithTF(tf::TransformListener& tfListener)
	{
		// For each pixel in camera image
		cv::Mat robotImage(mCameraModel.cameraInfo().width, mCameraModel.cameraInfo().height, CV_32F);
		for (int v = 0; v < robotImage.rows; v++)
		{
			for (int u = 0; u < robotImage.cols; u++)
			{
				// Create a ray through the pixel
				cv::Point2d pixel = cv::Point2d(u, v);
				cv::Point3d cvRay = mCameraModel.projectPixelTo3dRay(pixel);
				// Convert cvRay to ap::Ray
				ap::Ray ray;
				ray.point = Eigen::Vector3f::Zero();
				ray.vector.x() = cvRay.x; ray.vector.y() = cvRay.y; ray.vector.z() = cvRay.z;

				// For each frame in vector
				for (int frame = 0; frame = mMeshFrameIDs.size(); frame++)
				{
					// Lookup current transform
					tf::StampedTransform transform;
					tfListener.lookupTransform(mCameraFrameID, mMeshFrameIDs[frame], ros::Time(0), transform);


					// Get mesh for each frame

					// For each triangle in mesh
					// Check for intersection. If finite, set distance

				}

			}
		}

		// Return the matrix
		return robotImage;
	}

protected:
	std::string mCameraFrameID;
	std::vector<std::string> mMeshFrameIDs;

	urdf::Model mRobotModel;

	image_geometry::PinholeCameraModel mCameraModel;

	Assimp::Importer importer;
	std::vector<aiMesh> meshes;
};


int main(int argc, char** argv)
{
	ros::init(argc, argv, "project_robot_mesh");
	ros::NodeHandle m_nh;

	urdf::Model robotModel;

	std::string robotDescription;
	if (!m_nh.getParam("/robot_description", robotDescription))
	{
		ROS_FATAL("Parameter for robot description not provided");
	}
	robotModel.initString(robotDescription);

	MeshProjector(robotModel, "head_frame");

	ros::spin();
}
