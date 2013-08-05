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
#include <sensor_msgs/distortion_models.h>
#include <urdf/model.h>
#include <resource_retriever/retriever.h>

#include <assimp/mesh.h>
#include <assimp/scene.h>
#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>

#include <ap_robot_utils/geometry_utils.h>
#include <ap_robot_utils/pose_conversions.h>

image_geometry::PinholeCameraModel createIdealCamera()
{
	image_geometry::PinholeCameraModel cam;
	sensor_msgs::CameraInfo camInfo;
	camInfo.width = 640;
	camInfo.height = 480;

	//camInfo.distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;

	camInfo.K = (boost::array<double, 9ul>)
				{1.0, 0.0, camInfo.width/2.0,
				 0.0, 1.0, camInfo.height/2.0,
				 0.0, 0.0, 1.0};

	camInfo.P = (boost::array<double, 12ul>)
				{1.0, 0.0, camInfo.width/2.0, 0.0,
				 0.0, 1.0, camInfo.height/2.0, 0.0,
				 0.0, 0.0, 1.0, 0.0};

	camInfo.R = (boost::array<double, 9ul>)
				{1.0, 0.0, 0.0,
				 0.0, 1.0, 0.0,
				 0.0, 0.0, 1.0};

	cam.fromCameraInfo(camInfo);

	std::cerr << camInfo << std::endl;

	return cam;
}

class MeshProjector
{
public:
	//typedef std::map<std::string, Eigen::Isometry3f, std::less<std::string>, Eigen::aligned_allocator<std::pair<const std::string, Eigen::Isometry3f> > > TransformMap;
	typedef std::map<std::string, aiMatrix4x4> TransformMap;


	MeshProjector(urdf::Model& robotModel, const std::string camera_frame_id, ros::NodeHandle nh)
		: mRobotModel(robotModel),
		  mNH(nh)
	{
		this->mCameraFrameID = camera_frame_id;
		this->mCameraModel = createIdealCamera();
		//this->mCameraModel.cameraInfo().header.frame_id = camera_frame_id;

		for (std::pair<std::string, boost::shared_ptr<urdf::Link> >  linkPair : mRobotModel.links_)
		{
			boost::shared_ptr<urdf::Link> link = linkPair.second;
			mMeshFrameIDs.push_back(link->name);
		}

		std::cerr << mRobotModel.getName() << std::endl;
		this->init();
	}

	MeshProjector(urdf::Model& robotModel, const std::string camera_frame_id, ros::NodeHandle nh, const std::vector<std::string>& frame_ids)
		: mRobotModel(robotModel),
		  mNH(nh),
		  mMeshFrameIDs(frame_ids)
	{
		this->mCameraFrameID = camera_frame_id;
		this->mCameraModel = createIdealCamera();
		//this->mCameraModel.cameraInfo().header.frame_id = camera_frame_id;

		this->init();
	}

	void init()
	{
		resource_retriever::Retriever r;
		resource_retriever::MemoryResource resource;

		int totalMeshes = 0;
		int totalFaces = 0;
		for (int frame = 0; frame < mMeshFrameIDs.size(); frame++)
		{
			boost::shared_ptr<const urdf::Link> link = mRobotModel.getLink(mMeshFrameIDs[frame]);
			if (link == NULL || link->visual == NULL || link->visual->geometry == NULL ||
				!link->visual->geometry->type == urdf::Geometry::MESH)
			{
				continue;
			}

			boost::shared_ptr<urdf::Mesh>& mesh = (boost::shared_ptr<urdf::Mesh>&)link->visual->geometry;

			try
			{
				resource = r.get(mesh->filename);
			}
			catch (resource_retriever::Exception& e)
			{
				ROS_ERROR("Failed to retrieve file: %s", e.what());
				continue;
			}

			const aiScene* scene = importer.ReadFileFromMemory(resource.data.get(), resource.size, aiProcessPreset_TargetRealtime_Fast, ".stl");
			if (scene == NULL)
			{
				std::cerr << importer.GetErrorString() << std::endl;
				continue;
			}

			totalMeshes += scene->mNumMeshes;
			totalFaces += scene->mMeshes[0]->mNumFaces;
			std::pair<std::string, aiScene*> tempPair = std::pair<std::string, aiScene*>(mMeshFrameIDs[frame], importer.GetOrphanedScene());
			scenes.insert(tempPair);
		}

		std::cerr << "Initialized with " << totalMeshes << " meshes and " << totalFaces << " faces." << std::endl;
	}

	Eigen::Vector3f getEigenVector3f(aiVector3D* vec)
	{
		Eigen::Vector3f newVector;
		newVector.x() = vec->x;
		newVector.y() = vec->y;
		newVector.z() = vec->z;
		return newVector;
	}

	cv::Mat projectWithTF()
	{
		// For each frame in vector
		for (int frame = 0; frame < mMeshFrameIDs.size(); frame++)
		{
			// Lookup current transform
			tf::StampedTransform transform;

			try
			{
				mTFListener.lookupTransform(mCameraFrameID, mMeshFrameIDs[frame], ros::Time(0), transform);
				aiMatrix4x4 aTransform = ap::toAssimp(transform);
				TransformMap::value_type tfPair(mMeshFrameIDs[frame], aTransform);
				transforms.insert(tfPair);
			}
			catch (int err)
			{
				transforms.erase(mMeshFrameIDs[frame]);
			}
		}

		return projectWithEigen();
	}

	cv::Mat projectWithEigen()
	{
		// For each pixel in camera image
		std::cerr << "In function!" << std::endl;
		std::cerr << mCameraModel.cameraInfo().width << std::endl;
		cv::Mat robotImage(mCameraModel.cameraInfo().width, mCameraModel.cameraInfo().height, CV_32F);
		float* pixelPtr = (float*)robotImage.data;
		for (int v = 0; v < robotImage.rows; v++)
		{
			for (int u = 0; u < robotImage.cols; u++)
			{
				// Create a ray through the pixel
				int pixelIdx = u + (v * robotImage.cols);
				std::cerr << "Pixel (" << u << "," << v << ")" << std::endl;
				cv::Point2d pixel = cv::Point2d(u, v);
				cv::Point3d cvRay = mCameraModel.projectPixelTo3dRay(pixel);
				std::cerr << "Success." << std::endl;
				// Convert cvRay to ap::Ray
				ap::Ray ray;
				ray.point = Eigen::Vector3f::Zero();
				ray.vector.x() = cvRay.x; ray.vector.y() = cvRay.y; ray.vector.z() = cvRay.z;
				std::cerr << ray.vector.transpose() << std::endl;

				// For each frame in vector
				for (int frame = 0; frame < mMeshFrameIDs.size(); frame++)
				{
					// Lookup current transform
					aiMatrix4x4 transform;
					transform = transforms.at(mMeshFrameIDs[frame]);

					// Get copy of mesh for each frame
					std::cerr << "Getting frame " << frame << std::endl;
					aiScene* scene = scenes.at(mMeshFrameIDs[frame]);
					aiMesh mesh = *scene->mMeshes[0];
					std::cerr << scene->mMeshes[0]->HasFaces() << std::endl;

					// Transform mesh into camera frame
//					for (int i = 0; i < mesh.mNumVertices; i++)
//					{
//						aiVector3D newVertex = transform * mesh.mVertices[i];
//						mesh.mVertices[i] = newVertex;
//					}

					// For each triangle in mesh
					for (int i = 0; i < mesh.mNumFaces; i++)
					{
						std::cerr << "Getting face " << i << " of " << mesh.mNumFaces << std::endl;
						// Check for intersection. If finite, set distance
						aiFace face = mesh.mFaces[i];
//						if (NULL == face)
//						{
//							continue;
//						}

						if (face.mNumIndices != 3)
						{
							std::cerr << "??? " << face.mNumIndices << " vertices detected ???" << std::endl;
							continue;
						}

						std::cerr << "Creating Eigen vectors" << std::endl;
						Eigen::Vector3f vertexA = getEigenVector3f(&mesh.mVertices[face.mIndices[0]]);
						Eigen::Vector3f vertexB = getEigenVector3f(&mesh.mVertices[face.mIndices[1]]);
						Eigen::Vector3f vertexC = getEigenVector3f(&mesh.mVertices[face.mIndices[2]]);

						std::cerr << "Creating Eigen Triangle" << std::endl;
						ap::Triangle triangle(vertexA, vertexB, vertexC);

						Eigen::Vector3f intersection = ap::intersectRayTriangle(ray, triangle);
						if (std::isfinite(intersection.x()))
						{
							float d = intersection.norm();
							float val = pixelPtr[pixelIdx];
							if (val == 0 || val > d)
							{
								pixelPtr[pixelIdx] = d;
							}
						}
					}
				}
			}
		}

		// Return the matrix
		return robotImage;
	}

protected:
	std::string mCameraFrameID;
	std::vector<std::string> mMeshFrameIDs;
	tf::TransformListener mTFListener;
	ros::NodeHandle mNH;

	urdf::Model mRobotModel;

	image_geometry::PinholeCameraModel mCameraModel;

	Assimp::Importer importer;
	std::map<std::string, aiScene*> scenes;
	TransformMap transforms;
};

MeshProjector* gMP;

void timerCallback(const ros::TimerEvent&)
{
	std::cerr << "Timer!" << std::endl;
	gMP->projectWithTF();
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "project_robot_mesh");
	ros::NodeHandle nh;

	urdf::Model robotModel;

	std::string robotDescription;
	if (!nh.getParam("/robot_description", robotDescription))
	{
		ROS_FATAL("Parameter for robot description not provided");
	}
	robotModel.initString(robotDescription);

	gMP = new MeshProjector(robotModel, "/Body_PS1", nh);

	std::cerr << "Make a timer!" << std::endl;
	ros::Timer timer = nh.createTimer(ros::Duration(1), &timerCallback);
	timer.start();

	ros::spin();
}
