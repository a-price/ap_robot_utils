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
#include <assimp/postprocess.h>
#include <assimp/cimport.h>

#include <ap_robot_utils/geometry_utils.h>
#include <ap_robot_utils/pose_conversions.h>

std::ostream& operator <<(std::ostream& s, aiVector3D vec)
{
	s << "(" << vec.x << "," << vec.y << "," << vec.z << ")";
	return s;
}

ap::Mesh* meshAItoAP(const aiScene* inScene)
{
	ap::Mesh* outMesh = new ap::Mesh();

	// Find the triangle mesh
	int triMeshIdx = 0;
	for (int i = 0; i < inScene->mNumMeshes; i++)
	{
		if (inScene->mMeshes[i]->HasFaces() && inScene->mMeshes[i]->mFaces[0].mNumIndices == 3)
		{
			triMeshIdx = i;
		}
	}

	// Copy vertices
	outMesh->vertices.resize(inScene->mMeshes[triMeshIdx]->mNumVertices);
	for (int i = 0; i < inScene->mMeshes[triMeshIdx]->mNumVertices; i++)
	{
		outMesh->vertices[i] = Eigen::Vector3(inScene->mMeshes[triMeshIdx]->mVertices[i].x,
		                                      inScene->mMeshes[triMeshIdx]->mVertices[i].y,
		                                      inScene->mMeshes[triMeshIdx]->mVertices[i].z);
		//std::cerr << outMesh->vertices[i].transpose() << std::endl;
	}

	if (NULL == inScene->mMeshes[triMeshIdx]->mFaces)
	{
		std::cerr << "WTF?" << std::endl;
		return NULL;
	}
	outMesh->faces.resize(inScene->mMeshes[triMeshIdx]->mNumFaces);

	// Copy Faces (lists of vertex indices)
	for (int i = 0; i < inScene->mMeshes[triMeshIdx]->mNumFaces; i++)
	{
		if (3 != inScene->mMeshes[triMeshIdx]->mFaces[i].mNumIndices)
		{
			std::cerr << "WTF?" << std::endl;
			continue;
		}

		outMesh->faces[i].vertices[0] = inScene->mMeshes[triMeshIdx]->mFaces[i].mIndices[0];
		outMesh->faces[i].vertices[1] = inScene->mMeshes[triMeshIdx]->mFaces[i].mIndices[1];
		outMesh->faces[i].vertices[2] = inScene->mMeshes[triMeshIdx]->mFaces[i].mIndices[2];
	}

	return outMesh;
}

image_geometry::PinholeCameraModel createIdealCamera()
{
	image_geometry::PinholeCameraModel cam;
	sensor_msgs::CameraInfo camInfo;
	camInfo.width = 640;
	camInfo.height = 480;

	//camInfo.distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;

	camInfo.K = (boost::array<double, 9ul>)
				{100.0, 0.0, camInfo.width/2.0,
				 0.0, 100.0, camInfo.height/2.0,
				 0.0, 0.0, 1.0};

	camInfo.P = (boost::array<double, 12ul>)
				{100.0, 0.0, camInfo.width/2.0, 0.0,
				 0.0, 100.0, camInfo.height/2.0, 0.0,
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
	typedef std::map<std::string, Eigen::Isometry3, std::less<std::string>, Eigen::aligned_allocator<std::pair<const std::string, Eigen::Isometry3> > > TransformMap;
	//typedef std::map<std::string, aiMatrix4x4> TransformMap;
	typedef std::map<std::string, ap::Mesh*> MeshMap;


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

			const aiScene* scene = aiImportFileFromMemory((char*)resource.data.get(), resource.size,
			                                              aiProcess_ValidateDataStructure |
			                                              aiProcess_JoinIdenticalVertices |
			                                              aiProcess_ImproveCacheLocality |
			                                              aiProcess_Triangulate |
			                                              aiProcess_OptimizeGraph |
			                                              aiProcess_OptimizeMeshes |
			                                              aiProcess_FindDegenerates |
			                                              aiProcess_SortByPType,
			                                              ".stl");

			if (scene == NULL)
			{
//				std::cerr << importer.GetErrorString() << std::endl;
				continue;
			}

			totalMeshes += scene->mNumMeshes;
			totalFaces += scene->mMeshes[0]->mNumFaces;
			//MeshMap::value_type tempPair = MeshMap::value_type(mMeshFrameIDs[frame], *scene->mMeshes[0]);
			scenes.insert(MeshMap::value_type(mMeshFrameIDs[frame], meshAItoAP(scene)));
			transformedScenes.insert(MeshMap::value_type(mMeshFrameIDs[frame], meshAItoAP(scene)));
		}

		std::cerr << "Initialized with " << totalMeshes << " meshes and " << totalFaces << " faces." << std::endl;
	}

	Eigen::Vector3 getEigenVector3(aiVector3D* vec)
	{
		Eigen::Vector3 newVector;
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
				Eigen::Isometry3 aTransform = ap::toIsometry(transform);
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
		// Transform meshes into camera frame

		// For each frame in vector
		for (int frame = 0; frame < mMeshFrameIDs.size(); frame++)
		{
			// Lookup current transform
			Eigen::Isometry3 transform;
			transform = transforms.at(mMeshFrameIDs[frame]);

			// Get copy of mesh for each frame
			ap::Mesh* sourceMesh;
			ap::Mesh* transformedMesh;
			//std::cerr << "Getting frame " << frame << " : " << mMeshFrameIDs[frame] << std::endl;
			MeshMap::iterator scene_i = scenes.find(mMeshFrameIDs[frame]);
			if (scenes.end() == scene_i) { continue; }
			sourceMesh = scene_i->second;

			MeshMap::iterator scene_t = transformedScenes.find(mMeshFrameIDs[frame]);
			if (transformedScenes.end() == scene_t) { continue; }
			transformedMesh = scene_t->second;

			// Transform mesh into camera frame
			for (int i = 0; i < sourceMesh->vertices.size(); i++)
			{
				Eigen::Vector3 newVertex = transform * sourceMesh->vertices[i];
				//std::cerr << mesh->vertices[i].transpose() << "\t->\t" << newVertex.transpose() << std::endl;
				transformedMesh->vertices[i] = newVertex;
			}
		}

		// For each pixel in camera image
		cv::Mat robotImage(mCameraModel.cameraInfo().height, mCameraModel.cameraInfo().width, CV_32F);
		float* pixelPtr = (float*)robotImage.data;
		float maxDepth = 0;
		for (int v = 0; v < robotImage.rows; v++)
		{
			for (int u = 0; u < robotImage.cols; u++)
			{
				// Create a ray through the pixel
				int pixelIdx = u + (v * robotImage.cols);
				//std::cerr << "Pixel (" << u << "," << v << ")" << std::endl;
				cv::Point2d pixel = cv::Point2d(u, v);
				cv::Point3d cvRay = mCameraModel.projectPixelTo3dRay(pixel);
				// Convert cvRay to ap::Ray
				ap::Ray ray;
				ray.point = Eigen::Vector3::Zero();
				ray.vector.x() = cvRay.x; ray.vector.y() = cvRay.y; ray.vector.z() = cvRay.z;
				ray.vector.normalize();
				//std::cerr << ray.vector.transpose() << std::endl;

				// For each frame in vector
				for (int frame = 0; frame < mMeshFrameIDs.size(); frame++)
				{
					MeshMap::iterator scene_i = transformedScenes.find(mMeshFrameIDs[frame]);
					if (transformedScenes.end() == scene_i)
					{
						continue;
					}
					ap::Mesh* mesh = scene_i->second;

					// For each triangle in mesh
					for (int i = 0; i < mesh->faces.size(); i++)
					{
						// Check for intersection. If finite, set distance

						ap::Triangle triangle(mesh->vertices[mesh->faces[i].vertices[0]],
											  mesh->vertices[mesh->faces[i].vertices[1]],
											  mesh->vertices[mesh->faces[i].vertices[2]]);

						Eigen::Vector3 intersection = ap::intersectRayTriangle(ray, triangle);
						if (std::isfinite(intersection.x()))
						{
							float d = intersection.norm();
							float val = pixelPtr[pixelIdx];
							if (val == 0 || val > d)
							{
								pixelPtr[pixelIdx] = d;
							}
							if (d > maxDepth)
							{
								maxDepth = d;
							}
						}
					}
				}
			}
		}

		// Return the matrix
		if (maxDepth == 0) { maxDepth = 1;}
		return robotImage/maxDepth;
	}

protected:
	std::string mCameraFrameID;
	std::vector<std::string> mMeshFrameIDs;
	tf::TransformListener mTFListener;
	ros::NodeHandle mNH;

	urdf::Model mRobotModel;

	image_geometry::PinholeCameraModel mCameraModel;

//	Assimp::Importer importer;
	MeshMap scenes;
	MeshMap transformedScenes;
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
