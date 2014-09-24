/**
 * \file test_assimp_loader.cpp
 * \brief
 *
 * \author Andrew Price
 * \date August 6, 2013
 *
 * \copyright
 *
 * Copyright (c) 2013, Georgia Tech Research Corporation
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
#include <tf/transform_listener.h>
#include <boost/foreach.hpp>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/distortion_models.h>
#include <image_geometry/pinhole_camera_model.h>

#include <urdf/model.h>
#include <resource_retriever/retriever.h>

#include <assimp/mesh.h>
#include <assimp/scene.h>

#include <assimp/cimport.h>
#ifdef AI_CONFIG_PP_SBP_REMOVE
#undef AI_CONFIG_PP_SBP_REMOVE
#endif
#define AI_CONFIG_PP_SBP_REMOVE aiPrimitiveType_POINT | aiPrimitiveType_LINE
#include <assimp/postprocess.h>

#include <ap_robot_utils/geometry_utils.h>
#include <ap_robot_utils/pose_conversions.h>



typedef std::map<std::string, ap::Mesh*> MeshMap;
MeshMap scenes;
MeshMap transformedScenes;

typedef std::map<std::string, Eigen::Isometry3, std::less<std::string>, Eigen::aligned_allocator<std::pair<const std::string, Eigen::Isometry3> > > TransformMap;
TransformMap transforms;

image_geometry::PinholeCameraModel mCameraModel;
std::string mCameraFrameID;
std::vector<std::string> mMeshFrameIDs;

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


void init()
{
	//	Assimp::Importer importer;
	const aiScene* scene = aiImportFile("/home/arprice/catkin_workspace/src/ap_robot_utils/test/resources/cube.stla",
										aiProcess_ValidateDataStructure |
										aiProcess_JoinIdenticalVertices |
										aiProcess_ImproveCacheLocality |
										aiProcess_Triangulate |
										aiProcess_OptimizeGraph |
										aiProcess_OptimizeMeshes |
										aiProcess_FindDegenerates |
										aiProcess_SortByPType);
//	std::cerr << importer.GetErrorString() << std::endl;
	if (scene == NULL) { return; }

	scenes.insert(MeshMap::value_type("cube", meshAItoAP(scene)));
	transformedScenes.insert(MeshMap::value_type("cube", meshAItoAP(scene)));
}

int main(int argc, char** argv)
{
	mCameraModel = createIdealCamera();
	mMeshFrameIDs.push_back("cube");
	init();

	Eigen::Isometry3 cubeTF = Eigen::Isometry3::Identity();
	cubeTF.rotate(Eigen::AngleAxis<ap::decimal>(M_PI / 4.0, Eigen::Vector3::UnitZ()));

	cubeTF.translate(Eigen::Vector3(0.1,0.1,2));
	cubeTF.rotate(Eigen::AngleAxis<ap::decimal>(M_PI / 4.0, Eigen::Vector3::UnitY()));
	transforms.insert(TransformMap::value_type("cube", cubeTF));
	cv::Mat testImg = projectWithEigen();
	cv::imshow("img", testImg);
	cv::waitKey(0);
	return 0;
}
