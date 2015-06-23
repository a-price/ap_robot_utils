/**
 * \file geometry_utils.cpp
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

#include "ap_robot_utils/geometry_utils.h"
#include <cmath>
#include <Eigen/SVD>

/////// Import Code
#include <assimp/mesh.h>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
#include <assimp/Importer.hpp>

#include <set>
#include <stdexcept>
#include <vector>
#include <deque>

namespace ap
{
void setQuaternionDataVector(Eigen::Quaternion<ap::decimal>& q, const Eigen::Vector4& v)
{
	q.w() = v.w();
	q.x() = v.x();
	q.y() = v.y();
	q.z() = v.z();
}

void getQuaternionDataVector(const Eigen::Quaternion<ap::decimal>& q, Eigen::Vector4& v)
{
	v.w() = q.w();
	v.x() = q.x();
	v.y() = q.y();
	v.z() = q.z();
}

Eigen::Quaternion<ap::decimal> averageQuaternions(QuaternionStdVector& qs,
                                                  std::vector<ap::decimal>* weights)
{
	Eigen::Matrix4 accumulator = Eigen::Matrix4::Zero();
	const int n = qs.size();
	Eigen::Vector4 qVec;

	for (int i = 0; i < n; ++i)
	{
		qs[i].normalize();
	}

	if (weights != NULL && weights->size() == n)
	{
		ap::decimal totalWeight = 0;
		for (int i = 0; i < n; ++i)
		{
			Eigen::Quaternion<ap::decimal>& q = qs[i];
			getQuaternionDataVector(q, qVec);
			accumulator += (qVec * qVec.transpose()) * (*weights)[i];
			totalWeight += (*weights)[i];
		}

		accumulator /= totalWeight;
	}
	else
	{
		for (int i = 0; i < n; ++i)
		{
			Eigen::Quaternion<ap::decimal>& q = qs[i];
			getQuaternionDataVector(q, qVec);
			accumulator += qVec * qVec.transpose();
		}

		accumulator /= (ap::decimal)n;
	}

	Eigen::JacobiSVD<Eigen::Matrix4> svd(accumulator, Eigen::ComputeFullU);
	qVec = svd.matrixU().col(0);

	Eigen::Quaternion<ap::decimal> retVal;
	setQuaternionDataVector(retVal, qVec);
	retVal.normalize();

	return retVal;

}


// Transform a mesh
Mesh operator* (const Eigen::Isometry3& t, const Mesh& a)
{
	Mesh newMesh;
	newMesh.faces.insert(newMesh.faces.begin(), a.faces.begin(), a.faces.end());
	for (int i = 0; i < a.vertices.size(); ++i)
	{
		newMesh.vertices.push_back(t * a.vertices[i]);
	}
	return newMesh;
}

Mesh operator+ (const Mesh& a, const Eigen::Vector3& p)
{
	Mesh newMesh;
	newMesh.faces.insert(newMesh.faces.begin(), a.faces.begin(), a.faces.end());
	for (int i = 0; i < a.vertices.size(); ++i)
	{
		newMesh.vertices.push_back(p + a.vertices[i]);
	}
	return newMesh;
}

Mesh operator- (const Mesh& a, const Eigen::Vector3& p)
{
	return a + (-p);
}

std::ostream& operator <<(std::ostream& s, Ray r)
{
	s << "Ray: " << std::endl;
	s << "\tP: " << r.point.transpose() << std::endl;
	s << "\tV: " << r.vector.transpose() << std::endl;
	return s;
}


std::ostream& operator <<(std::ostream& s, Triangle tri)
{
	s << "Triangle: " << std::endl;
	s << "\tA: " << tri.vertexA.transpose() << std::endl;
	s << "\tB: " << tri.vertexB.transpose() << std::endl;
	s << "\tC: " << tri.vertexC.transpose() << std::endl;
	s << "\tNorm: " << tri.getNormal().transpose() << std::endl;
	return s;
}

std::ostream& operator <<(std::ostream& s, Mesh r)
{
	s << "Points:  " << std::endl;
	for (int i = 0; i < r.vertices.size(); ++i)
		s << "\t" << i << ":   " << r.vertices[i].transpose() << std::endl;
	s << "Faces:  " << std::endl;
	for (int i = 0; i < r.faces.size(); ++i)
	{
		s << "\t" << i << ":  " << r.faces[i].vertices[0] << ","
								<< r.faces[i].vertices[1] << ","
								<< r.faces[i].vertices[2] << std::endl;
	}
	return s;
}

Eigen::Vector3 intersectRayPlane(const Ray& r, const Plane& p)
{
	ap::decimal dist = -(r.point.dot(p.normal) - p.distance)/(r.vector.dot(p.normal));
	if (dist == 0) { return r.point; } // Plane contains ray origin
	Eigen::Vector3 intersection = r.point + (dist * r.vector);
	return intersection;
}

Eigen::Vector3 intersectRayTriangle(const Ray& r, const Triangle& t)
{
	// Find intersection of plane and ray
	Eigen::Vector3 result = intersectRayPlane(r, t.getPlane());
	if (!std::isfinite(result.x()))
	{
		// Parallel to plane, return NaNs
		return result;
	}

	// Check if inside of triangle
	for (int i = 0; i < 3; ++i)
	{
		Eigen::Vector3 v1 = (*t.vertices[i]) - r.point;
		Eigen::Vector3 v2 = (*t.vertices[(i+1)%3]) - r.point;
		Eigen::Vector3 v3 = (*t.vertices[(i+2)%3]) - r.point;
		Eigen::Vector3 tempNorm = v2.cross(v1).normalized();
		if (v3.dot(tempNorm) < 0) { tempNorm = -tempNorm; }

		ap::decimal d = r.point.dot(tempNorm);
		if (result.dot(tempNorm) < d)
		{
			result.x() = NAN;
			result.y() = NAN;
			result.z() = NAN;
			return result;
		}
	}

	// Check if in direction of ray
	if (r.vector.dot(result - r.point) < 0)
	{
		result.x() = NAN;
		result.y() = NAN;
		result.z() = NAN;
	}

	return result;
}

// TODO: Fix for non-convex surfaces
ap::decimal Mesh::volume() const
{
	const int n = faces.size();

	ap::decimal vol = 0;
	// Compute the signed volume of each facet to the origin
	for (int i = 0; i < n; ++i)
	{
		const Eigen::Vector3 p1 = vertices[faces[i].vertices[0]];
		const Eigen::Vector3 p2 = vertices[faces[i].vertices[1]];
		const Eigen::Vector3 p3 = vertices[faces[i].vertices[2]];

		ap::decimal signedVol = (p1).dot((p2).cross(p3)) / 6.0f;

		vol += fabs(signedVol);
	}

	return fabs(vol);
}

Eigen::Vector3 Mesh::com() const
{
	const int n = faces.size();
	Eigen::Vector3 c = Eigen::Vector3::Zero();
	for (int i = 0; i < n; ++i)
	{
		c += vertices[i];
	}
	return c / static_cast<ap::decimal>(n);
}

Eigen::Vector3 Mesh::cobb() const
{
	const int n = faces.size();
	if (0 == n) { return Eigen::Vector3::Zero(); }

	Eigen::Vector3 min = vertices[0];
	Eigen::Vector3 max = vertices[0];
	for (int i = 0; i < n; ++i)
	{
		const Eigen::Vector3& v = vertices[i];
		for (int j = 0; j < 3; ++j)
		{
			if (v[j] < min[j]) { min[j] = v[j]; }
			if (v[j] > max[j]) { max[j] = v[j]; }
		}
	}
	return (min+max) / 2.0;
}

void Mesh::computeAABB()
{
	minCoord = Eigen::Vector3( std::numeric_limits<ap::decimal>::infinity(),
	                           std::numeric_limits<ap::decimal>::infinity(),
	                           std::numeric_limits<ap::decimal>::infinity());
	maxCoord = Eigen::Vector3(-std::numeric_limits<ap::decimal>::infinity(),
	                          -std::numeric_limits<ap::decimal>::infinity(),
	                          -std::numeric_limits<ap::decimal>::infinity());

	const int numVertices = this->vertices.size();
	for (int i = 0; i < numVertices; ++i)
	{
		const Eigen::Vector3& v = this->vertices[i];
		if (v.x() < minCoord.x()) { minCoord.x() = v.x(); }
		if (v.x() > maxCoord.x()) { maxCoord.x() = v.x(); }
		if (v.y() < minCoord.y()) { minCoord.y() = v.y(); }
		if (v.y() > maxCoord.y()) { maxCoord.y() = v.y(); }
		if (v.z() < minCoord.z()) { minCoord.z() = v.z(); }
		if (v.z() > maxCoord.z()) { maxCoord.z() = v.z(); }
	}
}

void Mesh::aabb(ap::decimal& xMin, ap::decimal& yMin, ap::decimal& zMin, ap::decimal& xMax, ap::decimal& yMax, ap::decimal& zMax) const
{
	xMin = minCoord.x(); yMin = minCoord.y(); zMin = minCoord.z();
	xMax = maxCoord.x(); yMax = maxCoord.y(); zMax = maxCoord.z();
}

void Mesh::boundingSphere(ap::decimal& x, ap::decimal& y, ap::decimal& z, ap::decimal& r) const
{
	ap::decimal xMin,  yMin,  zMin,  xMax,  yMax,  zMax;
	this->aabb(xMin,  yMin,  zMin,  xMax,  yMax,  zMax);
	x = (xMax + xMin) / 2.0f;
	y = (yMax + yMin) / 2.0f;
	z = (zMax + zMin) / 2.0f;

	ap::decimal dX, dY, dZ;
	dX = xMax - xMin;
	dY = yMax - yMin;
	dZ = zMax - zMin;

	r = sqrt(dX*dX + dY*dY + dZ*dZ) / 2.0f;
}

/*********************************************************************************/
/* Model Import Code                                                             */
/*********************************************************************************/

typedef std::pair<uint, uint> IdxPair;

struct compareVectors
{
	bool operator() (const aiVector3D& s, const aiVector3D& r)
	{
		if (s.Length() < r.Length())
		{
			return true;
		}
		else if (s.Length() == r.Length())
		{
			if (s.x + s.y + s.z < r.x + r.y + r.z)
			{
				return true;
			}
			else if (s.x + s.y + s.z == r.x + r.y + r.z)
			{
				if (s.x < r.x)
				{
					return true;
				}
				else if(s.x == r.x)
				{
					if (s.y < r.y)
					{
						return true;
					}
					else if(s.y == r.y)
					{
						return s.z < r.z;
					}
					else
					{
						return false;
					}
				}
				else
				{
					return false;
				}
			}
			else
			{
				return false;
			}
		}
		return false;

	}
};

typedef std::set<aiVector3D, compareVectors> VectorSet;


uint getIteratorIndex(const std::vector<VectorSet::iterator>& vec, const VectorSet::iterator& iter)
{
	for (uint i = 0; i < vec.size(); ++i)
	{
		if (vec[i] == iter)
		{
			return i;
		}
	}

	std::cerr << "ERROR: Target not found." << std::endl;

	return 0;
}

bool meshAItoAP(const aiScene* inScene, Mesh* outMesh)
{
	// Find a triangle mesh
	int triMeshIdx = 0;
	for (int i = 0; i < inScene->mNumMeshes; i++)
	{
		if (inScene->mMeshes[i]->HasFaces() && inScene->mMeshes[i]->mFaces[0].mNumIndices == 3)
		{
			triMeshIdx = i;
		}
	}

	aiMesh* pMesh = inScene->mMeshes[triMeshIdx];

	// Copy vertices
	VectorSet vertices;

	std::pair<VectorSet::iterator, bool> insertResult;
	std::vector<VectorSet::iterator> indexToSetIterator;
	std::vector<int> indexMap;

	//NB: do this twice, iterators move around on insert
	for (int i = 0; i < pMesh->mNumVertices; i++)
	{
		vertices.insert(pMesh->mVertices[i]);
	}

	for (int i = 0; i < pMesh->mNumVertices; i++)
	{
		insertResult = vertices.insert(pMesh->mVertices[i]);
		indexMap.push_back(std::distance(vertices.begin(), insertResult.first));
		indexToSetIterator.push_back(insertResult.first);
	}

	// Create set iterator to index mapping
	VectorSet::iterator first = vertices.begin(), last = vertices.end();
	for (VectorSet::iterator iter = first; iter != last; ++iter)
	{
		outMesh->vertices.push_back(Eigen::Vector3(iter->x, iter->y, iter->z));
	}

	assert(NULL != pMesh->mFaces);
	outMesh->faces.resize(pMesh->mNumFaces);

	assert(pMesh->HasNormals());
	// Copy Faces (lists of vertex indices)
	for (int i = 0; i < pMesh->mNumFaces; ++i)
	{
		if (3 != pMesh->mFaces[i].mNumIndices)
		{
			std::cerr << "WTF?" << std::endl;
			continue;
		}

		Eigen::Vector3 c = Eigen::Vector3::Zero();
		for (int j = 0; j < 3; ++j)
		{
			uint originalIdx = pMesh->mFaces[i].mIndices[j];
			int shiftedIdx = std::distance(vertices.begin(), indexToSetIterator[originalIdx]);

			outMesh->faces[i].vertices[j] = shiftedIdx;
			c += outMesh->vertices[shiftedIdx];
		}

		// NB: Normals are generated per-vertex, not per face...
		aiVector3D normal = pMesh->mNormals[pMesh->mFaces[i].mIndices[0]].Normalize();
		outMesh->faces[i].normal = Eigen::Vector3(normal.x, normal.y, normal.z);
		outMesh->faces[i].center = c / 3.0;
	}

	return true;
}

ap::Mesh* loadMesh(const std::string filename)
{
	Assimp::Importer importer;
	const int postprocessingFlags =
			aiProcess_JoinIdenticalVertices |
			aiProcess_GenNormals |
			aiProcess_ImproveCacheLocality |
			aiProcess_Triangulate |
			aiProcess_OptimizeGraph |
			aiProcess_OptimizeMeshes |
			//aiProcess_FindDegenerates |
			aiProcess_FixInfacingNormals |
			aiProcess_SortByPType;

	const aiScene* pScene = importer.ReadFile(filename.c_str(),
											  aiProcess_ValidateDataStructure |
											  postprocessingFlags);

	ap::Mesh* pMesh = new ap::Mesh();

	if (pScene == NULL)
	{
		std::cerr << importer.GetErrorString() << std::endl;
		return nullptr;
	}

	if (!meshAItoAP(pScene, pMesh))
	{
		std::cerr << "Failed to load mesh: " << filename << std::endl;
		return nullptr;
	}

	return pMesh;
}

}
