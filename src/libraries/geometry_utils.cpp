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
									  std::vector<float>* weights)
{
	Eigen::Matrix4 accumulator = Eigen::Matrix4::Zero();
	const int n = qs.size();
	Eigen::Vector4 qVec;

	for (int i = 0; i < n; i++)
	{
		qs[i].normalize();
	}

	if (weights != NULL && weights->size() == n)
	{
		float totalWeight;
		for (int i = 0; i < n; i++)
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
		for (int i = 0; i < n; i++)
		{
			Eigen::Quaternion<ap::decimal>& q = qs[i];
			getQuaternionDataVector(q, qVec);
			accumulator += qVec * qVec.transpose();
		}

		accumulator /= (float)n;
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
	for (int i = 0; i < a.vertices.size(); i++)
	{
		newMesh.vertices.push_back(t * a.vertices[i]);
	}
	return newMesh;
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


Eigen::Vector3 intersectRayPlane(Ray r, Plane p)
{
	float temp = (r.point.dot(p.normal) + p.distance)/(r.vector.dot(p.normal));
	if (temp == 0) { temp = NAN; } // Plane contains ray origin
	Eigen::Vector3 intersection = r.point + (temp * r.vector);
	return intersection;
}

Eigen::Vector3 intersectRayTriangle(Ray r, Triangle t)
{
	// Find intersection of plane and ray
	Eigen::Vector3 result = intersectRayPlane(r, t.getPlane());
	if (!std::isfinite(result.x()))
	{
		// Parallel to plane, return NaNs
		return result;
	}

	// Check if inside of triangle
	bool insideTriangle = true;
	for (int i = 0; i < 3; i++)
	{
		Eigen::Vector3 v1 = (*t.vertices[i]) - r.point;
		Eigen::Vector3 v2 = (*t.vertices[(i+1)%3]) - r.point;
		Eigen::Vector3 tempNorm = v2.cross(v1).normalized();

		float d = r.point.dot(tempNorm);
		if (result.dot(tempNorm) > d)
		{
			insideTriangle = false;
			result.x() = NAN;
			result.y() = NAN;
			result.z() = NAN;
			break;
		}
	}

	return result;
}

// TODO: Fix for non-convex surfaces
float Mesh::volume() const
{
	int n = faces.size();

	float volume = 0;
	// Compute the signed volume of each facet to the origin
	for (int i = 0; i < n; ++i)
	{
		const Eigen::Vector3 p1 = vertices[faces[i].vertices[0]];
		const Eigen::Vector3 p2 = vertices[faces[i].vertices[1]];
		const Eigen::Vector3 p3 = vertices[faces[i].vertices[2]];

		float signedVol = (p1).dot((p2).cross(p3)) / 6.0f;

		volume += fabs(signedVol);
	}

	return fabs(volume);
}

}
