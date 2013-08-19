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
void setQuaternionDataVector(Eigen::Quaternionf& q, const Eigen::Vector4f& v)
{
	q.w() = v.w();
	q.x() = v.x();
	q.y() = v.y();
	q.z() = v.z();
}

void getQuaternionDataVector(const Eigen::Quaternionf& q, Eigen::Vector4f& v)
{
	v.w() = q.w();
	v.x() = q.x();
	v.y() = q.y();
	v.z() = q.z();
}

Eigen::Quaternionf averageQuaternions(QuaternionStdVector& qs,
									  std::vector<float>* weights)
{
	Eigen::Matrix4f accumulator = Eigen::Matrix4f::Zero();
	int n = qs.size();
	Eigen::Vector4f qVec;

	for (int i = 0; i < n; i++)
	{
		qs[i].normalize();
	}

	if (weights != NULL && weights->size() == n)
	{
		float totalWeight;
		for (int i = 0; i < n; i++)
		{
			Eigen::Quaternionf& q = qs[i];
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
			Eigen::Quaternionf& q = qs[i];
			getQuaternionDataVector(q, qVec);
			accumulator += qVec * qVec.transpose();
		}

		accumulator /= (float)n;
	}

	Eigen::JacobiSVD<Eigen::Matrix4f> svd(accumulator, Eigen::ComputeFullU);
	qVec = svd.matrixU().col(0);

	Eigen::Quaternionf retVal;
	setQuaternionDataVector(retVal, qVec);
	retVal.normalize();

	return retVal;

}

Eigen::Vector3f intersectRayPlane(Ray r, Plane p)
{
	float temp = (r.point.dot(p.normal) + p.distance)/(r.vector.dot(p.normal));
	if (temp == 0) { temp = NAN; } // Plane contains ray origin
	Eigen::Vector3f intersection = r.point + (temp * r.vector);
	return intersection;
}

Eigen::Vector3f intersectRayTriangle(Ray r, Triangle t)
{
	// Find intersection of plane and ray
	Eigen::Vector3f result = intersectRayPlane(r, t.getPlane());
	if (!std::isfinite(result.x()))
	{
		// Parallel to plane, return NaNs
		return result;
	}

	// Check if inside of triangle
	bool insideTriangle = true;
	for (int i = 0; i < 3; i++)
	{
		Eigen::Vector3f v1 = (*t.vertices[i]) - r.point;
		Eigen::Vector3f v2 = (*t.vertices[(i+1)%3]) - r.point;
		Eigen::Vector3f tempNorm = v2.cross(v1).normalized();

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

}
