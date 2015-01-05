/**
 * \file geometry_utils.h
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

#ifndef GEOMETRY_UTILS_H
#define GEOMETRY_UTILS_H

#include <deque>
#include <vector>

#include "ap_robot_utils/eigen_definitions.h"

#include <iostream>

namespace ap
{

typedef std::vector<Eigen::Quaternion<ap::decimal>, Eigen::aligned_allocator<Eigen::Quaternion<ap::decimal> > > QuaternionStdVector;
Eigen::Quaternion<ap::decimal> averageQuaternions(QuaternionStdVector& qs,
									  std::vector<float>* weights = NULL);



typedef Eigen::Vector3 Point;

class Ray
{
public:
	Eigen::Vector3 point;
	Eigen::Vector3 vector;
};

class Plane
{
public:
	Plane(const Eigen::Vector3& n, const Eigen::Vector3& p)
		: normal(n.normalized()),
		  point(p)
	{
		distance = normal.dot(point);
	}

	Plane(const Eigen::Vector3& n, float d)
		: normal(n.normalized()),
		  distance(d)
	{
		point = normal * distance;
	}

	Eigen::Vector3 point;
	Eigen::Vector3 normal;
	float distance;
};

class Triangle
{
public:
	Triangle()
	{
		vertices.push_back(&(this->vertexA));
		vertices.push_back(&(this->vertexB));
		vertices.push_back(&(this->vertexC));
	}

	Triangle(const Eigen::Vector3& a, const Eigen::Vector3& b, const Eigen::Vector3& c)
		: vertexA(a),
		  vertexB(b),
		  vertexC(c)
	{
		vertices.push_back(&(this->vertexA));
		vertices.push_back(&(this->vertexB));
		vertices.push_back(&(this->vertexC));
	}

	Eigen::Vector3 getNormal()
	{
		Eigen::Vector3 ab = vertexA - vertexB;
		Eigen::Vector3 ac = vertexA - vertexC;
		Eigen::Vector3 retVal = ab.cross(ac).normalized();
		if (!std::isfinite(retVal.x()))
		{
			std::cerr << "Error, NaN detected in output. Possible collinear vertices: " << std::endl <<
						 vertexA.transpose() << std::endl <<
						 vertexB.transpose() << std::endl <<
						 vertexC.transpose() << std::endl;
		}
		return retVal;
	}

	Plane getPlane()
	{
		Eigen::Vector3 normal = getNormal();
		Plane p(normal, vertexA);
		return p;
	}

	Eigen::Vector3 vertexA;
	Eigen::Vector3 vertexB;
	Eigen::Vector3 vertexC;

	std::deque<Eigen::Vector3*> vertices;
};


class Mesh
{
public:

	class Face
	{
	public:
		Face() {}
		Face(int a, int b, int c)
		{
			vertices[0] = a;
			vertices[1] = b;
			vertices[2] = c;
		}

		unsigned int vertices[3];
	};

	std::deque<Eigen::Vector3, Eigen::aligned_allocator<Eigen::Vector3> > vertices;
	std::deque<Face> faces;

	float volume() const;
};

// Transform a mesh
Mesh operator* (const Eigen::Isometry3& t, const Mesh& a);

std::ostream& operator <<(std::ostream& s, Ray r);

std::ostream& operator <<(std::ostream& s, Triangle tri);

std::ostream& operator <<(std::ostream& s, Mesh r);

Eigen::Vector3 intersectRayPlane(Ray r, Plane p);
Eigen::Vector3 intersectRayTriangle(Ray r, Triangle t);


}

#endif // GEOMETRY_UTILS_H
