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
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <iostream>

namespace ap
{

typedef std::vector<Eigen::Quaternionf, Eigen::aligned_allocator<Eigen::Quaternionf> > QuaternionStdVector;
Eigen::Quaternionf averageQuaternions(QuaternionStdVector& qs,
									  std::vector<float>* weights = NULL);



typedef Eigen::Vector3f Point;

class Ray
{
public:
	Eigen::Vector3f point;
	Eigen::Vector3f vector;
};

class Plane
{
public:
	Plane(Eigen::Vector3f& n, Eigen::Vector3f& p)
		: normal(n.normalized()),
		  point(p)
	{
		distance = normal.dot(point);
	}

	Plane(Eigen::Vector3f& n, float d)
		: normal(n.normalized()),
		  distance(d)
	{
		point = normal * distance;
	}

	Eigen::Vector3f point;
	Eigen::Vector3f normal;
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

	Triangle(const Eigen::Vector3f& a, const Eigen::Vector3f& b, const Eigen::Vector3f& c)
		: vertexA(a),
		  vertexB(b),
		  vertexC(c)
	{
		vertices.push_back(&(this->vertexA));
		vertices.push_back(&(this->vertexB));
		vertices.push_back(&(this->vertexC));
	}

	Eigen::Vector3f getNormal()
	{
		Eigen::Vector3f ab = vertexA - vertexB;
		Eigen::Vector3f ac = vertexA - vertexC;
		Eigen::Vector3f retVal = ab.cross(ac).normalized();
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
		Eigen::Vector3f normal = getNormal();
		Plane p(normal, vertexA);
		return p;
	}

	Eigen::Vector3f vertexA;
	Eigen::Vector3f vertexB;
	Eigen::Vector3f vertexC;

	std::deque<Eigen::Vector3f*> vertices;
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

	std::deque<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f> > vertices;
	std::deque<Face> faces;

	float volume() const;
};

// Transform a mesh
Mesh operator* (const Eigen::Isometry3f& t, const Mesh& a);

std::ostream& operator <<(std::ostream& s, Ray r);

std::ostream& operator <<(std::ostream& s, Triangle tri);

std::ostream& operator <<(std::ostream& s, Mesh r);

Eigen::Vector3f intersectRayPlane(Ray r, Plane p);
Eigen::Vector3f intersectRayTriangle(Ray r, Triangle t);


}

#endif // GEOMETRY_UTILS_H
