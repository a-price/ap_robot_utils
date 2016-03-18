/**
 * \file AugmentedMesh.cpp
 * \brief
 *
 * \author Andrew Price
 * \date 2016-3-9
 *
 * \copyright
 *
 * Copyright (c) 2016, Georgia Tech Research Corporation
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

#include "ap_robot_utils/AugmentedMesh.h"

#include <algorithm>

namespace ap
{

// Returns a righthanded orthonormal basis to complement vector u
template<class Vector>
void getOrthoBasis(const Vector& u,  Vector& v, Vector& w)
{
	assert(fabs(u.normSquared - 1.0) < 1e-9);
	if (u.x() > 0.5 || u.x()<-0.5 || u.y() > 0.5 || u.y()<-0.5)
	{
		v = Vector(u.y(), -u.x(), 0.0 );
	}
	else
	{
		v = Vector(0.0, u.z(), -u.y());
	}
	v.normalize();
	w = u.cross(v);
	w.normalize();
	return;
}

// Returns the projection of u onto the plane perpindicular to the unit vector v
//    This one is more stable when u and v are nearly equal.
template<class Vector>
inline Vector projectPerpUnitDiff ( const Vector& u, const Vector& v)
{
	Vector ans = u;
	ans -= v;
	ans -= ((ans.dot(v))*v);
	return ans;				// ans = (u-v) - ((u-v)^v)*v
}

// Rotate unit vector x in the direction of "dir": length of dir is rotation angle.
//		x must be a unit vector.  dir must be perpindicular to x.
template<class Vector>
Vector rotateUnitInDirection(const Vector& v, const Vector& dir)
{
	double theta = dir.squaredNorm();
	Vector vNew = v;
	if ( theta==0.0 )
	{
		return vNew;
	}
	else
	{
		theta = sqrt(theta);
		double costheta = cos(theta);
		double sintheta = sin(theta);
		Vector dirUnit = dir/theta;
		vNew = costheta*vNew + sintheta*dirUnit;
		return ( vNew );
	}
}

// Algorithm:
//    S. Buss and J. Fillmore, "Spherical Averages and
//      Applications to Spherical Splines and Interpolation."
//      ACM Transactions on Graphics 20 (2001) 95-126.
Eigen::Vector3 sphericalMean(const std::vector<Eigen::Vector3>& vs, const std::vector<decimal>& ws, const Eigen::Vector3& init,
                             decimal tolerance = 1e-12, decimal bkuptolerance = 1e-10, const size_t maxIters = 100)
{
	const size_t numVectors = vs.size();
	Eigen::Vector3 xVec(init);
	Eigen::Vector3 cLocalX, cLocalY;
	Eigen::Vector3 vPerp;
	decimal costheta, sintheta, theta;
	decimal sinthetaInv;
	Eigen::Matrix2 Hlocal;
	Eigen::Matrix2 Hessian;
	Eigen::Vector2 gradient;
	Eigen::Vector2 gradLocal;
	decimal cosphi, sinphi;
	decimal cosphiSq, sinphiSq;
	decimal tt, offdiag;
	Eigen::Vector2 xDispLocal;
	Eigen::Vector3 xDisp;

	for (int iter = 0; iter < maxIters; ++iter)
	{
		getOrthoBasis(xVec, cLocalX, cLocalY);

		gradient = Eigen::Vector2::Zero();
		Hessian = Eigen::Matrix2::Zero();

		for (size_t i = 0; i < numVectors; ++i)
		{
			const Eigen::Vector3& vi = vs[i];
			const decimal& wi = ws[i];

			vPerp = projectPerpUnitDiff(vi, xVec);
			sintheta = vPerp.norm();
			if (0.0 == sintheta)
			{
				Hessian += Eigen::DiagonalMatrix<decimal, 2>(wi, wi);
			}
			else
			{
				costheta = vi.dot(xVec);
				theta = atan2(sintheta, costheta);
				sinthetaInv = 1.0/sintheta;
				vPerp *= sinthetaInv;
				cosphi = vPerp.dot(cLocalX);
				sinphi = vPerp.dot(cLocalY);
				gradLocal = Eigen::Vector2(cosphi, sinphi);
				gradient += (wi*theta)*gradLocal;

				sinphiSq = sinphi * sinphi;
				cosphiSq = cosphi * cosphi;

				tt = wi*(theta*sinthetaInv)*costheta;
				offdiag = cosphi*sinphi*(wi-tt);
				Hlocal << wi*cosphiSq + tt*sinphiSq,
				          offdiag, offdiag,
				          wi*sinphiSq + tt*cosphiSq;
				Hessian += Hlocal;
			}
		}
		xDispLocal = Hessian.inverse() * gradient;
		xDisp = xDispLocal.x()*cLocalX + xDispLocal.y()*cLocalY;

		Eigen::Vector3 xVecOld = xVec;
		xVec = rotateUnitInDirection(xVec, xDisp);
		xVec.normalize();

		decimal err = (xVec-xVecOld).maxCoeff();
		if (err <= tolerance)
		{
			break;
		}
		else
		{
			if (err <= bkuptolerance)
			{
				tolerance = bkuptolerance;
			}
		}
	}
	std::cerr << "Avg: " << xVec.transpose() << std::endl;
	return xVec;
}

// Assumes r = 1
inline Eigen::Vector3 sphereToCart(const Eigen::Vector2& s)
{
	decimal sinTheta = sin(s[0]);
	decimal cosTheta = cos(s[0]);
	decimal sinPhi = sin(s[1]);
	decimal cosPhi = cos(s[1]);
	return Eigen::Vector3(cosTheta*sinPhi, sinTheta*sinPhi, cosPhi);
}

// Assumes r = 1
inline Eigen::Vector2 cartToSphere(const Eigen::Vector3& c)
{
	decimal r = c.norm();
	assert(r != 0);
	return Eigen::Vector2(atan2(c.y(), c.x()), acos(c.z()/r));
}

AugmentedMesh::AugmentedMesh()
    : Mesh(),
      hasVertexToFaceLookup(false),
      hasAdjacency(false),
      hasSurfaceCoordinates(false)
{
}

AugmentedMesh::AugmentedMesh(const Mesh& m)
    : Mesh(m),
      hasVertexToFaceLookup(false),
      hasAdjacency(false),
      hasSurfaceCoordinates(false)
{

}

void AugmentedMesh::computeVertexToFaceLookup()
{
	const int numFaces = faces.size();
	const int numVertices = vertices.size();

	vertexToFaceLookup.resize(numVertices);
	for (int i = 0; i < numFaces; ++i)
	{
		Mesh::Face& face = faces[i];
		for (int m = 0; m < 3; ++m)
		{
			vertexToFaceLookup[face.vertices[m]].push_back(i);
		}
	}

	hasVertexToFaceLookup = true;
}

void AugmentedMesh::computeAdjacency()
{
	const int numVertices = vertices.size();

	if (!hasVertexToFaceLookup) { computeVertexToFaceLookup(); }

	// Perform for all vertices
	for (int v = 0; v < numVertices; ++v)
	{
		const int numAdjacentFaces = vertexToFaceLookup[v].size();

		// For each vertex, loop through all pairs of adjacent faces
		for (int i = 0; i < numAdjacentFaces; ++i)
		{
			Mesh::Face& faceA = faces[vertexToFaceLookup[v][i]];
			for (int j = i+1; j < numAdjacentFaces; ++j)
			{
				Mesh::Face& faceB = faces[vertexToFaceLookup[v][j]];

				unsigned int match = 0;
				bool foundMatch = false;

				// Check for a match other than the current vertex
				for (int m = 0; m < 3; ++m)
				{
					if (faceA.vertices[m] == v) { continue; }
					for (int n = 0; n < 3; ++n)
					{
						if (faceB.vertices[n] == v) { continue; }

						if (faceA.vertices[m] == faceB.vertices[n])
						{
							foundMatch = true;
							match = faceA.vertices[m];
						}
					}
				}
				if (foundMatch)
				{
					// Face indices (vertex indices are v,match)
					size_t a = vertexToFaceLookup[v][i];
					size_t b = vertexToFaceLookup[v][j];

					// Add adjacency
					boost::add_edge(a, b, faceAdjacency);
					boost::add_edge(v, match, vertexAdjacency);
				}
			}
		}
	}

	hasAdjacency = true;
}

void AugmentedMesh::computeSurfaceCoordinates()
{
	const int numVertices = vertices.size();

	if (!hasAdjacency) { computeAdjacency(); }

	std::vector< Eigen::Vector2, Eigen::aligned_allocator<Eigen::Vector2> > sCoords(numVertices);

	// Compute coordinates for line-of-sight points
	// Perform for all vertices
	for (int v = 0; v < numVertices; ++v)
	{
		const Eigen::Vector3 cCoords = vertices[v] - cobb();
		const decimal r = cCoords.norm();
		if (r < 1e-6)
		{
			sCoords[v] = Eigen::Vector2::Zero();
		}
		else
		{
			sCoords[v] = Eigen::Vector2(atan2(cCoords.y(), cCoords.x()),
			                            acos(cCoords.z()/r));
		}
		std::cerr << sCoords[v].transpose() << std::endl;
	}

	std::vector< Eigen::Vector2, Eigen::aligned_allocator<Eigen::Vector2> > sCoordsNext(numVertices);
	for (int iter = 0; iter < 10; ++iter)
	{
		for (int v = 0; v < numVertices; ++v)
		{
			// Average of neighbors
			const int numNeighbors = boost::out_degree(v, vertexAdjacency);
			std::vector<Eigen::Vector3> neighborhood;

			Graph::adjacency_iterator e, eEnd;
			for (boost::tie(e, eEnd) = boost::adjacent_vertices(v, vertexAdjacency);
			     e != eEnd; ++e)
			{
				neighborhood.push_back(sphereToCart(sCoords[*e]));
			}
			neighborhood.push_back(sphereToCart(sCoords[v]));

			const decimal eta = 0.5; // "Learning rate"
			std::vector<decimal> w(numNeighbors+1, (1.0-eta)/static_cast<decimal>(numNeighbors));
			w.back() = eta;
			Eigen::Vector3 mean = sphericalMean(neighborhood, w, neighborhood.back());

			sCoordsNext[v] = cartToSphere(mean);
		}
		sCoords = sCoordsNext;
	}
	surfaceCoordinates = sCoords;
}

}
