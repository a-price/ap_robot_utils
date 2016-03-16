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

namespace ap
{

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
		const ap::decimal r = cCoords.norm();
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
	for (int iter = 0; iter < 1; ++iter)
	{
		for (int v = 0; v < numVertices; ++v)
		{
			// Average of neighbors
			Eigen::Vector2 avg = Eigen::Vector2::Zero();
			ap::decimal dWeight = 0;
			Graph::adjacency_iterator e, eEnd;
			for (boost::tie(e, eEnd) = boost::adjacent_vertices(v, vertexAdjacency);
			     e != eEnd; ++e)
			{
				std::cerr << v << "=>" << *e << std::endl;
				ap::decimal d = (vertices[*e]-vertices[v]).norm();
				if (fabs(sCoords[v][0]-sCoords[*e][0]) < M_PI)
				{
					avg += sCoords[*e]/d;
				}
				else
				{
					std::cerr << "avg err." << std::endl;
					ap::decimal newTheta = sCoords[*e][0];

					avg[1] += sCoords[*e][1]/d;
				}
				dWeight += 1.0/d;
			}
			avg /= (static_cast<ap::decimal>(boost::out_degree(v, vertexAdjacency))*dWeight);

			const ap::decimal eta = 0.5;
			sCoordsNext[v] = (1.0-eta)*sCoords[v] + eta*avg;

		}

		sCoords = sCoordsNext;
	}

	surfaceCoordinates = sCoords;
}

}
