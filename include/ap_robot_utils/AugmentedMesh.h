/**
 * \file AugmentedMesh.h
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

#ifndef AUGMENTEDMESH_H
#define AUGMENTEDMESH_H

#include "ap_robot_utils/geometry_utils.h"
#include <boost/graph/adjacency_list.hpp>
#include <Eigen/StdVector>

namespace ap
{

class AugmentedMesh : public Mesh
{
public:
	typedef boost::adjacency_list<boost::setS, boost::vecS, boost::undirectedS> Graph;

	AugmentedMesh();
	AugmentedMesh(const Mesh& m);

	std::vector< std::vector<size_t> > vertexToFaceLookup;
	bool hasVertexToFaceLookup;
	void computeVertexToFaceLookup();

	Graph vertexAdjacency;
	Graph faceAdjacency;
	bool hasAdjacency;
	void computeAdjacency();

	std::vector< Eigen::Vector2, Eigen::aligned_allocator<Eigen::Vector2> > surfaceCoordinates;
	bool hasSurfaceCoordinates;
	void computeSurfaceCoordinates();
};

}

#endif // AUGMENTEDMESH_H
