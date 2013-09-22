/**
 * \file pointcloud_utils.h
 * \brief
 *
 * \author Andrew Price
 * \date 9 22, 2013
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

#ifndef POINTCLOUD_UTILS_H
#define POINTCLOUD_UTILS_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/LU>
#include <Eigen/SVD>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <ap_robot_utils/geometry_utils.h>

namespace ap
{

typedef Eigen::Map<const Eigen::MatrixXf, Eigen::Aligned, Eigen::OuterStride<> > PointMapMatrix;

//template <typename PointType>
//Ray computeRotation(pcl::PointCloud<PointType>::Ptr& aCloud, pcl::PointCloud<PointType>::Ptr& bCloud)
Ray computeRotation(pcl::PointCloud<pcl::PointXYZ>::Ptr& aCloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& bCloud)
{
	// Get data as Eigen matrices
	Eigen::MatrixXf p_a = aCloud->getMatrixXfMap(3,4,0);
	Eigen::MatrixXf p_b = bCloud->getMatrixXfMap(3,4,0);

	// Compute delta vectors and midpoints from a to b
	Eigen::MatrixXf v = p_b - p_a;
	Eigen::MatrixXf m = p_a + (v/2);

	// Find the vector parallel to the axis of rotation (most perpendicular to the delta vectors)
	Eigen::JacobiSVD<Eigen::MatrixXf> svd(v, Eigen::ComputeFullU);
	Eigen::Vector3f alpha = svd.matrixU().col(2); // Eigenvector corresponding to smallest singular value

	// Project points into 2D plane normal to \alpha and passing through origin
	Eigen::MatrixXf m_proj = Eigen::MatrixXf::Constant(m.rows(), m.cols(), 0.0);
	for (int i = 0; i < m.cols(); i++)
	{
		m_proj.block<3,1>(0,i) = m - (m.block<3,1>(0,i).dot(alpha)*alpha);
	}

	// Solve for the intersection of the rays perpendicular to the delta and axis vectors
	Eigen::MatrixXf l = Eigen::MatrixXf::Constant(m.rows(), m.cols(), 0.0);
	for (int i = 0; i < m.cols(); i++)
	{
		l.block<3,1>(0,i) = v.block<3,1>(0,i).cross(alpha);
	}

	Eigen::Matrix3f A = Eigen::Matrix3f::Constant(0);
	Eigen::Vector3f b = Eigen::Vector3f::Constant(0);
	for (int i = 0; i < m.cols(); i++)
	{
		Eigen::Vector3f l_i = l.block<3,1>(0,i);
		A += Eigen::Matrix3f::Identity() - l_i*l_i.transpose();
		b += m.block<3,1>(0,i);
	}

	// Solve Ax+b=0
	Eigen::Vector3f x = A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(-b);

	Ray r;
	r.point = x;
	r.vector = alpha;

	return r;
}

}

#endif // POINTCLOUD_UTILS_H
