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

/**
 * @brief computeIntersection
 * @param p
 * @param v
 * @return
 *
 * @link http://math.stackexchange.com/questions/36398/point-closest-to-a-set-four-of-lines-in-3d
 */
Point computeIntersection(Eigen::Matrix3Xf& p, Eigen::Matrix3Xf& v)
{
	Eigen::Matrix3f A = Eigen::Matrix3f::Constant(0);
	Eigen::Vector3f b = Eigen::Vector3f::Constant(0);
	Eigen::Vector3f v_i;
	Eigen::Matrix3f projection;
	for (int i = 0; i < p.cols(); i++)
	{
		v_i = v.col(i);
		projection = Eigen::Matrix3f::Identity() - (v_i*v_i.transpose());
		A += projection;
		b += projection * p.col(i);
	}

	// Solve Ax+b=0
	Eigen::Vector3f x = A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);

	return x;
}


Ray computeRotation(pcl::PointCloud<pcl::PointXYZ>::Ptr& aCloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& bCloud)
{
	// Get data as Eigen matrices
	Eigen::Matrix3Xf p_a = aCloud->getMatrixXfMap(3,4,0);
	Eigen::Matrix3Xf p_b = bCloud->getMatrixXfMap(3,4,0);

	// Compute delta vectors and midpoints from a to b
	Eigen::Matrix3Xf v = p_b - p_a;
	Eigen::Matrix3Xf m = p_a + (v/2);

	// Find the vector parallel to the axis of rotation (most perpendicular to the delta vectors)
	Eigen::JacobiSVD<Eigen::Matrix3Xf> svd(v, Eigen::ComputeFullU);
	Eigen::Vector3f alpha = svd.matrixU().col(2); // Eigenvector corresponding to smallest singular value

	// Project points into 2D plane normal to \alpha and passing through origin
	Eigen::Matrix3Xf m_proj = Eigen::Matrix3Xf::Constant(m.rows(), m.cols(), 0.0);
	for (int i = 0; i < m.cols(); i++)
	{
		m_proj.col(i) = m.col(i) - (m.col(i).dot(alpha)*alpha);
	}

	// Solve for the intersection of the rays perpendicular to the delta and axis vectors
	Eigen::Matrix3Xf l = Eigen::Matrix3Xf::Constant(m.rows(), m.cols(), 0.0);
	for (int i = 0; i < m.cols(); i++)
	{
		l.col(i) = v.col(i).cross(alpha).normalized();
	}

	Eigen::Vector3f x = computeIntersection(m_proj, l);

	Ray r;
	r.point = x;
	r.vector = alpha;

	return r;
}

}

#endif // POINTCLOUD_UTILS_H
