/**
 * \file eigen_definitions.h
 * \brief
 *
 * \author Andrew Price
 * \date 2014-9-24
 *
 * \copyright
 *
 * Copyright (c) 2014, Georgia Tech Research Corporation
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

#ifndef AP_EIGEN_DEFINITIONS_H
#define AP_EIGEN_DEFINITIONS_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>

//#define DOUBLE_PRECISION
namespace ap
{
#ifdef DOUBLE_PRECISION
typedef double decimal;
#else
typedef float decimal;
#endif
}

namespace Eigen
{
typedef Matrix<ap::decimal, 2, 1> Vector2;
typedef Matrix<ap::decimal, 3, 1> Vector3;
typedef Matrix<ap::decimal, 4, 1> Vector4;
typedef Matrix<ap::decimal, 5, 1> Vector5;
typedef Matrix<ap::decimal, 6, 1> Vector6;
typedef Matrix<ap::decimal, Dynamic, 1> VectorX;

typedef Matrix<ap::decimal, 3, 3> Matrix3;
typedef Matrix<ap::decimal, 4, 4> Matrix4;

typedef Matrix<ap::decimal, 3, Eigen::Dynamic> Matrix3X;
typedef Matrix<ap::decimal, 6, Eigen::Dynamic> Matrix6X;
typedef Matrix<ap::decimal, Eigen::Dynamic, 3> MatrixX3;
typedef Matrix<ap::decimal, Eigen::Dynamic, 6> MatrixX6;
typedef Matrix<ap::decimal, 6, 6> Matrix6;

typedef Transform<ap::decimal,2,Isometry> Isometry2;
typedef Transform<ap::decimal,3,Isometry> Isometry3;

typedef std::vector<Vector3, aligned_allocator<Vector3> > Vector3List;
typedef std::vector<Vector6, aligned_allocator<Vector6> > Vector6List;

}

#endif // AP_EIGEN_DEFINITIONS_H
