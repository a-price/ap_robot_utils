/**
 * \file math_utils.h
 * \brief
 *
 * \author Andrew Price
 * \date 2015-1-7
 *
 * \copyright
 *
 * Copyright (c) 2015, Georgia Tech Research Corporation
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

#ifndef MATH_UTILS_H
#define MATH_UTILS_H

#include <ap_robot_utils/eigen_definitions.h>

namespace ap
{

inline ap::decimal randbetween(ap::decimal min, ap::decimal max)
{
	return (max - min) * ( (ap::decimal)rand() / (ap::decimal)RAND_MAX ) + min;
}

}

namespace Eigen
{
template<typename _Matrix_Type_>
bool pseudoInverse(const _Matrix_Type_ &a, _Matrix_Type_ &result, double
				   epsilon = std::numeric_limits<typename _Matrix_Type_::Scalar>::epsilon())
{
	bool doTranspose = false;
	_Matrix_Type_ a_oriented;
	if(a.rows()<a.cols())
	{
		doTranspose = true;
		a_oriented = a.transpose();
	}
	else
	{
		a_oriented = a;
	}

	Eigen::JacobiSVD< _Matrix_Type_ > svd = a_oriented.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);

	typename _Matrix_Type_::Scalar tolerance = epsilon * std::max(a_oriented.cols(),
																  a_oriented.rows()) * svd.singularValues().array().abs().maxCoeff();

	result = svd.matrixV() * _Matrix_Type_( (svd.singularValues().array().abs() >
											 tolerance).select(svd.singularValues().
															   array().inverse(), 0) ).asDiagonal() * svd.matrixU().adjoint();
	if (doTranspose)
	{
		result.transposeInPlace();
	}
	return true;
}
}

#endif // MATH_UTILS_H
