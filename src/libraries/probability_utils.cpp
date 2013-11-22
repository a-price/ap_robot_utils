/**
 * \file probability_utils.cpp
 * \brief
 *
 * \author Andrew Price
 * \date 9 30, 2013
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

#include "ap_robot_utils/probability_utils.h"
#include <cmath>

namespace ap
{


//Probability::Probability(double p, bool logMode)
//{
//	value = p;
//	useLogMode = logMode;
//}

//void Probability::setLogMode(bool mode)
//{
//	if (mode == useLogMode) // nothing new here
//	{
//		return;
//	}
//	else if (mode) // switch to log mode
//	{
//		value = log(value);
//	}
//	else
//	{
//		value = exp(value);
//	}

//	useLogMode = mode;
//}




//inline double addLogProbabilities(double logA, double logB)
//{
//	double C;
//	if (logB > logA)
//	{
//		// reuse C as temporary storage
//		C = logB;
//		logB = logA;
//		logA = C;
//	}

//	C = logB - logA; // C = log(B/A)

//	C = (C < 1e-9) ? 0 : log(1+exp(C));

//	return logA + C;
//}

//inline Probability operator +(const Probability& lhs, const Probability& rhs)
//{
//	Probability result(0);
//	if (lhs.useLogMode && !rhs.useLogMode)
//	{
//		result.useLogMode = true;
//		result.value = addLogProbabilities(lhs.value, log(rhs.value));
//	}
//	else if (!lhs.useLogMode && rhs.useLogMode)
//	{
//		result.useLogMode = true;
//		result.value = addLogProbabilities(log(lhs.value), rhs.value);
//	}
//	else if (lhs.useLogMode && rhs.useLogMode)
//	{
//		result.useLogMode = true;
//		result.value = addLogProbabilities(lhs.value, rhs.value);
//	}
//	else
//	{
//		result.useLogMode = false;
//		result.value = lhs.value + rhs.value;
//	}

//	return result;
//}

//inline Probability operator *(const Probability& lhs, const Probability& rhs)
//{
//	Probability result(0);
//	if (lhs.useLogMode && !rhs.useLogMode)
//	{
//		result.useLogMode = true;
//		result.value = (lhs.value + log(rhs.value));
//	}
//	else if (!lhs.useLogMode && rhs.useLogMode)
//	{
//		result.useLogMode = true;
//		result.value = (log(lhs.value) + rhs.value);
//	}
//	else if (lhs.useLogMode && rhs.useLogMode)
//	{
//		result.useLogMode = true;
//		result.value = (lhs.value + rhs.value);
//	}
//	else
//	{
//		result.useLogMode = false;
//		result.value = lhs.value * rhs.value;
//	}

//	return result;
//}

} // namespace ap
