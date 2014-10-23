/**
 * \file OrientationBins.cpp
 * \brief
 *
 * \author Andrew Price
 * \date 2014-10-23
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

#include "ap_robot_utils/OrientationBins.h"
#include <iostream>

namespace ap
{

// Instantiate static class members
std::array<Quaternion, NUM_90DEG_ORIENTATIONS> BaseOrientations::gOrientations;

BaseOrientations* BaseOrientations::gBaseOrientations;

BaseOrientations* BaseOrientations::getInstance()
{
	if (nullptr == gBaseOrientations)
	{
		 gBaseOrientations = new BaseOrientations();
	}

	return gBaseOrientations;
}

BaseOrientations::BaseOrientations()
{
	std::array<Eigen::Matrix3, 3> initials;
	initials[0] << 1,0,0,
	               0,1,0,
	               0,0,1;
	initials[1] << 0,1,0,
	               0,0,1,
	               1,0,0;
	initials[2] << 0,0,1,
	               1,0,0,
	               0,1,0;

	for (int i = 0; i < 3; ++i)
	{
		for (int j = 0; j < 2; ++j)
		{
			for (int k = 0; k < 4; ++k)
			{
				Eigen::Matrix3 temp = AngleAxis(M_PI/2.0*(float)k, Eigen::Vector3::UnitX()) * AngleAxis(M_PI*j, Eigen::Vector3::UnitY()) * initials[i];

				// Round to +/- 1's
				for (int n = 0; n < 9; ++n) { temp(n) = round(temp(n)); }

				int idx = (((2*i)+j)*4)+k;

				gOrientations[idx] = temp;
			}
		}
	}
}

} // namespace ap
