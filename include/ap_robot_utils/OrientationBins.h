/**
 * \file OrientationBins.h
 * \brief
 *
 * \author Andrew Price
 * \date October 21, 2014
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

#ifndef ORIENTATIONBINS_H
#define ORIENTATIONBINS_H

#include "ap_robot_utils/eigen_definitions.h"
#include "ap_robot_utils/shared_ptr.h"
#include <array>

namespace ap
{

const int NUM_90DEG_ORIENTATIONS = 24;

typedef Eigen::Quaternion<ap::decimal> Quaternion;
typedef Eigen::AngleAxis<ap::decimal> AngleAxis;

class BaseOrientations
{
public:
	static BaseOrientations* getInstance();
	const std::array<Quaternion, NUM_90DEG_ORIENTATIONS>& orientations() const
	{
		return gOrientations;
	}

protected:
	static std::array<Quaternion, NUM_90DEG_ORIENTATIONS> gOrientations;

private:
	BaseOrientations();
	BaseOrientations(const BaseOrientations&) {}
	BaseOrientations& operator=(const BaseOrientations&) { return *this; }

	static BaseOrientations* gBaseOrientations;
};

template <class T>
class OrientationBins
{
public:
	typedef typename std::array<shared_ptr<T>, NUM_90DEG_ORIENTATIONS>::iterator bin_iterator;
	shared_ptr<T>& search(const Quaternion& query);

protected:
	std::array<shared_ptr<T>, NUM_90DEG_ORIENTATIONS> mBins;
};

template <class T>
shared_ptr<T>& OrientationBins<T>::search(const Quaternion& query)
{
	Quaternion qryInv = query.inverse();
	ap::decimal minAngle = std::numeric_limits<ap::decimal>::max();

	std::array<Quaternion, NUM_90DEG_ORIENTATIONS>::const_iterator iter, endIter = BaseOrientations::getInstance()->orientations().end();
	bin_iterator targetIter, finalIter;

	for (iter = BaseOrientations::getInstance()->orientations().begin(), targetIter = mBins.begin();
	     iter != endIter;
	     ++iter, ++targetIter)
	{
		ap::decimal angle = ap::AngleAxis(qryInv * (*iter)).angle();

		if (angle < minAngle)
		{
			minAngle = angle;
			finalIter = targetIter;
		}
	}

	return *finalIter;
}

} // namespace ap


#endif // ORIENTATIONBINS_H
