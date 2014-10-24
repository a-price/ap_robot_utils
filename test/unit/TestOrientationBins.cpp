/**
 * \file TestOrientationBins.cpp
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


#include <cppunit/TestFixture.h>
#include <cppunit/extensions/HelperMacros.h>

#include <ap_robot_utils/OrientationBins.h>

class TestOrientationBins : public CppUnit::TestFixture
{
	CPPUNIT_TEST_SUITE( TestOrientationBins );
	CPPUNIT_TEST(TestInitialization);
	CPPUNIT_TEST(TestBinSearch);
	CPPUNIT_TEST_SUITE_END();
public:

	virtual void setUp() {}

	virtual void tearDown () {}

	void TestInitialization()
	{
		const std::array<ap::Quaternion, ap::NUM_90DEG_ORIENTATIONS>& bases = ap::BaseOrientations::getInstance()->orientations();

		for (int i = 0; i < ap::NUM_90DEG_ORIENTATIONS; ++i)
		{
			for (int j = i+1; j < ap::NUM_90DEG_ORIENTATIONS; ++j)
			{
				ap::decimal angle = ap::AngleAxis(bases[i]*bases[j].inverse()).angle();
//				std::cout << angle << std::endl;
				if (fabs(angle) < 0.99*M_PI/2.0)
				{
					std::cout << i << ":\n" << Eigen::Matrix3(bases[i]) << std::endl;
					std::cout << j << ":\n" << Eigen::Matrix3(bases[j]) << std::endl;
				}
				CPPUNIT_ASSERT(fabs(angle) > 0.99*M_PI/2.0);
			}
		}
	}

	void TestBinSearch()
	{
		ap::OrientationBins<ap::decimal> oBins;

		ap::shared_ptr<ap::decimal>& bin = oBins.search(ap::Quaternion(Eigen::Matrix3::Identity()));

		bin = ap::shared_ptr<ap::decimal>(new ap::decimal);
		(*bin) = 0.0;

		bin = oBins.search(ap::Quaternion(ap::AngleAxis(M_PI/8.0, Eigen::Vector3::UnitX())));
		CPPUNIT_ASSERT(*bin == 0.0);

		bin = oBins.search(ap::Quaternion(ap::AngleAxis(M_PI/2.0, Eigen::Vector3::UnitX())));
		CPPUNIT_ASSERT(bin == ap::shared_ptr<ap::decimal>());
	}

};

CPPUNIT_TEST_SUITE_REGISTRATION(TestOrientationBins);
