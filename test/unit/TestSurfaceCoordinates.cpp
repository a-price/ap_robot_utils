/**
 * \file TestSurfaceCoordinates.cpp
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

#include <cppunit/TestFixture.h>
#include <cppunit/extensions/HelperMacros.h>

#include "ap_robot_utils/AugmentedMesh.h"
#include "ap_robot_utils/ros_utils.h"

class TestSurfaceCoordinates : public CppUnit::TestFixture
{
	CPPUNIT_TEST_SUITE( TestSurfaceCoordinates );
	CPPUNIT_TEST(TestSimpleParameterization);
	CPPUNIT_TEST_SUITE_END();
public:

	virtual void setUp() {}

	virtual void tearDown () {}

	void TestSimpleParameterization()
	{
		ap::Mesh* m = ap::loadMesh(ap::parsePackageURL("package://ap_robot_utils/test/resources/cube.stl"));
		ap::AugmentedMesh am(*m);

		std::cerr << am.cobb().transpose() << std::endl;

		am.computeSurfaceCoordinates();
		for (int i = 0; i < 8; ++i)
		{
			std::cerr << am.surfaceCoordinates[i].transpose() << "\t:\t" << am.vertices[i].transpose() << std::endl;
		}
	}
};


CPPUNIT_TEST_SUITE_REGISTRATION(TestSurfaceCoordinates);
