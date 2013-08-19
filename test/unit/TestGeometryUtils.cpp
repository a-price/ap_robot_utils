/**
 * \file TestGeometryUtils.cpp
 * \brief
 *
 * \author Andrew Price
 * \date July 19, 2013
 *
 * \copyright
 *
 * Copyright (c) 2013, Georgia Tech Research Corporation
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

#include <ap_robot_utils/geometry_utils.h>

class TestGeometryUtils : public CppUnit::TestFixture
{
	CPPUNIT_TEST_SUITE( TestGeometryUtils );
	CPPUNIT_TEST(TestQuaternionAveraging);
	CPPUNIT_TEST(TestRayIntersection);
	CPPUNIT_TEST_SUITE_END();
public:

	virtual void setUp()
	{
		std::string urdf = "/home/arprice/catkin_workspace/src/drchubo/drchubo-v2/robots/drchubo-v2.urdf";

		srand(time(NULL));
	}

	virtual void tearDown () {}

	float randbetween(double min, double max)
	{
		return (max - min) * ( (double)rand() / (double)RAND_MAX ) + min;
	}

	void TestQuaternionAveraging()
	{
		ap::QuaternionStdVector qs;
		std::vector<float> weights;
		Eigen::Quaternionf resultA, resultB;

		qs.push_back(Eigen::Quaternionf(1,1,0,0));
		qs.push_back(Eigen::Quaternionf(1,0,0,1));
		weights.push_back(1);
		weights.push_back(1);

		// Test Standard
		resultA = ap::averageQuaternions(qs);
		std::cerr << resultA.matrix() << std::endl << std::endl;

		// Test 1's Weights
		resultB = ap::averageQuaternions(qs, &weights);
		std::cerr << resultB.matrix() << std::endl << std::endl;
		CPPUNIT_ASSERT(resultA.isApprox(resultB));

		// Test invalid weights default to even weights
		weights.push_back(1);
		resultB = ap::averageQuaternions(qs, &weights);
		std::cerr << resultB.matrix() << std::endl << std::endl;
		CPPUNIT_ASSERT(resultA.isApprox(resultB));

		// Test legit varying weights
		qs.push_back(Eigen::Quaternionf(1,0,1,0));
		resultB = ap::averageQuaternions(qs, &weights);
		std::cerr << resultB.matrix() << std::endl << std::endl;
	}

	void TestRayIntersection()
	{
		ap::Ray r;
		r.point = Eigen::Vector3f::Zero();
		r.vector = Eigen::Vector3f::UnitX();

		Eigen::Vector3f a = Eigen::Vector3f(1,1,-1);
		Eigen::Vector3f b = Eigen::Vector3f(1,0,1);
		Eigen::Vector3f c = Eigen::Vector3f(1,-1,-1);
		ap::Triangle t1(a, b, c);

		// Test plane intersection
		ap::Point expected = Eigen::Vector3f(1,0,0);
		ap::Point actual = ap::intersectRayPlane(r, t1.getPlane());

		std::cerr << "Expected: " << expected.transpose() << std::endl;
		std::cerr << "Actual: " << actual.transpose() << std::endl;

		CPPUNIT_ASSERT((expected - actual).norm() < 0.000001);

		// Test true intersection
		actual = ap::intersectRayTriangle(r, t1);

		std::cerr << "Expected: " << expected.transpose() << std::endl;
		std::cerr << "Actual: " << actual.transpose() << std::endl;

		CPPUNIT_ASSERT((expected - actual).norm() < 0.000001);

		// Test missed intersection
		Eigen::Vector3f b2 = Eigen::Vector3f(1,0,-2);
		ap::Triangle t2(a, b2, c);

		actual = ap::intersectRayTriangle(r, t2);

		std::cerr << "Expected: " << (Eigen::Vector3f::Ones() * NAN).transpose() << std::endl;
		std::cerr << "Actual: " << actual.transpose() << std::endl;

		CPPUNIT_ASSERT(!std::isfinite(actual.x()));

		// Test degenerate triangle
		b2 = Eigen::Vector3f(1,0,-1);
		t2 = ap::Triangle(a, b2, c);

		actual = ap::intersectRayTriangle(r, t2);

		std::cerr << "Expected: " << (Eigen::Vector3f::Ones() * NAN).transpose() << std::endl;
		std::cerr << "Actual: " << actual.transpose() << std::endl;

		CPPUNIT_ASSERT(!std::isfinite(actual.x()));

		// Test Parallel triangle
		r.vector = Eigen::Vector3f::UnitZ();
		actual = ap::intersectRayPlane(r, t1.getPlane());

		std::cerr << "Expected: " << (Eigen::Vector3f::Ones() * NAN).transpose() << std::endl;
		std::cerr << "Actual: " << actual.transpose() << std::endl;

		// TODO: Test Perpendicular, nonintersecting triangle
		//

		CPPUNIT_ASSERT(!std::isfinite(actual.x()));
	}

};

CPPUNIT_TEST_SUITE_REGISTRATION(TestGeometryUtils);
