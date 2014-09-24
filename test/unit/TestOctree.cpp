/**
 * \file TestOctree.cpp
 * \brief
 *
 * \author Andrew Price
 * \date January 8, 2014
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

#include <ap_robot_utils/Octree.h>

class TestOctree : public CppUnit::TestFixture
{
	CPPUNIT_TEST_SUITE( TestOctree );
	CPPUNIT_TEST(TestOctreeSearch);
	CPPUNIT_TEST(TestDrillDown);
	CPPUNIT_TEST(TestLeafIteration);
	CPPUNIT_TEST_SUITE_END();
public:

	virtual void setUp() {}

	virtual void tearDown () {}

	void TestOctreeSearch()
	{
		ap::shared_ptr<ap::Octree::Octree<float> > ot(new ap::Octree::Octree<float>() );
		ot->expand(3);
		Eigen::Vector3 query(0.5, 0.1, -0.8);
		ap::shared_ptr<ap::Octree::Element<float> > target = ot->search(query);
		CPPUNIT_ASSERT_EQUAL(Eigen::Vector3(0.4375, 0.0625, -0.4375), target->mCenter);
		CPPUNIT_ASSERT(!target->mHasChildren);

		query = Eigen::Vector3(0.2, 0.1, -0.3);
		target = ot->search(query);
		std::cout << target->mCenter << "\n" << target->mHasChildren << std::endl;
	}

	void TestDrillDown()
	{
		ap::shared_ptr<ap::Octree::Octree<float> > ot(new ap::Octree::Octree<float>() );
		Eigen::Vector3 query(rand()/RAND_MAX, rand()/RAND_MAX, rand()/RAND_MAX);
		ap::shared_ptr<ap::Octree::Element<float> > res = ot->search(query, 0, true);
		CPPUNIT_ASSERT(ot->mTree == res); // Root node is tree
		CPPUNIT_ASSERT(!ot->mTree->mHasChildren);

		res = ot->search(query, 3, true);
		CPPUNIT_ASSERT(ot->mTree->mHasChildren);
		CPPUNIT_ASSERT(res->mParent->mHasChildren);
	}

	void TestLeafIteration()
	{
		ap::shared_ptr<ap::Octree::Octree<float> > ot(new ap::Octree::Octree<float>() );
		ot->expand(2);
		Eigen::Vector3 query(rand()/RAND_MAX, rand()/RAND_MAX, rand()/RAND_MAX);
		ap::shared_ptr<ap::Octree::Element<float> > res = ot->search(query, 3, true);
		ap::shared_ptr<ap::Octree::Element<float> > leaf = ot->mTree->nextLeaf();
		leaf = leaf->nextLeaf();
		CPPUNIT_ASSERT(leaf->mChildIndexOfParent == 1);
		int safetyIters = 200;
		while(safetyIters-- > 0)
		{
			leaf->printPath();
			res = leaf->nextLeaf();
			// Check for end of line;
			if (ap::shared_ptr<ap::Octree::Element<float> >() == res)
			{
				break;
			}
			leaf = res;
		}
		CPPUNIT_ASSERT(safetyIters > 0); // check for possible endless loop
	}

};

CPPUNIT_TEST_SUITE_REGISTRATION(TestOctree);
