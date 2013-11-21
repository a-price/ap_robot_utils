/**
 * \file TestFelz.cpp
 * \brief
 *
 * \author Andrew Price
 * \date 11 20, 2013
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


#include <cppunit/TestFixture.h>
#include <cppunit/extensions/HelperMacros.h>

#include <ap_robot_utils/image_segmentation.h>

#include <opencv2/highgui/highgui.hpp>

class TestSegmentation : public CppUnit::TestFixture
{
	CPPUNIT_TEST_SUITE( TestSegmentation );
	CPPUNIT_TEST(TestImageSegmentation);
	CPPUNIT_TEST_SUITE_END();
public:

	virtual void setUp()
	{
		srand(time(NULL));
	}

	virtual void tearDown () {}

	void TestImageSegmentation()
	{
		cv::Mat img(3,3,CV_8UC3);
		img.at<cv::Vec3b>(0,0) = cv::Vec3b(1);
		img.at<cv::Vec3b>(0,1) = cv::Vec3b(2);
		img.at<cv::Vec3b>(0,2) = cv::Vec3b(3);
		img.at<cv::Vec3b>(1,0) = cv::Vec3b(120);
		img.at<cv::Vec3b>(1,1) = cv::Vec3b(124);
		img.at<cv::Vec3b>(1,2) = cv::Vec3b(128);
		img.at<cv::Vec3b>(2,0) = cv::Vec3b(250);
		img.at<cv::Vec3b>(2,1) = cv::Vec3b(251);
		img.at<cv::Vec3b>(2,2) = cv::Vec3b(252);

		cv::namedWindow("Hello", cv::WINDOW_NORMAL);
		cv::imshow("Hello", img);
		//cv::waitKey();

		ap::Segmentation s;
		ap::segmentFelzenszwalb(img, 2, s);

		ap::recolorSegmentation(img, s);

		cv::imshow("Hello", img);
		cv::waitKey();
	}

};

CPPUNIT_TEST_SUITE_REGISTRATION(TestSegmentation);
