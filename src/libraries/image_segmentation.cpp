/**
 * \file image_segmentation.cpp
 * \brief
 *
 * \author Andrew Price
 * \date November 19, 2013
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

#include "ap_robot_utils/image_segmentation.h"
#include "misc.h"
#include "segment-graph.h"

#include <iostream>
namespace ap
{


float diffSTD(const cv::Mat& image, int x1, int y1, int x2, int y2)
{
	int width = image.cols;
	cv::Vec3b pxA = image.at<cv::Vec3b>(x1 + y1 * width);
	cv::Vec3b pxB = image.at<cv::Vec3b>(x2 + y2 * width);
	return sqrt(square(pxA[0] - pxB[0]) +
				square(pxA[1] - pxB[1]) +
				square(pxA[2] - pxB[2]));
}

float diffHSV(const cv::Mat& image, int x1, int y1, int x2, int y2)
{
	int width = image.cols;
	cv::Vec3b pxA = image.at<cv::Vec3b>(x1 + y1 * width);
	cv::Vec3b pxB = image.at<cv::Vec3b>(x2 + y2 * width);
	return sqrt(square(pxA[0] - pxB[0]) +
				fabs(pxA[1] - pxB[1]) +
				fabs(pxA[2] - pxB[2]));
}


void segmentFelzenszwalb(const cv::Mat& input, Segmentation& s, const float c, unsigned int min_size, DifferenceFunction diff )
{
	// Grab the dimensions of the image
	const int width = input.cols;
	const int height = input.rows;

	// Construct the graph edge array
	edge* edges = new edge[width*height*4];
	memset(edges, 0, sizeof(edge)*width*height*4);

	// Loop through all pixels, adding edges down and to the right
	int num = 0;
	for (int y = 0; y < height; y++)
	{
		for (int x = 0; x < width; x++)
		{
			if (x < width-1)
			{
				edges[num].a = y * width + x;
				edges[num].b = y * width + (x+1);
				edges[num].w = (*diff)(input, x, y, x+1, y);
				++num;
			}

			if (y < height-1)
			{
				edges[num].a = y * width + x;
				edges[num].b = (y+1) * width + x;
				edges[num].w = (*diff)(input, x, y, x, y+1);
				++num;
			}

			if ((x < width-1) && (y < height-1))
			{
				edges[num].a = y * width + x;
				edges[num].b = (y+1) * width + (x+1);
				edges[num].w = (*diff)(input, x, y, x+1, y+1);
				++num;
			}

			if ((x < width-1) && (y > 0))
			{
				edges[num].a = y * width + x;
				edges[num].b = (y-1) * width + (x+1);
				edges[num].w = (*diff)(input, x, y, x+1, y-1);
				++num;
			}
		}
	}

	// Do the actual segmentation
	universe* u = segment_graph(width * height, num, edges, c);

	// Enforce minimum size for clusters
	for (int i = 0; i < num; i++)
	{
		int a = u->find(edges[i].a);
		int b = u->find(edges[i].b);
		if ((a != b) && ((u->size(a) < min_size) || (u->size(b) < min_size)))
			u->join(a, b);
	}

	// Create the segment containers
	s.clear();
	for (int comps = 0; comps < u->num_sets(); ++comps)
	{
		Segment temp;
		s.push_back(temp);
	}

	std::map<int, int> componentMap;
	int componentCount = 0;

	// Assign all pixels to their components
	for (int y = 0; y < height; y++)
	{
		for (int x = 0; x < width; x++)
		{
			int comp = u->find(y * width + x);
			int targetComponent;

			std::map<int, int>::iterator iter = componentMap.find(comp);
			if (iter == componentMap.end())
			{
				componentMap.insert(std::pair<int, int>(comp, componentCount));
				targetComponent = componentCount++;
			}
			else
			{
				targetComponent = iter->second;
			}

			assert(targetComponent < u->num_sets());

			s[targetComponent].push_back(cv::Point2i(x, y));
		}
	}

	delete [] edges;
	delete u;
}

void recolorSegmentation(const cv::Mat& colorIm, cv::Mat& recolorIm, const Segmentation &s, bool useAverageColor)
{
	assert(colorIm.data != recolorIm.data);

	recolorIm = cv::Mat(colorIm.rows, colorIm.cols, colorIm.type());

	for (int comp = 0; comp < s.size(); ++comp)
	{
		cv::Vec3b color;
		if (useAverageColor)
		{
			cv::Vec3f tempColor = cv::Vec3f(0,0,0);
			for (int element = 0; element < s[comp].size(); ++element)
			{
				cv::Point2i p = s[comp][element];
				assert(p.x + (p.y * colorIm.cols) < colorIm.rows * colorIm.cols);
				tempColor += colorIm.at<cv::Vec3b>(p);
			}
			color = tempColor / (float)s[comp].size();
		}
		else
		{
			// Create a random color for this component
			color = cv::Vec3b((uchar)random(), (uchar)random(), (uchar)random());
		}

		// Assign all elements of this component to this color
		for (int element = 0; element < s[comp].size(); ++element)
		{
			cv::Point2i p = s[comp][element];
			assert(p.x + (p.y * colorIm.cols) < colorIm.rows * colorIm.cols);
			recolorIm.at<cv::Vec3b>(p) = color;
		}
	}
}


}
