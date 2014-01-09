/**
 * \file Octree.h
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

#ifndef OCTREE_H
#define OCTREE_H

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <memory>
//#include <boost/shared_ptr.hpp>

namespace ap
{

namespace Octree
{

typedef void T;
//template <class T>
class Element
{
public:
	Eigen::Vector3f mCenter;
	float mCubeDiameter;
	std::shared_ptr<T> mData;
	Element* mParent;
	bool mHasChildren;

	Element(const Eigen::Vector3f& center = Eigen::Vector3f::Zero(), const float width = 1,
			Element* parent = NULL,
			std::shared_ptr<T> data = nullptr);

	/**
	 * @brief Expand this leaf into a branch with 8 children
	 * @param levels Number of levels to expand
	 */
	void expand(unsigned int levels = 1);

	/**
	 * @brief Returns the smallest containing element for the query point
	 * @param query Point for which to find smallest containing element
	 */
	std::shared_ptr<Element> search(Eigen::Vector3f& queryPt);

protected:
	std::shared_ptr<Element> mChildren[8];
};

typedef Element Octree;


//////////////////////////////////////////////
/////////////// Implementation ///////////////
//////////////////////////////////////////////

//template <class T>
Element::Element(const Eigen::Vector3f& center, const float width,
				 Element* parent,
				 std::shared_ptr<T> data)
{
	mCenter = center;
	mCubeDiameter = width;
	mParent = parent;
	mData = data;
	mHasChildren = false;

	for (int i = 0; i < 8; ++i)
	{
		mChildren[i] = std::shared_ptr<Element>();
	}
}

//template <class T>
void Element::expand(unsigned int levels)
{
	if (levels > 0)
	{
		mHasChildren = true;
		for (int i = 0; i < 8; ++i)
		{
			float newWidth = mCubeDiameter/2.0f;
			float newRadius = mCubeDiameter/4.0f;
			Eigen::Vector3f childCenter(mCenter[0] + ((i < 4) ? newRadius : -newRadius ),
										mCenter[1] + (((i % 4) < 2) ? newRadius : -newRadius ),
										mCenter[2] + (((i % 2) == 0) ? newRadius : -newRadius ));
			mChildren[i] = std::shared_ptr<Element>(new Element(childCenter, newWidth, this));
			if (levels > 1)
			{
				mChildren[i]->expand(levels - 1);
			}
		}
	}
}

//template <class T>
std::shared_ptr<Element> Element::search(Eigen::Vector3f& queryPt)
{
	// TODO: You can do this with only 3 comparisons
	// Find closest child
	float minD = std::numeric_limits<float>::max();
	int nearestIndex = 0;
	for (int i = 0; i < 8; ++i)
	{
		float d = (queryPt - mChildren[i]->mCenter).squaredNorm(); // Cheaper than regular norm
		if (d < minD)
		{
			minD = d;
			nearestIndex = i;
		}
	}

	if (mChildren[nearestIndex]->mHasChildren)
	{
		return mChildren[nearestIndex]->search(queryPt);
	}
	else
	{
		return mChildren[nearestIndex];
	}
}


} // namespace Octree
} // namespace ap


#endif // OCTREE_H
