/**
 * \file Quadtree.h
 * \brief
 *
 * \author Andrew Price
 * \date October 27, 2014
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

#ifndef QUADTREE_H
#define QUADTREE_H

#include "ap_robot_utils/eigen_definitions.h"
#include "ap_robot_utils/shared_ptr.h"

namespace ap
{

namespace Quadtree
{

//typedef float T;
template <class T>
class Element
{
public:
//	friend class Quadtree<T>;

	Eigen::Vector2 mCenter;
	float mCellDiameter;
	shared_ptr<T> mData;
	bool mHasChildren;
	int mChildIndexOfParent;

	Element(const Eigen::Vector2& center = Eigen::Vector2::Zero(), const float width = 1,
			shared_ptr<Element> parent = shared_ptr<Element>(),
			shared_ptr<T> data = nullptr);

	/**
	 * @brief Expand this leaf into a branch with 4 children
	 * @param levels Number of levels to expand
	 */
	void expand(unsigned int levels = 1);

	/**
	 * @brief Recursively removes leaves who have no data and no child nodes
	 */
	void prune();

	/**
	 * @brief Returns the smallest containing element for the query point
	 * @param query Point for which to find smallest containing element
	 */
	shared_ptr<Element> search(Eigen::Vector2& queryPt, const int maxLevel = -1, const bool drillDownToDepth = false);

	shared_ptr<Element> nextLeaf(const int startChild = 0);
	void printPath();

//protected:
	shared_ptr<Element> mChildren[4];
	shared_ptr<Element> mSelf;
	shared_ptr<Element> mParent;
};

template <class T>
class Quadtree
{
public:
	Quadtree(const Eigen::Vector2& center = Eigen::Vector2::Zero(), const float width = 1,
		   shared_ptr<T> data = nullptr) // Create root element and smart pointer to it
	{
		mTree = shared_ptr<Element<T> >(new Element<T> (center, width, NULL, data));
		mTree->mSelf = mTree;
	}

	void expand(unsigned int levels = 1)
	{
		mTree->expand(levels);
	}

	/**
	 * @brief Recursively removes leaves who have no data and no child nodes
	 */
	void prune()
	{
		mTree->prune();
	}

	shared_ptr<Element<T> > search(Eigen::Vector2& queryPt, const int maxLevel = -1, const bool drillDownToDepth = false)
	{
		return mTree->search(queryPt, maxLevel, drillDownToDepth);
	}

	shared_ptr<Element<T> > search(Eigen::Vector3& queryPt, const int maxLevel = -1, const bool drillDownToDepth = false)
	{
		ap::decimal w = sqrt(queryPt.x()*queryPt.x() + queryPt.y()*queryPt.y());
		Eigen::Vector2 queryPt2d(w, queryPt.z());
		return mTree->search(queryPt2d, maxLevel, drillDownToDepth);
	}

	shared_ptr<Element<T> > mTree;
};


//////////////////////////////////////////////
/////////////// Implementation ///////////////
//////////////////////////////////////////////

template <class T>
Element<T>::Element(const Eigen::Vector2& center, const float width,
				 shared_ptr<Element> parent,
				 shared_ptr<T> data)
{
	mCenter = center;
	mCellDiameter = width;
	mParent = parent;
	mData = data;
	mHasChildren = false;
	mChildIndexOfParent = 0;

	for (int i = 0; i < 4; ++i)
	{
		mChildren[i] = shared_ptr<Element>();
	}
}

template <class T>
void Element<T>::expand(unsigned int levels)
{
	if (levels > 0)
	{
		mHasChildren = true;
		for (int i = 0; i < 4; ++i)
		{
			float newWidth = mCellDiameter/2.0f;
			float newRadius = mCellDiameter/4.0f;
			Eigen::Vector2 childCenter(mCenter[0] + (((i % 4) < 2) ? newRadius : -newRadius ),
			                           mCenter[1] + (((i % 2) == 0) ? newRadius : -newRadius ));
			mChildren[i] = shared_ptr<Element<T> >(new Element<T>(childCenter, newWidth, mSelf));
			mChildren[i]->mSelf = mChildren[i];
			mChildren[i]->mChildIndexOfParent = i;
			if (levels > 1)
			{
				mChildren[i]->expand(levels - 1);
			}
		}
	}
}

template <class T>
void Element<T>::prune()
{
	if (mHasChildren)
	{
		bool childHasData = false;
		for (int i = 0; i < 4; ++i)
		{
			if (shared_ptr<Element<T> >() != mChildren[i])
			{
				mChildren[i]->prune();

				if (mChildren[i]->mHasChildren || shared_ptr<T>() != mChildren[i]->mData)
				{
					childHasData = true;
					break;
				}
			}
		}

		if (!childHasData)
		{
			for (int i = 0; i < 4; ++i)
			{
				// Don't forget to delete the self-pointer in the child
				mChildren[i]->mSelf = shared_ptr<Element<T> >();
				mChildren[i] = shared_ptr<Element<T> >();
			}
			mHasChildren = false;
		}
	}
}

template <class T>
shared_ptr<Element<T> > Element<T>::search(Eigen::Vector2& queryPt, const int maxLevel, const bool drillDownToDepth)
{
	assert(!(maxLevel < 0 && drillDownToDepth)); // This pair will drill down forever
	// Check for maxDepth
	int levelsRemaining = -1; // Defaults to unlimited levels
	if (maxLevel == 0)
	{
		return mSelf; // Found current level
	}
	else if(maxLevel > 0)
	{
		levelsRemaining = maxLevel - 1; // Decrement remaining depth
	}

	// Check for children
	if (!this->mHasChildren)
	{
		if (drillDownToDepth)
		{
			this->expand(1);
		}
		else
		{
			// Function has bottomed out
			return mSelf;
		}
	}

	// TODO: You can do this with only 3 comparisons
	// Find closest child
	float minD = std::numeric_limits<float>::max();
	int nearestIndex = 0;
	for (int i = 0; i < 4; ++i)
	{
		float d = (queryPt - mChildren[i]->mCenter).squaredNorm(); // Cheaper than regular norm
		if (d < minD)
		{
			minD = d;
			nearestIndex = i;
		}
	}

	return mChildren[nearestIndex]->search(queryPt, levelsRemaining, drillDownToDepth);
}

template <class T>
shared_ptr<Element<T> > Element<T>::nextLeaf(const int startChild)
{
	assert(startChild <= 4);
	if (!mHasChildren || startChild == 4)
	{
		if (shared_ptr<Element<T> >() != mParent)
		{
			return mParent->nextLeaf(mChildIndexOfParent + 1);
		}
		else
		{
			return shared_ptr<Element<T> >();
		}
	}
	else
	{
		if (!mChildren[startChild]->mHasChildren)
		{
			return mChildren[startChild];
		}
		else
		{
			return mChildren[startChild]->nextLeaf(0);
		}
	}
}

template <class T>
void Element<T>::printPath()
{
	std::cout << mChildIndexOfParent;
	if (shared_ptr<Element<T> >() != mParent)
	{
		std::cout << " <- ";
		mParent->printPath();
	}
	else
	{
		std::cout << std::endl;
	}
}


} // namespace Quadtree
} // namespace ap


#endif // QUADTREE_H
