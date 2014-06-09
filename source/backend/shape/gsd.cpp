/*******************************************************************************
 * gsd.cpp
 *
 * This module implements routines for generalised symmetric difference.
 *
 * ---------------------------------------------------------------------------
 *******************************************************************************/

// frame.h must always be the first POV file included (pulls in platform config)
#include "backend/frame.h"
#include "backend/math/vector.h"
#include "backend/bounding/bbox.h"
#include "backend/shape/gsd.h"
#include "backend/math/matrices.h"
#include "backend/scene/objects.h"
#include "backend/shape/quadrics.h"
#include "backend/shape/hfield.h"
#include "backend/scene/threaddata.h"

#include "lightgrp.h" // TODO

#include <algorithm>

// this must be the last file included
#include "base/povdebug.h"

namespace pov
{

void GSDInterUnion::Translate(const VECTOR Vector, const TRANSFORM *tr)
{
	for(vector<ObjectPtr>::iterator Current_Sib = children.begin(); Current_Sib != children.end(); Current_Sib++)
		Translate_Object (*Current_Sib, Vector, tr) ;

	Recompute_BBox(&BBox, tr);
}


void GSDInterUnion::Rotate(const VECTOR Vector, const TRANSFORM *tr)
{
	for(vector<ObjectPtr>::iterator Current_Sib = children.begin(); Current_Sib != children.end(); Current_Sib++)
		Rotate_Object (*Current_Sib, Vector, tr) ;

	Recompute_BBox(&BBox, tr);
}



void GSDInterUnion::Transform(const TRANSFORM *tr)
{
	for(vector<ObjectPtr>::iterator Current_Sib = children.begin(); Current_Sib != children.end(); Current_Sib++)
		Transform_Object(*Current_Sib, tr);

	Recompute_BBox(&BBox, tr);
}

void GSDInterUnion::Scale(const VECTOR Vector, const TRANSFORM *tr)
{
	for(vector<ObjectPtr>::iterator Current_Sib = children.begin(); Current_Sib != children.end(); Current_Sib++)
		Scale_Object (*Current_Sib, Vector, tr) ;

	Recompute_BBox(&BBox, tr);
}

void GSDInterUnion::Invert()
{
  selected.flip();
}

GSDInterUnion::GSDInterUnion():CompoundObject(IS_COMPOUND_OBJECT)
{
}

GSDInterMerge::GSDInterMerge():GSDInterUnion()
{
}

ObjectPtr GSDInterUnion::Copy()
{
  GSDInterUnion *New = new GSDInterUnion();
  Destroy_Transform(New->Trans);
  *New = *this;

	New->children.clear();
	New->children.reserve(children.size());
	for(vector<ObjectPtr>::iterator i(children.begin()); i != children.end(); i++)
		New->children.push_back(Copy_Object(*i));

	return (New);
}
ObjectPtr GSDInterMerge::Copy()
{
  GSDInterMerge *New = new GSDInterMerge();
  Destroy_Transform(New->Trans);
  *New = *this;

	New->children.clear();
	New->children.reserve(children.size());
	for(vector<ObjectPtr>::iterator i(children.begin()); i != children.end(); i++)
		New->children.push_back(Copy_Object(*i));

	return (New);
}

void GSDInterUnion::Determine_Textures(Intersection *isect, bool hitinside, WeightedTextureVector& textures, ColourInterpolation& ci, TraceThreadData *threaddata)
{
	ci = CI_RGB; /* unless overriden by a deeper compound object */
	if(!children.empty())
	{
		size_t firstinserted = textures.size();

		for(vector<ObjectPtr>::const_iterator Current_Sib = children.begin(); Current_Sib != children.end(); Current_Sib++)
		{
			if((*Current_Sib)->Inside(isect->IPoint, threaddata))
			{
				if((*Current_Sib)->Type & IS_COMPOUND_OBJECT)
					(*Current_Sib)->Determine_Textures(isect, hitinside, textures, ci, threaddata); // [JG] Possible issue for conflicting ci
				else if((*Current_Sib)->Texture != NULL)
					textures.push_back(WeightedTexture(1.0, (*Current_Sib)->Texture));
			}
		}

		COLC weight = 1.0f / max(COLC(textures.size() - firstinserted), 1.0f);

		for(size_t i = firstinserted; i < textures.size(); i++)
			textures[i].weight = weight;
	}
}

bool GSDInterUnion::All_Intersections(const Ray& ray, IStack& Depth_Stack, TraceThreadData *Thread)
{
	size_t Count;
  bool Found;
	IStack Local_Stack(Thread->stackPool);
	assert(Local_Stack->empty()); // verify that the IStack pulled from the pool is in a cleaned-up condition

	Thread->Stats()[Ray_GSD_Interunion_Tests]++;

	Found = false;

	for(vector<ObjectPtr>::const_iterator Current_Sib = children.begin(); Current_Sib != children.end(); Current_Sib++)
	{
		if ((*Current_Sib)->Bound.empty() == true || Ray_In_Bound(ray, (*Current_Sib)->Bound, Thread))
		{
			if((*Current_Sib)->All_Intersections(ray, Local_Stack, Thread))
			{
				while(Local_Stack->size() > 0)
				{
					Count = 1;

					for(vector<ObjectPtr>::const_iterator Inside_Sib = children.begin(); Inside_Sib != children.end(); Inside_Sib++)
					{
						if(*Inside_Sib != *Current_Sib)
						{
							if(Inside_Object(Local_Stack->top().IPoint, *Inside_Sib, Thread))
							{
								++Count;
							}
						}
					}

					if(selected[Count])
					{
						if(Clip.empty() || Point_In_Clip(Local_Stack->top().IPoint, Clip, Thread))
						{
							Local_Stack->top().Csg = this;

							Depth_Stack->push(Local_Stack->top());

							Found = true;
						}
					}

					Local_Stack->pop();
				}
			}
		}
	}

	if(Found)
		Thread->Stats()[Ray_GSD_Interunion_Tests_Succeeded]++;

	assert(Local_Stack->empty()); // verify that the IStack is in a cleaned-up condition (again)
	return (Found);
}

bool GSDInterMerge::All_Intersections(const Ray& ray, IStack& Depth_Stack, TraceThreadData *Thread)
{
	size_t Count;
  bool Found;
	IStack Local_Stack(Thread->stackPool);
	assert(Local_Stack->empty()); // verify that the IStack pulled from the pool is in a cleaned-up condition

	Thread->Stats()[Ray_GSD_Interunion_Tests]++;

	Found = false;

	for(vector<ObjectPtr>::const_iterator Current_Sib = children.begin(); Current_Sib != children.end(); Current_Sib++)
	{
		if ((*Current_Sib)->Bound.empty() == true || Ray_In_Bound(ray, (*Current_Sib)->Bound, Thread))
		{
			if((*Current_Sib)->All_Intersections(ray, Local_Stack, Thread))
			{
				while(Local_Stack->size() > 0)
				{
					Count = 1;

					for(vector<ObjectPtr>::const_iterator Inside_Sib = children.begin(); Inside_Sib != children.end(); Inside_Sib++)
					{
						if(*Inside_Sib != *Current_Sib)
						{
							if(Inside_Object(Local_Stack->top().IPoint, *Inside_Sib, Thread))
							{
								++Count;
							}
						}
					}
					// merge: keep only the intersection on border
					if(
							(selected[Count])
							&& ( ( (Count>0) && !selected[Count-1])
								||( (Count<(selected.size()-1)) && !selected[Count+1])
								)
						)
					{
						if(Clip.empty() || Point_In_Clip(Local_Stack->top().IPoint, Clip, Thread))
						{
							Local_Stack->top().Csg = this;

							Depth_Stack->push(Local_Stack->top());

							Found = true;
						}
					}

					Local_Stack->pop();
				}
			}
		}
	}

	if(Found)
		Thread->Stats()[Ray_GSD_Interunion_Tests_Succeeded]++;

	assert(Local_Stack->empty()); // verify that the IStack is in a cleaned-up condition (again)
	return (Found);
}

bool GSDInterUnion::Inside(const VECTOR IPoint, TraceThreadData *Thread) const
{
  size_t Count=0;
	for(vector<ObjectPtr>::const_iterator Current_Sib = children.begin(); Current_Sib != children.end(); Current_Sib++)
		if(Inside_Object(IPoint, (*Current_Sib), Thread))
		{
			++Count;
		}
	return selected[Count];
}


void GSDInterUnion::Compute_BBox()
{
	DBL Old_Volume, New_Volume;
	VECTOR NewMin, NewMax, TmpMin, TmpMax, Min, Max;

	Make_Vector(NewMin,  BOUND_HUGE,  BOUND_HUGE,  BOUND_HUGE);
	Make_Vector(NewMax, -BOUND_HUGE, -BOUND_HUGE, -BOUND_HUGE);

	for(vector<ObjectPtr>::iterator Current_Sib = children.begin(); Current_Sib != children.end(); Current_Sib++)
	{
		Make_min_max_from_BBox(TmpMin, TmpMax, (*Current_Sib)->BBox);

		NewMin[X] = min(NewMin[X], TmpMin[X]);
		NewMin[Y] = min(NewMin[Y], TmpMin[Y]);
		NewMin[Z] = min(NewMin[Z], TmpMin[Z]);
		NewMax[X] = max(NewMax[X], TmpMax[X]);
		NewMax[Y] = max(NewMax[Y], TmpMax[Y]);
		NewMax[Z] = max(NewMax[Z], TmpMax[Z]);
	}

	if((NewMin[X] > NewMax[X]) || (NewMin[Y] > NewMax[Y]) || (NewMin[Z] > NewMax[Z]))
		;// TODO MESSAGE    Warning(0, "Degenerate CSG bounding box (not used!).");
	else
	{
		New_Volume = (NewMax[X] - NewMin[X]) * (NewMax[Y] - NewMin[Y]) * (NewMax[Z] - NewMin[Z]);

		BOUNDS_VOLUME(Old_Volume, BBox);

		if(New_Volume < Old_Volume)
		{
			Make_BBox_from_min_max(BBox, NewMin, NewMax);

			/* Beware of bounding boxes too large. */

			if((BBox.Lengths[X] > CRITICAL_LENGTH) ||
					(BBox.Lengths[Y] > CRITICAL_LENGTH) ||
					(BBox.Lengths[Z] > CRITICAL_LENGTH))
				Make_BBox(BBox, -BOUND_HUGE/2, -BOUND_HUGE/2, -BOUND_HUGE/2, BOUND_HUGE, BOUND_HUGE, BOUND_HUGE);
		}
	}
}

}
