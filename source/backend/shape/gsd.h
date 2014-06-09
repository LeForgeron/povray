/*******************************************************************************
 * gsd.h
 *
 * This module contains all defines, typedefs, and prototypes for GSD.CPP.
 * Generalised Symmetric Difference
 *
 *******************************************************************************/

#ifndef GSD_H
#define GSD_H

namespace pov
{

/* GSD types */

enum GSD_TYPE{
GSD_INTERUNION_TYPE ,
GSD_INTERMERGE_TYPE
};

/*****************************************************************************
* Global typedefs
******************************************************************************/

class GSDInterUnion : public CompoundObject
{
	public:
		GSDInterUnion();
    vector<bool> selected;

		virtual void Normal(VECTOR, Intersection *, TraceThreadData *) const { }
		virtual void Translate(const VECTOR, const TRANSFORM *);
		virtual void Rotate(const VECTOR, const TRANSFORM *);
		virtual void Scale(const VECTOR, const TRANSFORM *);
		virtual void Transform(const TRANSFORM *);
		virtual void Compute_BBox();
		virtual void Invert();
		virtual ObjectPtr Copy();
		virtual bool All_Intersections(const Ray&, IStack&, TraceThreadData *);
		virtual bool Inside(const VECTOR, TraceThreadData *) const;

		void Determine_Textures(Intersection *isect, bool hitinside, WeightedTextureVector& textures, ColourInterpolation& ci, TraceThreadData *Threaddata);
};

class GSDInterMerge : public GSDInterUnion
{
	public:
		GSDInterMerge();
		virtual ObjectPtr Copy();

		virtual bool All_Intersections(const Ray&, IStack&, TraceThreadData *);
};

}

#endif
