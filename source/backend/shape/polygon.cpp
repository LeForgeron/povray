/*******************************************************************************
 * polygon.cpp
 *
 * This module implements functions that manipulate polygons.
 *
 * This module was written by Dieter Bayer [DB].
 *
 * ---------------------------------------------------------------------------
 * Persistence of Vision Ray Tracer ('POV-Ray') version 3.7.
 * Copyright 1991-2013 Persistence of Vision Raytracer Pty. Ltd.
 *
 * POV-Ray is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * POV-Ray is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Affero General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * ---------------------------------------------------------------------------
 * POV-Ray is based on the popular DKB raytracer version 2.12.
 * DKBTrace was originally written by David K. Buck.
 * DKBTrace Ver 2.0-2.12 were written by David K. Buck & Aaron A. Collins.
 * ---------------------------------------------------------------------------
 * $File: //depot/povray/smp/source/backend/shape/polygon.cpp $
 * $Revision: #32 $
 * $Change: 6085 $
 * $DateTime: 2013/11/10 07:39:29 $
 * $Author: clipka $
 *******************************************************************************/

/****************************************************************************
*
*  Explanation:
*
*  Syntax:
*
*  ---
*
*  The "inside polygon"-test was taken from:
*
*    E. Haines, "An Introduction to Ray-Tracing", ..., pp. 53-59
*
*  ---
*
*  May 1994 : Creation. [DB]
*
*  Oct 1994 : Changed polygon structure. Polygon points are now stored
*             in a seperate structure. This - together with the use of
*             a transformation - allows to keep just one point definition
*             for multiple copies of one polygon. [DB]
*
*****************************************************************************/

// frame.h must always be the first POV file included (pulls in platform config)
#include "backend/frame.h"
#include "backend/math/vector.h"
#include "backend/math/matrices.h"
#include "backend/scene/objects.h"
#include "backend/shape/polygon.h"
#include "backend/scene/threaddata.h"
#include "base/pov_err.h"
// Added to copy the texture list
#include "backend/texture/texture.h"

#include <algorithm>

// this must be the last file included
#include "base/povdebug.h"

namespace pov
{

/*****************************************************************************
* Local preprocessor defines
******************************************************************************/

/* Minimal intersection depth for a valid intersection. */
const DBL DEPTH_TOLERANCE = 1.0e-8;

/* If |x| < ZERO_TOLERANCE x is assumed to be 0. */
const DBL ZERO_TOLERANCE = 1.0e-10;



/*****************************************************************************
*
* FUNCTION
*
*   All_Polygon_Intersections
*
* INPUT
*
*   Object      - Object
*   Ray         - Ray
*   Depth_Stack - Intersection stack
*   
* OUTPUT
*
*   Depth_Stack
*   
* RETURNS
*
*   int - true, if a intersection was found
*   
* AUTHOR
*
*   Dieter Bayer
*   
* DESCRIPTION
*
*   Determine ray/polygon intersection and clip intersection found.
*
* CHANGES
*
*   May 1994 : Creation.
*
******************************************************************************/

bool Polygon::All_Intersections(const Ray& ray, IStack& Depth_Stack, TraceThreadData *Thread)
{
	DBL Depth;
	VECTOR IPoint;

	if (Intersect(ray, &Depth, Thread))
	{
		VEvaluateRay(IPoint, ray.Origin, Depth, ray.Direction);

		if (Clip.empty() || Point_In_Clip(IPoint, Clip, Thread))
		{
			Depth_Stack->push(Intersection(Depth, IPoint, this));

			return(true);
		}
	}

	return(false);
}



/*****************************************************************************
*
* FUNCTION
*
*   intersect_poylgon
*
* INPUT
*
*   Ray     - Ray
*   Polyg - Polygon
*   Depth   - Depth of intersection found
*   
* OUTPUT
*
*   Depth
*   
* RETURNS
*
*   int - true if intersection found
*   
* AUTHOR
*
*   Dieter Bayer
*   
* DESCRIPTION
*
*   Determine ray/polygon intersection.
*
* CHANGES
*
*   May 1994 : Creation.
*
******************************************************************************/

bool Polygon::Intersect(const Ray& ray, DBL *Depth, TraceThreadData *Thread) const
{
	DBL x, y, len;
	VECTOR p, d;

	/* Don't test degenerate polygons. */

	if (Test_Flag(this, DEGENERATE_FLAG))
		return(false);

	Thread->Stats()[Ray_Polygon_Tests]++;

	/* Transform the ray into the polygon space. */

	MInvTransPoint(p, ray.Origin, Trans);

	MInvTransDirection(d, ray.Direction, Trans);

	VLength(len, d);

	VInverseScaleEq(d, len);

	/* Intersect ray with the plane in which the polygon lies. */

	if (fabs(d[Z]) < ZERO_TOLERANCE)
		return(false);

	*Depth = -p[Z] / d[Z];

	if ((*Depth < DEPTH_TOLERANCE) || (*Depth > MAX_DISTANCE))
		return(false);

	/* Does the intersection point lie inside the polygon? */

	x = p[X] + *Depth * d[X];
	y = p[Y] + *Depth * d[Y];

	if (in_polygon(Data->Number, Data->Points, x, y))
	{
		Thread->Stats()[Ray_Polygon_Tests_Succeeded]++;

		*Depth /= len;

		return (true);
	}
	else
		return (false);
}



/*****************************************************************************
*
* FUNCTION
*
*   Inside_Polygon
*
* INPUT
*
*   IPoint - Intersection point
*   Object - Object
*   
* OUTPUT
*   
* RETURNS
*
*   int - always false
*   
* AUTHOR
*
*   Dieter Bayer
*   
* DESCRIPTION
*
*   Polygons can't be used in CSG.
*
* CHANGES
*
*   May 1994 : Creation.
*
******************************************************************************/

bool Polygon::Inside(const VECTOR, TraceThreadData *Thread) const
{
	return(false);
}



/*****************************************************************************
*
* FUNCTION
*
*   Polygon_Normal
*
* INPUT
*
*   Result - Normal vector
*   Object - Object
*   Inter  - Intersection found
*   
* OUTPUT
*
*   Result
*   
* RETURNS
*   
* AUTHOR
*
*   Dieter Bayer
*   
* DESCRIPTION
*
*   Calculate the normal of the polygon in a given point.
*
* CHANGES
*
*   May 1994 : Creation.
*
******************************************************************************/

void Polygon::Normal(VECTOR Result, Intersection *, TraceThreadData *) const
{
	Assign_Vector(Result, S_Normal);
}



/*****************************************************************************
*
* FUNCTION
*
*   Translate_Polygon
*
* INPUT
*
*   Object - Object
*   Vector - Translation vector
*   
* OUTPUT
*
*   Object
*   
* RETURNS
*   
* AUTHOR
*
*   Dieter Bayer
*   
* DESCRIPTION
*
*   Translate a polygon.
*
* CHANGES
*
*   May 1994 : Creation.
*
******************************************************************************/

void Polygon::Translate(const VECTOR, const TRANSFORM *tr)
{
	Transform(tr);
}



/*****************************************************************************
*
* FUNCTION
*
*   Rotate_Polygon
*
* INPUT
*
*   Object - Object
*   Vector - Rotation vector
*   
* OUTPUT
*
*   Object
*   
* RETURNS
*   
* AUTHOR
*
*   Dieter Bayer
*   
* DESCRIPTION
*
*   Rotate a polygon.
*
* CHANGES
*
*   May 1994 : Creation.
*
******************************************************************************/

void Polygon::Rotate(const VECTOR, const TRANSFORM *tr)
{
	Transform(tr);
}



/*****************************************************************************
*
* FUNCTION
*
*   Scale_Polygon
*
* INPUT
*
*   Object - Object
*   Vector - Scaling vector
*   
* OUTPUT
*
*   Object
*   
* RETURNS
*   
* AUTHOR
*
*   Dieter Bayer
*   
* DESCRIPTION
*
*   Scale a polygon.
*
* CHANGES
*
*   May 1994 : Creation.
*
******************************************************************************/

void Polygon::Scale(const VECTOR, const TRANSFORM *tr)
{
	Transform(tr);
}



/*****************************************************************************
*
* FUNCTION
*
*   Transform_Polygon
*
* INPUT
*
*   Object - Object
*   Trans  - Transformation to apply
*   
* OUTPUT
*
*   Object
*   
* RETURNS
*   
* AUTHOR
*
*   Dieter Bayer
*   
* DESCRIPTION
*
*   Transform a polygon by transforming all points
*   and recalculating the polygon.
*
* CHANGES
*
*   May 1994 : Creation.
*
******************************************************************************/

void Polygon::Transform(const TRANSFORM *tr)
{
	VECTOR N;

	if(Trans == NULL)
		Trans = Create_Transform();

	Compose_Transforms(Trans, tr);

	Make_Vector(N, 0.0, 0.0, 1.0);
	MTransNormal(S_Normal, N, Trans);

	VNormalizeEq(S_Normal);

	Compute_BBox();
}



/*****************************************************************************
*
* FUNCTION
*
*   Invert_Polygon
*
* INPUT
*
*   Object - Object
*   
* OUTPUT
*
*   Object
*   
* RETURNS
*   
* AUTHOR
*
*   Dieter Bayer
*   
* DESCRIPTION
*
*   Invert a polygon.
*
* CHANGES
*
*   May 1994 : Creation.
*
******************************************************************************/

void Polygon::Invert()
{
}



/*****************************************************************************
*
* FUNCTION
*
*   Create_Polygon
*
* INPUT
*   
* OUTPUT
*   
* RETURNS
*
*   POLYGON * - new polygon
*   
* AUTHOR
*
*   Dieter Bayer
*   
* DESCRIPTION
*
*   Create a new polygon.
*
* CHANGES
*
*   May 1994 : Creation.
*
******************************************************************************/

Polygon::Polygon() : ObjectBase(POLYGON_OBJECT)
{
	Trans = Create_Transform();

	Make_Vector(S_Normal, 0.0, 0.0, 1.0);

	Data = NULL;
	Textures = NULL;
	Strength = 1.0;
	Colour_Interpolation = CI_RGB;
}



/*****************************************************************************
*
* FUNCTION
*
*   Copy_Polygon
*
* INPUT
*
*   Object - Object
*   
* OUTPUT
*   
* RETURNS
*
*   void * - New polygon
*   
* AUTHOR
*
*   Dieter Bayer
*   
* DESCRIPTION
*
*   Copy a polygon structure.
*
* CHANGES
*
*   May 1994 : Creation.
*
******************************************************************************/

ObjectPtr Polygon::Copy()
{
	Polygon *New = new Polygon();

	Destroy_Transform(New->Trans);
	*New = *this;
	New->Trans = Copy_Transform(Trans);
	New->Data = Data;
	New->Data->References++;
	if (this->Textures)
	{
  // TODO: Share the Textures instead of copying
		New->Textures = (TEXTURE **)POV_MALLOC((New->Data->Number+1)*sizeof(TEXTURE*), "temporary polygon textures");
		for (int i=0;i<=New->Data->Number;i++)
		{
			if (this->Textures[i])
			{
				New->Textures[i] = Copy_Texture_Pointer((TEXTURE *)this->Textures[i]);
			}
			else
			{
				New->Textures[i] = NULL;
			}
		}
	}
	return (New);
}



/*****************************************************************************
*
* FUNCTION
*
*   Destroy_Polygon
*
* INPUT
*
*   Object - Object
*   
* OUTPUT
*
*   Object
*   
* RETURNS
*   
* AUTHOR
*
*   Dieter Bayer
*   
* DESCRIPTION
*
*   Destroy a polygon.
*
* CHANGES
*
*   May 1994 : Creation.
*
*   Dec 1994 : Fixed memory leakage. [DB]
*
******************************************************************************/

Polygon::~Polygon()
{
	if (--(Data->References) == 0)
	{
		POV_FREE (Data->Points);

		POV_FREE (Data);
	}

	if (Textures)
	{
		POV_FREE(Textures);
	}

	Destroy_Transform(Trans);
}



/*****************************************************************************
*
* FUNCTION
*
*   Compute_Polygon
*
* INPUT
*
*   Polyg - Polygon
*   Points  - 3D points describing the polygon
*   
* OUTPUT
*
*   Polyg
*   
* RETURNS
*   
* AUTHOR
*
*   Dieter Bayer
*   
* DESCRIPTION
*
*   Compute the following things for a given polygon:
*
*     - Polygon transformation
*
*     - Array of 2d points describing the shape of the polygon
*
* CHANGES
*
*   May 1994 : Creation.
*
******************************************************************************/

void Polygon::Compute_Polygon(int number, VECTOR *points)
{
	int i;
	DBL x, y, z, d;
	VECTOR o, u, v, w, N;
	MATRIX a, b;

	/* Create polygon data. */

	if (Data == NULL)
	{
		Data = reinterpret_cast<POLYGON_DATA *>(POV_MALLOC(sizeof(POLYGON_DATA), "polygon points"));

		Data->References = 1;

		Data->Number = number;

		Data->Points = reinterpret_cast<UV_VECT *>(POV_MALLOC(number*sizeof(UV_VECT), "polygon points"));
	}
	else
	{
		throw POV_EXCEPTION_STRING("Polygon data already computed.");
	}

	/* Get polygon's coordinate system (one of the many possible) */

	Assign_Vector(o, points[0]);

	/* Find valid, i.e. non-zero u vector. */

	for (i = 1; i < number; i++)
	{
		VSub(u, points[i], o);

		if (VSumSqr(u) > EPSILON)
		{
			break;
		}
	}

	if (i == number)
	{
		Set_Flag(this, DEGENERATE_FLAG);

;// TODO MESSAGE    Warning(0, "Points in polygon are co-linear. Ignoring polygon.");
	}

	/* Find valid, i.e. non-zero v and w vectors. */

	for (i++; i < number; i++)
	{
		VSub(v, points[i], o);

		VCross(w, u, v);

		if ((VSumSqr(v) > EPSILON) && (VSumSqr(w) > EPSILON))
		{
			break;
		}
	}

	if (i == number)
	{
		Set_Flag(this, DEGENERATE_FLAG);

;// TODO MESSAGE    Warning(0, "Points in polygon are co-linear. Ignoring polygon.");
	}

	VCross(u, v, w);
	VCross(v, w, u);

	VNormalize(u, u);
	VNormalize(v, v);
	VNormalize(w, w);

	MIdentity(a);
	MIdentity(b);

	a[3][0] = -o[X];
	a[3][1] = -o[Y];
	a[3][2] = -o[Z];

	b[0][0] =  u[X];
	b[1][0] =  u[Y];
	b[2][0] =  u[Z];

	b[0][1] =  v[X];
	b[1][1] =  v[Y];
	b[2][1] =  v[Z];

	b[0][2] =  w[X];
	b[1][2] =  w[Y];
	b[2][2] =  w[Z];

	MTimesC(Trans->inverse, a, b);

	MInvers(Trans->matrix, Trans->inverse);

	/* Project points onto the u,v-plane (3D --> 2D) */

	for (i = 0; i < number; i++)
	{
		x = points[i][X] - o[X];
		y = points[i][Y] - o[Y];
		z = points[i][Z] - o[Z];

		d = x * w[X] + y * w[Y] + z * w[Z];

		if (fabs(d) > ZERO_TOLERANCE)
		{
			Set_Flag(this, DEGENERATE_FLAG);

;// TODO MESSAGE      Warning(0, "Points in polygon are not co-planar. Ignoring polygons.");
		}

		Data->Points[i][X] = x * u[X] + y * u[Y] + z * u[Z];
		Data->Points[i][Y] = x * v[X] + y * v[Y] + z * v[Z];
	}

	Make_Vector(N, 0.0, 0.0, 1.0);
	MTransNormal(S_Normal, N, Trans);

	VNormalizeEq(S_Normal);

	Compute_BBox();
}



/*****************************************************************************
*
* FUNCTION
*
*   Compute_Polygon_BBox
*
* INPUT
*
*   Polyg - Polygon
*   
* OUTPUT
*
*   Polyg
*   
* RETURNS
*   
* AUTHOR
*
*   Dieter Bayer
*   
* DESCRIPTION
*
*   Calculate the bounding box of a polygon.
*
* CHANGES
*
*   May 1994 : Creation.
*
******************************************************************************/

void Polygon::Compute_BBox()
{
	int i;
	VECTOR p, Puv, Min, Max;

	Min[X] = Min[Y] = Min[Z] =  BOUND_HUGE;
	Max[X] = Max[Y] = Max[Z] = -BOUND_HUGE;

	for (i = 0; i < Data->Number; i++)
	{
		Puv[X] = Data->Points[i][X];
		Puv[Y] = Data->Points[i][Y];
		Puv[Z] = 0.0;

		MTransPoint(p, Puv, Trans);

		Min[X] = min(Min[X], p[X]);
		Min[Y] = min(Min[Y], p[Y]);
		Min[Z] = min(Min[Z], p[Z]);
		Max[X] = max(Max[X], p[X]);
		Max[Y] = max(Max[Y], p[Y]);
		Max[Z] = max(Max[Z], p[Z]);
	}

	Make_BBox_from_min_max(BBox, Min, Max);

	if (fabs(BBox.Lengths[X]) < SMALL_TOLERANCE)
	{
		BBox.Lower_Left[X] -= SMALL_TOLERANCE;
		BBox.Lengths[X]    += 2.0 * SMALL_TOLERANCE;
	}

	if (fabs(BBox.Lengths[Y]) < SMALL_TOLERANCE)
	{
		BBox.Lower_Left[Y] -= SMALL_TOLERANCE;
		BBox.Lengths[Y]    += 2.0 * SMALL_TOLERANCE;
	}

	if (fabs(BBox.Lengths[Z]) < SMALL_TOLERANCE)
	{
		BBox.Lower_Left[Z] -= SMALL_TOLERANCE;
		BBox.Lengths[Z]    += 2.0 * SMALL_TOLERANCE;
	}
}



/*****************************************************************************
*
* FUNCTION
*
*   in_polygon
*
* INPUT
*
*   Number - Number of points
*   Points - Points describing polygon's shape
*   u, v   - 2D-coordinates of the point to test
*   
* OUTPUT
*   
* RETURNS
*
*   int - true, if inside
*   
* AUTHOR
*
*   Eric Haines, 3D/Eye Inc, erich@eye.com
*   
* DESCRIPTION
*
* ======= Crossings Multiply algorithm ===================================
*
* This version is usually somewhat faster than the original published in
* Graphics Gems IV; by turning the division for testing the X axis crossing
* into a tricky multiplication test this part of the test became faster,
* which had the additional effect of making the test for "both to left or
* both to right" a bit slower for triangles than simply computing the
* intersection each time.  The main increase is in triangle testing speed,
* which was about 15% faster; all other polygon complexities were pretty much
* the same as before.  On machines where division is very expensive (not the
* case on the HP 9000 series on which I tested) this test should be much
* faster overall than the old code.  Your mileage may (in fact, will) vary,
* depending on the machine and the test data, but in general I believe this
* code is both shorter and faster.  This test was inspired by unpublished
* Graphics Gems submitted by Joseph Samosky and Mark Haigh-Hutchinson.
* Related work by Samosky is in:
*
* Samosky, Joseph, "SectionView: A system for interactively specifying and
* visualizing sections through three-dimensional medical image data",
* M.S. Thesis, Department of Electrical Engineering and Computer Science,
* Massachusetts Institute of Technology, 1993.
*
*
* Shoot a test ray along +X axis.  The strategy is to compare vertex Y values
* to the testing point's Y and quickly discard edges which are entirely to one
* side of the test ray.
*
* CHANGES
*
*   -
*
******************************************************************************/

bool Polygon::in_polygon(int number, UV_VECT *points, DBL u, DBL  v)
{
	register int i, yflag0, yflag1, inside_flag;
	register DBL ty, tx;
	register const DBL *vtx0, *vtx1, *first;

	tx = u;
	ty = v;

	vtx0 = &points[0][X];
	vtx1 = &points[1][X];

	first = vtx0;

	/* get test bit for above/below X axis */

	yflag0 = (vtx0[Y] >= ty);

	inside_flag = false;

	for (i = 1; i < number; )
	{
		yflag1 = (vtx1[Y] >= ty);

		/*
		 * Check if endpoints straddle (are on opposite sides) of X axis
		 * (i.e. the Y's differ); if so, +X ray could intersect this edge.
		 * The old test also checked whether the endpoints are both to the
		 * right or to the left of the test point.  However, given the faster
		 * intersection point computation used below, this test was found to
		 * be a break-even proposition for most polygons and a loser for
		 * triangles (where 50% or more of the edges which survive this test
		 * will cross quadrants and so have to have the X intersection computed
		 * anyway).  I credit Joseph Samosky with inspiring me to try dropping
		 * the "both left or both right" part of my code.
		 */

		if (yflag0 != yflag1)
		{
			/*
			 * Check intersection of pgon segment with +X ray.
			 * Note if >= point's X; if so, the ray hits it.
			 * The division operation is avoided for the ">=" test by checking
			 * the sign of the first vertex wrto the test point; idea inspired
			 * by Joseph Samosky's and Mark Haigh-Hutchinson's different
			 * polygon inclusion tests.
			 */

			if (((vtx1[Y]-ty) * (vtx0[X]-vtx1[X]) >= (vtx1[X]-tx) * (vtx0[Y]-vtx1[Y])) == yflag1)
			{
				inside_flag = !inside_flag;
			}
		}

		/* Move to the next pair of vertices, retaining info as possible. */

		if ((i < number-2) && (vtx1[X] == first[X]) && (vtx1[Y] == first[Y]))
		{
			vtx0 = &points[++i][X];
			vtx1 = &points[++i][X];

			yflag0 = (vtx0[Y] >= ty);

			first = vtx0;
		}
		else
		{
			vtx0 = vtx1;
			vtx1 = &points[++i][X];

			yflag0 = yflag1;
		}
	}

	return(inside_flag);
}


void Polygon::Determine_Textures(Intersection * isec, bool, WeightedTextureVector& wtv, ColourInterpolation& ci, TraceThreadData *)
{

	VECTOR P;
	DBL Zar,Yar,Xar,k;
	DBL sum,maxw,product,minw;
	int i;
	int Active=0;
	int nearer;
	int Count;

	ci = this->Colour_Interpolation;

	/* Transform the point into the blob space. */

	if (this->Trans != NULL)
	{
		MInvTransPoint(P, isec->IPoint, this->Trans);
	}
	else
	{
		Assign_Vector(P, isec->IPoint);
	}

	Count = this->Data->Number;
	DBL Weights[Count];
	/* Final weight (not normalised) is product of all squared distances
	 * to other points of the polygon.
	 * this->Texture[i] is null for duplicated point to ignore.
	 * Their distance is 1 in the first step.
	 * Then we reduce the max distance to 1
	 * (the scaling is not done for duplicated points)
	 *
	 * compute the product of all elements,
	 * replace the distance as product/distance 
	 * (for duplicated point, set to 0)
	 *
	 * Final stage is a classical normalisation
	 */
	maxw=0;
	minw=INFINITY;
	nearer=0;
	for(i=0;i<Count;i++)
	{
		if (this->Textures[i])
		{
			Xar=this->Data->Points[i][X]-P[X];
			Yar=this->Data->Points[i][Y]-P[Y];
			Zar=P[Z]; /* should be null, but distance is 3D */
			Weights[i]=Xar*Xar+Yar*Yar+Zar*Zar;
			if (Weights[i]>maxw)
			{
				maxw=Weights[i];
			} 
			if (Weights[i]<minw)
			{
				minw=Weights[i];
				nearer=i;
			}
		}
		else
		{
			Weights[i]=1.0;
		}
	}
	maxw=1.0/maxw; /* Divide once now, multiply many times later */
	k=1/exp(this->Strength);
	if (minw>0)
	{
		minw=0.0; /* reusing as max that time */
		for(i=0;i<Count;i++)
		{
			if (this->Textures[i])
			{
				Weights[i]*=maxw; /* reduce to less than 1. */
				/* now that W is less than 1, log(W)<0, so
				 * S*log(W) < 0, and therefor 0 < exp(S*log(W) <1
				 * which means no problem of overflow.
				 * (underflow are possible, but rounded to 0 should be fine)
				 *
				 * So here we compute W^S
				 */
				Weights[i]=exp(this->Strength * log(Weights[i]))*k;
				if (minw < Weights[i])
				{
					minw = Weights[i];
				}
			}
		}
		maxw = 1.0/minw;
		minw=INFINITY;
		for(i=0;i<Count;i++)
		{
			if (this->Textures[i])
			{
				Weights[i]*=maxw; /* reduce to less than 1. */
				if (minw > Weights[i])
				{
					minw = Weights[i];
				}
			}
		}
		product=1.0;
		for(i=0;i< Count;i++)
		{
			product*=Weights[i];
		}
	}
	else
	{
		product=0.0;
		minw=0.0;
	} 
	if ((product > 1e-275))
	{
		/* beware of underflow & overflow */
		for(i=0;i< Count;i++)
		{
			if (this->Textures[i])
			{
				Weights[i]=product/Weights[i];
			}
			else
			{
				Weights[i]=0;
			}
		}
	}
	else 
	{
		/* intersection is at a vertex of the polygon 
		 * so let's inverse the binary weights: 0 (distance) become 1
		 * non-null distance become 0.
		 *
		 * multiple textures are possible if the polygon has holes 
		 */
		k=(minw)*sqrt(Count);

		for(i=0;i< Count;i++)
		{
			if (this->Textures[i])
			{
				if (minw)
				{
					Weights[i]=(Weights[i]<=k)?(minw/Weights[i]):0.0;
				}
				else
				{
					Weights[i]=(Weights[i]<=k)?1.0:0.0;
				}
				if (Weights[i])
				{
					Active++;
				}
			}
			else
			{
				Weights[i]=0.0;
			}
		}
		if (!Active)
		{
			/* product was null per multiplication of very small values */
			Weights[nearer]=1.0;
		}
	} 


	/* Normalize weights so that their sum is 1. */
	/* A null sum is not possible, in theory */
	/* but an infinite value is (especially with big Strength) */
	sum = 0.0;
	maxw = 0.0;

	for (i = 0; i < Count; i++)
	{
		sum += Weights[i];
		if (maxw < Weights[i])
		{
			maxw = Weights[i];
		}
	}

	if ((sum == 0.0)&&(Active))
	{
		/* should not happen unless underflow due to very small Strength
		 * In such case, every texture get the same weigth which is 
		 * 1/Active
		 */
		for(i=0;i< Count;i++)
		{
			if (this->Textures[i])
			{
				wtv.push_back(WeightedTexture(1/Active,Textures[i]));
			}
		}
	}
	else if ((Active==0)&&(sum==0.0))
	{
		wtv.push_back(WeightedTexture(1.0,Textures[nearer]));
	} 
	else 
	{
		for(i=0;i< Count;i++)
		{
			if (Weights[i]&&Textures[i])
			{
				wtv.push_back(WeightedTexture(Weights[i]/sum,Textures[i]));
			}
		}
	}
}

}
