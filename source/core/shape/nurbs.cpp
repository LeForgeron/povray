//******************************************************************************
///
/// @file core/shape/nurbs.cpp
///
/// This module implements the nurbs primitive.
///
/// @author Jerome Grimbert
///
/// @copyright
/// @parblock
///
/// Persistence of Vision Ray Tracer ('POV-Ray') version 3.7.
/// Copyright 1991-2015 Persistence of Vision Raytracer Pty. Ltd.
///
/// POV-Ray is free software: you can redistribute it and/or modify
/// it under the terms of the GNU Affero General Public License as
/// published by the Free Software Foundation, either version 3 of the
/// License, or (at your option) any later version.
///
/// POV-Ray is distributed in the hope that it will be useful,
/// but WITHOUT ANY WARRANTY; without even the implied warranty of
/// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
/// GNU Affero General Public License for more details.
///
/// You should have received a copy of the GNU Affero General Public License
/// along with this program.  If not, see <http://www.gnu.org/licenses/>.
///
/// ----------------------------------------------------------------------------
///
/// POV-Ray is based on the popular DKB raytracer version 2.12.
/// DKBTrace was originally written by David K. Buck.
/// DKBTrace Ver 2.0-2.12 were written by David K. Buck & Aaron A. Collins.
///
/// @endparblock
///
//******************************************************************************

/****************************************************************************
*
*  Explanation:
*
*    -
*
*  Syntax:
*
*  nurbs
*  {
*     Xsize, Ysize [accuracy Avalue] 4DVector{Xsize*YSize}
*  }
*
*****************************************************************************/


// configcore.h must always be the first POV file included in core *.cpp files (pulls in platform config)
#include "core/configcore.h"
#include "core/shape/nurbs.h"

#include "core/render/ray.h"

#include "backend/math/matrices.h"
#include "backend/scene/objects.h"
#include "backend/scene/threaddata.h"
#include <algorithm>

// this must be the last file included
#include "base/povdebug.h"

namespace pov
{

/*****************************************************************************
* Local preprocessor defines
******************************************************************************/
const DBL VERY_SMALL_EPSILON = 1e-20;
/* yet another epsilon */
const DBL YAEP      = 1.0e-8;
#ifndef INFINITY
const DBL INFINITY  = 1.0e100;
#endif
/* following magical constant serves to detection of multiple intesections.
    according do NSK (Nishita, Sederberg,Kakimoto) set to 20% */
const DBL MAGICAL_CONSTANT  = ( 1.0 - 0.2 );




/*****************************************************************************
*
* FUNCTION
*
*   All_Nurbs_Intersections
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
*   int - true, if an intersection was found
*
* AUTHOR
*
*   Jerome Grimbert
*
* DESCRIPTION
*
*   Determine ray/nurbs intersection and clip intersection found.
*
* CHANGES
*
*   Jun 1994 : Creation.
*
******************************************************************************/

bool Nurbs::All_Intersections( const Ray& ray, IStack& Depth_Stack, SceneThreadData* Thread )
{
    Thread->Stats()[Ray_Nurbs_Tests]++;
    bool Found = false;
    BasicRay New_Ray;
    Vector2d interval[2];
    Vector2d bound[2];
    Vector3d n0, n1;
    DBL d0, d1;
    MInvTransRay( New_Ray, ray, Trans );
    New_Ray.Direction.normalize();
    findPlanes( New_Ray, n0, d0, n1, d1 );
    Grid a, b;

    if( false == a.planePointsDistances( n0, d0, wc[X], wc[Y], wc[Z], weight ) )
    {
        return Found;
    }

    if( false == b.planePointsDistances( n1, d1, wc[X], wc[Y], wc[Z], weight ) )
    {
        return Found;
    }

    interval[0][0] = interval[1][0] = 0.0;

    if( weight.dimX() > weight.dimY() )
    {
        /* small trick to ensure that we first split nurbs in smaller order */
        interval[0][1] = 1.0 + YAEP;
        interval[1][1] = 1.0;
    }
    else
    {
        interval[0][1] = 1.0;
        interval[1][1] = 1.0 + YAEP;
    }

    bound[1][1] = bound[0][1] =  INFINITY;
    bound[1][0] = bound[0][0] = -INFINITY;
    Found = findSolution( a, b, interval[0], interval[1], bound[0], bound[1], New_Ray, Depth_Stack, Thread );

    if( Found )
    {
        Thread->Stats()[Ray_Nurbs_Tests_Succeeded]++;
    }

    return ( Found );
}



/*****************************************************************************
*
* FUNCTION
*
*   Inside_Nurbs
*
* INPUT
*
*   IPoint - Intersection point
*
* OUTPUT
*
* RETURNS
*
*   int - true if inside
*
* AUTHOR
*
*   Jerome Grimbert
*
* DESCRIPTION
*
*   A nurbs is not a solid, so an inside test doesn't make sense.
*
* CHANGES
*
*
******************************************************************************/

bool Nurbs::Inside( const Vector3d& IPoint, TraceThreadData* Thread ) const
{
    return false;
}



/*****************************************************************************
*
* FUNCTION
*
*   Nurbs_Normal
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
*   Jerome Grimbert
*
* DESCRIPTION
*
*   Calculate the normal of the nurbs for a given point.
*
* CHANGES
*
******************************************************************************/

void Nurbs::Normal( Vector3d& Result, Intersection* Inter, TraceThreadData* Thread ) const
{
    /* Use preocmputed normal. */
    Result = Inter->INormal;
}



/*****************************************************************************
*
* FUNCTION
*
*   Translate_Nurbs
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
*   Jerome Grimbert
*
* DESCRIPTION
*
*   Translate an ovus.
*
* CHANGES
*
*
******************************************************************************/

void Nurbs::Translate( const Vector3d&, const TRANSFORM* tr )
{
    Transform( tr );
}



/*****************************************************************************
*
* FUNCTION
*
*   Rotate_Nurbs
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
*   Jerome Grimbert
*
* DESCRIPTION
*
*   Rotate an ovus.
*
* CHANGES
*
*
******************************************************************************/

void Nurbs::Rotate( const Vector3d&, const TRANSFORM* tr )
{
    Transform( tr );
}



/*****************************************************************************
*
* FUNCTION
*
*   Scale_Nurbs
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
*   Jerome Grimbert
*
* DESCRIPTION
*
*   Scale an ovus.
*
* CHANGES
*
*
******************************************************************************/

void Nurbs::Scale( const Vector3d&, const TRANSFORM* tr )
{
    Transform( tr );
}



/*****************************************************************************
*
* FUNCTION
*
*   Transform_Nurbs
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
*   Jerome Grimbert
*
* DESCRIPTION
*
*   Transform an ovus and recalculate its bounding box.
*
* CHANGES
*
*
******************************************************************************/

void Nurbs::Transform( const TRANSFORM* tr )
{
    if( Trans == NULL )
    { Trans = Create_Transform(); }

    Compose_Transforms( Trans, tr );
    Compute_BBox();
}



/*****************************************************************************
*
* FUNCTION
*
*   Create_Nurbs
*
* INPUT
*
* OUTPUT
*
* RETURNS
*
*   Nurbs * - new nurbs
*
* AUTHOR
*
*   Jerome Grimbert
*
* DESCRIPTION
*
*   Create a new nurbs.
*
* CHANGES
*
*
******************************************************************************/

Nurbs::Nurbs( const size_t x, const size_t y ): NonsolidObject( NURBS_OBJECT ), weight( x, y ), accuracy( 0.01 )
{
    Trans = Create_Transform();
    wc[X] = Grid( x, y );
    wc[Y] = Grid( x, y );
    wc[Z] = Grid( x, y );
    minbox = Vector3d( BOUND_HUGE, BOUND_HUGE, BOUND_HUGE );
    maxbox = Vector3d( -BOUND_HUGE, -BOUND_HUGE, -BOUND_HUGE );
}



/*****************************************************************************
*
* FUNCTION
*
*   Copy_Nurbs
*
* INPUT
*
*   Object - Object
*
* OUTPUT
*
* RETURNS
*
*   void * - New nurbs
*
* AUTHOR
*
*   Jerome Grimbert
*
* DESCRIPTION
*
*   Copy a nurbs.
*
* CHANGES
*
*
******************************************************************************/

ObjectPtr Nurbs::Copy()
{
    Nurbs* New = new Nurbs();
    Destroy_Transform( New->Trans );
    *New = *this;
    New->Trans = Copy_Transform( Trans );
    return ( New );
}



/*****************************************************************************
*
* FUNCTION
*
*   Destroy_Nurbs
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
*   Jerome Grimbert
*
* DESCRIPTION
*
*   Destroy a ovus.
*
* CHANGES
*
*
******************************************************************************/

Nurbs::~Nurbs()
{
    Destroy_Transform( Trans );
}



/*****************************************************************************
*
* FUNCTION
*
*   Compute_Nurbs_BBox
*
* INPUT
*
*   Nurbs - nurbs
*
* OUTPUT
*
*   Nurbs
*
* RETURNS
*
* AUTHOR
*
*   Jerome Grimbert
*
* DESCRIPTION
*
*   Calculate the bounding box of a nurbs.
*
* CHANGES
*
*
******************************************************************************/

void Nurbs::Compute_BBox()
{
    Make_BBox_from_min_max( BBox, minbox, maxbox );
    Recompute_BBox( &BBox, Trans );
}



/*****************************************************************************
*
* FUNCTION
*
*   Nurbs_UVCoord
*
* INPUT
*
* OUTPUT
*
* RETURNS
*
* AUTHOR
*
*   Jerome Grimbert
*
* DESCRIPTION
*
*   -
*
* CHANGES
*
*   -
*
******************************************************************************/

void Nurbs::UVCoord( Vector2d& Result, const Intersection* Inter, TraceThreadData* Thread ) const
{
    /* Use preocmputed uv coordinates. */
    Result = Inter->Iuv;
}

/************************** The hard stuff starts here *********************/

Nurbs::Grid::Grid( const size_t x, const size_t y ): xsize( x ), ysize( y )
{
    content.resize( y );

    for( size_t i = 0; i < y; ++i )
    {
        content.at( i ).resize( x );
    }
}

Nurbs::Grid::~Grid()
{
}

Nurbs::Grid::Grid(): xsize( 0 ), ysize( 0 )
{
}

DBL Nurbs::Grid::get( const size_t x, const size_t y )const
{
    return content.at( y ).at( x );
}

DBL Nurbs::Grid::set( const size_t x, const size_t y, const DBL& v )
{
    content.at( y ).at( x ) = v;
    return v;
}

void Nurbs::Grid::deCasteljauX( Grid& first, Grid& second, const DBL w, const DBL dw )const
{
    first = Grid( xsize, ysize );
    second = Grid( xsize, ysize );
    std::vector<DBL> input;
    input.resize( xsize );

    for( size_t c = 0; c < ysize; ++c )
    {
        for( size_t i = 0; i < xsize; ++i )
        {
            input[i] = content[c][i];
        }

        deCasteljauForward( input, first.content[c], w, dw );
        deCasteljauBackward( input, second.content[c], w, dw );
    }
}

void Nurbs::Grid::deCasteljauY( Grid& first, Grid& second, const DBL w, const DBL dw )const
{
    first = Grid( xsize, ysize );
    second = Grid( xsize, ysize );
    std::vector<DBL> input;
    std::vector<DBL> output1;
    std::vector<DBL> output2;
    input.resize( ysize );
    output1.resize( ysize );
    output2.resize( ysize );

    for( size_t c = 0; c < xsize; ++c )
    {
        for( size_t i = 0; i < ysize; ++i )
        {
            input[i] = content[i][c];
        }

        deCasteljauForward( input, output1, w, dw );
        deCasteljauBackward( input, output2, w, dw );

        for( size_t i = 0; i < ysize; ++i )
        {
            first.content[i][c] = output1[i];
            second.content[i][c] = output2[i];
        }
    }
}

void Nurbs::Grid::deCasteljauForward( const std::vector<DBL>& in, std::vector<DBL>& out, const DBL w, const DBL dw )const
{
    out = in;
    std::vector<DBL> work;

    for( size_t k = 1; k < out.size(); ++k )
    {
        work = out;

        for( size_t i = k; i < out.size(); ++i )
        {
            out[i] = dw * work[i - 1] + w * work[i];
        }
    }
}

void Nurbs::Grid::deCasteljauBackward( const std::vector<DBL>& in, std::vector<DBL>& out, const DBL w, const DBL dw )const
{
    std::vector<DBL> work = in;
    std::reverse( work.begin(), work.end() );
    deCasteljauForward( work, out, dw, w );
    std::reverse( out.begin(), out.end() );
}

void Nurbs::Grid::checkIntersection( const DBL p, const DBL n, const DBL pw, const DBL nw, DBL& min, DBL& max )const
{
    if( ( p <= 0 ) ^ ( n <= 0 ) )
    {
        DBL tmp = pw + ( nw - pw ) * p / ( p - n );
        min = std::min( min, tmp );
        max = std::max( max, tmp );
    }
}

bool Nurbs::Grid::getMinMaxX( DBL& min, DBL& max )const
{
    min = 2.0;
    max = -2.0;
    size_t first, last;
    first = last = 0;
    const DBL ratio = 1.0 / ( xsize - 1 );

    for( size_t i = 0; i < ysize; ++i )
    {
        if( content[i][0] < 0.0 )
        {
            ++first;
        }

        if( content[i][xsize - 1] < 0.0 )
        {
            ++last;
        }

        for( size_t j = 1; j < xsize; ++j )
        {
            for( size_t k = 0; k < j; ++k )
            {
                checkIntersection( get( k, i ), get( j, i ), k * ratio, j * ratio, min, max );
            }
        }
    }

    if( ( first ) && ( first != ysize ) )
    {
        min = 0.0;
    }

    if( ( last ) && ( last != ysize ) )
    {
        max = 1.0;
    }

    return ( min <= max );
}
bool Nurbs::Grid::getMinMaxY( DBL& min, DBL& max )const
{
    min = 2.0;
    max = -2.0;
    size_t first, last;
    first = last = 0;
    const DBL ratio = 1.0 / ( ysize - 1 );

    for( size_t i = 0; i < xsize; ++i )
    {
        if( content[0][i] < 0.0 )
        {
            ++first;
        }

        if( content[ysize - 1][i] < 0.0 )
        {
            ++last;
        }

        for( size_t j = 1; j < ysize; ++j )
        {
            for( size_t k = 0; k < j; ++k )
            {
                checkIntersection( get( i, k ), get( i, j ),  k * ratio, j * ratio, min, max );
            }
        }
    }

    if( ( first ) && ( first != xsize ) )
    {
        min = 0.0;
    }

    if( ( last ) && ( last != xsize ) )
    {
        max = 1.0;
    }

    return ( min <= max );
}

bool Nurbs::Grid::planePointsDistances( const Vector3d& normal, const DBL d, const Grid& x, const Grid& y, const Grid& z, const Grid& w )
{
    *this = Grid( w.xsize, w.ysize );
    DBL min, max, v;
    min = INFINITY;
    max = -INFINITY;

    for( size_t i = 0; i < ysize; ++i )
    {
        for( size_t j = 0; j < xsize; ++j )
        {
            v = set( j, i, x.get( j, i ) * normal[X] + y.get( j, i ) * normal[Y] + z.get( j, i ) * normal[Z] + w.get( j, i ) * d );
            min = std::min( min, v );
            max = std::max( max, v );
        }
    }

    if( ( min > 0.0 ) || ( max < 0.0 ) )
    {
        return false;
    }
    else
    {
        return true;
    }
}
void Nurbs::Grid::linePointsDistances( const Vector2d& line, const Grid& a, const Grid& b )
{
    *this = Grid( a.xsize, a.ysize );

    for( size_t i = 0; i < ysize; ++i )
    {
        for( size_t j = 0; j < xsize; ++j )
        {
            set( j, i, a.get( j, i ) * line[0] + b.get( j, i ) * line[1] );
        }
    }
}
void Nurbs::set( const size_t x, const size_t y, const VECTOR_4D& v )
{
    weight.set( x, y, v[W] );
    wc[X].set( x, y, v[W]*v[X] );
    wc[Y].set( x, y, v[W]*v[Y] );
    wc[Z].set( x, y, v[W]*v[Z] );
    minbox[X] = std::min( minbox[X], v[X] );
    minbox[Y] = std::min( minbox[Y], v[Y] );
    minbox[Z] = std::min( minbox[Z], v[Z] );
    maxbox[X] = std::max( maxbox[X], v[X] );
    maxbox[Y] = std::max( maxbox[Y], v[Y] );
    maxbox[Z] = std::max( maxbox[Z], v[Z] );
}

void Nurbs::Grid::point( const DBL u, const DBL v, bool vFirst, DBL& p0, DBL& p10, DBL& p11 )const
{
    Grid d1, d2, d3, d4;
    // relation is p0 = a.p10 + b.p11 (with b = 1.0 - a)
    // p10 and p11 are needed for tangent/normal computation

    if( vFirst )
    {
        deCasteljauY( d1, d2, v, 1.0 - v );
        d2.reduceY();
        d2.deCasteljauX( d3, d4, u, 1.0 - u );
        p0 = d4.get( 0, 0 );// the final de Casteljau value
        p10 = d4.get( 1, 0 );
        p11 = d3.get( xsize - 2, 0 );
    }
    else
    {
        deCasteljauX( d1, d2, u, 1.0 - u );
        d2.reduceX();
        d2.deCasteljauY( d3, d4, v, 1.0 - v );
        p0 = d4.get( 0, 0 );// the final de Casteljau value
        p10 = d4.get( 0, 1 );
        p11 = d3.get( 0, ysize - 2 );
    }
}

bool Nurbs::lineU( const Grid& a, const Grid& b, Vector2d& l )const
{
    DBL tmp;
    l[0] =    b.getBottomLeft() - b.getTopLeft() + b.getBottomRight() - b.getTopRight();
    l[1] = -( a.getBottomLeft() - a.getTopLeft() + a.getBottomRight() - a.getTopRight() );
    tmp = l[0] * l[0] + l[1] * l[1];

    if( tmp < VERY_SMALL_EPSILON )
    {
        return false;
    }

    tmp = sqrt( tmp );
    l[0] /= tmp;
    l[1] /= tmp;
    return true;
}

bool Nurbs::lineV( const Grid& a, const Grid& b, Vector2d& l )const
{
    DBL tmp;
    l[0] =    b.getTopRight() - b.getTopLeft() + b.getBottomRight() - b.getBottomLeft();
    l[1] = -( a.getTopRight() - a.getTopLeft() + a.getBottomRight() - a.getBottomLeft() );
    tmp = l[0] * l[0] + l[1] * l[1];

    if( tmp < VERY_SMALL_EPSILON )
    {
        return false;
    }

    tmp = sqrt( tmp );
    l[0] /= tmp;
    l[1] /= tmp;
    return true;
}

bool Nurbs::bounds( const Grid& a, const Grid& b, Vector2d& la, Vector2d& lb )const
{
    bool ret = false;
    ret |= a.bounds( la );
    ret |= b.bounds( lb );
    return !ret;
}

bool Nurbs::Grid::bounds( Vector2d& l )const
{
    bool ret = false;
    DBL& min = l[0];
    DBL& max = l[1];
    DBL v = content[0][0];
    min = max = v;

    for( size_t y = 0; y < ysize; ++y )
    {
        for( size_t x = 0; x < xsize; ++x )
        {
            v = content[y][x];
            min = std::min( min, v );
            max = std::max( max, v );
        }
    }

    ret |= ( min > EPSILON ) || ( max < -EPSILON );
    return ret;
}
bool Nurbs::addSolution( const DBL u, const DBL v,  const BasicRay& ray, IStack& depthStack, SceneThreadData* Thread )
{
    DBL c1, c2, c3, w1, w2, w3, n1, n2, n3;
    DBL d;
    int i, coord;
    bool ret = false;
    Vector3d IPoint, Real_Pt, v1, v2, Normal_Vector, Real_Normal;
    Vector2d UV;

    if( ( u < 0.0 ) || ( u > 1.0 ) || ( v < 0.0 ) || ( v > 1.0 ) )
    {
        return 0;
    }

    // select the grid to use to compute IPoint
    if( fabs( ray.Direction[X] ) > 0.5 )
    {
        coord = X;
    }
    else if( fabs( ray.Direction[Y] ) > 0.5 )
    {
        coord = Y;
    }
    else
    {
        coord = Z;
    }

    const Grid& cp = wc[coord];
    bool vFirst = weight.dimX() > weight.dimY();
    cp.point( u, v, vFirst, c1, c2, c3 );
    weight.point( u, v, vFirst, w1, w2, w3 );
    c1 /= w1;
    c2 = ( c2 / w2 - c3 / w3 );
    d = ( c1 - ray.Origin[coord] ) / ( ray.Direction[coord] );

    if( d < 4.0 * accuracy )
    {
        return false;
    }

    IPoint = ray.Origin + d * ray.Direction;
    MTransPoint( Real_Pt, IPoint, Trans );

    if( Clip.empty() || ( Point_In_Clip( Real_Pt, Clip, Thread ) ) )
    {
        // compute the normal
        for( i = 0; i <= Z; ++i )
        {
            if( i == coord )   /* already computed  */
            {
                v1[i] = c2;
            }
            else
            {
                wc[i].point( u, v, vFirst, n1, n2, n3 );
                v1[i] = ( n2 / w2 - n3 / w3 );
            }
        }

        vFirst = !vFirst;

        for( i = 0; i <= Z; ++i )
        {
            wc[i].point( u, v, vFirst, n1, n2, n3 );
            v2[i] = ( n2 / w2 - n3 / w3 );
        }

        Normal_Vector = cross( v1, v2 );
        Normal_Vector.normalize();
        MTransNormal( Real_Normal, Normal_Vector, Trans );
        Real_Normal.normalize();
        UV[U] = u;
        UV[V] = v;
        depthStack->push( Intersection( d, Real_Pt, Real_Normal, UV, this ) );
        ret = true;
    }

    return ret;
}

void Nurbs::findPlanes( const BasicRay& ray, Vector3d& n1, DBL& d1, Vector3d& n2, DBL& d2 )const
{
    // precondition: ray.Direction is normalized
    //
    if( fabs( ray.Direction[X] ) < 0.5 )
    {
        n1[X] =     0.0;
        n1[Y] =  ray.Direction[Z];
        n1[Z] = -ray.Direction[Y];
    }
    else
    {
        n1[X] = -ray.Direction[Z];
        n1[Y] =     0.0;
        n1[Z] =  ray.Direction[X];
    }

    n1.normalize();
    n2 = cross( ray.Direction, n1 );
    d1 = -dot( ray.Origin, n1 );
    d2 = -dot( ray.Origin, n2 );
}

bool Nurbs::findSolution( Grid& a, Grid& b, Vector2d i0, Vector2d i1, Vector2d b0, Vector2d b1, const BasicRay& ray, IStack& Depth_Stack, SceneThreadData* Thread )
{
    int iii;
    Vector2d  Line;
    DBL  tmp;
    bool  in_half = false;
    Grid d;
    DBL  min, max;

    if( ( ( b1[1] - b1[0] ) < accuracy )
        && ( ( b0[1] - b0[0] ) < accuracy ) )
    {
        /* end of iteration ...*/
        return addSolution( i0[0] + i0[1] / 2.0, i1[0] + i1[1] / 2.0, ray, Depth_Stack, Thread );
    }

    if( i0[1] > i1[1] )
    {
        iii = 0;

        if( lineU( a, b, Line ) == false )
        {
            if( lineV( a, b, Line ) == false )
            { in_half = true; } /* degenerated patch -=> split in half */
            else
            {
                tmp = -Line[0];
                Line[0] = Line[1];
                Line[1] = tmp;
            }
        }
    }
    else
    {
        iii = 1;

        if( lineV( a, b, Line ) == false )
        {
            if( lineU( a, b, Line ) == false )
            { in_half = true; } /* degenerated patch -=> split in half */
            else
            {
                tmp = -Line[0];
                Line[0] = Line[1];
                Line[1] = tmp;
            }
        }
    }

    if( false == in_half )
    {
        d.linePointsDistances( Line, a, b );

        if( 0 == iii )
        {
            if( false == d.getMinMaxX( min, max ) )
            {
                return false;
            }
        }
        else
        {
            if( false == d.getMinMaxY( min, max ) )
            {
                return false;
            }
        }

        tmp = max - min ;

        if( tmp > MAGICAL_CONSTANT )
        { in_half = true; }
        else
        {
            {
                Grid ptmp10, ptmp11, ptmp20, ptmp21;

                if( min < 0.5 )
                {
                    if( 0 == iii )
                    {
                        a.deCasteljauX( ptmp10 , ptmp20, min, 1.0 - min );
                        b.deCasteljauX( ptmp11 , ptmp21, min, 1.0 - min );
                        ptmp20.deCasteljauX( a, ptmp10, ( max - min ) / ( 1.0 - min ), ( 1.0 - max ) / ( 1.0 - min ) );
                        ptmp21.deCasteljauX( b, ptmp11, ( max - min ) / ( 1.0 - min ), ( 1.0 - max ) / ( 1.0 - min ) );
                    }
                    else
                    {
                        a.deCasteljauY( ptmp10, ptmp20, min, 1.0 - min );
                        b.deCasteljauY( ptmp11, ptmp21, min, 1.0 - min );
                        ptmp20.deCasteljauY( a, ptmp10, ( max - min ) / ( 1.0 - min ), ( 1.0 - max ) / ( 1.0 - min ) );
                        ptmp21.deCasteljauY( b, ptmp11, ( max - min ) / ( 1.0 - min ), ( 1.0 - max ) / ( 1.0 - min ) );
                    }
                }
                else
                {
                    if( 0 == iii )
                    {
                        a.deCasteljauX( ptmp10, ptmp20, max, 1.0 - max );
                        b.deCasteljauX( ptmp11, ptmp21, max, 1.0 - max );
                        ptmp10.deCasteljauX( ptmp20, a, min / max, 1.0 - min / max );
                        ptmp11.deCasteljauX( ptmp21, b, min / max, 1.0 - min / max );
                    }
                    else
                    {
                        a.deCasteljauY( ptmp10, ptmp20, max, 1.0 - max );
                        b.deCasteljauY( ptmp11, ptmp21, max, 1.0 - max );
                        ptmp10.deCasteljauY( ptmp20, a, min / max, 1.0 - min / max );
                        ptmp11.deCasteljauY( ptmp21, b, min / max, 1.0 - min / max );
                    }
                }
            }

            if( false == bounds( a, b, b0, b1 ) )
            {
                return 0;
            }

            if( 0 == iii )
            {
                i0[0] += i0[1] * min;
                i0[1] = i0[1] * tmp;
            }
            else
            {
                i1[0] += i1[1] * min;
                i1[1] = i1[1] * tmp;
            }

            return findSolution( a, b, i0, i1, b0, b1, ray, Depth_Stack, Thread );
        }
    }

    /*  (in_half == 1) */
    {
        Grid ld0, ld1, rd0, rd1;
        Vector2d ni0, ni1;
        bool ret = false;

        if( 0 == iii )
        {
            a.deCasteljauX( ld0, rd0, 0.5, 0.5 );
            b.deCasteljauX( ld1, rd1, 0.5, 0.5 );
            tmp = i0[1] /= 2.0;
        }
        else
        {
            a.deCasteljauY( ld0, rd0, 0.5, 0.5 );
            b.deCasteljauY( ld1, rd1, 0.5, 0.5 );
            tmp = i1[1] /= 2.0;
        }

        ni0 = i0;
        ni1 = i1;

        if( 0 == iii )
        {
            ni0[0] += tmp;
        }
        else
        {
            ni1[0] += tmp;
        }

        if( false != bounds( ld0, ld1, b0, b1 ) )
        { ret |= findSolution( ld0, ld1, i0, i1, b0, b1, ray, Depth_Stack, Thread ); }

        if( false != bounds( rd0, rd1, b0, b1 ) )
        { ret |= findSolution( rd0, rd1, ni0, ni1, b0, b1, ray, Depth_Stack, Thread ); }

        return ret;
    }
}

}