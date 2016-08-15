//******************************************************************************
///
/// @file core/shape/nurbs.h
///
/// This module implements the header for the nurbs and rational_bezier_patch primitive.
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

#ifndef NURBS_H
#define NURBS_H

#include "backend/scene/objects.h"
#include "backend/math/vector.h"
#include "backend/math/matrices.h"


namespace pov
{

/*****************************************************************************
 * Global preprocessor defines
 ******************************************************************************/

#define RATIONAL_BEZIER_PATCH_OBJECT (PATCH_OBJECT)
#define NURBS_OBJECT (PATCH_OBJECT)


/*****************************************************************************
 * Global typedefs
 ******************************************************************************/

class RationalBezierPatch : public ObjectBase, public UVMeshable
{
private:
    typedef VECTOR Vector3d;
    typedef UV_VECT Vector2d;
    class Grid
    {
    public:
        Grid();
        Grid( const size_t x, const size_t y );
        ~Grid();
        void deCasteljauX( Grid& first, Grid& second, const DBL w, const DBL dw )const;
        void deCasteljauY( Grid& first, Grid& second, const DBL w, const DBL dw )const;
        void deCasteljauXF( Grid& first, const DBL w, const DBL dw )const;
        void deCasteljauYF( Grid& first, const DBL w, const DBL dw )const;
        void deCasteljauXB( Grid& second, const DBL w, const DBL dw )const;
        void deCasteljauYB( Grid& second, const DBL w, const DBL dw )const;
        bool getMinMaxX( DBL& min, DBL& max )const;
        bool getMinMaxY( DBL& min, DBL& max )const;
        DBL get( const size_t x, const size_t y )const;
        DBL set( const size_t x, const size_t y, const DBL& v );
        DBL getBottomLeft()const{return content.at(ysize-1).at(0);}
        DBL getBottomRight()const{return content.at(ysize-1).at(xsize-1);}
        DBL getTopLeft()const{return content.at(0).at(0);}
        DBL getTopRight()const{return content.at(0).at(xsize-1);}
        size_t dimX()const {return xsize;}
        size_t dimY()const {return ysize;}
        bool planePointsDistances( const Vector3d& normal, const DBL d, const Grid& x, const Grid& y, const Grid& z, const Grid& w );
        void linePointsDistances( const Vector2d& line, const Grid& a, const Grid& b );
        void point( const DBL u, const DBL v, bool vFirst, DBL & p0, DBL & p10, DBL & p11 )const;
        bool bounds(Vector2d  l)const;

    private:
        size_t xsize;
        size_t ysize;
        std::vector<std::vector<DBL> > content;
        void deCasteljauForward( const std::vector<DBL>& in, std::vector<DBL>& out, const DBL w, const DBL dw )const;
        void deCasteljauBackward( const std::vector<DBL>& in, std::vector<DBL>& out, const DBL w, const DBL dw )const;
        void checkIntersection( const DBL p, const DBL n, const DBL pw, const DBL nw, DBL& min, DBL & max )const;
        void reduceX(){xsize = 1;}
        void reduceY(){ysize = 1;}
    };
    class BasicRay
    {
    public:
      VECTOR Origin;
      VECTOR Direction;
    };
    void MInvTransRay( BasicRay& a, const Ray& ray, const TRANSFORM* tr)
    {
      DBL len;
      MInvTransPoint( a.Origin, ray.Origin , tr );
      MInvTransDirection( a.Direction, ray.Direction , tr );
      VLength( len, a.Direction );
      VInverseScaleEq( a.Direction, len );
    }

    Grid wc[3];/**< weigthed X coordinates */
    Grid weight;/**< original weights */
    DBL accuracy;/**< limit for solver */
    Vector3d minbox;
    Vector3d maxbox;
    /** for copy */
    RationalBezierPatch(): ObjectBase( RATIONAL_BEZIER_PATCH_OBJECT ) {}
    bool lineU(const Grid& a, const Grid& b, Vector2d& l)const;
    bool lineV(const Grid& a, const Grid& b, Vector2d& l)const;
    bool bounds(const Grid& a, const Grid&b, Vector2d la, Vector2d lb)const;
    bool addSolution(const DBL u, const DBL v, const BasicRay& ray, IStack& Depth_Stack, SceneThreadData* Thread);
    bool findSolution( Grid& a, Grid& b, Vector2d i0, Vector2d i1, Vector2d b0, Vector2d b1, const BasicRay& ray, IStack& Depth_Stack, SceneThreadData* Thread);
    void findPlanes( const BasicRay& ray, Vector3d& n0, DBL& d0, Vector3d& n1, DBL& d1)const;
public:
    RationalBezierPatch( const size_t x, const size_t y );
    void set( const size_t x, const size_t y, const VECTOR_4D& v );
    void setAccuracy( const DBL a ) {accuracy = a;}
    virtual void evalVertex( VECTOR r, const DBL u, const DBL v )const;
    virtual void evalNormal( VECTOR r, const DBL u, const DBL v )const;
    virtual void minUV( UV_VECT r )const;
    virtual void maxUV( UV_VECT r )const;
    virtual ~RationalBezierPatch();

    virtual ObjectPtr Copy();

    virtual bool All_Intersections( const Ray&, IStack&, TraceThreadData* );
    virtual bool Inside( const VECTOR, TraceThreadData* ) const;
    virtual void Normal( VECTOR, Intersection*, TraceThreadData* ) const;
    virtual void UVCoord( UV_VECT, const Intersection*, TraceThreadData* ) const;
    virtual void Translate( const VECTOR, const TRANSFORM* );
    virtual void Rotate( const VECTOR, const TRANSFORM* );
    virtual void Scale( const VECTOR, const TRANSFORM* );
    virtual void Transform( const TRANSFORM* );
    virtual void Invert( ){}
    virtual void Compute_BBox();
};

class Nurbs: public ObjectBase, public UVMeshable
{
private:
    class Point4D
    {
    private:
      DBL coordinate[4];
    public:
      Point4D(); 
      Point4D( const VECTOR_4D& v ); 
      Point4D operator=(const Point4D pt);
      Point4D operator+(const Point4D pt)const;
      Point4D operator*(const DBL m)const;
      Point4D operator/(const DBL m)const;
      void asVector(VECTOR res)const;
    };
    std::vector< std::vector< Point4D > > cp;
    std::vector< DBL > uknots;
    std::vector< DBL > vknots;
    size_t usize;
    size_t vsize;
    size_t uorder;
    size_t vorder;
    Point4D deBoor(int k, int order, int i, double x, const std::vector< DBL > & knots, const std::vector< Point4D > & ctrlPoints )const;
    int whichInterval( DBL x, size_t order, const std::vector< DBL > & knots)const;
public:
    virtual void evalVertex( VECTOR r, const DBL u, const DBL v )const;
    virtual void evalNormal( VECTOR r, const DBL u, const DBL v )const;
    virtual void minUV( UV_VECT r )const;
    virtual void maxUV( UV_VECT r )const;
    virtual ~Nurbs();
    Nurbs( const size_t x, const size_t y, const size_t uo, const size_t vo );
    void setControlPoint( const size_t x, const size_t y, const VECTOR_4D& v );
    void setUKnot( const size_t i, const DBL v );
    void setVKnot( const size_t i, const DBL v );
    /** for copy */
    Nurbs(): ObjectBase( NURBS_OBJECT ) {}


    virtual ObjectPtr Copy();

    virtual bool All_Intersections( const Ray&, IStack&, TraceThreadData* );
    virtual bool Inside( const VECTOR, TraceThreadData* ) const;
    virtual void Normal( VECTOR, Intersection*, TraceThreadData* ) const;
    virtual void UVCoord( UV_VECT, const Intersection*, TraceThreadData* ) const;
    virtual void Translate( const VECTOR, const TRANSFORM* );
    virtual void Rotate( const VECTOR, const TRANSFORM* );
    virtual void Scale( const VECTOR, const TRANSFORM* );
    virtual void Transform( const TRANSFORM* );
    virtual void Invert( ){}
    virtual void Compute_BBox();


};
}

#endif
