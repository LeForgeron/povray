

#include <vector>

#include <boost/thread.hpp>
#include <boost/scoped_array.hpp>

// frame.h must always be the first POV file included (pulls in platform config)
#include "backend/frame.h"
#include "backend/colour/colour_old.h"
#include "backend/math/vector.h"
#include "backend/math/chi2.h"
#include "backend/math/matrices.h"
#include "backend/render/trace.h"
#include "backend/render/tracepixel.h"
#include "backend/support/jitter.h"
#include "backend/texture/normal.h"
#include "backend/texture/pigment.h"

// this must be the last file included
#include "base/povdebug.h"

namespace pov
{
#define SQRT2       1.4142135623730950488016887242097   ///< sqrt(2)
#define SQRT2_2     0.70710678118654752440084436210485  ///< sqrt(2)/2
#define SQRT3_2     0.86602540378443864676372317075294 ///< sqrt(3)/2
	bool TracePixel::ProjectionPlateCarreeCameraRay(Ray &ray, DBL x, DBL y)
	{
		Vector3d cam_pos;
		Vector3d V1;
		Vector3d V2;
		TRANSFORM trans1,trans2;
		cam_pos = cameraLocation+cameraDirection; /* center of world */
		/* x : rotate around up */
		Compute_Axis_Rotation_Transform (&trans1,*cameraUp,TWO_M_PI*(0.5-x));
		/* y : rotate around right, before x */
		Compute_Axis_Rotation_Transform (&trans2,*cameraRight,M_PI*(y-0.5));
		/* x=y=1/2 :  no move
		 * no point in composing transformation, we are not reusing them
		 */
		MTransPoint(*V1,*cameraDirection,&trans2);
		MTransPoint(*V2,*V1,&trans1);
		cam_pos -= V2;
		Assign_Vector(ray.Origin,*cam_pos);
		Assign_Vector(ray.Direction,*V2);
    return true;
	}
	bool TracePixel::ProjectionEqualAreaCameraRay(Ray &ray, DBL x, DBL y, DBL cx)
	{
		Vector3d cam_pos;
		Vector3d V1;
		Vector3d V2;
		TRANSFORM trans1,trans2;
		cam_pos = cameraLocation+cameraDirection; /* center of world */
		/* x : rotate around up */
		Compute_Axis_Rotation_Transform (&trans1,*cameraUp,-M_PI*x);
		/* y : rotate around right, before x */
		Compute_Axis_Rotation_Transform (&trans2,*cameraRight,M_PI_2/asin(cx)*asin(y*cx));
		/* x=y=1/2 :  no move
		 * no point in composing transformation, we are not reusing them
		 */
		MTransPoint(*V1,*cameraDirection,&trans2);
		MTransPoint(*V2,*V1,&trans1);
		cam_pos -= V2;
		Assign_Vector(ray.Origin,*cam_pos);
		Assign_Vector(ray.Direction,*V2);
    return true;
	}
	bool TracePixel::ProjectionMollweideCameraRay(Ray &ray, DBL x, DBL y)
	{
		Vector3d cam_pos;
		Vector3d V1;
		Vector3d V2;
		TRANSFORM trans1,trans2;
		DBL theta, lambda, phi, ct;
		cam_pos = cameraLocation+cameraDirection; /* center of world */
		/* Formula from http://mathworld.wolfram.com/MollweideProjection.html */
    /* adjust range */
		x *= M_PI;
    theta = asin(y); /* no need to divide & multiply by SQRT2 here */
    ct = cos(theta);
		if (!ct)
			return false;

		lambda = -x/ct; /* same, avoid 2.0*SQRT2/M_PI factor */
		if (fabs(lambda)>M_PI)
			return false;

		if (abs((2.0*theta + sin(2.0*theta))/M_PI)>1.0)
			return false;

		phi = asin((2.0*theta + sin(2.0*theta))/M_PI);

		/* x : rotate around up */
		Compute_Axis_Rotation_Transform (&trans1,*cameraUp,lambda);
		/* y : rotate around right, before x */
		Compute_Axis_Rotation_Transform (&trans2,*cameraRight,phi);
		/* x=y=0 :  no move
		 * no point in composing transformation, we are not reusing them
		 */
		MTransPoint(*V1,*cameraDirection,&trans2);
		MTransPoint(*V2,*V1,&trans1);
		cam_pos -= V2;
		Assign_Vector(ray.Origin,*cam_pos);
		Assign_Vector(ray.Direction,*V2);
    return true;
	}
	bool TracePixel::ProjectionAitoffHammerCameraRay(Ray &ray, DBL x, DBL y)
	{
		Vector3d cam_pos;
		Vector3d V1;
		Vector3d V2;
		TRANSFORM trans1,trans2;
		cam_pos = cameraLocation+cameraDirection; /* center of world */
		/* adjust range , normal ratio 2:1 */
		x *= 2.0* SQRT2;
		y *= SQRT2;
		DBL z,lambda,phi;
		z = (0.25*x)*(0.25*x)+(0.5*y)*(0.5*y);
		if (z > 0.5)
			return false;

		z = sqrt(1.0-z);
		lambda = 2.0*atan2(z*x,(2.0*(2.0*z*z-1.0)));
		phi = asin(z*y);
		/* lambda : rotate around up */
		Compute_Axis_Rotation_Transform (&trans1,*cameraUp,-lambda);
		/* phi : rotate around right, before x */
		Compute_Axis_Rotation_Transform (&trans2,*cameraRight,phi);
		/* x=y=0 :  no move
		 * no point in composing transformation, we are not reusing them
		 */
		MTransPoint(*V1,*cameraDirection,&trans2);
		MTransPoint(*V2,*V1,&trans1);
		cam_pos -= V2;
		Assign_Vector(ray.Origin,*cam_pos);
		Assign_Vector(ray.Direction,*V2);
    return true;
	}
	bool TracePixel::ProjectionVanDerGrintenCameraRay(Ray &ray, DBL x, DBL y)
	{
		Vector3d cam_pos;
		Vector3d V1;
		Vector3d V2;
		TRANSFORM trans1,trans2;

		cam_pos = cameraLocation+cameraDirection; /* center of world */
#if 0
		DBL xx,yy,c1,c2,c3,d,a1,m1,theta,lambda,phi;
		DBL sqradius;
		/* formula from http://mathworld.wolfram.com/vanderGrintenProjection.html */
		/* Bogus in Dec-2011 */
		/* adjust range: avoid M_PI divide and multiply */
		xx = x; 
		yy = y; 
		sqradius = (xx*xx+yy*yy); 
		if (sqradius>1.0)
			return false;
		c1 = -fabs(yy)*(1+sqradius); 
		c2 = c1 - 2.0*yy*yy +xx*xx; 
		c3 = -2.0*c1 +1.0 +2.0*yy*yy +sqradius*sqradius; 
		d = yy*yy/c3 + 1.0/27.0* ((2.0*c2*c2*c2/(c3*c3*c3))-9.0*(c1*c2)/(c3*c3)); 
		a1 = (1.0/c3)*(c1-(c2*c2)/(3.0*c3)); 
		m1 = 2.0/sqrt(-1.0/3.0*a1); 
		theta = 1.0/3.0*fabs(acos(3.0*d/(a1*m1))); 

		if (xx !=0.0)
		{
			lambda = M_PI/2.0*(sqradius -1.0 +sqrt(1.0+2.0*(xx*xx-yy*yy)+sqradius*sqradius))/xx;
		}
		else
		{
			lambda = 0.0;
		}

		if (yy <0.0)
		{
			phi = -M_PI*(-m1*cos(theta+(M_PI/3.0))-c2/(3.0*c3));
		}
		else if (yy >0.0)
		{
			phi = M_PI*(-m1*cos(theta+(M_PI/3.0))-c2/(3.0*c3));// pi * ((-2/3*cos(2*pi/3))- -4/(3*8) ) = pi ( -2/3* -1/2  + 1/6) = pi (2/6+1/6) = pi (1/2)
		}
		else 
		{
			phi = 0.0;
		}
#else
		x *=M_PI;
		y *=M_PI;
		DBL x2,ay,y2,r,r2,c1,c3,c2,c0,al,m,d,t;
		DBL lambda, phi;
		x2 = x * x;
		ay = fabs(y);
		y2 = y * y;
		r  = x2 + y2;
		if (r > M_PI*M_PI)
			return false;
		r2 = r * r;
		c1 = -M_PI * ay * (r + M_PI * M_PI);
		c3 = r2 + TWO_M_PI * (ay *r + M_PI * (y2 + M_PI * (ay + M_PI_2)));
		c2 = c1 + (M_PI * M_PI) * (r -3.0*y2);
		c0 = M_PI * ay;
		c2 /= c3;
		al = c1 / c3 - c2*c2/3.0;
		m = 2.0 * sqrt( -al/3.0);
		d = 2.0/27.0 * c2 * c2 * c2 + (c0 * c0 - c2*c1/3.0) / c3;

		d *= 3.0 /(al*m);
		d = acos(d);
		if (y < 0)
			phi = -M_PI * (m * cos((d + 4.0* M_PI)/3.0) - c2/3.0);
		else
			phi = M_PI * (m * cos((d + 4.0* M_PI)/3.0) - c2/3.0);

		t = r2 + 2.0*M_PI*M_PI * (x2 - y2 + (M_PI*M_PI/2.0));
		if (x != 0.0)
			lambda = 0.5 * ( r -M_PI*M_PI + sqrt(t))/x;
		else
			lambda = 0;
#endif

		/* x : rotate around up */
		Compute_Axis_Rotation_Transform (&trans1,*cameraUp,-lambda);
		/* y : rotate around right, before x */
		Compute_Axis_Rotation_Transform (&trans2,*cameraRight,phi);
		/* x=y=0 :  no move
		 * no point in composing transformation, we are not reusing them
		 */
		MTransPoint(*V1,*cameraDirection,&trans2);
		MTransPoint(*V2,*V1,&trans1);
		cam_pos -= V2;
		Assign_Vector(ray.Origin,*cam_pos);
		Assign_Vector(ray.Direction,*V2);
    return true;
	}
	/* ECK4_XC : 2/sqrt(pi*(4+pi)) */
	/* ECK4_YC : 2*sqrt(pi/(4+pi)) */
	/* ECK4_YC / ECK4_XC == pi */
#define ECK4_XC .4222382003157712014929445259585221076624
#define ECK4_YC 1.3265004281770023222060941830274718740086
	bool TracePixel::ProjectionEckert4CameraRay(Ray &ray, DBL x, DBL y)
	{
		Vector3d cam_pos;
		Vector3d V1;
		Vector3d V2;
		TRANSFORM trans1,trans2;
		cam_pos = cameraLocation+cameraDirection; /* center of world */
		DBL phi,lambda,theta;
		/* adjust range,  normal ratio is 2:1; you cannot invent ECK4_YC alone */
		x*= 2.0 * M_PI * ECK4_XC; /* also 2.0 * ECK4_YC */
		y*= ECK4_YC; 
		/* from http://mathworld.wolfram.com/EckertIVProjection.html */
	
		theta = asin(y/2.0*sqrt((4.0+M_PI)/M_PI));
		lambda = sqrt(M_PI*(4.0+M_PI))*x/(2.0*(1+cos(theta)));
		phi = asin((theta +sin(theta)*cos(theta)+2.0*sin(theta))/(2.0+M_PI_2));
		if (fabs(lambda) > M_PI)
			return false;

		/* x : rotate around up */
		Compute_Axis_Rotation_Transform (&trans1,*cameraUp,-lambda);
		/* y : rotate around right, before x */
		Compute_Axis_Rotation_Transform (&trans2,*cameraRight,phi);
		/* x=y=0 :  no move
		 * no point in composing transformation, we are not reusing them
		 */
		MTransPoint(*V1,*cameraDirection,&trans2);
		MTransPoint(*V2,*V1,&trans1);
		cam_pos -= V2;
		Assign_Vector(ray.Origin,*cam_pos);
		Assign_Vector(ray.Direction,*V2);
    return true;
	}
	/* ECK6_ADJ := pi * 2 / sqrt(2+pi) */
#define ECK6_ADJ 2.7709649675782468540085734337823276068619
	bool TracePixel::ProjectionEckert6CameraRay(Ray &ray, DBL x, DBL y)
	{
		Vector3d cam_pos;
		Vector3d V1;
		Vector3d V2;
		TRANSFORM trans1,trans2;
		cam_pos = cameraLocation+cameraDirection; /* center of world */
		/* adjust range, normal ratio is 2:1 */
		x*= ECK6_ADJ;
		y*= ECK6_ADJ/2.0;

		DBL lambda,phi, theta;
		theta = 0.5 * sqrt(2.0+M_PI)*y;
		phi = asin((theta+sin(theta))/(1+M_PI_2));
		lambda = sqrt(2.0+M_PI)*x /(1+cos(theta));
		if (fabs(lambda) > M_PI)
			return false;
		/* x : rotate around up */
		Compute_Axis_Rotation_Transform (&trans1,*cameraUp,-lambda);
		/* y : rotate around right, before x */
		Compute_Axis_Rotation_Transform (&trans2,*cameraRight,phi);
		/* x=y=0 :  no move
		 * no point in composing transformation, we are not reusing them
		 */
		MTransPoint(*V1,*cameraDirection,&trans2);
		MTransPoint(*V2,*V1,&trans1);
		cam_pos -= V2;
		Assign_Vector(ray.Origin,*cam_pos);
		Assign_Vector(ray.Direction,*V2);
    return true;
	}
	bool TracePixel::ProjectionMillerCameraRay(Ray &ray, DBL x, DBL y)
	{
		Vector3d cam_pos;
		Vector3d V1;
		Vector3d V2;
		TRANSFORM trans1,trans2;
		/* normal ratio is about 1.3638862 */
		/* adjust range */
		y *= 2.303412543;
		DBL lambda, phi;
		lambda = M_PI*x;
		phi = 5.0/4.0 *atan(sinh(4.0/5.0*y));
		cam_pos = cameraLocation+cameraDirection; /* center of world */
		/* x : rotate around up */
		Compute_Axis_Rotation_Transform (&trans1,*cameraUp,-lambda);
		/* y : rotate around right, before x */
		Compute_Axis_Rotation_Transform (&trans2,*cameraRight,phi);
		/* x=y=0 :  no move
		 * no point in composing transformation, we are not reusing them
		 */
		MTransPoint(*V1,*cameraDirection,&trans2);
		MTransPoint(*V2,*V1,&trans1);
		cam_pos -= V2;
		Assign_Vector(ray.Origin,*cam_pos);
		Assign_Vector(ray.Direction,*V2);
    return true;
	}
	bool TracePixel::ProjectionMercatorCameraRay(Ray &ray, DBL x, DBL y)
	{
		Vector3d cam_pos;
		Vector3d V1;
		Vector3d V2;
		TRANSFORM trans1,trans2;
		cam_pos = cameraLocation+cameraDirection; /* center of world */
		/* x : rotate around up */
		Compute_Axis_Rotation_Transform (&trans1,*cameraUp,TWO_M_PI*(0.5-x));
		/* y : rotate around right, before x */
		Compute_Axis_Rotation_Transform (&trans2,*cameraRight,atan(sinh(TWO_M_PI/aspectRatio*(y-0.5))));
		/* x=y=1/2 :  no move
		 * no point in composing transformation, we are not reusing them
		 */
		MTransPoint(*V1,*cameraDirection,&trans2);
		MTransPoint(*V2,*V1,&trans1);
		cam_pos -= V2;
		Assign_Vector(ray.Origin,*cam_pos);
		Assign_Vector(ray.Direction,*V2);
    return true;
	}
	bool TracePixel::ProjectionLambertAzimuthalCameraRay(Ray &ray, DBL x, DBL y)
	{
		Vector3d cam_pos;
		Vector3d V1;
		Vector3d V2;
		DBL lambda;
		DBL theta;
		DBL c;
		DBL rho;

    rho = sqrt((x*x)+(y*y));
		if (rho>4.0) 
		{
			return false;
		}
    c = 2.0*asin(rho/2.0);
		/* simplified from lambda0 =0 & theta1 =0
		 * See Wolfram 
   http://mathworld.wolfram.com/LambertAzimuthalEqual-AreaProjection.html
	   *
		 * Everything is there, excepted the range for x,y : -2 to +2 
		 */
		lambda = -atan2(x * sin(c),(rho*cos(c)));
		theta = asin(y*sin(c)/rho);
			
		TRANSFORM trans1,trans2;
		cam_pos = cameraLocation+cameraDirection; /* center of world */
		/* x : rotate around up */
		Compute_Axis_Rotation_Transform (&trans1,*cameraUp,lambda  );
		/* y : rotate around right, before x */
		Compute_Axis_Rotation_Transform (&trans2,*cameraRight,theta  );
		/* no point in composing transformation, we are not reusing them
		 */
		MTransPoint(*V1,*cameraDirection,&trans2);
		MTransPoint(*V2,*V1,&trans1);
		cam_pos -= V2;
		Assign_Vector(ray.Origin,*cam_pos);
		Assign_Vector(ray.Direction,*V2);
    return true;
	}
	/* projected coordinates of unfolded object,
	 * in [0,1] x [0,1] sheet
	 */
	static const Vector2d Tetra2d[6]=
	{
		Vector2d(0,0),
		Vector2d(1,0),
		Vector2d(0.5,SQRT3_2),
		Vector2d(1.5,SQRT3_2),
		Vector2d(2,0),
		Vector2d(2.5,SQRT3_2)
	};
	/* spherical coordinates of 3D object: azimuth, elevation in degrees */
	static const Vector2d Tetra3d[4]=
	{
		Vector2d(0,90),
		Vector2d(0,-19.47),
		Vector2d(120,-19.47),
		Vector2d(-120,-19.47)
	};
	/* associated index of 3D coordinates from unfolded vertex */
	static const unsigned int Tetra2to3[6]= { 0,1,2,3,0,2 };
       
   typedef struct { unsigned int a,b,c;} Trindex;
   typedef struct { unsigned int a[4];} Teindex;
   /* index of 2D vertex for each 2D triangles */
   static const Trindex TetraIndex[4]={{0,1,2},{1,2,3},{4,3,1},{4,3,5}};
   static const Vector2d Octa2d[10]=
   {
      Vector2d(0,0),
      Vector2d(0,1),
      Vector2d(SQRT3_2,0.5),
      Vector2d(SQRT3_2,1.5),
      Vector2d(2.0*SQRT3_2,0),
      Vector2d(2.0*SQRT3_2,1),
      Vector2d(3.0*SQRT3_2,0.5),
      Vector2d(3.0*SQRT3_2,1.5),
      Vector2d(4.0*SQRT3_2,1),
      Vector2d(4.0*SQRT3_2,0)
   };
   static const Vector2d Octa3d[6]=
   {
		Vector2d(0,90),
		Vector2d(0,0),
		Vector2d(-90,0),
		Vector2d(180,0),
		Vector2d(90,0),
		Vector2d(0,-90)
   };
   static const unsigned int Octa2to3[10]={0,1,2,5,0,3,4,5,1,0};
   static const Trindex OctaIndex[8]={{0,1,2},{1,2,3},
		 {5,3,2},{4,2,5},{4,5,6},{5,6,7},{8,7,6},{8,6,9}};
#if 0
   static const Vector2d Icosa2d[22]={
    Vector2d(0.5,SQRT3_2),
    Vector2d(1,0),
    Vector2d(1.5,SQRT3_2),
    Vector2d(2,0),
    Vector2d(2.5,SQRT3_2),
    Vector2d(3,0),
    Vector2d(3.5,SQRT3_2),
    Vector2d(4,0),
    Vector2d(4.5,SQRT3_2),
    Vector2d(5,0),
    Vector2d(5.5,SQRT3_2),

    Vector2d(0,2.0*SQRT3_2),
    Vector2d(1,2.0*SQRT3_2),
    Vector2d(2,2.0*SQRT3_2),
    Vector2d(3,2.0*SQRT3_2),
    Vector2d(4,2.0*SQRT3_2),
    Vector2d(5,2.0*SQRT3_2),
    Vector2d(0.5,3.0*SQRT3_2),
    Vector2d(1.5,3.0*SQRT3_2),
    Vector2d(2.5,3.0*SQRT3_2),
    Vector2d(3.5,3.0*SQRT3_2),
    Vector2d(4.5,3.0*SQRT3_2)
   };
	static const Vector2d Icosa3d[12]={
		Vector2d(0,90),
		Vector2d(0,26.565),
		Vector2d(-72,26.565),
		Vector2d(-144,26.565),
		Vector2d(144,26.565),
		Vector2d(72,26.565),
		Vector2d(36,-26.565),
		Vector2d(-36,-26.565),
		Vector2d(-108,-26.565),
		Vector2d(180,-26.565),
		Vector2d(108,-26.565),
		Vector2d(0,-90)
  };
	static const unsigned int Icosa2to3[22]={1,0,2,0,3, 0,4,0,5,0, 1,
		6,7,8,9,10, 6,
		11,11,11,11,11};
	static const Trindex IcosaIndex[20]={
		{0,1,2},{2,3,4},{4,5,6},{6,7,8},{8,9,10},
		{0,12,2},{2,13,4},{4,14,6},{6,15,8},{8,16,10},
		{11,0,12},{12,2,13},{13,4,14},{14,6,15},{15,8,16},
		{11,17,12},{12,18,13},{13,19,14},{14,20,15},{15,21,16}
		};
#else
   static const Vector2d Icosa2d[22]={
    Vector2d(0.0,SQRT3_2),
    Vector2d(0.5,0),
    Vector2d(1.0,SQRT3_2),
    Vector2d(1.5,0),
    Vector2d(2.0,SQRT3_2),
    Vector2d(2.5,0),
    Vector2d(3.0,SQRT3_2),
    Vector2d(3.5,0),
    Vector2d(4.0,SQRT3_2),
    Vector2d(4.5,0),

    Vector2d(5.0,SQRT3_2), //10

    Vector2d(5.5,2.0*SQRT3_2), //11
    Vector2d(0.5,2.0*SQRT3_2),
    Vector2d(1.5,2.0*SQRT3_2),
    Vector2d(2.5,2.0*SQRT3_2),
    Vector2d(3.5,2.0*SQRT3_2),
    Vector2d(4.5,2.0*SQRT3_2),
    Vector2d(5.0,3.0*SQRT3_2),//17
    Vector2d(1.0,3.0*SQRT3_2),
    Vector2d(2.0,3.0*SQRT3_2),
    Vector2d(3.0,3.0*SQRT3_2),
    Vector2d(4.0,3.0*SQRT3_2)
   };
	static const Vector2d Icosa3d[12]={
		Vector2d(0,90),
		Vector2d(0,26.565),
		Vector2d(-72,26.565),
		Vector2d(-144,26.565),
		Vector2d(144,26.565),
		Vector2d(72,26.565),
		Vector2d(36,-26.565),
		Vector2d(-36,-26.565),
		Vector2d(-108,-26.565),
		Vector2d(180,-26.565),
		Vector2d(108,-26.565),
		Vector2d(0,-90)
  };
	static const unsigned int Icosa2to3[22]={1,0,2,0,3, 0,4,0,5,0, 1,
		7,7,8,9,10, 6,
		11,11,11,11,11};
	static const Trindex IcosaIndex[20]={
		{0,1,2},{2,3,4},{4,5,6},{6,7,8},{8,9,10},
		{0,12,2},{2,13,4},{4,14,6},{6,15,8},{8,16,10},

		{11,10,16},{12,2,13},{13,4,14},{14,6,15},{15,8,16},
		{11,17,16},{12,18,13},{13,19,14},{14,20,15},{15,21,16}
		};
  static const Vector2d Cube2d[14]={
    Vector2d(0,0), //0
    Vector2d(1,0),
    Vector2d(1,1), //2
    Vector2d(0,1),
    Vector2d(2,0), //4
    Vector2d(2,1),
    Vector2d(3,0), //6
    Vector2d(3,1),
    Vector2d(2,2), //8
    Vector2d(3,2),
    Vector2d(4,1), //10
    Vector2d(4,2),
    Vector2d(5,1), //12
    Vector2d(5,2),
  };
	static const Vector2d CubeFace3d[6]={
    Vector2d(0.0,0.0),
    Vector2d(270.0,0.0),
    Vector2d(180.0,0.0),
    Vector2d(0.0,-90.0),
    Vector2d(90.0,0.0),
    Vector2d(0.0,90.0)
  };
// Vertical elevation for cube vertex: asin(1/sqrt(3))
#define Hilt 35.264389683
	static const Vector2d Cube3d[8]={
    Vector2d(45.0,Hilt), // 0
    Vector2d(-45.0,Hilt),
    Vector2d(-45.0,-Hilt), // 2
    Vector2d(45.0,-Hilt),
    Vector2d(225.0,Hilt), // 4
    Vector2d(135.0,Hilt),
    Vector2d(135.0,-Hilt), // 6
    Vector2d(225.0,-Hilt),
  };
	static const unsigned int Cube2to3[14]={0,1,2,3, 4,7, 5,6, 2,3, 5,0, 4,1
  };
	static const Teindex CubeIndex[6]={
  {0,1,2,3},{1,4,5,2},{7,6,4,5},
  {5,7,9,8},{7,10,11,9},{10,12,13,11}
  };
#endif
   /* For equilateral triangle */
   void TracePixel::weight_in_triangle(Ray &ray, const DBL x, const DBL y,
            const Vector2d a,
            const Vector2d b,
            const Vector2d c,
            const Vector2d na,
            const Vector2d nb,
            const Vector2d nc
            )
   {
      DBL wa,wb,wc,wp,wm;
      Vector2d p(x,y),np;
      Vector2d pa,pb,pc;
      Vector2d qa,qb,qc;
      DBL dm1,dm2,dm3;
      DBL xpa = p.x()-a.x();
      DBL xpb = p.x()-b.x();
      DBL xpc = p.x()-c.x();

      DBL xbc = b.x()-c.x();
      DBL xba = b.x()-a.x();
      DBL xab = a.x()-b.x();
      DBL xac = a.x()-c.x();
      DBL xca = c.x()-a.x();
      DBL xcb = c.x()-b.x();

      DBL ypa = p.y()-a.y();
      DBL ypb = p.y()-b.y();
      DBL ypc = p.y()-c.y();

      DBL ybc = b.y()-c.y();
      DBL yba = b.y()-a.y();
      DBL yab = a.y()-b.y();
      DBL yac = a.y()-c.y();
      DBL yca = c.y()-a.y();
      DBL ycb = c.y()-b.y();

      dm1 = xpa*ybc-ypa*xbc;
      dm2 = xpb*yca-ypb*xca;
      dm3 = xpc*yab-ypc*xab;
      if (dm1*dm1>0.0)
      {
         DBL r = (xba*ybc-yba*xbc)/dm1;
         DBL s = (xpa*yba-ypa*xba)/dm1;
         Vector2d q(b+(c-b)*s);
         qa = q-a;

         pa = p-a;
         wa = 1.0 - pa.length()/qa.length();
      }
      else 
      {
         wa = 1.0;
      }

      if (dm2*dm2>0.0)
      {
         DBL s = (xpb*ycb-ypb*xcb)/dm2;
         Vector2d q(c+(a-c)*s);
         qb = q-b;

         pb = p-b;
         wb = 1.0 - pb.length()/qb.length();
      }
      else 
      {
         wb = 1.0;
      }
      if (dm3*dm3>0.0)
      {
         DBL s = (xpc*yac-ypc*xac)/dm3;
         Vector2d q(a+(b-a)*s);
         qc = q-c;

         pc = p-c;
         wc = 1.0 - pc.length()/qc.length();
      }
      else 
      {
         wc = 1.0;
      }
      wp = 1.0/(wa+wb+wc);
      wa *= wp;
      wb *= wp;
      wc *= wp;
      {
         Vector3d n3a(
               cos(M_PI_180*na.x())*cos(M_PI_180*na.y()),
               sin(M_PI_180*na.x())*cos(M_PI_180*na.y()),
               sin(M_PI_180*na.y()));
         Vector3d n3b(
               cos(M_PI_180*nb.x())*cos(M_PI_180*nb.y()),
               sin(M_PI_180*nb.x())*cos(M_PI_180*nb.y()),
               sin(M_PI_180*nb.y()));
         Vector3d n3c(
               cos(M_PI_180*nc.x())*cos(M_PI_180*nc.y()),
               sin(M_PI_180*nc.x())*cos(M_PI_180*nc.y()),
               sin(M_PI_180*nc.y()));
         Vector3d n3p;

         /* using Vector3d::operator*(DBL) */
         n3p = n3a*wa; 
         n3p+= n3b*wb;
         n3p+= n3c*wc;
         n3p.normalize();
         np.y() = asin(n3p.z())/M_PI_180;
         np.x() = atan2(n3p.y(),n3p.x())/M_PI_180;
      }
      /* nc is the 3D direction, using spherical coordinate; translate to cartesian */
      Vector3d cam_pos,V1,V2;
      TRANSFORM trans1,trans2;
      cam_pos = cameraLocation+cameraDirection; /* center of the world */
      /* x : rotate around up */
      Compute_Axis_Rotation_Transform(&trans1,*cameraUp,M_PI_180*np.x());
      /* y : rotate around right, before x */
      Compute_Axis_Rotation_Transform(&trans2,*cameraRight,M_PI_180*np.y());
      /* 0,0: no move */
      MTransPoint(*V1,*cameraDirection,&trans2);
      MTransPoint(*V2,*V1,&trans1);
      cam_pos -= V2;
      Assign_Vector(ray.Origin,*cam_pos);
      Assign_Vector(ray.Direction,*V2);
   }

   /* weighting in a isocele triangle with a right angle in third point
    * the other two vertices also share a coordinate
    * and the radius of the third point is 1/sqrt(3) whereas the radius of the
    * two other vertices is 1
    */
   void TracePixel::weight_in_rectriangle(Ray &ray, const DBL x, const DBL y,
            const Vector2d a,
            const Vector2d b,
            const Vector2d c,
            const Vector2d na,
            const Vector2d nb,
            const Vector2d nc
            )
   {
      DBL wa,wb,la,lb,lp;
      Vector2d ca,cb,p(x,y),cp,np;
      ca = a-c;
      cb = b-c;
      cp = p-c;
      la = ca.lengthSqr();
      lb = cb.lengthSqr();
			wa = (ca.x()*cp.x()+ca.y()*cp.y())/(la);
			wb = (cb.x()*cp.x()+cb.y()*cp.y())/(lb);
      {
         Vector3d n3a(
               cos(M_PI_180*na.x())*cos(M_PI_180*na.y()),
               sin(M_PI_180*na.x())*cos(M_PI_180*na.y()),
               sin(M_PI_180*na.y()));
         Vector3d n3b(
               cos(M_PI_180*nb.x())*cos(M_PI_180*nb.y()),
               sin(M_PI_180*nb.x())*cos(M_PI_180*nb.y()),
               sin(M_PI_180*nb.y()));
         Vector3d n3c(
               cos(M_PI_180*nc.x())*cos(M_PI_180*nc.y()),
               sin(M_PI_180*nc.x())*cos(M_PI_180*nc.y()),
               sin(M_PI_180*nc.y()));
         n3c /= sqrt(3);
         n3a -= n3c;
         n3b -= n3c;
         n3a *= wa; 
         n3b *= wb; 
         Vector3d n3p(n3c+n3a+n3b);
         n3p.normalize();
         np.y() = asin(n3p.z())/M_PI_180;
         np.x() = atan2(n3p.y(),n3p.x())/M_PI_180;
      }
      /* nc is the 3D direction, using spherical coordinate; translate to cartesian */
      Vector3d cam_pos,V1,V2;
      TRANSFORM trans1,trans2;
      cam_pos = cameraLocation+cameraDirection; /* center of the world */
      /* x : rotate around up */
      Compute_Axis_Rotation_Transform(&trans1,*cameraUp,M_PI_180*np.x());
      /* y : rotate around right, before x */
      Compute_Axis_Rotation_Transform(&trans2,*cameraRight,M_PI_180*np.y());
      /* 0,0: no move */
      MTransPoint(*V1,*cameraDirection,&trans2);
      MTransPoint(*V2,*V1,&trans1);
      cam_pos -= V2;
      Assign_Vector(ray.Origin,*cam_pos);
      Assign_Vector(ray.Direction,*V2);
   }
   static bool inside_triangle(const DBL x,const DBL y, 
         const Vector2d &a, const Vector2d &b, const Vector2d &c)
   {
      /* computes part of cross product to check (x,y) is always
       * on the same side of each triangle segment
       */
      Vector2d i=a,j=b,k=c;
      Vector2d m(x,y),n(x,y),o(x,y);
      i -= b;
      m -= b;

      j -= c;
      n -= c;

      k -= a;
      o -= a;

      DBL e,f,g;
#define CROSS_2D(a,b) ((a).x()*(b).y() - (a).y()*(b).x())
      e = CROSS_2D(i,m);
      f = CROSS_2D(j,n);
      g = CROSS_2D(k,o);
#define SIGN(a) ((a<0)?-1:(a>0)?1:0)
      int r,s,t;
      r = SIGN(e);
      s = SIGN(f);
      t = SIGN(g);
      return ((r*s >  0) &&(s*t >  0) &&(r*t >  0));
   }

	bool TracePixel::ProjectionTetraCameraRay(Ray &ray, DBL x, DBL y)
	{
		/* adjust x,y */
		x *= 2.5;
		y *= SQRT3_2;
		for(int i=0;i<4;i++)
		{
			if (inside_triangle(x,y,
						Tetra2d[TetraIndex[i].a],
						Tetra2d[TetraIndex[i].b],
						Tetra2d[TetraIndex[i].c]))
			{
				/* compute the weight of each vertex in 2D triangle and apply to 3D */
				weight_in_triangle(ray, x,y, 
						Tetra2d[TetraIndex[i].a],
						Tetra2d[TetraIndex[i].b],
						Tetra2d[TetraIndex[i].c],
						Tetra3d[Tetra2to3[TetraIndex[i].a]],
						Tetra3d[Tetra2to3[TetraIndex[i].b]],
						Tetra3d[Tetra2to3[TetraIndex[i].c]]);

				return true;
			}
		}         
		return false;
	}
	bool TracePixel::ProjectionCubeCameraRay(Ray &ray, DBL x, DBL y)
	{
		/* adjust x,y */
		x *= 5;
		y *= 2;
		for(int i=0;i<6;i++)
		{
			Vector2d cent;
			cent = Cube2d[CubeIndex[i].a[0]];
			cent += Cube2d[CubeIndex[i].a[1]];
			cent += Cube2d[CubeIndex[i].a[2]];
			cent += Cube2d[CubeIndex[i].a[3]];
			cent /= 4;
			for(int j=0;j<4;j++)
			{
				if (inside_triangle(x,y,
							Cube2d[CubeIndex[i].a[j]],
							Cube2d[CubeIndex[i].a[(j+1)%4]],
							cent))
				{
					/* compute the weight of each vertex in 2D triangle and apply to 3D */
					weight_in_rectriangle(ray, x,y, 
							Cube2d[CubeIndex[i].a[j]],
							Cube2d[CubeIndex[i].a[(j+1)%4]],
							cent,
							Cube3d[Cube2to3[CubeIndex[i].a[j]]],
							Cube3d[Cube2to3[CubeIndex[i].a[(j+1)%4]]],
							CubeFace3d[i]);
					return true;
				}
			}         
		}         
		return false;
	}
	bool TracePixel::ProjectionOctaCameraRay(Ray &ray, DBL x, DBL y)
	{
		/* adjust x,y */
		x *= 4*SQRT3_2;
		y *= 1.5;
		for(int i=0;i<8;i++)
		{
			if (inside_triangle(x,y,
						Octa2d[OctaIndex[i].a],
						Octa2d[OctaIndex[i].b],
						Octa2d[OctaIndex[i].c]))
			{
				/* compute the weight of each vertex in 2D triangle and apply to 3D */
				weight_in_triangle(ray, x,y, 
						Octa2d[OctaIndex[i].a],
						Octa2d[OctaIndex[i].b],
						Octa2d[OctaIndex[i].c],
						Octa3d[Octa2to3[OctaIndex[i].a]],
						Octa3d[Octa2to3[OctaIndex[i].b]],
						Octa3d[Octa2to3[OctaIndex[i].c]]);

				return true;
			}
		}         
		return false;
	}
	bool TracePixel::ProjectionIcosaCameraRay(Ray &ray, DBL x, DBL y)
	{
		/* adjust x,y */
		x *= 5.5;
		y *= 3*SQRT3_2;
		for(int i=0;i<20;i++)
		{
			if (inside_triangle(x,y,
						Icosa2d[IcosaIndex[i].a],
						Icosa2d[IcosaIndex[i].b],
						Icosa2d[IcosaIndex[i].c]))
			{
				/* compute the weight of each vertex in 2D triangle and apply to 3D */
				weight_in_triangle(ray, x,y, 
						Icosa2d[IcosaIndex[i].a],
						Icosa2d[IcosaIndex[i].b],
						Icosa2d[IcosaIndex[i].c],
						Icosa3d[Icosa2to3[IcosaIndex[i].a]],
						Icosa3d[Icosa2to3[IcosaIndex[i].b]],
						Icosa3d[Icosa2to3[IcosaIndex[i].c]]);

				return true;
			}
		}         
		return false;
	}
}
