//******************************************************************************
///
/// @file core/material/warp.h
///
/// Declarations related to warps.
///
/// @copyright
/// @parblock
///
/// Persistence of Vision Ray Tracer ('POV-Ray') version 3.8.
/// Copyright 1991-2019 Persistence of Vision Raytracer Pty. Ltd.
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

#ifndef POVRAY_CORE_WARP_H
#define POVRAY_CORE_WARP_H

// Module config header file must be the first file included within POV-Ray unit header files
#include "core/configcore.h"

// C++ variants of C standard header files
//  (none at the moment)

// C++ standard header files
#include <vector>

// POV-Ray header files (base module)
//  (none at the moment)

// POV-Ray header files (core module)
#include "core/math/matrix.h"
#include "core/math/vector.h"

namespace pov
{

//##############################################################################
///
/// @defgroup PovCoreMaterialWarp Texture Warps
/// @ingroup PovCore
///
/// @{

/*****************************************************************************
* Global typedefs
******************************************************************************/

struct GenericWarp
{
    virtual ~GenericWarp() {}
    virtual GenericWarp* Clone() const = 0;
    virtual bool WarpPoint(Vector3d& rP) const = 0;
    virtual bool WarpNormal(Vector3d& rN) const = 0;
    virtual bool UnwarpNormal(Vector3d& rN) const = 0;
};

typedef GenericWarp* WarpPtr;
typedef const GenericWarp* ConstWarpPtr;
typedef std::vector<WarpPtr> WarpList;


struct BlackHoleWarp final : public GenericWarp
{
    BlackHoleWarp() :
        GenericWarp(), Center(0.0), Repeat_Vector(0.0), Uncertainty_Vector(0.0), Strength(1.0), Radius(1.0),
        Radius_Squared(1.0), Inverse_Radius(1.0), Power(2.0), Type(0), Inverted(false), Repeat(false), Uncertain(false)
    {}
    BlackHoleWarp(const BlackHoleWarp& old) :
        GenericWarp(old), Center(old.Center), Repeat_Vector(old.Repeat_Vector),
        Uncertainty_Vector(old.Uncertainty_Vector), Strength(old.Strength), Radius(old.Radius),
        Radius_Squared(old.Radius_Squared), Inverse_Radius(old.Inverse_Radius), Power(old.Power), Type(old.Type),
        Inverted(old.Inverted), Repeat(old.Repeat), Uncertain(old.Uncertain)
    {}

    virtual GenericWarp* Clone() const override { return new BlackHoleWarp(*this); }
    virtual bool WarpPoint(Vector3d& rP) const override;
    virtual bool WarpNormal(Vector3d& rN) const override;
    virtual bool UnwarpNormal(Vector3d& rN) const override;

    Vector3d    Center;
    Vector3d    Repeat_Vector;
    Vector3d    Uncertainty_Vector;
    DBL         Strength;
    DBL         Radius;
    DBL         Radius_Squared;
    DBL         Inverse_Radius;
    DBL         Power;
    short       Type;
    bool        Inverted;
    bool        Repeat;
    bool        Uncertain;
};

struct CubicWarp final : public GenericWarp
{
    CubicWarp() : GenericWarp() {}
    CubicWarp(const CubicWarp& old) : GenericWarp(old) {}

    virtual GenericWarp* Clone() const override { return new CubicWarp(*this); }
    virtual bool WarpPoint(Vector3d& rP) const override;
    virtual bool WarpNormal(Vector3d& rN) const override;
    virtual bool UnwarpNormal(Vector3d& rN) const override;
};

struct CylindricalWarp final : public GenericWarp
{
    CylindricalWarp() :
        GenericWarp(), Orientation_Vector(0.0, 0.0, 1.0), DistExp(0.0)
    {}
    CylindricalWarp(const CylindricalWarp& old) :
        GenericWarp(old), Orientation_Vector(old.Orientation_Vector), DistExp(old.DistExp)
    {}

    virtual GenericWarp* Clone() const override { return new CylindricalWarp(*this); }
    virtual bool WarpPoint(Vector3d& rP) const override;
    virtual bool WarpNormal(Vector3d& rN) const override;
    virtual bool UnwarpNormal(Vector3d& rN) const override;

    Vector3d Orientation_Vector;
    DBL DistExp;
};

struct IdentityWarp final : public GenericWarp
{
    IdentityWarp() : GenericWarp() {}
    IdentityWarp(const IdentityWarp& old) : GenericWarp(old) {}

    virtual GenericWarp* Clone() const override { return new IdentityWarp(*this); }
    virtual bool WarpPoint(Vector3d& rP) const override;
    virtual bool WarpNormal(Vector3d& rN) const override;
    virtual bool UnwarpNormal(Vector3d& rN) const override;
};

struct PlanarWarp final : public GenericWarp
{
    PlanarWarp() :
        GenericWarp(), Orientation_Vector(0.0, 0.0, 1.0), OffSet(0.0)
    {}
    PlanarWarp(const PlanarWarp& old) :
        GenericWarp(old), Orientation_Vector(old.Orientation_Vector), OffSet(old.OffSet)
    {}

    virtual GenericWarp* Clone() const override { return new PlanarWarp(*this); }
    virtual bool WarpPoint(Vector3d& rP) const override;
    virtual bool WarpNormal(Vector3d& rN) const override;
    virtual bool UnwarpNormal(Vector3d& rN) const override;

    Vector3d Orientation_Vector;
    DBL OffSet;
};

struct RepeatWarp final : public GenericWarp
{
    RepeatWarp() :
        GenericWarp(), Axis(-1), Width(0.0), Flip(1.0), Offset(0.0)
    {}
    RepeatWarp(const RepeatWarp& old) :
        GenericWarp(old), Axis(old.Axis), Width(old.Width), Flip(old.Flip), Offset(old.Offset)
    {}

    virtual GenericWarp* Clone() const override { return new RepeatWarp(*this); }
    virtual bool WarpPoint(Vector3d& rP) const override;
    virtual bool WarpNormal(Vector3d& rN) const override;
    virtual bool UnwarpNormal(Vector3d& rN) const override;

    int Axis;
    SNGL Width;
    Vector3d Flip, Offset;
};

struct SphericalWarp final : public GenericWarp
{
    SphericalWarp() :
        GenericWarp(), Orientation_Vector(0.0, 0.0, 1.0), DistExp(0.0)
    {}
    SphericalWarp(const SphericalWarp& old) :
        GenericWarp(old), Orientation_Vector(old.Orientation_Vector), DistExp(old.DistExp)
    {}

    virtual GenericWarp* Clone() const override { return new SphericalWarp(*this); }
    virtual bool WarpPoint(Vector3d& rP) const override;
    virtual bool WarpNormal(Vector3d& rN) const override;
    virtual bool UnwarpNormal(Vector3d& rN) const override;

    Vector3d Orientation_Vector;
    DBL DistExp;
};

struct ToroidalWarp final : public GenericWarp
{
    ToroidalWarp() :
        GenericWarp(), Orientation_Vector(0.0, 0.0, 1.0), DistExp(0.0), MajorRadius(1.0)
    {}
    ToroidalWarp(const ToroidalWarp& old) :
        GenericWarp(old), Orientation_Vector(old.Orientation_Vector), DistExp(old.DistExp),
        MajorRadius(old.MajorRadius)
    {}

    virtual GenericWarp* Clone() const override { return new ToroidalWarp(*this); }
    virtual bool WarpPoint(Vector3d& rP) const override;
    virtual bool WarpNormal(Vector3d& rN) const override;
    virtual bool UnwarpNormal(Vector3d& rN) const override;

    Vector3d Orientation_Vector;
    DBL DistExp;
    DBL MajorRadius;
};

struct TransformWarp final : public GenericWarp
{
    TransformWarp() :
        GenericWarp()
    {
        MIdentity(Trans.matrix);
        MIdentity(Trans.inverse);
    }
    TransformWarp(const TransformWarp& old) :
        GenericWarp(old), Trans(old.Trans)
    {}

    virtual GenericWarp* Clone() const override { return new TransformWarp(*this); }
    virtual bool WarpPoint(Vector3d& rP) const override;
    virtual bool WarpNormal(Vector3d& rN) const override;
    virtual bool UnwarpNormal(Vector3d& rN) const override;

    TRANSFORM Trans;
};


struct GenericTurbulenceWarp : public GenericWarp
{
    GenericTurbulenceWarp() :
        GenericWarp(), Turbulence(0.0), Octaves(6), Omega(0.5), Lambda(2.0)
    {}
    GenericTurbulenceWarp(const GenericTurbulenceWarp& old) :
        GenericWarp(old), Turbulence(old.Turbulence), Octaves(old.Octaves), Lambda(old.Lambda), Omega(old.Omega)
    {}

    virtual GenericWarp* Clone() const override = 0;
    virtual bool WarpPoint(Vector3d& rP) const override;
    virtual bool WarpNormal(Vector3d& rN) const override;
    virtual bool UnwarpNormal(Vector3d& rN) const override;

    Vector3d Turbulence;
    int Octaves;
    SNGL Lambda, Omega;
};

/// Genuine turbulence warp.
struct TurbulenceWarp final : public GenericTurbulenceWarp
{
    TurbulenceWarp() : GenericTurbulenceWarp() {}
    TurbulenceWarp(const TurbulenceWarp& old) : GenericTurbulenceWarp(old) {}

    virtual GenericWarp* Clone() const override { return new TurbulenceWarp(*this); }
};

/// Turbulence tied to a pattern.
struct ClassicTurbulence final : public GenericTurbulenceWarp
{
    ClassicTurbulence(bool hbp) :
        GenericTurbulenceWarp(), handledByPattern(hbp)
    {}
    ClassicTurbulence(const ClassicTurbulence& old) :
        GenericTurbulenceWarp(old), handledByPattern(old.handledByPattern)
    {}

    virtual GenericWarp* Clone() const override { return new ClassicTurbulence(*this); }

    bool handledByPattern;
};

/// rotate/screw warp
struct RotateWarp final : public GenericWarp
{
    RotateWarp() : GenericWarp(), twoPiPerUnit( TWO_M_PI ) {}
    RotateWarp(const RotateWarp& old) : GenericWarp(old), twoPiPerUnit(old.twoPiPerUnit) {}

    virtual GenericWarp* Clone() const override { return new RotateWarp(*this); }
    virtual bool WarpPoint(Vector3d& rP) const override;
    virtual bool WarpNormal(Vector3d& rN) const override;
    virtual bool UnwarpNormal(Vector3d& rN) const override;

    float twoPiPerUnit;
};

/// hyperbolic disc warp
struct DiscWarp final : public GenericWarp
{
    DiscWarp(): GenericWarp(), Radius(1.0), Scale(1.0){}
    DiscWarp(const DiscWarp& old): GenericWarp(old),Radius(old.Radius), Scale(old.Scale){}

    virtual GenericWarp* Clone() const override { return new DiscWarp(*this); }
    virtual bool WarpPoint(Vector3d& rP) const override;
    virtual bool WarpNormal(Vector3d& rN) const override;
    virtual bool UnwarpNormal(Vector3d& rN) const override;
    
    DBL Radius;
    DBL Scale;
};
/// wrap plane around surface warp
struct ConeWarp final: public GenericWarp
{
  ConeWarp(): GenericWarp(), Center(0.0, 0.0, 0.0), Param(0.0, 0.0, 0.0), Radius(0.0), Inverted(false) {}
  ConeWarp(const ConeWarp& old): GenericWarp(old), Center(old.Center), Param(old.Param), Radius(old.Radius), Inverted(old.Inverted) {}
  virtual GenericWarp * Clone() const override {return new ConeWarp(*this); }
  virtual bool WarpPoint(Vector3d& rP) const override;
  virtual bool WarpNormal(Vector3d& rN) const override;
  virtual bool UnwarpNormal(Vector3d& rN) const override;

  Vector3d    Center;
  Vector3d    Param;
  DBL         Radius;
  bool        Inverted;
};
struct TorusWarp final: public GenericWarp
{
  TorusWarp(): GenericWarp(), Center(0.0, 0.0, 0.0), Param(0.0, 0.0, 0.0), Radius(0.0), Inverted(false)  {}
  TorusWarp(const TorusWarp& old): GenericWarp(old), Center(old.Center), Param(old.Param), Radius(old.Radius), Inverted(old.Inverted)  {}
  virtual GenericWarp * Clone() const override {return new TorusWarp(*this); }
  virtual bool WarpPoint(Vector3d& rP) const override;
  virtual bool WarpNormal(Vector3d& rN) const override;
  virtual bool UnwarpNormal(Vector3d& rN) const override;

  Vector3d    Center;
  Vector3d    Param;
  DBL         Radius;
  bool        Inverted;
};
struct CylinderWarp final: public GenericWarp
{
  CylinderWarp(): GenericWarp(), Center(0.0, 0.0, 0.0), Param(0.0, 0.0, 0.0), Inverted(false)  {}
  CylinderWarp(const CylinderWarp& old): GenericWarp(old),  Center(old.Center), Param(old.Param), Inverted(old.Inverted)  {}
  virtual GenericWarp * Clone() const override {return new CylinderWarp(*this); }
  virtual bool WarpPoint(Vector3d& rP) const override;
  virtual bool WarpNormal(Vector3d& rN) const override;
  virtual bool UnwarpNormal(Vector3d& rN) const override;

  Vector3d    Center;
  Vector3d    Param;
  bool        Inverted;
};
struct SphereWarp final: public GenericWarp
{
  SphereWarp(): GenericWarp(), Center(0.0, 0.0, 0.0), Param(0.0, 0.0, 0.0), Inverted(false)  {}
  SphereWarp(const SphereWarp& old): GenericWarp(old),  Center(old.Center), Param(old.Param), Inverted(old.Inverted)  {}
  virtual GenericWarp * Clone() const override {return new SphereWarp(*this); }
  virtual bool WarpPoint(Vector3d& rP) const override;
  virtual bool WarpNormal(Vector3d& rN) const override;
  virtual bool UnwarpNormal(Vector3d& rN) const override;

  Vector3d    Center;
  Vector3d    Param;
  bool        Inverted;
};
/*****************************************************************************
* Global functions
******************************************************************************/

void Warp_EPoint (Vector3d& TPoint, const Vector3d& EPoint, const TPATTERN *TPat);
void Destroy_Warps (WarpList& rWarps);
void Copy_Warps (WarpList& rNew, const WarpList& old);
void Warp_Normal (Vector3d& TNorm, const Vector3d& ENorm, const TPATTERN *TPat, bool DontScaleBumps);
void UnWarp_Normal (Vector3d& TNorm, const Vector3d& ENorm, const TPATTERN *TPat, bool DontScaleBumps);

/// @}
///
//##############################################################################

}
// end of namespace pov

#endif // POVRAY_CORE_WARP_H
