//******************************************************************************
///
/// @file core/shape/lemon.h
///
/// Declarations related to the ovus geometric primitive.
///
/// @author Jerome Grimbert
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

#ifndef POVRAY_CORE_LEMON_H
#define POVRAY_CORE_LEMON_H

// Module config header file must be the first file included within POV-Ray unit header files
#include "core/configcore.h"

// C++ variants of C standard header files
// C++ standard header files
//  (none at the moment)

// POV-Ray header files (base module)
#include "base/messenger_fwd.h"

// POV-Ray header files (core module)
#include "core/coretypes.h"
#include "core/scene/object.h"
#include "core/shape/uvmeshable.h"

namespace pov
{

//##############################################################################
///
/// @addtogroup PovCoreShape
///
/// @{

//******************************************************************************
///
/// @name Object Types
///
/// @{

#define LEMON_OBJECT (STURM_OK_OBJECT)

/// @}
///
//******************************************************************************

class Lemon final : public ObjectBase, public UVMeshable
{
    private:
        struct LEMON_INT final
        {
            DBL d;  /* Distance of intersection point               */
            Vector3d n;/* Normal */
        };
    public:
        Lemon();
        virtual ~Lemon() override;

        virtual ObjectPtr Copy() override;

        virtual bool All_Intersections(const Ray&, IStack&, TraceThreadData *) override;
        virtual bool Inside(const Vector3d&, TraceThreadData *) const override;
        virtual void Normal(Vector3d&, Intersection *, TraceThreadData *) const override;
        virtual void UVCoord(Vector2d&, const Intersection *) const override;
        virtual void Translate(const Vector3d&, const TRANSFORM *) override;
        virtual void Rotate(const Vector3d&, const TRANSFORM *) override;
        virtual void Scale(const Vector3d&, const TRANSFORM *) override;
        virtual void Transform(const TRANSFORM *) override;
        virtual void Compute_BBox() override;

        void Compute_Lemon_Data(GenericMessenger& messenger, const MessageContext& context);

        Vector3d apex;          /* Center of the top of the lemon */
        Vector3d base;          /* Center of the bottom of the lemon */
        DBL apex_radius;        /* Radius of the lemon at the top */
        DBL base_radius;        /* Radius of the lemon at the bottom */
        DBL inner_radius;       /* Radius of the inner circle */
        DBL HorizontalPosition; /* horizontal position of the center of the inner circle */
        DBL VerticalPosition;   /* vertical position of the center of the inner circle */
        Vector3d uref; ///< direction for origin of u in uv_mapping


        virtual void evalVertex( Vector3d& r, const DBL u, const DBL v, TraceThreadData *Thread )const override;
        virtual void evalNormal( Vector3d& r, const DBL u, const DBL v, TraceThreadData *Thread )const override;
        virtual void minUV( Vector2d& r )const override;
        virtual void maxUV( Vector2d& r )const override;
    protected:
        int Intersect(const Vector3d& P, const Vector3d& D, LEMON_INT *Intersection, RenderStatistics& stats) const;
        void CalcUV(const Vector3d& IPoint, Vector2d& Result) const;
};

/// @}
///
//##############################################################################

}
// end of namespace pov

#endif // POVRAY_CORE_OVUS_H
