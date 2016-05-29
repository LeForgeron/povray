/*******************************************************************************
 * lemon.h
 *
 * This module contains all defines, typedefs, and prototypes for LEMON.CPP.
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
 * $File: $
 * $Revision: $
 * $Change: $
 * $DateTime: $
 * $Author: $
 *******************************************************************************/

#ifndef LEMON_H
#define LEMON_H

#include "backend/control/messagefactory.h"
#include "base/textstream.h"


namespace pov
{

/*****************************************************************************
* Global preprocessor defines
******************************************************************************/

#define LEMON_OBJECT (STURM_OK_OBJECT)



/*****************************************************************************
* Global typedefs
******************************************************************************/

class Lemon : public ObjectBase
{
    private:
        struct LEMON_INT
        {
            DBL d;  /* Distance of intersection point               */
            VECTOR n;/* Normal */
        };
    public:
        VECTOR apex;            /* Center of the top of the lemon */
        VECTOR base;            /* Center of the bottom of the lemon */
        DBL apex_radius;        /* Radius of the lemon at the top */
        DBL base_radius;        /* Radius of the lemon at the bottom */
        DBL inner_radius;       /* Radius of the inner circle */
        DBL HorizontalPosition; /* horizontal position of the center of the inner circle */
        DBL VerticalPosition;   /* vertical position of the center of the inner circle */

        Lemon();
        virtual ~Lemon();

        virtual ObjectPtr Copy();

        virtual bool All_Intersections(const Ray&, IStack&, TraceThreadData *);
        virtual bool Inside(const VECTOR, TraceThreadData *) const;
        virtual void Normal(VECTOR, Intersection *, TraceThreadData *) const;
        virtual void UVCoord(UV_VECT, const Intersection *, TraceThreadData *) const;
        virtual void Translate(const VECTOR, const TRANSFORM *);
        virtual void Rotate(const VECTOR, const TRANSFORM *);
        virtual void Scale(const VECTOR, const TRANSFORM *);
        virtual void Transform(const TRANSFORM *);
        virtual void Invert();
        virtual void Compute_BBox();

        void Compute_Lemon_Data( MessageFactory & messageFactory, pov_base::ITextStream *FileHandle, pov_base::ITextStream::FilePos & Token_File_Pos, int Token_Col_No );
    protected:
        int Intersect(const VECTOR & P, const VECTOR & D, LEMON_INT *Intersection, TraceThreadData *Thread) const;
        void CalcUV(const VECTOR IPoint, UV_VECT Result) const;
};

}

#endif
