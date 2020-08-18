/********************************************************************
* 
*   Description: aimissimkin.c
*   Kinematics for the AIMIS-FYT Photoreactiv Polymere Printer  
*   consisting of five Joints 0;1;2;3;4 and four axis X; Z Z; B; W, 
*   where W is the Extrusion stepmotor.  
* 
*   This File is derived from the maxkins.c Kinematics for Chris Radek's
*   tabletop 5 axis mill.
*
*   Copyright (C) 2020  AIMIS-FYT
*
*   This program is free software: you can redistribute it and/or modify
*   it under the terms of the GNU General Public License as published by
*   the Free Software Foundation, either version 3 of the License, or
*   (at your option) any later version.
*
*   This program is distributed in the hope that it will be useful,
*   but WITHOUT ANY WARRANTY; without even the implied warranty of
*   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*   GNU General Public License for more details.
*
*   You should have received a copy of the GNU General Public License
*   along with this program.  If not, see <https://www.gnu.org/licenses/>.
* 
********************************************************************/

#include "kinematics.h"		/* these decls */
#include "posemath.h"
#include "hal.h"
#include "rtapi.h"
#include "rtapi_math.h"

#define d2r(d) ((d)*PM_PI/180.0)
#define r2d(r) ((r)*180.0/PM_PI)

#ifndef hypot
#define hypot(a,b) (sqrt((a)*(a)+(b)*(b)))
#endif

struct haldata {
    hal_float_t *pivot_length;
} *haldata;

int kinematicsForward(const double *joints,
		      EmcPose * pos,
		      const KINEMATICS_FORWARD_FLAGS * fflags,
		      KINEMATICS_INVERSE_FLAGS * iflags)
{
    // B correction
    double zb = (*(haldata->pivot_length) ) * cos(d2r(joints[3]));
    double xb = (*(haldata->pivot_length) ) * sin(d2r(joints[3]));
        
    pos->tran.x = joints[0];
    pos->tran.z = joints[2] - zb + *(haldata->pivot_length);
    pos->b = joints[3];
    pos->w = joints[4];

    return 0;
}

int kinematicsInverse(const EmcPose * pos,
		      double *joints,
		      const KINEMATICS_INVERSE_FLAGS * iflags,
		      KINEMATICS_FORWARD_FLAGS * fflags)
{
    // B correction
    double zb = (*(haldata->pivot_length) ) * cos(d2r(pos->b));
    double xb = (*(haldata->pivot_length) ) * sin(d2r(pos->b));
        
    // C correction
   double xyr = hypot(pos->tran.x, pos->tran.y);
   double xytheta = atan2(pos->tran.y, pos->tran.x) - d2r(pos->c);

    joints[0] = xyr * cos(xytheta) + xb;
    joints[1] = pos->tran.z + zb  - *(haldata->pivot_length);
    joints[2] = pos->tran.z + zb  - *(haldata->pivot_length);

    joints[3] = pos->b;
    joints[4] = pos->w;

    return 0;
}

KINEMATICS_TYPE kinematicsType()
{
    return KINEMATICS_BOTH;
}

#include "rtapi.h"		/* RTAPI realtime OS API */
#include "rtapi_app.h"		/* RTAPI realtime module decls */

EXPORT_SYMBOL(kinematicsType);
EXPORT_SYMBOL(kinematicsInverse);
EXPORT_SYMBOL(kinematicsForward);
MODULE_LICENSE("GPL");

int comp_id;
int rtapi_app_main(void) {
    int result;
    comp_id = hal_init("aimiskin");
    if(comp_id < 0) return comp_id;

    haldata = hal_malloc(sizeof(struct haldata));

    result = hal_pin_float_new("aimiskin.pivot-length", HAL_IO, &(haldata->pivot_length), comp_id);
    if(result < 0) goto error;

    *(haldata->pivot_length) = 0.666;

    hal_ready(comp_id);
    return 0;

error:
    hal_exit(comp_id);
    return result;
}

