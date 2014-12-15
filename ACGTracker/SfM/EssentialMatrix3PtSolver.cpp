/*===========================================================================*\
 *                                                                           *
 *  ACG Tracker from the ECCV'14 Paper                                       *
 *  Scalable 6-DOF Localization on Mobile Devices                            *
 *  Copyright (C) 2014 by Computer Graphics Group, RWTH Aachen               *
 *  Author: Sven Middelberg <middelberg@cs.rwth-aachen.de>                   *
 *  www.rwth.graphics                                                        *
 *                                                                           *
 *---------------------------------------------------------------------------*
 *  This file is part of ACG Tracker                                         *
 *                                                                           *
 *  ACG Tracker is free software: you can redistribute it and/or modify      *
 *  it under the terms of the GNU General Public License as published by     *
 *  the Free Software Foundation, either version 3 of the License, or        *
 *  (at your option) any later version.                                      *
 *                                                                           *
 *  ACG Tracker is distributed in the hope that it will be useful,           *
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of           *
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the            *
 *  GNU General Public License for more details.                             *
 *                                                                           *
 *  You should have received a copy of the GNU General Public License        *
 *  along with ACG Tracker. If not, see <http://www.gnu.org/licenses/>.      *
 *                                                                           *
\*===========================================================================*/

#include <cassert>
#include <array>
#include <Accelerate/Accelerate.h>
#include <algorithm>

#include "EssentialMatrix3PtSolver.h"
#include "Utils/PolynomialSolver.h"

namespace ACGT
{
    // the function returns the 6x10 constraint matrix that contains the determinant and trace constraints
    // vt holds transpose(V) from the svd A=U*S*V^T (since A has rank 3 the last three rows of V^T contain the
    // basis vectors of the nullspace of the Longuet-Higgins constraint of the essential matrix E: (x',y',1)*E*(x,y,1)^T=0)
    void setupConstraintMatrix(const __CLPK_doublereal vt[36], __CLPK_doublereal c[60])
    {
        const __CLPK_doublereal e000 = vt[3+0*6];
        const __CLPK_doublereal e001 = vt[3+1*6];
        const __CLPK_doublereal e002 = vt[3+2*6];
        const __CLPK_doublereal e010 = vt[3+3*6];
        const __CLPK_doublereal e012 = vt[3+4*6];
        const __CLPK_doublereal e021 = vt[3+5*6];
        
        const __CLPK_doublereal e100 = vt[4+0*6];
        const __CLPK_doublereal e101 = vt[4+1*6];
        const __CLPK_doublereal e102 = vt[4+2*6];
        const __CLPK_doublereal e110 = vt[4+3*6];
        const __CLPK_doublereal e112 = vt[4+4*6];
        const __CLPK_doublereal e121 = vt[4+5*6];
        
        const __CLPK_doublereal e200 = vt[5+0*6];
        const __CLPK_doublereal e201 = vt[5+1*6];
        const __CLPK_doublereal e202 = vt[5+2*6];
        const __CLPK_doublereal e210 = vt[5+3*6];
        const __CLPK_doublereal e212 = vt[5+4*6];
        const __CLPK_doublereal e221 = vt[5+5*6];
        
        const __CLPK_doublereal e0012 = e001*e001;
        const __CLPK_doublereal e0102 = e010*e010;
        const __CLPK_doublereal e0122 = e012*e012;
        const __CLPK_doublereal e0212 = e021*e021;
        
        const __CLPK_doublereal e1012 = e101*e101;
        const __CLPK_doublereal e1102 = e110*e110;
        const __CLPK_doublereal e1122 = e112*e112;
        const __CLPK_doublereal e1212 = e121*e121;
        
        const __CLPK_doublereal e2012 = e201*e201;
        const __CLPK_doublereal e2102 = e210*e210;
        const __CLPK_doublereal e2122 = e212*e212;
        const __CLPK_doublereal e2212 = e221*e221;
        
        c[0+0*6] = e002*e010*e021 - e001*e002*e012 - e000*e012*e021 - e000*e001*e010;
        c[0+1*6] = e002*e010*e121 - e000*e010*e101 - e001*e010*e100 - e001*e002*e112 - e001*e012*e102 - e002*e012*e101 - e000*e012*e121 - e000*e021*e112 - e000*e001*e110 + e002*e021*e110 + e010*e021*e102 - e012*e021*e100;
        c[0+2*6] = e002*e010*e221 - e000*e010*e201 - e001*e010*e200 - e001*e002*e212 - e001*e012*e202 - e002*e012*e201 - e000*e012*e221 - e000*e021*e212 - e000*e001*e210 + e002*e021*e210 + e010*e021*e202 - e012*e021*e200;
        c[0+3*6] = e002*e110*e121 - e001*e100*e110 - e010*e100*e101 - e001*e102*e112 - e002*e101*e112 - e012*e101*e102 - e000*e112*e121 - e000*e101*e110 + e010*e102*e121 - e012*e100*e121 - e021*e100*e112 + e021*e102*e110;
        c[0+4*6] = e002*e110*e221 - e000*e110*e201 - e001*e100*e210 - e001*e110*e200 - e010*e100*e201 - e010*e101*e200 - e001*e102*e212 - e001*e112*e202 - e002*e101*e212 - e002*e112*e201 - e012*e101*e202 - e012*e102*e201 - e000*e112*e221 - e000*e121*e212 - e000*e101*e210 + e002*e121*e210 + e010*e102*e221 + e010*e121*e202 - e012*e100*e221 - e012*e121*e200 - e021*e100*e212 + e021*e102*e210 + e021*e110*e202 - e021*e112*e200;
        c[0+5*6] = e002*e210*e221 - e001*e200*e210 - e010*e200*e201 - e001*e202*e212 - e002*e201*e212 - e012*e201*e202 - e000*e212*e221 - e000*e201*e210 + e010*e202*e221 - e012*e200*e221 - e021*e200*e212 + e021*e202*e210;
        c[0+6*6] = e102*e110*e121 - e101*e102*e112 - e100*e112*e121 - e100*e101*e110;
        c[0+7*6] = e102*e110*e221 - e100*e110*e201 - e101*e110*e200 - e101*e102*e212 - e101*e112*e202 - e102*e112*e201 - e100*e112*e221 - e100*e121*e212 - e100*e101*e210 + e102*e121*e210 + e110*e121*e202 - e112*e121*e200;
        c[0+8*6] = e102*e210*e221 - e101*e200*e210 - e110*e200*e201 - e101*e202*e212 - e102*e201*e212 - e112*e201*e202 - e100*e212*e221 - e100*e201*e210 + e110*e202*e221 - e112*e200*e221 - e121*e200*e212 + e121*e202*e210;
        c[0+9*6] = e202*e210*e221 - e201*e202*e212 - e200*e212*e221 - e200*e201*e210;
        
        c[1+0*6] = e000*e0012 - 2.0*e002*e001*e021 + e000*e0102 + 2.0*e002*e010*e012 - e000*e0122 - e000*e0212;
        c[1+1*6] = e110*(2.0*e000*e010 + 2.0*e002*e012) - e002*(2.0*e001*e121 + 2.0*e021*e101) + e0012*e100 + e0102*e100 - e0122*e100 - e0212*e100 + 2.0*e000*e001*e101 - 2.0*e000*e012*e112 - 2.0*e001*e021*e102 + 2.0*e002*e010*e112 + 2.0*e010*e012*e102 - 2.0*e000*e021*e121;
        c[1+2*6] = e210*(2.0*e000*e010 + 2.0*e002*e012) - e002*(2.0*e001*e221 + 2.0*e021*e201) + e0012*e200 + e0102*e200 - e0122*e200 - e0212*e200 + 2.0*e000*e001*e201 - 2.0*e000*e012*e212 - 2.0*e001*e021*e202 + 2.0*e002*e010*e212 + 2.0*e010*e012*e202 - 2.0*e000*e021*e221;
        c[1+3*6] = e010*(2.0*e100*e110 + 2.0*e102*e112) - e102*(2.0*e001*e121 + 2.0*e021*e101) + e000*e1012 + e000*e1102 - e000*e1122 - e000*e1212 + 2.0*e001*e100*e101 - 2.0*e002*e101*e121 + 2.0*e002*e110*e112 - 2.0*e012*e100*e112 + 2.0*e012*e102*e110 - 2.0*e021*e100*e121;
        c[1+4*6] = e010*(2.0*e100*e210 + 2.0*e110*e200 + 2.0*e102*e212 + 2.0*e112*e202) - e102*(2.0*e001*e221 + 2.0*e021*e201) - e002*(2.0*e101*e221 + 2.0*e121*e201) - e202*(2.0*e001*e121 + 2.0*e021*e101) + 2.0*e000*e101*e201 + 2.0*e001*e100*e201 + 2.0*e001*e101*e200 + 2.0*e000*e110*e210 - 2.0*e000*e112*e212 + 2.0*e002*e110*e212 + 2.0*e002*e112*e210 - 2.0*e012*e100*e212 + 2.0*e012*e102*e210 + 2.0*e012*e110*e202 - 2.0*e012*e112*e200 - 2.0*e000*e121*e221 - 2.0*e021*e100*e221 - 2.0*e021*e121*e200;
        c[1+5*6] = e010*(2.0*e200*e210 + 2.0*e202*e212) - e202*(2.0*e001*e221 + 2.0*e021*e201) + e000*e2012 + e000*e2102 - e000*e2122 - e000*e2212 + 2.0*e001*e200*e201 - 2.0*e002*e201*e221 + 2.0*e002*e210*e212 - 2.0*e012*e200*e212 + 2.0*e012*e202*e210 - 2.0*e021*e200*e221;
        c[1+6*6] = e100*e1012 - 2.0*e102*e101*e121 + e100*e1102 + 2.0*e102*e110*e112 - e100*e1122 - e100*e1212;
        c[1+7*6] = e210*(2.0*e100*e110 + 2.0*e102*e112) - e102*(2.0*e101*e221 + 2.0*e121*e201) + e1012*e200 + e1102*e200 - e1122*e200 - e1212*e200 + 2.0*e100*e101*e201 - 2.0*e100*e112*e212 - 2.0*e101*e121*e202 + 2.0*e102*e110*e212 + 2.0*e110*e112*e202 - 2.0*e100*e121*e221;
        c[1+8*6] = e110*(2.0*e200*e210 + 2.0*e202*e212) - e202*(2.0*e101*e221 + 2.0*e121*e201) + e100*e2012 + e100*e2102 - e100*e2122 - e100*e2212 + 2.0*e101*e200*e201 - 2.0*e102*e201*e221 + 2.0*e102*e210*e212 - 2.0*e112*e200*e212 + 2.0*e112*e202*e210 - 2.0*e121*e200*e221;
        c[1+9*6] = e200*e2012 - 2.0*e202*e201*e221 + e200*e2102 + 2.0*e202*e210*e212 - e200*e2122 - e200*e2212;
        
        c[2+0*6] = e001*(e0012 - e0102 - e0122 + e0212);
        c[2+1*6] = 3.0*e101*e0012 - 2.0*e110*e001*e010 - 2.0*e112*e001*e012 + 2.0*e121*e001*e021 - e101*e0102 - e101*e0122 + e101*e0212;
        c[2+2*6] = 3.0*e201*e0012 - 2.0*e210*e001*e010 - 2.0*e212*e001*e012 + 2.0*e221*e001*e021 - e201*e0102 - e201*e0122 + e201*e0212;
        c[2+3*6] = 3.0*e001*e1012 - 2.0*e010*e101*e110 - 2.0*e012*e101*e112 + 2.0*e021*e101*e121 - e001*e1102 - e001*e1122 + e001*e1212;
        c[2+4*6] = e021*(2.0*e101*e221 + 2.0*e121*e201) + 6.0*e001*e101*e201 - 2.0*e001*e110*e210 - 2.0*e010*e101*e210 - 2.0*e010*e110*e201 - 2.0*e001*e112*e212 - 2.0*e012*e101*e212 - 2.0*e012*e112*e201 + 2.0*e001*e121*e221;
        c[2+5*6] = 3.0*e001*e2012 - 2.0*e010*e201*e210 - 2.0*e012*e201*e212 + 2.0*e021*e201*e221 - e001*e2102 - e001*e2122 + e001*e2212;
        c[2+6*6] = e101*(e1012 - e1102 - e1122 + e1212);
        c[2+7*6] = 3.0*e201*e1012 - 2.0*e210*e101*e110 - 2.0*e212*e101*e112 + 2.0*e221*e101*e121 - e201*e1102 - e201*e1122 + e201*e1212;
        c[2+8*6] = 3.0*e101*e2012 - 2.0*e110*e201*e210 - 2.0*e112*e201*e212 + 2.0*e121*e201*e221 - e101*e2102 - e101*e2122 + e101*e2212;
        c[2+9*6] = e201*(e2012 - e2102 - e2122 + e2212);
        
        c[3+0*6] = e002*e0012 + 2.0*e000*e001*e021 - e002*e0102 + 2.0*e000*e010*e012 + e002*e0122 - e002*e0212;
        c[3+1*6] = e112*(2.0*e000*e010 + 2.0*e002*e012) + e000*(2.0*e001*e121 + 2.0*e021*e101) + e0012*e102 - e0102*e102 + e0122*e102 - e0212*e102 + 2.0*e001*e002*e101 + 2.0*e000*e012*e110 + 2.0*e001*e021*e100 - 2.0*e002*e010*e110 + 2.0*e010*e012*e100 - 2.0*e002*e021*e121;
        c[3+2*6] = e212*(2.0*e000*e010 + 2.0*e002*e012) + e000*(2.0*e001*e221 + 2.0*e021*e201) + e0012*e202 - e0102*e202 + e0122*e202 - e0212*e202 + 2.0*e001*e002*e201 + 2.0*e000*e012*e210 + 2.0*e001*e021*e200 - 2.0*e002*e010*e210 + 2.0*e010*e012*e200 - 2.0*e002*e021*e221;
        c[3+3*6] = e100*(2.0*e001*e121 + 2.0*e021*e101) + e012*(2.0*e100*e110 + 2.0*e102*e112) + e002*e1012 - e002*e1102 + e002*e1122 - e002*e1212 + 2.0*e001*e101*e102 + 2.0*e000*e101*e121 + 2.0*e000*e110*e112 + 2.0*e010*e100*e112 - 2.0*e010*e102*e110 - 2.0*e021*e102*e121;
        c[3+4*6] = e200*(2.0*e001*e121 + 2.0*e021*e101) + e100*(2.0*e001*e221 + 2.0*e021*e201) + e000*(2.0*e101*e221 + 2.0*e121*e201) + e012*(2.0*e100*e210 + 2.0*e110*e200 + 2.0*e102*e212 + 2.0*e112*e202) + 2.0*e001*e101*e202 + 2.0*e001*e102*e201 + 2.0*e002*e101*e201 + 2.0*e000*e110*e212 + 2.0*e000*e112*e210 - 2.0*e002*e110*e210 + 2.0*e010*e100*e212 - 2.0*e010*e102*e210 - 2.0*e010*e110*e202 + 2.0*e010*e112*e200 + 2.0*e002*e112*e212 - 2.0*e002*e121*e221 - 2.0*e021*e102*e221 - 2.0*e021*e121*e202;
        c[3+5*6] = e200*(2.0*e001*e221 + 2.0*e021*e201) + e012*(2.0*e200*e210 + 2.0*e202*e212) + e002*e2012 - e002*e2102 + e002*e2122 - e002*e2212 + 2.0*e001*e201*e202 + 2.0*e000*e201*e221 + 2.0*e000*e210*e212 + 2.0*e010*e200*e212 - 2.0*e010*e202*e210 - 2.0*e021*e202*e221;
        c[3+6*6] = e102*e1012 + 2.0*e100*e101*e121 - e102*e1102 + 2.0*e100*e110*e112 + e102*e1122 - e102*e1212;
        c[3+7*6] = e212*(2.0*e100*e110 + 2.0*e102*e112) + e100*(2.0*e101*e221 + 2.0*e121*e201) + e1012*e202 - e1102*e202 + e1122*e202 - e1212*e202 + 2.0*e101*e102*e201 + 2.0*e100*e112*e210 + 2.0*e101*e121*e200 - 2.0*e102*e110*e210 + 2.0*e110*e112*e200 - 2.0*e102*e121*e221;
        c[3+8*6] = e200*(2.0*e101*e221 + 2.0*e121*e201) + e112*(2.0*e200*e210 + 2.0*e202*e212) + e102*e2012 - e102*e2102 + e102*e2122 - e102*e2212 + 2.0*e101*e201*e202 + 2.0*e100*e201*e221 + 2.0*e100*e210*e212 + 2.0*e110*e200*e212 - 2.0*e110*e202*e210 - 2.0*e121*e202*e221;
        c[3+9*6] = e202*e2012 + 2.0*e200*e201*e221 - e202*e2102 + 2.0*e200*e210*e212 + e202*e2122 - e202*e2212;
        
        c[4+0*6] = -e010*(e0012 - e0102 - e0122 + e0212);
        c[4+1*6] = - e110*e0012 - 2.0*e101*e001*e010 + 3.0*e110*e0102 + 2.0*e112*e010*e012 - 2.0*e121*e010*e021 + e110*e0122 - e110*e0212;
        c[4+2*6] = - e210*e0012 - 2.0*e201*e001*e010 + 3.0*e210*e0102 + 2.0*e212*e010*e012 - 2.0*e221*e010*e021 + e210*e0122 - e210*e0212;
        c[4+3*6] = - e010*e1012 - 2.0*e001*e101*e110 + 3.0*e010*e1102 + 2.0*e012*e110*e112 - 2.0*e021*e110*e121 + e010*e1122 - e010*e1212;
        c[4+4*6] = 6.0*e010*e110*e210 - 2.0*e001*e110*e201 - 2.0*e010*e101*e201 - 2.0*e001*e101*e210 + 2.0*e010*e112*e212 + 2.0*e012*e110*e212 + 2.0*e012*e112*e210 - 2.0*e010*e121*e221 - 2.0*e021*e110*e221 - 2.0*e021*e121*e210;
        c[4+5*6] = - e010*e2012 - 2.0*e001*e201*e210 + 3.0*e010*e2102 + 2.0*e012*e210*e212 - 2.0*e021*e210*e221 + e010*e2122 - e010*e2212;
        c[4+6*6] = -e110*(e1012 - e1102 - e1122 + e1212);
        c[4+7*6] = - e210*e1012 - 2.0*e201*e101*e110 + 3.0*e210*e1102 + 2.0*e212*e110*e112 - 2.0*e221*e110*e121 + e210*e1122 - e210*e1212;
        c[4+8*6] = - e110*e2012 - 2.0*e101*e201*e210 + 3.0*e110*e2102 + 2.0*e112*e210*e212 - 2.0*e121*e210*e221 + e110*e2122 - e110*e2212;
        c[4+9*6] = -e210*(e2012 - e2102 - e2122 + e2212);
        
        c[5+0*6] = -e012*(e0012 - e0102 - e0122 + e0212);
        c[5+1*6] = - e112*e0012 - 2.0*e101*e001*e012 + e112*e0102 + 2.0*e110*e010*e012 + 3.0*e112*e0122 - 2.0*e121*e012*e021 - e112*e0212;
        c[5+2*6] = - e212*e0012 - 2.0*e201*e001*e012 + e212*e0102 + 2.0*e210*e010*e012 + 3.0*e212*e0122 - 2.0*e221*e012*e021 - e212*e0212;
        c[5+3*6] = - e012*e1012 - 2.0*e001*e101*e112 + e012*e1102 + 2.0*e010*e110*e112 + 3.0*e012*e1122 - 2.0*e021*e112*e121 - e012*e1212;
        c[5+4*6] = 2.0*e010*e110*e212 - 2.0*e001*e112*e201 - 2.0*e012*e101*e201 - 2.0*e001*e101*e212 + 2.0*e010*e112*e210 + 2.0*e012*e110*e210 + 6.0*e012*e112*e212 - 2.0*e012*e121*e221 - 2.0*e021*e112*e221 - 2.0*e021*e121*e212;
        c[5+5*6] = - e012*e2012 - 2.0*e001*e201*e212 + e012*e2102 + 2.0*e010*e210*e212 + 3.0*e012*e2122 - 2.0*e021*e212*e221 - e012*e2212;
        c[5+6*6] = -e112*(e1012 - e1102 - e1122 + e1212);
        c[5+7*6] = - e212*e1012 - 2.0*e201*e101*e112 + e212*e1102 + 2.0*e210*e110*e112 + 3.0*e212*e1122 - 2.0*e221*e112*e121 - e212*e1212;
        c[5+8*6] = - e112*e2012 - 2.0*e101*e201*e212 + e112*e2102 + 2.0*e110*e210*e212 + 3.0*e112*e2122 - 2.0*e121*e212*e221 - e112*e2212;
        c[5+9*6] = -e212*(e2012 - e2102 - e2122 + e2212);
    }
    
    std::vector<Eigen::Matrix3d> solveE3Pt(const std::vector<Eigen::Vector3d>& NDCs1, const std::vector<Eigen::Vector3d>& NDCs2)
    {
        assert(NDCs1.size() == NDCs2.size());
        
        std::vector<Eigen::Matrix3d> candidates;
        
        // check if there are at least three correspondences
        if (NDCs1.size() < 3)
        {
            // cannot compute E
            return candidates;
        }
        
        char jobu = 'N';
        char jobvt = 'A';
        __CLPK_integer m = NDCs1.size();
        __CLPK_integer n = 6;
        std::vector<__CLPK_doublereal> a(NDCs1.size()*6);
        
        // store in each row of a: (x',xy'-x'y,xx'+yy',y',x,y)
        
        vDSP_vmulD((double*)NDCs1.data(), 3, (double*)NDCs2.data(), 3, a.data(), 1, NDCs1.size());
        for (int i=0; i<NDCs1.size(); i++)
            a[i] += 1.0;
        
        vDSP_vmulD((double*)NDCs1.data()+1, 3, (double*)NDCs2.data(), 3, a.data()+NDCs1.size(), 1, NDCs1.size());
        vDSP_vsubD((double*)NDCs1.data(), 3, (double*)NDCs2.data(), 3, a.data()+2*NDCs1.size(), 1, NDCs1.size());
        vDSP_vmulD((double*)NDCs1.data(), 3, (double*)NDCs2.data()+1, 3, a.data()+3*NDCs1.size(), 1, NDCs1.size());
        for (int i=0; i<NDCs1.size(); i++)
        {
            a[4*NDCs1.size()+i] = NDCs2[i].y();
            a[5*NDCs1.size()+i] = NDCs1[i].y();
        }
        
        __CLPK_integer lDA = NDCs1.size();
        std::vector<__CLPK_doublereal> sigma(NDCs1.size());
        std::vector<__CLPK_doublereal> u(NDCs1.size()*NDCs1.size());
        __CLPK_integer lDU = NDCs1.size();
        __CLPK_doublereal vt[36];
        __CLPK_integer lDVT = 6;
        
        __CLPK_integer lWork = 2.0*std::max((__CLPK_integer)1,std::max(3*std::min(m,n)+std::max(m,n),5*std::min(m,n)));
        std::vector<__CLPK_doublereal> work(lWork);
        __CLPK_integer info;
        
        dgesvd_(&jobu, // compute no columns of U
                &jobvt, // compute all columns of VT
                &m, // 3 rows, one for each correspondence
                &n, // 6 columns, one for each E_13, E_21, E_22, E_23, E_31, E_32
                a.data(), // matrix that should be decomposed
                &lDA, // leading dimension of a
                sigma.data(), // singular values (diagonal entries of Sigma)
                u.data(), // U
                &lDU, // leading dimension of u
                vt, // V^T
                &lDVT, // leading dimension of V^T
                work.data(), // workspace
                &lWork,
                &info);
        
        __CLPK_doublereal c[60];
        setupConstraintMatrix(vt,c);
        
        m = 6;
        n = 6;
        __CLPK_integer nrhs = 4;
        lDA = 6;
        __CLPK_integer lDB = 6;
        __CLPK_integer jptv[6] = {0,0,0,0,0,0};
        char cmach = 'E';
        __CLPK_doublereal rcond = slamch_(&cmach);
        __CLPK_integer rank;
        dgelsy_(&m, &m, &nrhs, c, &lDA, c+36 , &lDB, jptv, &rcond, &rank, work.data(), &lWork, &info);
        
        // Set up quartic <k>(b) = b^4 + B*b^3 + C*b^2 + D*b + E (see paper Frauendorfer et al.) and find its real roots:
        
        std::array<double,4> roots;
        int numRoots = PolynomialSolver::solveQuartic(roots.data(), c[5+36], c[11+36]-c[4+36], c[17+36]-c[10+36], c[23+36]-c[16+36], -c[22+36]);
        
        const __CLPK_doublereal e000 = vt[3+0*6];
        const __CLPK_doublereal e001 = vt[3+1*6];
        const __CLPK_doublereal e002 = vt[3+2*6];
        const __CLPK_doublereal e010 = vt[3+3*6];
        const __CLPK_doublereal e012 = vt[3+4*6];
        const __CLPK_doublereal e021 = vt[3+5*6];
        
        const __CLPK_doublereal e100 = vt[4+0*6];
        const __CLPK_doublereal e101 = vt[4+1*6];
        const __CLPK_doublereal e102 = vt[4+2*6];
        const __CLPK_doublereal e110 = vt[4+3*6];
        const __CLPK_doublereal e112 = vt[4+4*6];
        const __CLPK_doublereal e121 = vt[4+5*6];
        
        const __CLPK_doublereal e200 = vt[5+0*6];
        const __CLPK_doublereal e201 = vt[5+1*6];
        const __CLPK_doublereal e202 = vt[5+2*6];
        const __CLPK_doublereal e210 = vt[5+3*6];
        const __CLPK_doublereal e212 = vt[5+4*6];
        const __CLPK_doublereal e221 = vt[5+5*6];
        
        for (int i=0; i<numRoots; i++)
        {
            const double coeffB = roots[i];
            const double coeffA = -c[5+36]*coeffB*coeffB*coeffB - c[11+36]*coeffB*coeffB - c[17+36]*coeffB -c[23+36];
            
            Eigen::Matrix3d candidate;
            candidate <<  coeffA*e000+coeffB*e100+e200, coeffA*e001+coeffB*e101+e201, coeffA*e002+coeffB*e102+e202,
            coeffA*e010+coeffB*e110+e210,                          0.0, coeffA*e012+coeffB*e112+e212,
            -coeffA*e002-coeffB*e102-e202, coeffA*e021+coeffB*e121+e221, coeffA*e000+coeffB*e100+e200;
            
            candidates.push_back(candidate);
        }
        
        return candidates;
    }
}