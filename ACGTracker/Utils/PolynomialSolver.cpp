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

#include <cmath>

namespace ACGT
{
    namespace PolynomialSolver
    {
        /*
         * Solves the linear equation a*x+b=0.
         * Returns the number of real solutions.
         * The real solution, if existent, is stored in the double at the solution pointer.
         * If there are an infinity number of solutions (a = b = 0) the function returns -1. In this case nothing is stored as the solution.
         */
        int solveLinear(double *roots, const double a, const double b)
        {
            if (a==0) {
                if (b==0) {
                    return -1;
                } else {
                    return 0;
                }
            }
            
            *roots = -b/a;
            return 1;
        }
        
        /*
         * Solves the quadratic equation a*x^2+b*x+c=0.
         * Returns the number of real solutions.
         * The real solutions are stored in the solutions array provided by the caller.
         * If there are an infinity number of solutions (a = b = c = 0) the function returns -1. In this case nothing is stored in the solutions array
         */
        int solveQuadratic(double *roots, const double a, const double b, const double c)
        {
            // check if equation is (at most) linear
            if (a==0) {
                return solveLinear(roots, b, c);
            }
            
            // not linear, transform to monic form x^2+p*x+q=0
            const double p = b/a;
            const double q = c/a;
            
            // compute discriminant
            const double D = p*p/4.0-q;
            
            if (D < 0.0) {
                // no real solution exists
                return 0;
            }
            
            if (D == 0.0) {
                // one real solution
                roots[0] = -p/2.0;
                return 1;
            }
            
            // two real solutions
            double sqrtD = sqrt(D);
            roots[0] = -p/2.0-sqrtD;
            roots[1] = -p/2.0+sqrtD;
            return 2;
        }
        
        // Finds a real non-negative root of a cubic (3rd degree) polynomial with Cardano's method.
        // The polynomial has the form f(x) = x^3+a*x^2+b*x+c .
        // The method returns the first real non-negative root it finds.
        // If no non-negative root exists, *root will be a negative root.
        void nnRootCubicPolynomial(double *root, const double a, const double b, const double c)
        {
            // Compute depressed cubic z^3 + p*z + q by substitution x = z - a/3
            const double p = b-a*a/3.0;
            const double q = 2.0/27.0*a*a*a-a*b/3.0+c;
            
            // solve equation z^3 + p*z + q = 0
            const double discriminant = 0.25*q*q+p*p*p/27.0;
            
            if (discriminant > 0) {
                // only one real solutions (and two non-real complex solutions)
                const double sqrtDis = sqrt(discriminant);
                *root = cbrt(-0.5*q+sqrtDis)+cbrt(-0.5*q-sqrtDis)-a/3.0;
            } else if (discriminant == 0) {
                // three real solutions
                *root = cbrt(-4.0*q)-a/3.0;
                if (*root < 0) {
                    *root = cbrt(0.5*q)-a/3.0;
                }
            } else {
                // casus irreducibilis: three real solutions
                const double tmp1 = sqrt(-4.0/3.0*p);
                const double tmp2 = acos(-0.5*q*sqrt(-27.0/(p*p*p)))/3.0;
                *root = tmp1*cos(tmp2)-a/3.0;
                if (*root < 0) {
                    *root = -tmp1*cos(tmp2+M_PI/3.0)-a/3.0;
                    if (*root < 0) {
                        *root = -tmp1*cos(tmp2-M_PI/3.0)-a/3.0;
                    }
                }
            }
        }
        
        // Finds all real roots of a quartic (4rd degree) polynomial by factorization of the quartic into a product of two quadratic (2nd degree) polynomials.
        // The quartic polynomial has the form f(x) = a*x^4+b*x^3+c*x^2+d*x+e .
        // The method returns the number of real roots of the quartic polynomial.
        // The real roots are stored in the array roots.
        // The caller has to pass a double array of sufficient size (at least 4).
        int solveQuartic(double *roots, double a, double b, double c, double d, double e)
        {
            if (a == 0.0) {
                return 0;
            }
            // normalize
            b = b/a;
            c = c/a;
            d = d/a;
            e = e/a;
            
            // Compute depressed quartic z^4 + k*z^2 + l*z + m by substitution x = z - b/4
            const double k = -0.375*b*b+c;
            const double l = 0.125*b*b*b-0.5*b*c+d;
            const double m = -0.01171875*b*b*b*b+0.0625*b*b*c-0.25*b*d+e;
            
            int numRealRoots = 0;
            if (l==0) {
                // biquadratic equation
                const double discriminant = 0.25*k*k-m;
                if (discriminant > 0) {
                    const double sqrtDis = sqrt(discriminant);
                    double squaredRoot = -0.5*k-sqrtDis;
                    if (squaredRoot > 0) {
                        roots[numRealRoots++] = sqrt(squaredRoot)-0.25*b;
                        roots[numRealRoots++] = -sqrt(squaredRoot)-0.25*b;
                    } else if (squaredRoot == 0) {
                        roots[numRealRoots++] = -0.25*b;
                    }
                    squaredRoot = -0.5*k+sqrtDis;
                    if (squaredRoot > 0) {
                        roots[numRealRoots++] = sqrt(squaredRoot)-0.25*b;
                        roots[numRealRoots++] = -sqrt(squaredRoot)-0.25*b;
                    } else if (squaredRoot == 0) {
                        roots[numRealRoots++] = -0.25*b;
                    }
                } else if (discriminant == 0) {
                    const double squaredRoot = -0.5*k;
                    if (squaredRoot > 0) {
                        roots[numRealRoots++] = sqrt(squaredRoot)-0.25*b;
                        roots[numRealRoots++] = -sqrt(squaredRoot)-0.25*b;
                    } else if (squaredRoot == 0) {
                        roots[numRealRoots++] = -0.25*b;
                    }
                }
                return numRealRoots;
            }
            
            // Factorize depressed quartic into <k'> = (z^2+p*z+q)*(z^2+r*z+s)
            // To find p,q,r,s compute roots of cubic equation P^3 + x*P^2 + y*P + z = 0:
            double p,q,r,s;
            nnRootCubicPolynomial(&p, 2.0*k, k*k-4.0*m, -l*l);
            p = sqrt(p);
            r = -p;
            s = (k+p*p+l/p)/2.0;
            q = (k+p*p-l/p)/2.0;
            
            // find real roots of first quadratic polynom z^2+p*z+q
            double discriminant = 0.25*p*p-q;
            if (discriminant > 0) {
                const double sqrtDis = sqrt(discriminant);
                roots[numRealRoots++] = -0.5*p-sqrtDis-0.25*b;
                roots[numRealRoots++] = -0.5*p+sqrtDis-0.25*b;
            } else if (discriminant == 0) {
                roots[numRealRoots++] = -0.5*p-0.25*b;
            }
            
            // find real roots of second quadratic polynom z^2+r*z+s
            discriminant = 0.25*r*r-s;
            if (discriminant > 0) {
                const double sqrtDis = sqrt(discriminant);
                roots[numRealRoots++] = -0.5*r-sqrtDis-0.25*b;
                roots[numRealRoots++] = -0.5*r+sqrtDis-0.25*b;
            } else if (discriminant == 0) {
                roots[numRealRoots++] = -0.5*r-0.25*b;
            }
            return numRealRoots;
        }
    }
}