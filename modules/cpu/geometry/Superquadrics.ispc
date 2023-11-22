// Copyright 2019 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

// ospray
#include "common/Data.ih"
#include "common/DifferentialGeometry.ih"
#include "common/Ray.ih"
#include "geometry/Geometry.ih"
#include "rkcommon/math/box.ih"
#include "rkcommon/math/vec.ih"
// c++ shared
#include "SuperquadricsShared.h"

OSPRAY_BEGIN_ISPC_NAMESPACE

const int PLANECOUNT = 9;
const vec4f planes[9] = {
	{1.0, 1.0, 0.0, 0.0},
	{1.0,-1.0, 0.0, 0.0},
	{1.0, 0.0, 1.0, 0.0},
	{1.0, 0.0,-1.0, 0.0},
	{0.0, 1.0, 1.0, 0.0},
	{0.0, 1.0,-1.0, 0.0},
	{1.0, 0.0, 0.0, 0.0},
	{0.0, 1.0, 0.0, 0.0},
	{0.0, 0.0, 1.0, 0.0}};

const float EPSILON = 1.0e-10;
const float MIN_VALUE = -1.01;
const float MAX_VALUE =  1.01;
const float BOUND_HUGE = 2.0e+10;
const float DEPTH_TOLERANCE = 1.0e-4; // Minimal intersection depth for a valid intersection.
const float ZERO_TOLERANCE = 1.0e-10; // If |x| < ZERO_TOLERANCE, x is regarded to be 0.
const int   MAX_ITERATIONS = 20;

static void Superquadrics_postIntersect(const Geometry* uniform geometry, varying DifferentialGeometry& dg, const varying Ray& ray, uniform int64 flags)
{
	Superquadrics* uniform self = (Superquadrics* uniform)geometry;

	dg.Ng = dg.Ns = ray.Ng;

	// make epsilon large enough to not get lost when computing
	// |CO| = |center-ray.org| ~ radius for 2ndary rays
	dg.epsilon = self->radius[ray.primID] * ulpEpsilon;
}

export void Superquadrics_bounds(const RTCBoundsFunctionArguments *uniform args)
{
	Superquadrics* uniform self = (Superquadrics* uniform)args->geometryUserPtr;
	uniform int primID = args->primID;
	uniform float radius = self->radius[primID];
	uniform vec3f center = self->vertex[primID];
	box3fa *uniform out = (box3fa* uniform) args->bounds_o;
	*out = make_box3fa(center - radius, center + radius);
}

/// Intersect a ray with an axis aligned unit box.
bool intersect_box(const vec3f& P, const vec3f& D, float& dmin, float& dmax)
{
    float tmin = 0.0, tmax = 0.0;

    // Left/right.
    if(abs(D.x) > EPSILON) {
        if(D.x > EPSILON) {
            dmin = (MIN_VALUE - P.x) / D.x;
            dmax = (MAX_VALUE - P.x) / D.x;
        }
        else {
            dmax = (MIN_VALUE - P.x) / D.x;
            dmin = (MAX_VALUE - P.x) / D.x;
        }

        if(dmin > dmax) return false;
    }
    else {
        if((P.x < MIN_VALUE) || (P.x > MAX_VALUE))
			return false;
        dmin = -BOUND_HUGE;
        dmax =  BOUND_HUGE;
    }

    // Top/bottom.
    if(abs(D.y) > EPSILON) {
        if(D.y > EPSILON) {
            tmin = (MIN_VALUE - P.y) / D.y;
            tmax = (MAX_VALUE - P.y) / D.y;
        }
        else {
            tmax = (MIN_VALUE - P.y) / D.y;
            tmin = (MAX_VALUE - P.y) / D.y;
        }

        if(tmax < dmax) {
            if(tmin > dmin) {
                if(tmin > tmax) return false;
                dmin = tmin;
            }
            else {
                if(dmin > tmax) return false;
            }

            dmax = tmax;
        }
        else {
            if(tmin > dmin) {
                if(tmin > dmax) return false;
                dmin = tmin;
            }
        }
    }
    else {
        if((P.y < MIN_VALUE) || (P.y > MAX_VALUE)) {
            return false;
        }
    }

    // Front/back.
    if(abs(D.z) > EPSILON) {
        if(D.z > EPSILON) {
            tmin = (MIN_VALUE - P.z) / D.z;
            tmax = (MAX_VALUE - P.z) / D.z;
        }
        else {
            tmax = (MIN_VALUE - P.z) / D.z;
            tmin = (MAX_VALUE - P.z) / D.z;
        }

        if(tmax < dmax) {
            if(tmin > dmin) {
                if(tmin > tmax) return false;
                dmin = tmin;
            }
            else {
                if(dmin > tmax) return false;
            }

            dmax = tmax;
        }
        else {
            if(tmin > dmin) {
                if(tmin > dmax) return false;
                dmin = tmin;
            }
        }
    }
    else {
        if((P.z < MIN_VALUE) || (P.z > MAX_VALUE)) {
            return false;
        }
    }

    return true;
}

float evaluate_g(float x, float y, float e)
{
    float g = 0.0;

    if(x > y) {
        g = 1.0 + pow(y / x, e);
        if(g != 1.0)
            g = pow(g, 1.0 / e);
        g *= x;
    }
    else if(y != 0.0) {
        g = 1.0 + pow(x / y, e);
        if(g != 1.0)
            g = pow(g, 1.0 / e);
        g *= y;
    }

    return g;
}

/// Computes the superellipsoid value at the given location.
float evaluate_superellipsoid(const vec3f& P, const uniform vec2f& exponent)
{
    return evaluate_g(evaluate_g(abs(P.x), abs(P.y), exponent.x), abs(P.z), exponent.y) - 1.0f;
}

// Home in on the root of a superquadric using a combination of secant and bisection methods.
// This routine requires that the sign of the function be different at P0 and P1, it will fail drastically if this isn't the case.
void solve_hit1(float v0, const vec3f& tP0, float v1, const vec3f& tP1, vec3f& P, const uniform vec2f& exponent)
{
    int i;
    float x, v2, v3;
    vec3f P0, P1, P2, P3;

    P0 = tP0;
    P1 = tP1;

    // The sign of v0 and v1 changes between P0 and P1, this means there is an intersection point in there somewhere.
    for(i = 0; i < MAX_ITERATIONS; i++) {
        if(abs(v0) < ZERO_TOLERANCE) {
            // Near point is close enough to an intersection - just use it.
            P = P0;
            break;
        }

        if(abs(v1) < ZERO_TOLERANCE) {
            // Far point is close enough to an intersection.
            P = P1;
            break;
        }

        // Look at the chord connecting P0 and P1.
        // Assume a line between the points.
        x = abs(v0) / abs(v1 - v0);
        P2 = P1 - P0;
        P2 = P0 + x * P2;
        v2 = evaluate_superellipsoid(P2, exponent);

        // Look at the midpoint between P0 and P1.
        P3 = P1 - P0;
        P3 = P0 + 0.5 * P3;
        v3 = evaluate_superellipsoid(P3, exponent);

        if(v2 * v3 < 0.0) {
            // We can move both ends.
            v0 = v2;
            P0 = P2;
            v1 = v3;
            P1 = P3;
        }
        else {
            if(abs(v2) < abs(v3)) {
                // secant method is doing better.
                if(v0 * v2 < 0.0) {
                    v1 = v2;
                    P1 = P2;
                }
                else {
                    v0 = v2;
                    P0 = P2;
                }
            }
            else {
                // bisection method is doing better.
                if(v0 * v3 < 0.0) {
                    v1 = v3;
                    P1 = P3;
                }
                else {
                    v0 = v3;
                    P0 = P3;
                }
            }
        }
    }

    if(i == MAX_ITERATIONS) {
        // The loop never quite closed in on the result - just use the point
        // closest to zero.  This really shouldn't happen since the max number
        // of iterations is enough to converge with straight bisection.
        if(abs(v0) < abs(v1)) {
            P = P0;
        }
        else {
            P = P1;
        }
    }
}

/// Try to find the root of a superquadric using Newtons method.
bool check_hit2(const vec3f& P, const vec3f& D, float t0, vec3f& P0, float v0, float t1, float& t, vec3f& Q, const uniform vec2f& exponent)
{
    int i;
    float dt0, dt1, v1, deltat, maxdelta;

	const float eps = 1.0e-5f;

    dt0 = t0;
    dt1 = t0 + 1.0e-4f * (t1 - t0);
    maxdelta = t1 - t0;

    for(i = 0; (dt0 < t1) && (i < MAX_ITERATIONS); i++) {
        vec3f P1 = P + dt1 * D;
        v1 = evaluate_superellipsoid(P1, exponent);

        if(v0 * v1 < 0.0f) {
            // Found a crossing point, go back and use normal root solving.
            solve_hit1(v0, P0, v1, P1, Q, exponent);
            P0 = Q - P;
            t = length(P0);
            return true;
        }
        else {
            if(abs(v1) < eps) {
                Q = P + dt1 * D;
                t = dt1;
                return true;
            }
            else {
                if(((v0 > 0.0) && (v1 > v0)) || ((v0 < 0.0) && (v1 < v0))) {
                    // We definitely failed.
                    break;
                }
                else {
                    if(v1 == v0) {
                        break;
                    }
                    else {
                        deltat = v1 * (dt1 - dt0) / (v1 - v0);
                    }
                }
            }
        }

        if(abs(deltat) > maxdelta) {
            break;
		}

        v0 = v1;
        dt0 = dt1;
        dt1 -= deltat;
    }

    return false;
}

void Superquadrics_intersect_kernel(const RTCIntersectFunctionNArguments* uniform args, const uniform bool isOcclusionTest)
{
	// Make sure to set the mask
	if(!args->valid[programIndex])
		return;

	Superquadrics* uniform self = (Superquadrics* uniform)args->geometryUserPtr;
	uniform int primID = args->primID;

	// This assumes that the args->rayhit is actually a pointer to a varying ray!
	varying Ray* uniform ray = (varying Ray* uniform)args->rayhit;

	uniform float radius = self->radius[primID];
	uniform vec3f center = self->vertex[primID];
	uniform linear3f tensor = self->tensor[primID];
	uniform vec2f exponent = self->exponent[primID];

    // Transform the ray into the superellipsoid space.
	vec3f P = tensor * (ray->org - center);
	vec3f D = tensor * ray->dir;
	float len = length(D);
	D = D / len;

    // Intersect bounding box.
	float t1, t2;
    if(!intersect_box(P, D, t1, t2)) {
		return;
	}

    // Test if superellipsoid lies 'behind' the ray origin.
	if(t2 < DEPTH_TOLERANCE)
		return;
	if(t1 < DEPTH_TOLERANCE)
		t1 = DEPTH_TOLERANCE;

	int cnt = 2;
    float dists[9 + 2];
    dists[0] = t1;
    dists[1] = t2;

    // Intersect ray with planes cutting superellipsoids in pieces.
	// Find all the places where the ray intersects the set of
	// subdividing planes through the superquadric.  Return the
	// number of valid hits (within the bounding box).

    // Since min and max dist are the distance to two of the bounding planes
    // we are considering, there is a high probablity of missing them due to
    // round off error. Therefore we adjust min and max.
    float margin = EPSILON * (t2 - t1);
    float mindist = t1 - margin;
    float maxdist = t2 + margin;

    // Check the sets of planes that cut apart the superquadric.
	int i;
    for(i = 0; i < PLANECOUNT; i++) {
        float d = D.x * planes[i].x + D.y * planes[i].y + D.z * planes[i].z;

        if(abs(d) < EPSILON)
            continue; // Can't possibly get a hit for this combination of ray and plane.

        float t = (planes[i].w - (P.x * planes[i].x + P.y * planes[i].y + P.z * planes[i].z)) / d;

        if((t >= mindist) && (t <= maxdist)) {
            dists[cnt++] = t;
        }
    }

    // Sort the results for further processing.
	// Todo: Replace this bubble sort implemention with something more efficient.
	bool done;
	do {
		done = true;
		for(i = 1; i < cnt; i++) {
			if(dists[i] < dists[i-1]) {
				float temp = dists[i];
				dists[i] = dists[i-1];
				dists[i-1] = temp;
				done = false;
			}
		}
	}
	while(!done);

	vec3f P0 = P + dists[0] * D;
	float v0 = evaluate_superellipsoid(P0, exponent);

	float tnear = BOUND_HUGE;
	if(abs(v0) < ZERO_TOLERANCE) {
		tnear = dists[0] / len;
	}
	else {
		for(i = 1; i < cnt; i++) {
			vec3f P1 = P + dists[i] * D;
			float v1 = evaluate_superellipsoid(P1, exponent);

			if(abs(v1) < ZERO_TOLERANCE) {
				tnear = dists[i] / len;
				break;
			}
			else {
				if(v0 * v1 < 0.0) {
					// Opposite signs: there must be a root between.
					vec3f P2;
					solve_hit1(v0, P0, v1, P1, P2, exponent);

					vec3f P3 = P2 - P;
					float t = length(P3);

					tnear = t / len;
					break;
				}
				else {
					// Although there was no sign change, we may actually be approaching
					// the surface. In this case, we are being fooled by the shape of the
					// surface into thinking there isn't a root between sample points.
					float t;
					vec3f P2;
					if(check_hit2(P, D, dists[i-1], P0, v0, dists[i], t, P2, exponent)) {
						tnear = t / len;
						break;
					}
				}
			}

			v0 = v1;
			P0 = P1;
		}
	}

	// Discard intersections behind the view point.
	if(tnear == BOUND_HUGE || tnear <= ray->t0 || tnear >= ray->t) {
		return;
	}

	ray->t = tnear;

	if(isOcclusionTest) {
		ray->t = neg_inf;
	}
	else {
		ray->primID = primID;
		ray->geomID = args->geomID;
		ray->instID = args->context->instID[0];

	    // Intersection point in the superellipsoid space.
		P = P + D * (tnear * len);

		// Calculate surface normal in superellipsoid space.
		float r, z2n = 0.0;
		if(P.z != 0.0) {
			z2n = pow(abs(P.z), exponent.y);
	        P.z = z2n / P.z;
		}

		if(abs(P.x) > abs(P.y)) {
			r = pow(abs(P.y / P.x), exponent.x);
			P.x = (1.0 - z2n) / P.x;
			P.y = (P.y != 0.0) ? (1.0 - z2n) * r / P.y : 0.0;
		}
		else if(P.y != 0.0) {
			r = pow(abs(P.x / P.y), exponent.x);
			P.x = (P.x != 0.0) ? (1.0 - z2n) * r / P.x : 0.0;
			P.y = (1.0 - z2n) / P.y;
		}

		if(P.z != 0.0)
			P.z *= (1.0 + r);

		// Transform the normal from superellipsoid space to view space.
		ray->Ng = normalize(transposed(tensor) * P);
	}
}

export void *uniform Superquadrics_intersect_addr()
{
  return Superquadrics_intersect_kernel;
}

export void *uniform Superquadrics_postIntersect_addr()
{
  return Superquadrics_postIntersect;
}

OSPRAY_END_ISPC_NAMESPACE
