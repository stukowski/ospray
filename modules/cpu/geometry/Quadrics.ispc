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
#include "QuadricsShared.h"

OSPRAY_BEGIN_ISPC_NAMESPACE

static void Quadrics_postIntersect(const Geometry* uniform geometry, varying DifferentialGeometry& dg, const varying Ray& ray, uniform int64 flags)
{
	Quadrics* uniform self = (Quadrics* uniform)geometry;

	dg.Ng = dg.Ns = ray.Ng;

	// make epsilon large enough to not get lost when computing
	// |CO| = |center-ray.org| ~ radius for 2ndary rays
	dg.epsilon = self->radius[ray.primID] * ulpEpsilon;
}

export void Quadrics_bounds(const RTCBoundsFunctionArguments *uniform args)
{
	Quadrics* uniform self = (Quadrics* uniform)args->geometryUserPtr;
	uniform int primID = args->primID;
	uniform float radius = self->radius[primID];
	uniform vec3f center = self->vertex[primID];
	box3fa *uniform out = (box3fa* uniform) args->bounds_o;
	*out = make_box3fa(center - radius, center + radius);
}

SYCL_EXTERNAL unmasked void Quadrics_intersect_kernel(const RTCIntersectFunctionNArguments* uniform args, const uniform bool isOcclusionTest)
{
	// Make sure to set the mask
	if(!args->valid[programIndex])
		return;
	args->valid[programIndex] = 0;

	Quadrics* uniform self = (Quadrics* uniform)args->geometryUserPtr;
	uniform int primID = args->primID;

	// This assumes that the args->rayhit is actually a pointer to a varying ray!
	varying Ray* uniform ray = (varying Ray* uniform)args->rayhit;

	uniform float radius = self->radius[primID];
	uniform vec3f center = self->vertex[primID];
	uniform int primID_10 = primID * 10;
	uniform float coeff_a = self->coeff[primID_10 + 0];
	uniform float coeff_b = self->coeff[primID_10 + 1];
	uniform float coeff_c = self->coeff[primID_10 + 2];
	uniform float coeff_d = self->coeff[primID_10 + 3];
	uniform float coeff_e = self->coeff[primID_10 + 4];
	uniform float coeff_f = self->coeff[primID_10 + 5];
	uniform float coeff_g = self->coeff[primID_10 + 6];
	uniform float coeff_h = self->coeff[primID_10 + 7];
	uniform float coeff_i = self->coeff[primID_10 + 8];
	uniform float coeff_j = self->coeff[primID_10 + 9];

	// This epsilon leads to border artifacts when rendering semi-transparent ellipsoids,
	// because the ray hits the front-facing side of the ellipsoid but not its back-facing
	// side when the ray is close to the edge of the ellipsoid.
	uniform float epsilon = log(radius);
	if(epsilon < 0.f) epsilon = -1.f / epsilon;

	// This is to reduce the border artifacts somewhat
	// (but not too much, because we may risk self-intersections of secondary rays):
	epsilon *= 0.1f;

	vec3f ro = ray->org - center;

	float Aq = (coeff_a*(ray->dir.x * ray->dir.x)) +
		(2.0f * coeff_b * ray->dir.x * ray->dir.y) +
		(2.0f * coeff_c * ray->dir.x * ray->dir.z) +
		(coeff_e * (ray->dir.y * ray->dir.y)) +
		(2.0f * coeff_f * ray->dir.y * ray->dir.z) +
		(coeff_h * (ray->dir.z * ray->dir.z));

	float Bq = 2.0f * (
		(coeff_a * ro.x * ray->dir.x) +
		(coeff_b * ((ro.x * ray->dir.y) + (ray->dir.x * ro.y))) +
		(coeff_c * ((ro.x * ray->dir.z) + (ray->dir.x * ro.z))) +
		(coeff_d * ray->dir.x) +
		(coeff_e * ro.y * ray->dir.y) +
		(coeff_f * ((ro.y * ray->dir.z) + (ray->dir.y * ro.z))) +
		(coeff_g * ray->dir.y) +
		(coeff_h * ro.z * ray->dir.z) +
		(coeff_i * ray->dir.z));

	float Cq = (coeff_a * (ro.x * ro.x)) +
		(2.0f * coeff_b * ro.x * ro.y) +
		(2.0f * coeff_c * ro.x * ro.z) +
		(2.0f * coeff_d * ro.x) +
		(coeff_e * (ro.y * ro.y)) +
		(2.0f * coeff_f * ro.y * ro.z) +
		(2.0f * coeff_g * ro.y) +
		(coeff_h * (ro.z * ro.z)) +
		(2.0f * coeff_i * ro.z) +
		coeff_j;

	bool hit = false;
	if(Aq == 0.0f) {
		float t1 = - Cq / Bq;
		if(t1 > ray->t0 && t1 < ray->t) {
			hit = true;
			ray->t = t1;
		}
	}
	else {
		float disc = (Bq * Bq - 4.0f * Aq * Cq);
		if(disc > 0.0f) {
			disc = sqrt(disc);
			float t_out = (-Bq + disc) / (2.0 * Aq);
			float t_in = (-Bq - disc) / (2.0 * Aq);
			if(t_in > ray->t0 && t_in < ray->t) {
				hit = true;
				ray->t = t_in;
			}
			else if(t_out > (ray->t0 + epsilon) && t_out < ray->t) {
				hit = true;
				ray->t = t_out;
			}
		}
	}
	if(hit) {				
		if (isOcclusionTest) {	
			ray->t = neg_inf;
		}
		else {
			ray->primID = primID;
			ray->geomID = args->geomID;
			ray->instID = args->context->instID[0];
			vec3f pnt = ray->org + ray->t * ray->dir;

			ray->Ng.x = (coeff_a*(pnt.x - center.x) +
			coeff_b*(pnt.y - center.y) +
			coeff_c*(pnt.z - center.z) + coeff_d);

			ray->Ng.y = (coeff_b*(pnt.x - center.x) +
			coeff_e*(pnt.y - center.y) +
			coeff_f*(pnt.z - center.z) + coeff_g);

			ray->Ng.z = (coeff_c*(pnt.x - center.x) +
			coeff_f*(pnt.y - center.y) +
			coeff_h*(pnt.z - center.z) + coeff_i);

			ray->Ng = normalize(ray->Ng);
			ray->u = 0.f;
          	ray->v = 0.f;
			args->valid[programIndex] = -1;
		}
	}
}

export void *uniform Quadrics_intersect_addr()
{
  return Quadrics_intersect_kernel;
}

export void *uniform Quadrics_postIntersect_addr()
{
  return Quadrics_postIntersect;
}

OSPRAY_END_ISPC_NAMESPACE
