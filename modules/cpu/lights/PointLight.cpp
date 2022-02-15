// Copyright 2009-2022 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#include "PointLight.h"
#include "lights/Light_ispc.h"
#include "lights/PointLight_ispc.h"

namespace ospray {

void *PointLight::createIE(const void *instance) const
{
  void *ie = ispc::PointLight_create();
  ispc::Light_set(ie, visible, (const ispc::Instance *)instance);
  ispc::PointLight_set(ie,
      (ispc::vec3f &)position,
      (ispc::vec3f &)radiance,
      (ispc::vec3f &)radIntensity,
      radius,
      intensityDistribution.data(),
      (const ispc::vec2i &)intensityDistribution.size,
      (ispc::vec3f &)direction,
      (const ispc::vec3f &)intensityDistribution.c0);
  return ie;
}

std::string PointLight::toString() const
{
  return "ospray::PointLight";
}

void PointLight::commit()
{
  Light::commit();
  position = getParam<vec3f>("position", vec3f(0.f));
  radius = getParam<float>("radius", 0.f);

  // per default perpendicular to direction
  direction = getParam<vec3f>("direction", vec3f(0.f, 0.f, 1.f));
  intensityDistribution.c0 = std::abs(direction.x) < std::abs(direction.y)
      ? vec3f(0.0f, direction.z, direction.y)
      : vec3f(direction.z, 0.0f, direction.x);
  intensityDistribution.readParams(*this);

  queryIntensityQuantityType(intensityDistribution
          ? OSP_INTENSITY_QUANTITY_SCALE
          : OSP_INTENSITY_QUANTITY_INTENSITY);
  processIntensityQuantityType();
}

void PointLight::processIntensityQuantityType()
{
  radIntensity = 0.0f;
  radiance = 0.0f;
  const float sphereArea = 4.0f * M_PI * radius * radius;

  // converting from the chosen intensity quantity type to radiance
  // (for r > 0) or radiative intensity (r = 0)
  if (intensityDistribution
          ? intensityQuantity == OSP_INTENSITY_QUANTITY_SCALE
          : intensityQuantity == OSP_INTENSITY_QUANTITY_INTENSITY) {
    radIntensity = coloredIntensity;
    if (radius > 0.0f) {
      // the visible surface are of a sphere in one direction is equal
      // to the surface area of a disk oriented to this direction
      radiance = coloredIntensity / (sphereArea / 4.0f);
    }
    return;
  }
  if (!intensityDistribution) {
    if (intensityQuantity == OSP_INTENSITY_QUANTITY_POWER) {
      radIntensity = coloredIntensity / (4.0f * M_PI);
      if (radius > 0.0f)
        radiance = coloredIntensity / (M_PI * sphereArea);
      return;
    }
    if (intensityQuantity == OSP_INTENSITY_QUANTITY_RADIANCE) {
      // a virtual point light has no surface area therefore
      // radIntensity stays zero
      if (radius > 0.0f)
        radiance = coloredIntensity;
      return;
    }
  }

  postStatusMsg(OSP_LOG_WARNING)
      << toString() << " unsupported 'intensityQuantity' value";
}

} // namespace ospray
