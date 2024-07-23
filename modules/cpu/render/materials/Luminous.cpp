// Copyright 2009 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#include "Luminous.h"
#ifndef OSPRAY_TARGET_SYCL
// ispc
#include "render/materials/Luminous_ispc.h"
#endif

namespace ospray {
namespace pathtracer {

Luminous::Luminous(api::ISPCDevice &device)
    : AddStructShared(device.getDRTDevice(), device, FFO_MATERIAL_LUMINOUS)
{
#ifndef OSPRAY_TARGET_SYCL
  getSh()->super.getBSDF = reinterpret_cast<ispc::Material_GetBSDFFunc>(
      ispc::Luminous_getBSDF_addr());
  getSh()->super.getTransparency =
      reinterpret_cast<ispc::Material_GetTransparencyFunc>(
          ispc::Luminous_getTransparency_addr());
  getSh()->super.getEmission = reinterpret_cast<ispc::Material_GetEmissionFunc>(
      ispc::Luminous_getEmission_addr());
#endif
}

std::string Luminous::toString() const
{
  return "ospray::pathtracer::Luminous";
}

void Luminous::commit()
{
  emissiveColor = getMaterialParam3f("color", vec3f(1.f));
  const vec3f radiance =
      emissiveColor.factor * getParam<float>("intensity", 1.f);
  const float transparency = getParam<float>("transparency", 0.f);

  getSh()->radiance = radiance;
  getSh()->emissionMap = emissiveColor.tex;
  getSh()->transparency = transparency;
  getSh()->super.isEmissive = isEmissive();
}

} // namespace pathtracer
} // namespace ospray
