// Copyright 2019 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

// ospray
#include "Superquadrics.h"
#include "common/Data.h"
#ifndef OSPRAY_TARGET_SYCL
// ispc-generated files
#include "geometry/Superquadrics_ispc.h"
#else
namespace ispc {
void Quadrics_bounds(const RTCBoundsFunctionArguments *args);
}
#endif

namespace ospray {

Superquadrics::Superquadrics(api::ISPCDevice& device) : AddStructShared(device.getIspcrtContext(), device, FFG_USER_GEOMETRY)
{
#ifndef OSPRAY_TARGET_SYCL
    getSh()->super.postIntersect = ispc::Superquadrics_postIntersect_addr();
    getSh()->super.intersect = ispc::Superquadrics_intersect_addr();
#endif
}

std::string Superquadrics::toString() const
{
    return "ospray::Superquadrics";
}

void Superquadrics::commit()
{
    vertexData = getParamDataT<vec3f>("superquadric.center", true);
    tensorData = getParamDataT<LinearSpace3f>("superquadric.tensor", true);
    exponentData = getParamDataT<vec2f>("superquadric.exponent", true);
    radiusData = getParamDataT<float>("superquadric.radius", true);

    if(!vertexData || !vertexData->compact())
        throw std::runtime_error("Superquadrics needs compact 'center' data!");
    if(!tensorData || !tensorData->compact())
        throw std::runtime_error("Superquadrics needs compact 'tensor' data!");
    if(!exponentData || !exponentData->compact())
        throw std::runtime_error("Superquadrics needs compact 'exponent' data!");
    if(!radiusData || !radiusData->compact())
        throw std::runtime_error("Superquadrics needs compact 'radius' data!");

    createEmbreeUserGeometry((RTCBoundsFunction)&ispc::Superquadrics_bounds);

    getSh()->vertex = vertexData->data();
    getSh()->tensor = tensorData->data();
    getSh()->exponent = exponentData->data();
    getSh()->radius = radiusData->data();
    getSh()->super.numPrimitives = numPrimitives();

    postCreationInfo(numPrimitives());
}

size_t Superquadrics::numPrimitives() const
{
    return vertexData ? vertexData->size() : 0;
}

} // namespace ospray
