// Copyright 2019 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

// ospray
#include "Quadrics.h"
#include "common/Data.h"
#ifndef OSPRAY_TARGET_SYCL
// ispc-generated files
#include "geometry/Quadrics_ispc.h"
#else
namespace ispc {
void Quadrics_bounds(const RTCBoundsFunctionArguments *args);
}
#endif

namespace ospray {

Quadrics::Quadrics(api::ISPCDevice& device) : AddStructShared(device.getIspcrtContext(), device, FeatureFlagsGeometry(RTC_FEATURE_FLAG_FILTER_FUNCTION_IN_GEOMETRY))
{
#ifndef OSPRAY_TARGET_SYCL
    getSh()->super.postIntersect = ispc::Quadrics_postIntersect_addr();
    getSh()->super.intersect = ispc::Quadrics_intersect_addr();
#endif
}

std::string Quadrics::toString() const
{
    return "ospray::Quadrics";
}

void Quadrics::commit()
{
    vertexData = getParamDataT<vec3f>("quadric.center", true);
    coeffData = getParamDataT<float>("quadric.coeff", true);
    radiusData = getParamDataT<float>("quadric.radius", true);

    if(!vertexData || !vertexData->compact())
        throw std::runtime_error("Quadrics needs compact 'center' data!");
    if(!coeffData || !coeffData->compact())
        throw std::runtime_error("Quadrics needs compact 'coeff' data!");
    if(!radiusData || !radiusData->compact())
        throw std::runtime_error("Quadrics needs compact 'radius' data!");

    createEmbreeUserGeometry((RTCBoundsFunction)&ispc::Quadrics_bounds);

    getSh()->vertex = vertexData->data();
    getSh()->coeff = coeffData->data();
    getSh()->radius = radiusData->data();
    getSh()->super.numPrimitives = numPrimitives();

    postCreationInfo(numPrimitives());
}

size_t Quadrics::numPrimitives() const
{
    return vertexData ? vertexData->size() : 0;
}

} // namespace ospray
