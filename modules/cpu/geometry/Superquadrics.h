// Copyright 2019 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "Geometry.h"
// ispc shared
#include "SuperquadricsShared.h"

namespace ospray {

struct OSPRAY_SDK_INTERFACE Superquadrics
    : public AddStructShared<Geometry, ispc::Superquadrics>
{
    Superquadrics(api::ISPCDevice& device);
    virtual ~Superquadrics() override = default;

    virtual std::string toString() const override;
    virtual void commit() override;
    virtual size_t numPrimitives() const override;

protected:
    Ref<const DataT<vec3f>> vertexData;
    Ref<const DataT<LinearSpace3f>> tensorData;
    Ref<const DataT<vec2f>> exponentData;
    Ref<const DataT<float>> radiusData;
};

} // namespace ospray
