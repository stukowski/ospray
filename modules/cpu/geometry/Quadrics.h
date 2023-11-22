// Copyright 2019 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "Geometry.h"
// ispc shared
#include "QuadricsShared.h"

namespace ospray {

struct QuadricsCoeff {
    float a; float b; float c;
    float d; float e; float f;
    float g; float h; float i; float j;
};

struct OSPRAY_SDK_INTERFACE Quadrics
    : public AddStructShared<Geometry, ispc::Quadrics>
{
    Quadrics(api::ISPCDevice& device);
    virtual ~Quadrics() override = default;

    virtual std::string toString() const override;
    virtual void commit() override;
    virtual size_t numPrimitives() const override;

protected:
    Ref<const DataT<vec3f>> vertexData;
    Ref<const DataT<float>> coeffData;
    Ref<const DataT<float>> radiusData;
};

} // namespace ospray
