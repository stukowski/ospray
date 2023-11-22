// Copyright 2022 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "GeometryShared.h"

#ifdef __cplusplus
namespace ispc {
#endif // __cplusplus

struct Quadrics
{
    Geometry super;
    vec3f* vertex;
    float* coeff;
    float* radius;
};

#ifdef __cplusplus
} // namespace ispc
#endif // __cplusplus
