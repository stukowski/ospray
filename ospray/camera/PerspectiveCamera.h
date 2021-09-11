// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "camera/Camera.h"
#include "rkcommon/math/box.h"

namespace ospray {

struct OSPRAY_SDK_INTERFACE PerspectiveCamera : public Camera
{
  PerspectiveCamera();
  virtual ~PerspectiveCamera() override = default;

  virtual std::string toString() const override;
  virtual void commit() override;

  // Project the bounding box to the screen. Does not support motion blur,
  // depth of field, or steroe. The projected box will be returned in normalized
  // [0, 1] coordinates in the framebuffer, the z coordinate will store the min
  // and max depth, in box.lower, box.upper respectively
  // TODO: we can support this in the future.
  box3f projectBox(const box3f &b) const;

  // Data members //

  float fovy;
  float aspect;
  float apertureRadius;
  float focusDistance;
  bool architectural; // orient image plane to be parallel to 'up' and shift the
                      // lens

  OSPStereoMode stereoMode;
  float interpupillaryDistance; // distance between the two cameras (stereo)
};

} // namespace ospray
