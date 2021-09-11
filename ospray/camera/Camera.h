// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "common/MotionTransform.h"
#include "common/Util.h"

namespace ospray {

//! base camera class abstraction
/*! the base class itself does not do anything useful; look into
    perspectivecamera etc for that */
struct OSPRAY_SDK_INTERFACE Camera : public MotionTransform
{
  Camera();
  ~Camera() override;

  std::string toString() const override;
  void commit() override;

  static Camera *createInstance(const char *identifier);
  template <typename T>
  static void registerType(const char *type);

  // Data members //

  // if motionBlur in local camera space; otherwise in world-space:
  vec3f pos; // position of the camera
  vec3f dir; // main direction of the camera
  vec3f up; // up direction of the camera

  float nearClip{1e-6f}; // near clipping distance
  // definition of the image region, may even be outside of [0..1]^2
  // to simulate sensor shift
  vec2f imageStart; // lower left corner
  vec2f imageEnd; // upper right corner
  range1f shutter{0.5f, 0.5f}; // start and end time of camera shutter time

  void setDevice(RTCDevice embreeDevice);

 private:
  template <typename BASE_CLASS, typename CHILD_CLASS>
  friend void registerTypeHelper(const char *type);
  static void registerType(const char *type, FactoryFcn<Camera> f);
  RTCDevice embreeDevice{nullptr};
  RTCGeometry embreeGeometry{nullptr};
};

OSPTYPEFOR_SPECIALIZATION(Camera *, OSP_CAMERA);

// Inlined defintions /////////////////////////////////////////////////////////

template <typename T>
inline void Camera::registerType(const char *type)
{
  registerTypeHelper<Camera, T>(type);
}

} // namespace ospray
