// Copyright 2009 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#include "PathTracer.h"
#include "PathTracerData.h"
#include "camera/Camera.h"
#include "common/FeatureFlagsEnum.h"
#include "common/World.h"
#include "fb/FrameBuffer.h"
#include "geometry/GeometricModel.h"
#include "lights/Light.h"
#include "render/Material.h"

#ifdef OSPRAY_TARGET_SYCL
#include <sycl/sycl.hpp>
#include "common/FeatureFlags.ih"
namespace ispc {
SYCL_EXTERNAL void PathTracer_renderTask(Renderer *uniform _self,
    FrameBuffer *uniform fb,
    Camera *uniform camera,
    World *uniform world,
    const uint32 *uniform taskIDs,
    const sycl::nd_item<3> taskIndex,
    const uniform FeatureFlagsHandler &ffh);
}
#else
// ispc exports
#include "math/Distribution1D_ispc.h"
#include "render/pathtracer/PathTracer_ispc.h"
#endif

namespace ospray {

PathTracer::PathTracer(api::ISPCDevice &device)
    : AddStructShared(device.getIspcrtContext(), device)
{}

std::string PathTracer::toString() const
{
  return "ospray::PathTracer";
}

void PathTracer::commit()
{
  Renderer::commit();

  getSh()->rouletteDepth = getParam<int>("roulettePathLength", 5);
  getSh()->maxScatteringEvents = getParam<int>("maxScatteringEvents", 20);
  getSh()->maxRadiance = getParam<float>("maxContribution", inf);
  getSh()->numLightSamples = getParam<int>("lightSamples", -1);

  // Set shadow catcher plane
  const vec4f shadowCatcherPlane =
      getParam<vec4f>("shadowCatcherPlane", vec4f(0.f));
  const vec3f normal = vec3f(shadowCatcherPlane);
  const float l = length(normal);
  getSh()->shadowCatcher = l > 0.f;
  const float rl = rcp(l);
  getSh()->shadowCatcherPlane = vec4f(normal * rl, shadowCatcherPlane.w * rl);

  scanForGeometryLights = getParam<bool>("geometryLights", true);
  getSh()->backgroundRefraction = getParam<bool>("backgroundRefraction", false);
}

void *PathTracer::beginFrame(FrameBuffer *, World *world)
{
  if (!world)
    return nullptr;

  if (!world->pathtracerData
      || (scanForGeometryLights
          && !world->pathtracerData->scannedForGeometryLights)) {
    // Create PathTracerData object
    std::unique_ptr<PathTracerData> pathtracerData =
        rkcommon::make_unique<PathTracerData>(
            *world, scanForGeometryLights, *this);

    world->getSh()->pathtracerData = pathtracerData->getSh();
    world->pathtracerData = std::move(pathtracerData);
  }

  return nullptr;
}

AsyncEvent PathTracer::renderTasks(FrameBuffer *fb,
    Camera *camera,
    World *world,
    void *,
    const utility::ArrayView<uint32_t> &taskIDs,
    bool wait) const
{
  AsyncEvent event;
  auto *rendererSh = getSh();
  auto *fbSh = fb->getSh();
  auto *cameraSh = camera->getSh();
  auto *worldSh = world->getSh();
  const size_t numTasks = taskIDs.size();

#ifdef OSPRAY_TARGET_SYCL
  const uint32_t *taskIDsPtr = taskIDs.data();
  event = device.getSyclQueue().submit([&](sycl::handler &cgh) {
    FeatureFlags ff = world->getFeatureFlags();
    if (world->pathtracerData->getSh()->numGeoLights)
      ff.other |= FFO_LIGHT_GEOMETRY;
    ff |= featureFlags;
    ff |= fb->getFeatureFlags();
    ff |= camera->getFeatureFlags();
    cgh.set_specialization_constant<ispc::specFeatureFlags>(ff);

    cgh.parallel_for(fb->getDispatchRange(numTasks),
        [=](sycl::nd_item<3> taskIndex, sycl::kernel_handler kh) {
          ispc::FeatureFlagsHandler ffh(kh);
          ispc::PathTracer_renderTask(&rendererSh->super,
              fbSh,
              cameraSh,
              worldSh,
              taskIDsPtr,
              taskIndex,
              ffh);
        });
  });

  if (wait)
    event.wait_and_throw();
#else
  (void)wait;
  ispc::PathTracer_renderTasks(
      &rendererSh->super, fbSh, cameraSh, worldSh, taskIDs.data(), numTasks);
#endif
  return event;
}

} // namespace ospray
