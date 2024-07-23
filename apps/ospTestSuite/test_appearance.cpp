// Copyright 2020 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#include "test_appearance.h"
#include "ArcballCamera.h"
#include "ospray_testing.h"
#include "rkcommon/utility/multidim_index_sequence.h"

namespace OSPRayTestScenes {

Texture2D::Texture2D()
{
  rendererType = "pathtracer";
  samplesPerPixel = 8;
}

void Texture2D::SetUp()
{
  Base::SetUp();
  camera.setParam("position", vec3f(4.f, 3.f, 0.f));
  // flip image to have origin in upper left corner, plus mirror
  camera.setParam("imageStart", vec2f(1.f, 1.f));
  camera.setParam("imageEnd", vec2f(0.f, 0.f));
  auto params = GetParam();
  OSPTextureFilter filter = std::get<0>(params);

  // create (4*2) x 5 grid
  constexpr int cols = 8;
  constexpr int rows = 5;
  std::vector<vec3f> vertex;
  std::vector<vec2f> texcoord;
  rkcommon::index_sequence_2D iidx(vec2i(cols, rows));
  // Generate each quad, each needs its own vertex coordinates w/ unique
  // texcoords when we're testing w/ texture coordinates on
  for (auto i : iidx) {
    const int idx_start = vertex.size();
    vertex.push_back(vec3f(i.x, i.y * 1.2f, 5.3f));
    vertex.push_back(vec3f(i.x + 1.f, i.y * 1.2f, 5.3f));
    vertex.push_back(vec3f(i.x + 1.f, (i.y + 1.f) * 1.2f, 5.3f));
    vertex.push_back(vec3f(i.x, (i.y + 1.f) * 1.2f, 5.3f));

    texcoord.push_back(vec2f(0.f, 0.f));
    texcoord.push_back(vec2f(1.f, 0.f));
    texcoord.push_back(vec2f(1.f, 1.f));
    texcoord.push_back(vec2f(0.f, 1.f));
  }
  cpp::Geometry mesh("mesh");
  mesh.setParam("vertex.position", cpp::CopiedData(vertex));
  if (std::get<3>(params)) {
    mesh.setParam("vertex.texcoord", cpp::CopiedData(texcoord));
  }
  mesh.setParam("quadSoup", true);
  mesh.commit();

  // create textures:
  // columns=#channels, rows=type=[byte, byte, float, short, half]
  std::array<OSPTextureFormat, 4 *rows> format = {OSP_TEXTURE_R8,
      OSP_TEXTURE_RA8,
      OSP_TEXTURE_RGB8,
      OSP_TEXTURE_RGBA8,

      OSP_TEXTURE_L8,
      OSP_TEXTURE_LA8,
      OSP_TEXTURE_SRGB,
      OSP_TEXTURE_SRGBA,

      OSP_TEXTURE_R32F,
      OSP_TEXTURE_RA32F,
      OSP_TEXTURE_RGB32F,
      OSP_TEXTURE_RGBA32F,

      OSP_TEXTURE_R16,
      OSP_TEXTURE_RA16,
      OSP_TEXTURE_RGB16,
      OSP_TEXTURE_RGBA16,

      OSP_TEXTURE_R16F,
      OSP_TEXTURE_RA16F,
      OSP_TEXTURE_RGB16F,
      OSP_TEXTURE_RGBA16F};

  std::array<OSPDataType, 4 *rows> eltype = {OSP_UCHAR,
      OSP_VEC2UC,
      OSP_VEC3UC,
      OSP_VEC4UC,

      OSP_UCHAR,
      OSP_VEC2UC,
      OSP_VEC3UC,
      OSP_VEC4UC,

      OSP_FLOAT,
      OSP_VEC2F,
      OSP_VEC3F,
      OSP_VEC4F,

      OSP_USHORT,
      OSP_VEC2US,
      OSP_VEC3US,
      OSP_VEC4US,

      OSP_USHORT,
      OSP_VEC2US,
      OSP_VEC3US,
      OSP_VEC4US};

  std::array<vec4uc, 15> dbyte;
  std::array<vec4us, 15> dshort;
  std::array<vec4f, 15> dfloat;
  std::array<vec4us, 15> dhalf;
  rkcommon::index_sequence_2D didx(vec2i(3, 5));
  for (auto idx : didx) {
    auto i = didx.flatten(idx);
    // the center texel should be 127 / 32767 / 0.5, to test normal maps
    dbyte[i] = vec4uc(idx.x * 80 + 47, idx.y * 56 + 15, 204, i * 15 + 40);
    dshort[i] = vec4us(
        idx.x * 20480 + 12287, idx.y * 14336 + 4095, 52428, i * 3932 + 9830);
    dfloat[i] = vec4f(idx.x * 0.3125f + 0.1875f,
        idx.y * 0.21875f + 0.0625f,
        0.8f,
        i * 0.06f + 0.15f);
  }
  // float to half conversion
  uint32_t *pf = (uint32_t *)dfloat.data();
  uint16_t *ph = (uint16_t *)dhalf.data();
  for (int i = 0; i < 4 * 15; i++, pf++, ph++)
    *ph = (((*pf >> 23) - 112) << 10) | ((*pf >> 13) & 0x3ff);

  std::array<void *, rows> addr = {
      dbyte.data(), dbyte.data(), dfloat.data(), dshort.data(), dhalf.data()};
  std::array<int, rows> stride = {4, 4, 16, 8, 8};

  cpp::GeometricModel model(mesh);
  std::array<cpp::Material, cols * rows> material;
  for (auto idx : iidx) {
    auto i = iidx.flatten(idx);
    auto fmt = format[i / 2];
    auto eltp = eltype[i / 2];
    cpp::Texture tex("texture2d");
    tex.setParam("format", fmt);
    tex.setParam("filter", filter);
    auto tmp = ospNewSharedData(addr[idx.y], eltp, 3, stride[idx.y], 5);
    auto data = ospNewData(eltp, 3, 5);
    ospCopyData(tmp, data);
    ospRelease(tmp);
    tex.setParam("data", data);
    tex.commit();
    ospRelease(data);
    cpp::Material mat("obj");
    mat.setParam("kd", vec3f(0.8));
    mat.setParam(i & 1 ? "map_bump" : "map_kd", tex);
    mat.commit();
    material[i] = mat;
  }
  model.setParam("material", cpp::CopiedData(material));
  AddModel(model);

  { // set up backplate texture
    std::array<vec3uc, 125 * 94> bpdata;
    bool toggle = false;
    for (auto &el : bpdata) {
      el = toggle ? vec3uc(5, 10, 5) : vec3uc(10, 5, 10);
      toggle = !toggle;
    }
    cpp::CopiedData texData(bpdata.data(), vec2ul(125, 94));

    cpp::Texture backplateTex("texture2d");
    backplateTex.setParam("format", OSP_TEXTURE_RGB8);
    backplateTex.setParam("filter", OSP_TEXTURE_FILTER_NEAREST);
    backplateTex.setParam("data", texData);
    backplateTex.commit();

    renderer.setParam("map_backplate", backplateTex);
  }

  cpp::Light light1("distant");
  cpp::Light light2("distant");

  // two light sets
  if (std::get<2>(params)) {
    // default light
    light1.setParam("intensity", 3.f);
    // highlighting normal direction
    light2.setParam("direction", vec3f(0.7f, 0.2f, 0.f));
    light2.setParam("intensity", 5.f);
    light2.setParam("color", vec3f(0.f, 0.f, 0.5f));
  } else {
    // perpendicular bright lights, testing center normal
    light1.setParam("direction", vec3f(0.5f, -1.f, 0.f));
    light1.setParam("color", vec3f(999.f, 0.f, 0.0f));
    light2.setParam("direction", vec3f(-0.5f, 1.f, 0.f));
    light2.setParam("color", vec3f(0.f, 999.f, 0.0f));
  }

  AddLight(light1);
  AddLight(light2);

  renderer.setParam("mipMapBias", std::get<1>(params));
}

static cpp::Texture createTexture2D(uint32_t width = 32,
    uint32_t height = 32,
    vec2ui wrapMode = vec2ui(OSP_TEXTURE_WRAP_REPEAT),
    OSPTextureFilter filter = OSP_TEXTURE_FILTER_LINEAR)
{
  // Prepare texel data
  std::vector<vec3uc> dbyte(width * height);
  rkcommon::index_sequence_2D idx(vec2i(width, height));
  for (auto i : idx)
    dbyte[idx.flatten(i)] =
        vec3uc(i.x * (256 / width), i.y * (256 / height), i.x * (256 / width));

  // Create texture object
  cpp::Texture tex("texture2d");
  tex.setParam("format", OSP_TEXTURE_RGB8);
  tex.setParam("filter", filter);
  tex.setParam("wrapMode", wrapMode);
  tex.setParam("data", cpp::CopiedData(dbyte.data(), vec2ul(width, height)));
  tex.commit();
  return tex;
}

static cpp::Geometry createQuads(
    const int cols, const int rows, const float gap, const bool withTC = false)
{
  cpp::Geometry quads("mesh");
  {
    // Create quads data
    std::vector<vec3f> position(4 * cols * rows);
    rkcommon::index_sequence_2D idx(vec2i(cols, rows));

    for (auto i : idx) {
      auto l = static_cast<vec2f>(i) * gap;
      auto u = l + (.75f * gap);

      position[4 * idx.flatten(i) + 0] = vec3f(l.x, l.y, 0.f);
      position[4 * idx.flatten(i) + 1] = vec3f(l.x, u.y, 0.f);
      position[4 * idx.flatten(i) + 2] = vec3f(u.x, u.y, 0.f);
      position[4 * idx.flatten(i) + 3] = vec3f(u.x, l.y, 0.f);
    }

    // Set quads parameters
    quads.setParam(
        "vertex.position", cpp::CopiedData(position.data(), 4 * cols * rows));

    if (withTC) {
      std::vector<vec2f> texcoord(4 * cols * rows);

      for (auto i : idx) {
        texcoord[4 * idx.flatten(i) + 0] = vec2f(-1.f, -1.f);
        texcoord[4 * idx.flatten(i) + 1] = vec2f(-1.f, 3.f);
        texcoord[4 * idx.flatten(i) + 2] = vec2f(3.f, 3.f);
        texcoord[4 * idx.flatten(i) + 3] = vec2f(3.f, -1.f);
      }

      quads.setParam(
          "vertex.texcoord", cpp::CopiedData(texcoord.data(), 4 * cols * rows));
    }

    quads.setParam("quadSoup", true);
    quads.commit();
  }

  return quads;
}

Texture2DTransform::Texture2DTransform()
{
  rendererType = GetParam();
}

void Texture2DTransform::SetUp()
{
  Base::SetUp();

  camera.setParam("position", vec3f(4.f, 4.f, -10.f));
  camera.setParam("direction", vec3f(0.f, 0.f, 1.f));
  camera.setParam("up", vec3f(0.f, 1.f, 0.f));

  // Create quad geometry
  constexpr int cols = 2;
  constexpr int rows = 2;
  cpp::Geometry quads = createQuads(cols, rows, 5.f);

  // Create materials
  std::array<cpp::Material, cols * rows> materials;
  cpp::Texture tex = createTexture2D();
  for (int i = 0; i < cols * rows; i++) {
    cpp::Material mat("obj");
    mat.setParam("map_kd", tex);
    mat.commit();
    materials[i] = mat;
  }

  // Set scale
  materials[1].setParam("map_kd.scale", vec2f(.5f));
  materials[1].commit();

  // Set rotation
  materials[2].setParam("map_kd.rotation", 45.f);
  materials[2].commit();

  // Set translation
  materials[3].setParam("map_kd.translation", vec2f(.5f));
  materials[3].commit();

  // Create geometric model
  cpp::GeometricModel model(quads);
  model.setParam("material", cpp::CopiedData(materials));
  AddModel(model);

  cpp::Light ambient("ambient");
  ambient.setParam("intensity", 0.5f);
  AddLight(ambient);
}

Texture2DWrapMode::Texture2DWrapMode()
{
  rendererType = "scivis";
}

void Texture2DWrapMode::SetUp()
{
  Base::SetUp();

  auto params = GetParam();
  OSPTextureFilter filter = std::get<1>(params);

  camera.setParam("position", vec3f(4.f, 4.f, -8.f));
  camera.setParam("direction", vec3f(0.f, 0.f, 1.f));
  camera.setParam("up", vec3f(0.f, 1.f, 0.f));

  // Create quad geometry
  constexpr int cols = 3;
  constexpr int rows = 3;
  cpp::Geometry quads = createQuads(cols, rows, 3.f, true);

  // Create materials
  std::array<cpp::Material, cols * rows> materials;

  for (int i = 0; i < rows; i++) {
    for (int j = 0; j < cols; j++) {
      cpp::Texture tex = createTexture2D(4, 4, vec2ui(j, i), filter);
      cpp::Material mat("obj");
      mat.setParam("map_kd", tex);
      mat.commit();
      materials[(i * cols) + j] = mat;
    }
  }

  // Create geometric model
  cpp::GeometricModel model(quads);
  model.setParam("material", cpp::CopiedData(materials));
  AddModel(model);

  cpp::Light ambient("ambient");
  ambient.setParam("intensity", 0.5f);
  AddLight(ambient);

  renderer.setParam("mipMapBias", std::get<0>(params));
}

Texture2DMipMapping::Texture2DMipMapping()
{
  auto params = GetParam();
  rendererType = std::get<0>(params);
  samplesPerPixel = 4;
}

void Texture2DMipMapping::SetUp()
{
  Base::SetUp();

  auto params = GetParam();

  auto builder = ospray::testing::newBuilder("mip_map_textures");
  ospray::testing::setParam(builder, "filter", (uint32_t)std::get<2>(params));
  ospray::testing::setParam(builder, "tcScale", std::get<4>(params));
  ospray::testing::commit(builder);

  world = ospray::testing::buildWorld(builder);
  ospray::testing::release(builder);

  std::string cameraType = std::get<1>(params);
  camera = cpp::Camera(cameraType);
  if (cameraType == "perspective")
    camera.setParam("aspect", imgSize.x / (float)imgSize.y);
  if (cameraType == "orthographic")
    camera.setParam("height", 10.0f);
  camera.setParam(
      "position", vec3f(0.f, 0.f, cameraType == "panoramic" ? -4.f : -6.f));

  OSPStereoMode stereoMode = std::get<5>(params);
  if (cameraType == "perspective" || cameraType == "panoramic")
    camera.setParam("stereoMode", stereoMode);
  if (stereoMode == OSP_STEREO_TOP_BOTTOM) {
    camera.setParam("imageStart", vec2f(0.25f));
    camera.setParam("imageEnd", vec2f(0.75f));
  }

  renderer.setParam("mipMapBias", std::get<3>(params));

  cpp::Light distant("distant");
  distant.setParam("direction", vec3f(0.f, 0.f, 1.f));
  distant.setParam("intensity", 3.f);
  AddLight(distant);
}

RendererMaterialList::RendererMaterialList()
{
  rendererType = GetParam();
}

void RendererMaterialList::SetUp()
{
  Base::SetUp();

  // Setup geometry //

  cpp::Geometry sphereGeometry("sphere");

  constexpr int dimSize = 3;

  rkcommon::index_sequence_2D numSpheres(dimSize);

  std::vector<vec3f> spheres;
  std::vector<uint32_t> index;
  std::vector<cpp::Material> materials;

  auto makeObjMaterial = [](vec3f Kd, vec3f Ks) -> cpp::Material {
    cpp::Material mat("obj");
    mat.setParam("kd", Kd);
    mat.setParam("ks", Ks);
    mat.commit();

    return mat;
  };

  for (auto i : numSpheres) {
    auto i_f = static_cast<vec2f>(i);
    spheres.emplace_back(i_f.x, i_f.y, 0.f);

    auto l = i_f / (dimSize - 1);
    materials.push_back(
        makeObjMaterial(lerp(l.x, vec3f(0.1f), vec3f(0.f, 0.f, 1.f)),
            lerp(l.y, vec3f(0.f), vec3f(1.f))));

    index.push_back(static_cast<uint32_t>(numSpheres.flatten(i)));
  }

  sphereGeometry.setParam("sphere.position", cpp::CopiedData(spheres));
  sphereGeometry.setParam("radius", 0.4f);
  sphereGeometry.commit();

  cpp::GeometricModel model(sphereGeometry);
  model.setParam("material", cpp::CopiedData(index));

  AddModel(model);

  // Setup renderer material list //

  renderer.setParam("material", cpp::CopiedData(materials));

  // Create the world to get bounds for camera setup //

  if (!instances.empty())
    world.setParam("instance", cpp::CopiedData(instances));

  instances.clear();

  world.commit();

  auto worldBounds = world.getBounds<box3f>();

  ArcballCamera arcballCamera(worldBounds, imgSize);

  camera.setParam("position", arcballCamera.eyePos());
  camera.setParam("direction", arcballCamera.lookDir());
  camera.setParam("up", arcballCamera.upDir());

  // Setup lights //

  cpp::Light ambient("ambient");
  ambient.setParam("intensity", 0.5f);
  AddLight(ambient);
}

PTBackgroundRefraction::PTBackgroundRefraction()
{
  rendererType = "pathtracer";
  samplesPerPixel = 64;
}

void PTBackgroundRefraction::SetUp()
{
  bool backgroundRefraction = GetParam();

  Base::SetUp();

  // Setup geometry //
  cpp::Geometry boxGeometry("box");

  rkcommon::index_sequence_3D numBoxes(vec3i(2, 2, 1));

  std::vector<box3f> boxes;

  for (auto i : numBoxes) {
    auto f3 = static_cast<vec3f>(i);
    auto lower = f3 * vec3f(1.5f, 1.5f, 0.0f) + vec3f(-1.25f, -1.25f, 2.5f);
    auto upper = lower + vec3f(1.f, 1.f, 0.27f);
    boxes.emplace_back(lower, upper);
  }

  boxGeometry.setParam("box", cpp::CopiedData(boxes));
  boxGeometry.commit();

  cpp::GeometricModel model(boxGeometry);

  std::vector<cpp::Material> materials;
  materials.emplace_back(cpp::Material("thinGlass"));
  materials.emplace_back(cpp::Material("glass"));
  materials.emplace_back(cpp::Material("glass"));
  materials.back().setParam("eta", 1.2f);
  materials.emplace_back(cpp::Material("obj"));
  materials.back().setParam("d", 0.2f);
  materials.back().setParam("kd", vec3f(0.7f, 0.5f, 0.1f));
  for (auto &m : materials)
    m.commit();
  model.setParam("material", cpp::CopiedData(materials));
  AddModel(model);

  renderer.setParam("backgroundRefraction", backgroundRefraction);

  { // set up backplate texture
    std::vector<vec4uc> bpdata;
    bpdata.push_back(vec4uc(199, 60, 10, 255));
    bpdata.push_back(vec4uc(60, 199, 40, 255));
    bpdata.push_back(vec4uc(80, 40, 199, 255));
    bpdata.push_back(vec4uc(99, 10, 99, 255));

    cpp::CopiedData texData(bpdata.data(), vec2ul(2, 2));

    cpp::Texture backplateTex("texture2d");
    backplateTex.setParam("format", OSP_TEXTURE_RGBA8);
    backplateTex.setParam("data", texData);
    backplateTex.commit();

    renderer.setParam("map_backplate", backplateTex);
  }

  cpp::Light light("sunSky");
  light.setParam("turbidity", 8.0f);
  light.setParam("intensity", 0.005f);
  AddLight(light);
  cpp::Light mirrorlight("sunSky");
  mirrorlight.setParam("up", vec3f(0.0f, -1.0f, 0.0f));
  AddLight(mirrorlight);
}

// Test Instantiations //////////////////////////////////////////////////////
INSTANTIATE_TEST_SUITE_P(Transparency,
    FromOsprayTesting,
    ::testing::Combine(::testing::Values("transparency"),
        ::testing::Values("scivis", "pathtracer", "ao"),
        ::testing::Values(16)));

TEST_P(RendererMaterialList, material_list)
{
  PerformRenderTest();
}

INSTANTIATE_TEST_SUITE_P(MaterialLists,
    RendererMaterialList,
    ::testing::Values("scivis", "pathtracer"));

INSTANTIATE_TEST_SUITE_P(TestScenesPtMaterials,
    FromOsprayTesting,
    ::testing::Combine(::testing::Values("test_pt_alloy_roughness",
                           "test_pt_carpaint",
                           "test_pt_glass",
                           "test_pt_thinglass",
                           "test_pt_luminous",
                           "test_pt_material_tex",
                           "test_pt_metal_roughness",
                           "test_pt_metallic_flakes",
                           "test_pt_mix_tex",
                           "test_pt_obj",
                           "test_pt_plastic",
                           "test_pt_principled_metal",
                           "test_pt_principled_plastic",
                           "test_pt_principled_glass",
                           "test_pt_principled_tex",
                           "test_pt_velvet"),
        ::testing::Values("pathtracer"),
        ::testing::Values(64)));

TEST_P(Texture2D, filter)
{
  PerformRenderTest();
}

INSTANTIATE_TEST_SUITE_P(Appearance,
    Texture2D,
    ::testing::Combine(::testing::Values(OSP_TEXTURE_FILTER_LINEAR,
                           OSP_TEXTURE_FILTER_NEAREST),
        ::testing::Values(0.0f),
        ::testing::Bool(),
        ::testing::Bool()));

INSTANTIATE_TEST_SUITE_P(AppearanceMipMap,
    Texture2D,
    ::testing::Combine(::testing::Values(OSP_TEXTURE_FILTER_LINEAR,
                           OSP_TEXTURE_FILTER_NEAREST),
        ::testing::Values(6.0f),
        ::testing::Bool(),
        ::testing::Values(false)));

TEST_P(Texture2DTransform, simple)
{
  PerformRenderTest();
}

INSTANTIATE_TEST_SUITE_P(
    Appearance, Texture2DTransform, ::testing::Values("scivis"));

TEST_P(Texture2DWrapMode, wrap)
{
  PerformRenderTest();
}

INSTANTIATE_TEST_SUITE_P(Appearance,
    Texture2DWrapMode,
    ::testing::Combine(::testing::Values(0.0f, 5.0f),
        ::testing::Values(
            OSP_TEXTURE_FILTER_NEAREST, OSP_TEXTURE_FILTER_LINEAR)));

TEST_P(Texture2DMipMapping, mipMapping)
{
  PerformRenderTest();
}

INSTANTIATE_TEST_SUITE_P(MipMap,
    Texture2DMipMapping,
    ::testing::Combine(::testing::Values("scivis", "pathtracer", "ao"),
        ::testing::Values("perspective"),
        ::testing::Values(
            OSP_TEXTURE_FILTER_NEAREST, OSP_TEXTURE_FILTER_LINEAR),
        ::testing::Values(0.0f, 1.0f),
        ::testing::Values(1.0f),
        ::testing::Values(OSP_STEREO_NONE)));

INSTANTIATE_TEST_SUITE_P(MipMapScale,
    Texture2DMipMapping,
    ::testing::Combine(::testing::Values("pathtracer"),
        ::testing::Values("perspective"),
        ::testing::Values(OSP_TEXTURE_FILTER_LINEAR),
        ::testing::Values(1.0f),
        ::testing::Values(0.3f, 3.1f),
        ::testing::Values(OSP_STEREO_NONE)));

INSTANTIATE_TEST_SUITE_P(MipMapCamera,
    Texture2DMipMapping,
    ::testing::Combine(::testing::Values("pathtracer"),
        ::testing::Values("perspective", "orthographic", "panoramic"),
        ::testing::Values(OSP_TEXTURE_FILTER_LINEAR),
        ::testing::Values(1.0f),
        ::testing::Values(1.0f),
        ::testing::Values(OSP_STEREO_NONE, OSP_STEREO_TOP_BOTTOM)));

TEST_P(PTBackgroundRefraction, backgroundRefraction)
{
  PerformRenderTest();
}

INSTANTIATE_TEST_SUITE_P(Appearance, PTBackgroundRefraction, ::testing::Bool());

} // namespace OSPRayTestScenes
