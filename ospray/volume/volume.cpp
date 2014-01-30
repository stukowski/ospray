#include "volume.h"
#include "volume_ispc.h"
#include "naive32_ispc.h"
#include "bricked32_ispc.h"

namespace ospray {
  long long volumeMagicCookie = 0x123456789012345LL;

  const char *scalarTypeName[] = {
    "unorm8", "float", "double", "unspecified"
  };
  const int scalarTypeSize[] = {
    1,4,8,0
  };

  template<>
  NaiveVolume<uint8>::NaiveVolume(const vec3i &size, const uint8 *internalData)
    : Volume(size,ospray::UNORM8)
  { 
    ispcPtr = ispc::_Naive32Volume1uc_create((ispc::vec3i&)size,internalData); 
  }
  template<>
  BrickedVolume<uint8>::BrickedVolume(const vec3i &size, const uint8 *internalData)
    : Volume(size,ospray::UNORM8)
  { 
    ispcPtr = ispc::_Bricked32Volume1uc_create((ispc::vec3i&)size,internalData); 
  }
  template<>
  NaiveVolume<float>::NaiveVolume(const vec3i &size, const float *internalData)
    : Volume(size,ospray::FLOAT)
  { 
    ispcPtr = ispc::_Naive32Volume1f_create((ispc::vec3i&)size,internalData); 
  }
  template<>
  BrickedVolume<float>::BrickedVolume(const vec3i &size, const float *internalData)
    : Volume(size,ospray::FLOAT)
  { 
    ispcPtr = ispc::_Bricked32Volume1f_create((ispc::vec3i&)size,internalData); 
  }


  template<>
  std::string NaiveVolume<uint8>::toString() const { return "NaiveVolume<uint8>"; }
  template<>
  std::string BrickedVolume<uint8>::toString() const { return "BrickedVolume<uint8>"; }
  template<>
  std::string NaiveVolume<float>::toString() const { return "NaiveVolume<float>"; }
  template<>
  std::string BrickedVolume<float>::toString() const { return "BrickedVolume<float>"; }


  template<>
  void NaiveVolume<uint8>::setRegion(const vec3i &where, 
                                     const vec3i &size, 
                                     const float *data)
  {
    uint8 t[long(size.x)*size.y*size.z];
    for (long i=0;i<long(size.x)*size.y*size.z;i++)
      t[i] = uint8(255.999f*data[i]);
    ispc::_Naive32Volume1uc_setRegion(ispcPtr,(const ispc::vec3i&)where,
                                       (const ispc::vec3i&)size,t);
  }

  template<>
  void BrickedVolume<uint8>::setRegion(const vec3i &where, 
                                     const vec3i &size, 
                                     const float *data)
  {
    uint8 t[long(size.x)*size.y*size.z];
    for (long i=0;i<long(size.x)*size.y*size.z;i++)
      t[i] = uint8(255.999f*data[i]);
    ispc::_Bricked32Volume1uc_setRegion(ispcPtr,(const ispc::vec3i&)where,
                                       (const ispc::vec3i&)size,t);
  }
  template<>
  void NaiveVolume<uint8>::setRegion(const vec3i &where, 
                                     const vec3i &size, 
                                     const uint8 *data)
  {
    ispc::_Naive32Volume1uc_setRegion(ispcPtr,(const ispc::vec3i&)where,
                                       (const ispc::vec3i&)size,data);
  }
  template<>
  void BrickedVolume<uint8>::setRegion(const vec3i &where, 
                                     const vec3i &size, 
                                     const uint8 *data)
  {
    ispc::_Bricked32Volume1uc_setRegion(ispcPtr,(const ispc::vec3i&)where,
                                       (const ispc::vec3i&)size,data);
  }





  template<>
  void NaiveVolume<float>::setRegion(const vec3i &where, 
                                     const vec3i &size, 
                                     const uint8 *data)
  {
    float t[long(size.x)*size.y*size.z];
    for (long i=0;i<long(size.x)*size.y*size.z;i++)
      t[i] = float(255.999f*data[i]);
    ispc::_Naive32Volume1f_setRegion(ispcPtr,(const ispc::vec3i&)where,
                                      (const ispc::vec3i&)size,t);
  }

  template<>
  void BrickedVolume<float>::setRegion(const vec3i &where, 
                                     const vec3i &size, 
                                     const uint8 *data)
  {
    float t[long(size.x)*size.y*size.z];
    for (long i=0;i<long(size.x)*size.y*size.z;i++)
      t[i] = float(255.999f*data[i]);
    ispc::_Bricked32Volume1f_setRegion(ispcPtr,(const ispc::vec3i&)where,
                                       (const ispc::vec3i&)size,t);
  }
  template<>
  void NaiveVolume<float>::setRegion(const vec3i &where, 
                                     const vec3i &size, 
                                     const float *data)
  {
    ispc::_Naive32Volume1f_setRegion(ispcPtr,(const ispc::vec3i&)where,
                                       (const ispc::vec3i&)size,data);
  }
  template<>
  void BrickedVolume<float>::setRegion(const vec3i &where, 
                                     const vec3i &size, 
                                     const float *data)
  {
    ispc::_Bricked32Volume1f_setRegion(ispcPtr,(const ispc::vec3i&)where,
                                       (const ispc::vec3i&)size,data);
  }





  template<typename T>
  void loadRaw(Volume *volume, FILE *file, const vec3i &size)
  {
    assert(volume);
    assert(file);
    assert(size.x > 0);
    T t[size.x];
    for (int z=0;z<size.z;z++)
      for (int y=0;y<size.y;y++) {
        int n = fread(t,sizeof(T),size.x,file);
        if (n != size.x)
          throw std::runtime_error("ospray::loadRaw: read incomplete data");
        volume->setRegion(vec3i(0,y,z),vec3i(size.x,1,1),t);
      }
  }

  Volume *loadRaw(const char *fileName, 
                  ScalarType inputType, long sx, long sy, long sz,
                  long flags)
  {
    const vec3i size(sx,sy,sz);
    FILE *file = fopen(fileName,"rb");
    if (!file)
      throw std::runtime_error("ospray::loadRaw(): could not open volume '"
                               +std::string(fileName)+"'");

    long dataSize = sx *sy * sz * scalarTypeSize[inputType];

    Volume *volume = NULL;
    switch (flags & Volume::LAYOUT) {
    case Volume::NAIVE: 
      {
        switch(inputType) {
        case ospray::UNORM8:
          volume = new NaiveVolume<uint8>(size);
          loadRaw<unsigned char>(volume,file,size); 
          break;
        default:
          throw std::runtime_error
            (std::string("ospray::loadRaw() not yet implemented for input data type ")
             +ospray::scalarTypeName[inputType]+" and naive layout");
        }; 
      } break;
    case Volume::BRICKED: 
      {
        switch(inputType) {
        case ospray::UNORM8:
          volume = new BrickedVolume<uint8>(size);
          loadRaw<unsigned char>(volume,file,size); 
          break;
        default:
          throw std::runtime_error
            (std::string("ospray::loadRaw() not yet implemented for input data type ")
             +ospray::scalarTypeName[inputType]+" and naive layout");
        }; 
      } break;
    default:
      throw std::runtime_error
        (std::string("ospray::loadRaw() not yet implemented given layout"));
    }
    fclose(file);
    return volume;
  }

  template<typename T>
  Volume *createVolume(const vec3i &size, 
                       const long layout,
                       const T *internal)
  {
    switch (layout & Volume::LAYOUT) {
    case Volume::BRICKED:
      return new BrickedVolume<T>(size,internal);
      break;
    case Volume::NAIVE:
      return new NaiveVolume<T>(size,internal);
      break;
    default:
      NOTIMPLEMENTED;
    }
  }

  Volume *createVolume(const vec3i &size, 
                       const ScalarType scalar,
                       const long layout,
                       const void *data = NULL)
  {
    switch (scalar) {
    case FLOAT:
      return createVolume<float>(size,layout,(const float*)data);
    default:
      NOTIMPLEMENTED;
    }
  }

  Volume *resampleVolume(Volume *src, const vec3i &newSize, const long flags)
  {
    Volume *volume = NULL;
    if ((flags & Volume::LAYOUT) == Volume::BRICKED)
      volume = new BrickedVolume<float>(newSize);
    else
      volume = new NaiveVolume<float>(newSize);
    ispc::resampleVolume(volume->inISPC(),src->inISPC());
    return volume;
  }



  struct VolumeTrailer {
    long  long magicCookie;
    ScalarType scalarType;
    int   layout;
    vec3i size;
  };
  void saveVolume(Volume *volume, const std::string &fileName)
  {
    assert(volume);
    assert(volume->inISPC());
    int64_t dataSize = 0;
    void *dataPtr = NULL;
    ispc::getInternalRepresentation(volume->inISPC(),dataSize,dataPtr);
    assert(dataSize > 0);
    assert(dataPtr != NULL);
    FILE *file = fopen(fileName.c_str(),"wb");
    if (!file) 
      throw std::runtime_error("could not open volume data file '"
                               +fileName+"' for writing");
    VolumeTrailer trailer;
    trailer.size = volume->size;
    trailer.layout = volume->layout();
    trailer.scalarType = volume->scalarType;
    trailer.magicCookie = volumeMagicCookie;
    int rc;
    rc = fwrite(dataPtr,dataSize,1,file);
    assert(rc == 1);
    rc = fwrite(&trailer,sizeof(trailer),1,file);
    assert(rc == 1);
    fclose(file);
  }

  Volume *loadVolume(const std::string &fileName)
  {
    FILE *file = fopen(fileName.c_str(),"rb");
    assert(file);

    VolumeTrailer trailer;
    fseek(file,-sizeof(trailer),SEEK_END);
    long long sz = ftell(file);
    PRINT(sz);
    fread(&trailer,sizeof(trailer),1,file);
    assert(trailer.magicCookie == volumeMagicCookie);
    PRINT(trailer.size);
    void *data = malloc(sz);
    fread(data,sz,1,file);
    Volume *volume
      = createVolume(trailer.size,trailer.scalarType,trailer.layout,data);
    fclose(file);
    return volume;
  }

  void WrapperVolume::commit()
  {
    /* this code current does all the loading internally in ospray -
       this should not be; see comments in WrapperVolume */
    const vec3i size = getParam3f("dimensions",vec3f(-1));
    Assert(size.x > 0);
    Assert(size.y > 0);
    Assert(size.z > 0);
    const char *fileName = getParamString("filename",NULL);
    Assert(fileName);

    internalRep = loadRaw(fileName,UNORM8,size.x,size.y,size.z,Volume::BRICKED);
    ispcEquivalent = internalRep->inISPC();
  }
}