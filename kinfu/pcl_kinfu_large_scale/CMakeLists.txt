find_package(CUDA REQUIRED)
message(STATUS "kinfu: CUDA ${CUDA_VERSION} found.")

include(${CMAKE_CURRENT_SOURCE_DIR}/remove_vtk_definitions_hack.cmake)
REMOVE_VTK_DEFINITIONS()

set(KinfuLargeScale_INCLUDE_DIRS
  ${CMAKE_CURRENT_SOURCE_DIR}/containers/include
  ${CMAKE_CURRENT_SOURCE_DIR}/utils/include
  ${CMAKE_CURRENT_SOURCE_DIR}/kinfu_large_scale/include
  ${CMAKE_CURRENT_SOURCE_DIR}/kinfu_large_scale/src
  )

set(KinfuLargeScale_INCLUDE_DIRS
  ${KinfuLargeScale_INCLUDE_DIRS}
  ${CUDA_INCLUDE_DIRS}
  PARENT_SCOPE
  )

include_directories(
  ${KinfuLargeScale_INCLUDE_DIRS}
  )

include_directories(
  ${PCL_INCLUDE_DIRS}
  ${CUDA_INCLUDE_DIRS}
  )

link_directories(${PCL_LIBRARY_DIRS})

add_library(
  KinfuLargeScaleCPP
  SHARED
  containers/src/device_memory.cpp
  containers/src/error.cpp
  containers/src/initialization.cpp

  kinfu_large_scale/src/color_volume.cpp
  kinfu_large_scale/src/cyclical_buffer.cpp
  kinfu_large_scale/src/kinfu.cpp
  kinfu_large_scale/src/marching_cubes.cpp
  kinfu_large_scale/src/raycaster.cpp
  kinfu_large_scale/src/screenshot_manager.cpp
  kinfu_large_scale/src/standalone_marching_cubes.cpp
  kinfu_large_scale/src/tsdf_volume.cpp
  kinfu_large_scale/src/world_model.cpp
  )

target_link_libraries(KinfuLargeScaleCPP
  ${PCL_LIBRARIES}
  ${CUDA_LIBRARIES}
  )

set(CUDA_NVCC_FLAGS "-D_FORCE_INLINES -gencode arch=compute_30,code=sm_30 -gencode arch=compute_35,code=sm_35 -gencode arch=compute_50,code=sm_50 -gencode arch=compute_52,code=sm_52 -gencode arch=compute_60,code=sm_60 -gencode arch=compute_61,code=sm_61 -gencode arch=compute_62,code=sm_62")
set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS} "--ftz=true --prec-div=false --prec-sqrt=false")	
set(CUDA_VERBOSE_BUILD OFF)
cuda_add_library(KinfuLargeScaleCUDA
  SHARED 
  kinfu_large_scale/src/cuda/bilateral_pyrdown.cu
  kinfu_large_scale/src/cuda/colors.cu
  kinfu_large_scale/src/cuda/coresp.cu
  kinfu_large_scale/src/cuda/estimate_combined.cu
  kinfu_large_scale/src/cuda/estimate_tranform.cu
  kinfu_large_scale/src/cuda/extract.cu
  kinfu_large_scale/src/cuda/image_generator.cu
  kinfu_large_scale/src/cuda/maps.cu
  kinfu_large_scale/src/cuda/marching_cubes.cu
  kinfu_large_scale/src/cuda/normals_eigen.cu
  kinfu_large_scale/src/cuda/pointer_shift.cu
  kinfu_large_scale/src/cuda/push.cu
  kinfu_large_scale/src/cuda/ray_caster.cu
  kinfu_large_scale/src/cuda/tsdf_volume.cu
  )

target_link_libraries(KinfuLargeScaleCUDA
  ${PCL_LIBRARIES}
  ${CUDA_LIBRARIES}
  )

set(KinfuLargeScale_LIBRARIES
  KinfuLargeScaleCPP
  KinfuLargeScaleCUDA
  ${PCL_LIBRARIES}
  ${CUDA_LIBRARIES}
  PARENT_SCOPE)
