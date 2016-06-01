project(boundingmesh-lib)

# Offer the user the choice of overriding the installation directories
set(INSTALL_LIB_DIR lib CACHE PATH "Installation directory for libraries")
set(INSTALL_BIN_DIR bin CACHE PATH "Installation directory for executables")
set(INSTALL_INCLUDE_DIR include CACHE PATH
  "Installation directory for header files")
if(WIN32 AND NOT CYGWIN)
  # TODO : Is this directory name correct ?
  set(DEF_INSTALL_CMAKE_DIR CMake)
else()
  set(DEF_INSTALL_CMAKE_DIR lib/cmake/BoundingMesh)
endif()
set(INSTALL_CMAKE_DIR ${DEF_INSTALL_CMAKE_DIR} CACHE PATH
  "Installation directory for CMake files")

# Make relative paths absolute (needed later on)
foreach(p LIB BIN INCLUDE CMAKE)
  set(var INSTALL_${p}_DIR)
  if(NOT IS_ABSOLUTE "${${var}}")
    set(${var} "${CMAKE_INSTALL_PREFIX}/${${var}}")
  endif()
endforeach()

# Find the packages needed to compile boundingmesh
find_package(Eigen 3 REQUIRED)
find_package(Coin)
#find_package(GMP REQUIRED)
#find_package(CGAL)
#set(QHULL_USE_STATIC True)
find_package(QHULL)

#${CGAL_INCLUDE_DIRS} ${GMP_INCLUDE_DIRS}
set(BOUNDINGMESH_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR} ${EIGEN_INCLUDE_DIRS})
set(BOUNDINGMESHINSTALLED_INCLUDE_DIRS ${INSTALL_INCLUDE_DIR} ${EIGEN_INCLUDE_DIRS})


if(COIN_FOUND)
	add_definitions(-DCOIN_AVAILABLE -DCOIN_NOT_DLL)
	set(BOUNDINGMESH_INCLUDE_DIRS ${BOUNDINGMESH_INCLUDE_DIRS} ${COIN_INCLUDE_DIRS})
  set(BOUNDINGMESHINSTALLED_INCLUDE_DIRS ${BOUNDINGMESHINSTALLED_INCLUDE_DIRS} ${COIN_INCLUDE_DIRS})
endif(COIN_FOUND)

if(QHULL_FOUND)
	add_definitions(-DQHULL_AVAILABLE)
	set(BOUNDINGMESH_INCLUDE_DIRS ${BOUNDINGMESH_INCLUDE_DIRS} ${QHULL_INCLUDE_DIRS})
  set(BOUNDINGMESHINSTALLED_INCLUDE_DIRS ${BOUNDINGMESHINSTALLED_INCLUDE_DIRS} ${QHULL_INCLUDE_DIRS})
else(QHULL_FOUND)
	message("Not building boundingmesh-convexhull, would need QHULL library.")
endif(QHULL_FOUND)

set(BOUNDINGMESH_INCLUDE_DIRS ${BOUNDINGMESH_INCLUDE_DIRS} PARENT_SCOPE)
#set(BOUNDINGMESHINSTALLED_INCLUDE_DIRS ${BOUNDINGMESHINSTALLED_INCLUDE_DIRS} PARENT_SCOPE)

include_directories(
	${BOUNDINGMESH_INCLUDE_DIRS}
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")

add_library(
	boundingmesh
	STATIC
	Primitives.cpp
	Mesh.cpp
	Decimator.cpp
	MetricGenerator.cpp
	ContractionUtils.cpp
)

set(BOUNDINGMESH_LIBRARIES boundingmesh)

if(COIN_FOUND)
	set(BOUNDINGMESH_LIBRARIES ${BOUNDINGMESH_LIBRARIES} ${COIN_LIBRARIES})
endif(COIN_FOUND)

if(QHULL_FOUND)
	set(BOUNDINGMESH_LIBRARIES ${BOUNDINGMESH_LIBRARIES} ${QHULL_LIBRARIES})
endif(QHULL_FOUND)


set(BOUNDINGMESH_LIBRARIES ${BOUNDINGMESH_LIBRARIES} PARENT_SCOPE)

set_target_properties(boundingmesh PROPERTIES PUBLIC_HEADER "boundingmesh.h;Primitives.h;Mesh.h;Decimator.h")

install(TARGETS boundingmesh EXPORT BoundingMeshTargets ARCHIVE DESTINATION "${INSTALL_LIB_DIR}/static" PUBLIC_HEADER DESTINATION "${INSTALL_INCLUDE_DIR}/boundingmesh")

export(TARGETS boundingmesh FILE "${PROJECT_BINARY_DIR}/BoundingMeshTargets.cmake")
export(PACKAGE BoundingMesh)

set(CONF_LIBRARIES ${BOUNDINGMESH_LIBRARIES})
set(CONF_CXX_FLAGS ${CMAKE_CXX_FLAGS})
set(CONF_INCLUDE_DIRS "${BOUNDINGMESH_INCLUDE_DIRS}")
configure_file(BoundingMeshConfig.cmake.in "${PROJECT_BINARY_DIR}/BoundingMeshConfig.cmake" @ONLY)

set(CONF_INCLUDE_DIRS "${BOUNDINGMESHINSTALLED_INCLUDE_DIRS}")
configure_file(BoundingMeshConfig.cmake.in "${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/BoundingMeshConfig.cmake" @ONLY)

install(FILES "${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/BoundingMeshConfig.cmake" DESTINATION "${INSTALL_CMAKE_DIR}" COMPONENT dev)

install(EXPORT BoundingMeshTargets DESTINATION "${INSTALL_CMAKE_DIR}")
