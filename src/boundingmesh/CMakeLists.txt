project(boundingmesh-lib)

find_package(Eigen 3 REQUIRED)
find_package(Coin)
#find_package(GMP REQUIRED)
#find_package(CGAL)
#set(QHULL_USE_STATIC True)
find_package(QHULL)

#${CGAL_INCLUDE_DIRS} ${GMP_INCLUDE_DIRS}
set(BOUNDINGMESH_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})

if(COIN_FOUND)
	add_definitions(-DCOIN_AVAILABLE -DCOIN_NOT_DLL)
	set(BOUNDINGMESH_INCLUDE_DIRS ${BOUNDINGMESH_INCLUDE_DIRS} ${COIN_INCLUDE_DIRS})
endif(COIN_FOUND)

if(QHULL_FOUND)
	add_definitions(-DQHULL_AVAILABLE)
	set(BOUNDINGMESH_INCLUDE_DIRS ${BOUNDINGMESH_INCLUDE_DIRS} ${QHULL_INCLUDE_DIRS})
else(QHULL_FOUND)
	message("Not building boundingmesh-convexhull, would need QHULL library.")
endif(QHULL_FOUND)

set(BOUNDINGMESH_INCLUDE_DIRS ${BOUNDINGMESH_INCLUDE_DIRS} PARENT_SCOPE)

include_directories(
	${BOUNDINGMESH_INCLUDE_DIRS}
)

add_library(
	boundingmesh
	STATIC
	Primitives.cpp
	Mesh.cpp
	Decimator.cpp
	MetricGenerator.cpp
	ContractionUtils.cpp
)

set(
	HDRS
	boundingmesh.h
	Primitives.h
	Mesh.h
	Decimator.h
	MetricGenerator.h
	ContractionUtils.h
)

set(BOUNDINGMESH_LIBRARIES boundingmesh)

if(COIN_FOUND)
	set(BOUNDINGMESH_LIBRARIES ${BOUNDINGMESH_LIBRARIES} ${COIN_LIBRARIES})
endif(COIN_FOUND)

if(QHULL_FOUND)
	set(BOUNDINGMESH_LIBRARIES ${BOUNDINGMESH_LIBRARIES} ${QHULL_LIBRARIES})
endif(QHULL_FOUND)


set(BOUNDINGMESH_LIBRARIES ${BOUNDINGMESH_LIBRARIES} PARENT_SCOPE)

set_target_properties(
	boundingmesh
	PROPERTIES
	VERSION ${VERSION}
)

install(
	FILES ${HDRS} 
	DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)
install(
	TARGETS boundingmesh
	EXPORT boundingmesh-export
	ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
	LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
	RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

include(CMakePackageConfigHelpers)
configure_package_config_file(
	boundingmesh-config.cmake.in
	boundingmesh-config.cmake
	INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}
)
install(
	FILES ${CMAKE_CURRENT_BINARY_DIR}/boundingmesh-config.cmake
	DESTINATION ${CMAKE_INSTALL_LIBDIR}
)
export(
	TARGETS boundingmesh
	FILE ${CMAKE_CURRENT_BINARY_DIR}/boundingmesh-export.cmake
)
install(
	EXPORT boundingmesh-export
	DESTINATION ${CMAKE_INSTALL_LIBDIR}
	FILE boundingmesh-export.cmake
)
