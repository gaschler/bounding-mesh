project(boundingmesh-lib)

find_package(Eigen 3 REQUIRED)
find_package(Coin)
find_package(CGAL)
find_package(GMP)

set(BOUNDINGMESH_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})

if(COIN_FOUND)
	add_definitions(-DCOIN_AVAILABLE -DCOIN_NOT_DLL)
	set(BOUNDINGMESH_INCLUDE_DIRS ${BOUNDINGMESH_INCLUDE_DIRS} ${COIN_INCLUDE_DIRS})
endif(COIN_FOUND)

if (CGAL_FOUND)
	add_definitions(-DCGAL_AVAILABLE -DCGAL_DISABLE_ROUNDING_MATH_CHECK)
	set(BOUNDINGMESH_INCLUDE_DIRS ${BOUNDINGMESH_INCLUDE_DIRS} ${CGAL_INCLUDE_DIRS})
	set(BOUNDINGMESH_INCLUDE_DIRS ${BOUNDINGMESH_INCLUDE_DIRS} ${Boost_INCLUDE_DIR})
	#set(BOUNDINGMESH_INCLUDE_DIRS ${BOUNDINGMESH_INCLUDE_DIRS} ${GMP_INCLUDE_DIR})
else(CGAL_FOUND)
	message("Not building bounding convex decomposition, would need CGAL library.")	
endif(CGAL_FOUND)

set(BOUNDINGMESH_INCLUDE_DIRS ${BOUNDINGMESH_INCLUDE_DIRS} PARENT_SCOPE)

include_directories(
	${BOUNDINGMESH_INCLUDE_DIRS}
)

set(
	BOUNDINGMESH_SRCS
	ContractionUtils.cpp
	Decimator.cpp
	FileUtils.cpp
	Mesh.cpp
	MetricGenerator.cpp
	Primitives.cpp
)

if(USE_EIGENQUADPROG)
	list(APPEND BOUNDINGMESH_SRCS ../../thirdparty/EigenQP.cpp)
endif()

add_library(
	boundingmesh
	STATIC
	${BOUNDINGMESH_SRCS}
)

set(
	BOUNDINGMESHDECOMPOSITION_SRCS
	SegmenterDownsampling.cpp
	SegmenterSimple.cpp
	SimpleOptimizer.cpp
	Split.cpp
	VoxelSet.cpp
	VoxelSubset.cpp
)

add_library(
	boundingmeshdecomposition
	STATIC
	${BOUNDINGMESHDECOMPOSITION_SRCS}
)

target_link_libraries(
    boundingmeshdecomposition
    boundingmesh
)

set(
	BOUNDINGMESH_HDRS
	boundingmesh.h
	ContractionUtils.h
	Decimator.h
	FileUtils.h
	Mesh.h
	MetricGenerator.h
	OptimizerInferface.h
	Primitives.h
	Segmenter.h
	SegmenterDownsampling.h
	SegmenterSimple.h
	SimpleOptimizer.h
	Split.h
	VoxelSet.h
	VoxelSubset.h
)

set(BOUNDINGMESH_LIBRARIES boundingmesh boundingmeshdecomposition)

if(COIN_FOUND)
	set(BOUNDINGMESH_LIBRARIES ${BOUNDINGMESH_LIBRARIES} ${COIN_LIBRARIES})
endif(COIN_FOUND)

if(CGAL_FOUND)
	set(BOUNDINGMESH_LIBRARIES ${BOUNDINGMESH_LIBRARIES} ${CGAL_LIBRARIES})
	set(BOUNDINGMESH_LIBRARIES ${BOUNDINGMESH_LIBRARIES} ${Boost_LIBRARIES})
endif(CGAL_FOUND)
set(BOUNDINGMESH_LIBRARIES ${BOUNDINGMESH_LIBRARIES} ${GMP_LIBRARIES})

set(BOUNDINGMESH_LIBRARIES ${BOUNDINGMESH_LIBRARIES} PARENT_SCOPE)

set_target_properties(
	boundingmesh
	PROPERTIES
	VERSION ${VERSION}
)

if(GTest_FOUND)
	include(GoogleTest)
	add_executable(OptimizerTest OptimizerTest.cpp)
	target_link_libraries(OptimizerTest boundingmesh gmock gmock_main pthread)
	gtest_add_tests(TARGET OptimizerTest)
endif(GTest_FOUND)

set_target_properties(
	boundingmeshdecomposition
	PROPERTIES
	VERSION ${VERSION}
)

install(
	FILES ${BOUNDINGMESH_HDRS}
	DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)
install(
	TARGETS boundingmesh
	EXPORT boundingmesh-export
	ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
	LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
	RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
install(
	TARGETS boundingmeshdecomposition
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
