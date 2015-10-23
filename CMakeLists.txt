cmake_minimum_required(VERSION 2.8.8)

enable_testing()
include(GNUInstallDirs)
include(CheckCXXCompilerFlag)

project(boundingmesh)
set(VERSION 0.1)
add_definitions(-DBOUNDINGMESH_VERSION="${VERSION}")

option(BOUNDINGMESH_VECTORIZE "use faster vectorized math (recommended unless build fails with Eigen errors)" ON)
if(NOT ${BOUNDINGMESH_VECTORIZE})
	add_definitions(-DEIGEN_DONT_VECTORIZE -DEIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT)
endif()
if(WIN32)
	add_definitions(
		-D_CRT_SECURE_NO_WARNINGS
		-D_SCL_SECURE_NO_WARNINGS
		-D_USE_MATH_DEFINES
		-D_WIN32_WINNT=0x501
		-DNOMINMAX
		-DWIN32_LEAN_AND_MEAN
	)
else()
	check_cxx_compiler_flag("-std=c++0x" HAS_CXX_COMPILER_FLAG_STDCXX0X)
	if(HAS_CXX_COMPILER_FLAG_STDCXX0X)
	    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
	endif()
endif()


set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/Modules)

set(CMAKE_VERBOSE_MAKEFILE ON) #Enables Eclipse highlighting

include_directories(
	${EIGEN_INCLUDE_DIRS}
)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR})
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR})

add_subdirectory(src/boundingmesh)
add_subdirectory(src/boundingmesh-bin)
add_subdirectory(src/boundingmesh-gui)
add_subdirectory(src/extras)
add_subdirectory(src/test)

install(
	FILES
	${CMAKE_CURRENT_SOURCE_DIR}/README.md
	${CMAKE_CURRENT_SOURCE_DIR}/LICENSE
	DESTINATION ${CMAKE_INSTALL_BINDIR}
)

