cmake_minimum_required(VERSION 3.2)

enable_testing()
include(GNUInstallDirs)
include(CheckCXXCompilerFlag)

project(boundingmesh)
set(VERSION 0.2)
add_definitions(-DBOUNDINGMESH_VERSION="${VERSION}")

find_package(GTest)
cmake_policy(SET CMP0057 NEW)

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
	option(MORE_COMPILE_ERRORS "compile with strict error settings" ON)
	if(MORE_COMPILE_ERRORS)
		set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Werror -Werror=reorder -Werror=uninitialized -Wformat -Werror=return-type -Werror=missing-braces -Werror=switch -Wpedantic")
	endif(MORE_COMPILE_ERRORS)
endif()

option(USE_EIGENQUADPROG "use QuadProg++ Eigen fork (forces us to LGPL)" ON)
if(USE_EIGENQUADPROG)
	add_definitions(
		-DUSE_EIGENQUADPROG=1
	)
endif()

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/Modules)

include_directories(
	${CMAKE_CURRENT_SOURCE_DIR}/src
	${CMAKE_CURRENT_SOURCE_DIR}/thirdparty
	${EIGEN_INCLUDE_DIRS}
)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR})
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR})

add_subdirectory(src/boundingmesh)
add_subdirectory(src/boundingmesh-bin)
add_subdirectory(src/boundingmesh-gui)

install(
	FILES
	${CMAKE_CURRENT_SOURCE_DIR}/README.md
	${CMAKE_CURRENT_SOURCE_DIR}/LICENSE
	DESTINATION ${CMAKE_INSTALL_DATADIR}/boundingmesh
)

