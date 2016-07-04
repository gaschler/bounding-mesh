set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/test)

project(boundingmesh-tests)

find_package(Coin)

include_directories(
	${CMAKE_CURRENT_SOURCE_DIR}/../extras
	${BOUNDINGMESH_INCLUDE_DIRS}
)

find_package(CGAL)
if(CGAL_FOUND)
	if((NOT MSVC) OR MSVC_VERSION GREATER 1699)
		add_executable(
			testCgalEigenQuadraticProgramming
			testCgalEigenQuadraticProgramming.cpp
			../../thirdparty/EigenQP.cpp
		)
		target_compile_definitions(
			testCgalEigenQuadraticProgramming
			PUBLIC
			-DCGAL_DISABLE_ROUNDING_MATH_CHECK
		)	
		target_include_directories(
			testCgalEigenQuadraticProgramming
			PUBLIC
			""
			${CGAL_INCLUDE_DIRS}
		)
		target_link_libraries(
			testCgalEigenQuadraticProgramming
			${BOUNDINGMESH_LIBRARIES}
			${CGAL_LIBRARIES}
		)
		
		add_test(testCgalEigenQuadraticProgramming ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/testCgalEigenQuadraticProgramming)
	endif()
endif()


find_package(FreeSOLID)
if(FREESOLID_FOUND)
	include_directories(
		${FREESOLID_INCLUDE_DIRS}
	)	
	add_definitions(
		""
		${COIN_DEFINITIONS}
	)

	add_executable(
		testFigures
		testFigures.cpp
	)

	target_link_libraries(
		testFigures
		${BOUNDINGMESH_LIBRARIES}
	)

	add_test(testFigures ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/testFigures "${CMAKE_SOURCE_DIR}/examples")

endif(FREESOLID_FOUND)
