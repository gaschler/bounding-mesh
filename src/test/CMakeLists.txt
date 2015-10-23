set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/test)

project(boundingmesh-tests)

find_package(Coin)
FIND_PACKAGE(FreeSOLID REQUIRED)
if(FREESOLID_FOUND)
	include_directories(
		${CMAKE_CURRENT_SOURCE_DIR}/../extras
		${BOUNDINGMESH_INCLUDE_DIRS}
		${FREESOLID_INCLUDE_DIRS}
	)	
	add_definitions(
		""
		${COIN_DEFINITIONS}
	)

	add_executable(
		testFile
		testFile.cpp
		../extras/utils.cpp
		../extras/measuring.cpp
	)

	target_link_libraries(
		testFile
		${BOUNDINGMESH_LIBRARIES}
		${FREESOLID_LIBRARIES}
	)

	add_test(testTorus "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/testFile" "${CMAKE_SOURCE_DIR}/examples/torus/torus.off")
	
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
