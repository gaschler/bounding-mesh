project(boundingmesh-extras)

find_package(Coin)
find_package(FreeSOLID)
if(FREESOLID_FOUND)
	include_directories(
		${CMAKE_CURRENT_SOURCE_DIR}
		${BOUNDINGMESH_INCLUDE_DIRS}
		${FREESOLID_INCLUDE_DIRS}
	)
	add_executable(
		boundingmesh-measure
		measuring_main.cpp
		measuring.cpp
		utils.cpp
	)
	target_compile_definitions(
		boundingmesh-measure
		PRIVATE
		""
		${COIN_DEFINITIONS}
		${QT_DEFINITIONS}
		${SOQT_DEFINITIONS}
	)
	target_link_libraries(
		boundingmesh-measure
		${BOUNDINGMESH_LIBRARIES}
		${FREESOLID_LIBRARIES}
	)
	add_dependencies(boundingmesh-measure boundingmesh)
else()
	message("Not building boundingmesh-measure, would need SOLID library.")
endif()

