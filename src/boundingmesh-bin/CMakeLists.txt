include_directories(
	${BOUNDINGMESH_INCLUDE_DIRS}
)

add_executable(
	boundingmesh-bin
	boundingmesh-cli.cpp
)

set_target_properties(
	boundingmesh-bin
	PROPERTIES
	OUTPUT_NAME boundingmesh	
)

target_link_libraries(
    boundingmesh-bin
    ${BOUNDINGMESH_LIBRARIES}
)

add_dependencies(boundingmesh-bin boundingmesh)

add_test(testBoundingmeshTorus ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/boundingmesh -v 100 "${CMAKE_SOURCE_DIR}/examples/torus/torus.off" "${CMAKE_BINARY_DIR}/torus_decimated.off")
add_test(testBoundingmeshTeapot ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/boundingmesh -v 1000 "${CMAKE_SOURCE_DIR}/examples/teapot/teapot.off" "${CMAKE_BINARY_DIR}/teapot_decimated.off")

add_executable(
	bounding-convex-decomposition
	bounding-convex-decomposition-cli.cpp
)

target_link_libraries(
	bounding-convex-decomposition
	${BOUNDINGMESH_LIBRARIES}
)

add_dependencies(bounding-convex-decomposition boundingmesh boundingmeshdecomposition)

add_test(testBoundingConvexDecompositionBunny ${CMAKE_BINARY_DIR}/bounding-convex-decomposition -x 20000 "${CMAKE_SOURCE_DIR}/examples/bunny/bunny.off" "${CMAKE_BINARY_DIR}/bunny_decomposition.wrl")
add_test(testBoundingConvexDecompositionTeapot ${CMAKE_BINARY_DIR}/bounding-convex-decomposition "${CMAKE_SOURCE_DIR}/examples/teapot/teapot.off" "${CMAKE_BINARY_DIR}/teapot_decomposition.wrl")

install(
	TARGETS
	boundingmesh-bin
	bounding-convex-decomposition
	ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
	LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
	RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

find_package(Coin)
if(COIN_FOUND AND WIN32)
	add_custom_command(TARGET boundingmesh-bin
		POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${COIN_INCLUDE_DIRS}/../bin/coin3.dll"
        $<TARGET_FILE_DIR:boundingmesh-bin>
	)  
	install(
		FILES
		"${COIN_INCLUDE_DIRS}/../bin/coin3.dll"
		DESTINATION ${CMAKE_INSTALL_BINDIR}
	)
endif()
