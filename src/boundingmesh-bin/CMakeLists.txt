project(boundingmesh-bin)

include_directories(
	${BOUNDINGMESH_INCLUDE_DIRS}
	${CMAKE_CURRENT_SOURCE_DIR}/../extras
)

add_executable(
	boundingmesh-bin
	main.cpp
	../extras/utils.cpp
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

install(
	TARGETS
	boundingmesh-bin
	ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
	LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
	RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

find_package(Coin)
if(COIN_FOUND AND WIN32)
	install(
		FILES
		"${COIN_INCLUDE_DIRS}/../bin/coin3.dll"
		DESTINATION ${CMAKE_INSTALL_BINDIR}
	)
endif()