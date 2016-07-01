project(boundingmesh-gui)

set(ENV{QT_SELECT} "qt4")
find_package(Qt4 COMPONENTS QtCore QtGui QtOpenGL)
set(QT_USE_QTMAIN TRUE)
include(${QT_USE_FILE})
find_package(Coin)
find_package(OpenGL)
find_package(SoQt)
if(COIN_FOUND AND OPENGL_FOUND AND QT_FOUND AND SOQT_FOUND)
	cmake_minimum_required(VERSION 2.8.8)
	include(${QT_USE_FILE})
	qt4_wrap_cpp(
		MOC_SRCS
		gui.h
	)
	qt4_add_resources(
		RCC_SRCS
		res.qrc
	)
	add_executable(
		boundingmesh-gui
		WIN32
		${MOC_SRCS}
		${RCC_SRCS}
		gui.cpp
		ViewerMesh.cpp
		boundingmesh-gui.rc
	)
	target_compile_definitions(
		boundingmesh-gui
		PRIVATE
		""
		${COIN_DEFINITIONS}
		${QT_DEFINITIONS}
		${SOQT_DEFINITIONS}
	)
	target_include_directories(
		boundingmesh-gui
		PRIVATE
		${BOUNDINGMESH_INCLUDE_DIRS}
		${COIN_INCLUDE_DIRS}
		${OPENGL_INCLUDE_DIR}
		${QT_INCLUDES}
		${SOQT_INCLUDE_DIRS}
		${CMAKE_CURRENT_SOURCE_DIR}/../boundingmesh-bin/
		#############${CMAKE_CURRENT_SOURCE_DIR}/../extras/
	)
	target_link_libraries(
		boundingmesh-gui
		${BOUNDINGMESH_LIBRARIES}
		${COIN_LIBRARIES}
		${OPENGL_LIBRARIES}
		${QT_LIBRARIES}
		${SOQT_LIBRARIES}
	)
	set_target_properties(
		boundingmesh-gui
		PROPERTIES
		VERSION ${VERSION}
	)
	set_property(
		SOURCE MyFile.cpp 
		APPEND_STRING PROPERTY COMPILE_FLAGS " -mwindows "
	)
	install(
		TARGETS
		boundingmesh-gui
		ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
		LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
		RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
	)
	file(
		COPY 
		${CMAKE_CURRENT_SOURCE_DIR}/../../examples/torus/torus.off
		${CMAKE_CURRENT_SOURCE_DIR}/../../examples/teapot/teapot.off
		${CMAKE_CURRENT_SOURCE_DIR}/../../examples/bunny/bunny.off
    	DESTINATION ${CMAKE_BINARY_DIR}/
    )
	install(
		FILES 
		${CMAKE_CURRENT_SOURCE_DIR}/../../examples/torus/torus.off
		${CMAKE_CURRENT_SOURCE_DIR}/../../examples/teapot/teapot.off
		${CMAKE_CURRENT_SOURCE_DIR}/../../examples/bunny/bunny.off
		DESTINATION ${CMAKE_INSTALL_BINDIR}
	)
	
	if(WIN32)
		install(
			FILES
			"${COIN_INCLUDE_DIRS}/../bin/coin3.dll"
			"${COIN_INCLUDE_DIRS}/../bin/soqt1.dll"
			"${QT_QTGUI_INCLUDE_DIR}/../../bin/QtGui4.dll"
			"${QT_QTGUI_INCLUDE_DIR}/../../bin/QtCore4.dll"
			"${QT_QTGUI_INCLUDE_DIR}/../../bin/QtOpenGL4.dll"
			DESTINATION ${CMAKE_INSTALL_BINDIR}
		)
	endif()
else()
	message("Not building boundingmesh-gui, would need Qt4, Coin, SoQt, and OpenGL libraries.")
endif()

