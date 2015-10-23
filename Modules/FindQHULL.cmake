include(FindPackageHandleStandardArgs)
include(SelectLibraryConfigurations)

file(
	GLOB
	QHULL_INCLUDE_DIR
	$ENV{QHULLDIR}/include
	$ENV{HOME}/include
	/usr/local/include
	/usr/include
	$ENV{ProgramW6432}/qhull*/include
	$ENV{ProgramFiles}/qhull*/include
)

find_path(
	QHULL_INCLUDE_DIRS
	NAMES
	qhull/qhull_a.h
	libqhull/qhull_a.h
	HINTS
	${QHULL_INCLUDE_PATHS}
)

mark_as_advanced(QHULL_INCLUDE_DIRS)

file(
	GLOB
	QHULL_LIBRARY_PATHS
	$ENV{QHULLDIR}/lib
	$ENV{HOME}/lib
	/usr/local/lib
	/usr/lib
	$ENV{ProgramW6432}/qhull*/lib
	$ENV{ProgramFiles}/qhull*/lib
)

find_library(
	QHULL_LIBRARY_DEBUG
	NAMES
	libqhullstaticd libqhulld qhullstaticd qhulld
	HINTS
	${QHULL_LIBRARY_PATHS}
)

find_library(
	QHULL_LIBRARY_RELEASE
	NAMES
	libqhullstatic libqhull qhullstatic qhull 
	HINTS
	${QHULL_LIBRARY_PATHS}
)

select_library_configurations(QHULL)

find_package_handle_standard_args(
	QHULL
	DEFAULT_MSG
	QHULL_INCLUDE_DIRS
	QHULL_LIBRARIES
)