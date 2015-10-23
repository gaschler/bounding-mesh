include(FindPackageHandleStandardArgs)
include(SelectLibraryConfigurations)

file(
	GLOB
	FREESOLID_INCLUDE_PATHS
	$ENV{FREESOLIDDIR}/include
	$ENV{HOME}/include
	/usr/local/include
	/usr/include
	$ENV{ProgramW6432}/freesolid*/include
	$ENV{ProgramFiles}/freesolid*/include
	$ENV{ProgramW6432}/solid*/include
	$ENV{ProgramFiles}/solid*/include
)

find_path(
	FREESOLID_INCLUDE_DIRS
	NAMES
	SOLID/SOLID.h
	HINTS
	${FREESOLID_INCLUDE_PATHS}
)

mark_as_advanced(FREESOLID_INCLUDE_DIRS)

file(
	GLOB
	FREESOLID_LIBRARY_PATHS
	$ENV{FREESOLIDDIR}/lib
	$ENV{HOME}/lib
	/usr/local/lib
	/usr/lib
	$ENV{ProgramW6432}/freesolid*/lib
	$ENV{ProgramFiles}/freesolid*/lib
	$ENV{ProgramW6432}/solid*/lib
	$ENV{ProgramFiles}/solid*/lib
)

find_library(
	FREESOLID_LIBRARY_DEBUG
	NAMES
	FreeSOLIDd solid3_d solidd solidsd
	HINTS
	${FREESOLID_LIBRARY_PATHS}
)

find_library(
	FREESOLID_LIBRARY_RELEASE
	NAMES
	FreeSOLID solid3 solid solids
	HINTS
	${FREESOLID_LIBRARY_PATHS}
)

select_library_configurations(FREESOLID)

find_package_handle_standard_args(
	FREESOLID
	DEFAULT_MSG
	FREESOLID_INCLUDE_DIRS
	FREESOLID_LIBRARIES
)
