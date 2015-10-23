include(FindPackageHandleStandardArgs)
include(SelectLibraryConfigurations)

file(
	GLOB
	COIN_INCLUDE_PATHS
	$ENV{COINDIR}/include
	$ENV{COIN3DDIR}/include
	$ENV{HOME}/include
	/usr/local/include
	/usr/include/Coin*
	/usr/include
	$ENV{SystemDrive}/Coin*/include
	$ENV{ProgramW6432}/Coin*/include
	$ENV{ProgramFiles}/Coin*/include
)

find_path(
	COIN_INCLUDE_DIRS
	NAMES
	Inventor/So.h
	HINTS
	${COIN_INCLUDE_PATHS}
)

mark_as_advanced(COIN_INCLUDE_DIRS)

file(
	GLOB
	COIN_LIBRARY_PATHS
	$ENV{COINDIR}/lib
	$ENV{COIN3DDIR}/lib
	$ENV{HOME}/lib
	/usr/local/lib
	/usr/lib
	$ENV{SystemDrive}/Coin*/lib
	$ENV{ProgramW6432}/Coin*/lib
	$ENV{ProgramFiles}/Coin*/lib
)

find_library(
	COIN_LIBRARY_DEBUG
	NAMES
	Coind coin2d coin3d
	HINTS
	${COIN_LIBRARY_PATHS}
)

find_library(
	COIN_LIBRARY_RELEASE
	NAMES
	Coin coin2 coin3
	HINTS
	${COIN_LIBRARY_PATHS}
)

select_library_configurations(COIN)

set(COIN_DEFINITIONS -DCOIN_DLL)

mark_as_advanced(COIN_DEFINITIONS)

find_package_handle_standard_args(
	Coin
	DEFAULT_MSG
	COIN_INCLUDE_DIRS
	COIN_LIBRARIES
)