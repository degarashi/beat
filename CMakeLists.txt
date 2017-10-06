cmake_minimum_required(VERSION 2.8)
project(beat)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -O0 -ggdb3 -DDEBUG")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_C_FLAGS_DEBUG} -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_C_FLAGS} -O0 -ggdb3 -std=c++14 -Wall -Wextra -ftemplate-depth=1024")
add_definitions(-DSSE=${SSE})
include_directories(.)
# デバッグビルドの際はGLibのデバッグモードをONにする
# if(CMAKE_BUILD_TYPE STREQUAL "Debug")
	# add_definitions(-D_GLIBCXX_DEBUG)
# endif()
aux_source_directory(. LIBSRC_FILE)
add_library(beat SHARED ${LIBSRC_FILE})
set_target_properties(beat PROPERTIES VERSION 0.0.1 SOVERSION 0)
install(TARGETS beat LIBRARY DESTINATION lib)

# ------ テストコード生成 ------
if(NOT without-test)
	enable_testing()

	include(aux.cmake)
	find_package(GTest REQUIRED)
	include_directories(${GTEST_INCLUDE_DIRS})
	set(TESTEXE_LIB
		beat
		${GTEST_MAIN_LIBRARIES}
		${GTEST_LIBRARIES}
		${CMAKE_THREAD_LIBS_INIT}
	)
	aux_source_directory(tests/common COMMON)
	# testsディレクトリ以下のソースが対象
	aux_source_directory(tests TEST_SRC)
	foreach(SRC IN LISTS TEST_SRC)
		list(FIND COMMON ${SRC} OUTPUT)
		if(${OUTPUT} LESS 0)
			GetFilename(${SRC}, SRCNAME)
			AddTest(${SRCNAME} "${SRC};${COMMON}" "${TESTEXE_LIB}")
		endif()
	endforeach()
endif()
