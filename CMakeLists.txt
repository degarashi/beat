cmake_minimum_required(VERSION 2.8)
project(beat)

set(COMMON_COMPILE lubee/common_compile)
set(CXX_VERSION 17)
include(${COMMON_COMPILE}/cxx_options.cmake)
include(${COMMON_COMPILE}/compile_commands.cmake)
include(${COMMON_COMPILE}/aux.cmake)
string(CONCAT CMAKE_CXX_FLAGS " -Wno-c99-extensions")
string(CONCAT CMAKE_CXX_FLAGS " -Wno-gnu-zero-variadic-macro-arguments")
string(CONCAT CMAKE_CXX_FLAGS " -Wno-gnu-anonymous-struct")
string(CONCAT CMAKE_CXX_FLAGS " -Wno-nested-anon-types")

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
add_definitions(-DSSE=${SSE})
include_directories(
	${CMAKE_CURRENT_SOURCE_DIR}
)
aux_source_directory(
	${CMAKE_CURRENT_SOURCE_DIR}/src
	LIBSRC_FILE
)
aux_source_directory(
	${CMAKE_CURRENT_SOURCE_DIR}/src/broad_collision/ntree
	NTREE_FILE
)
add_library(beat SHARED ${LIBSRC_FILE} ${NTREE_FILE})
set_target_properties(beat PROPERTIES VERSION 0.0.1 SOVERSION 0)
install(TARGETS beat LIBRARY DESTINATION lib)

# ------ テストコード生成 ------
if(NOT without-unittest)
	enable_testing()

	LoadGTestLibs(LIBS)
	set(TESTEXE_LIB beat ${LIBS})
	aux_source_directory(src/tests/common COMMON)
	# testsディレクトリ以下のソースが対象
	aux_source_directory(src/tests TEST_SRC)
	foreach(SRC IN LISTS TEST_SRC)
		list(FIND COMMON ${SRC} OUTPUT)
		if(${OUTPUT} LESS 0)
			GetFilename(${SRC}, SRCNAME)
			AddTest(${PROJECT_NAME} ${SRCNAME} "${SRC};${COMMON}" "${TESTEXE_LIB}")
		endif()
	endforeach()
endif()
