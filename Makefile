LIB_NAME			:= beat
COMMON_MAKE_PATH	:= lubee
WITHOUT_UNITTEST	?= NO
SSE					?= 2
MAKE_GDBINIT		:= YES

OPT_SSE					= -DSSE=$(SSE)
OPT_WITHOUT_UNITTEST	= -Dwithout-unittest=$(WITHOUT_UNITTEST)

ADDITIONAL_CMAKE_OPTION	:= $(OPT_SSE) $(OPT_WITHOUT_UNITTEST)
include lubee/common_compile/common.make
