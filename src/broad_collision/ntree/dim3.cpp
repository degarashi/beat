#include "dim3.hpp"
#include "lubee/src/error.hpp"

namespace beat {
	namespace ntree {
		namespace g3 {
			Morton_t Index::_SeparateBits2(Morton_t x) {
				D_Assert0(x < (1<<10));				// 00000000 : 00000000 : 00000012 : 3456789A
				x = (x | (x<<16)) & 0x030000ff;		// 00000012 : 00000000 : 00000000 : 3456789A
				x = (x | (x<<8)) & 0x0300f00f;		// 00000012 : 00000000 : 34560000 : 0000789A
				x = (x | (x<<4)) & 0x030c30c3;		// 00000012 : 00003400 : 00560000 : 7800009A
				x = (x | (x<<2)) & 0x09249249;		// 00001002 : 00300400 : 50060070 : 0800900A
				return x;
			}
			Morton_t Index::_ComposeBits2(Morton_t x) {
				x = (x | (x>>2)) & 0x030c30c3;
				x = (x | (x>>4)) & 0x0300f00f;
				x = (x | (x>>8)) & 0x030000ff;
				x = (x | (x>>16)) & 0x000003ff;
				return x;
			}
		}
	}
}
