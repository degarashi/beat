#pragma once
#include "../triangle2d.hpp"
#include "frea/src/random/vector.hpp"

namespace beat {
	namespace g2 {
		namespace random {
			template <class RD>
			Triangle GenTriangle(RD&& rd) {
				const auto genp = [&rd](){ return frea::random::GenVecUnit<Vec2>(rd); };
				Triangle ret(genp(), genp(), genp());
				ret.clockwise();
				return ret;
			}
			template <class RD>
			Triangle GenTriangleArea(RD&& rd, const float min) {
				for(;;) {
					const auto ret = GenTriangle(rd);
					if(ret.area() >= min)
						return ret;
				}
			}
		}
	}
}
