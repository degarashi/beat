#pragma once
#include "../aabb2d.hpp"
#include "frea/random/vector.hpp"

namespace beat {
	namespace g2 {
		namespace random {
			template <class RD>
			AABB GenAABB(RD&& rd) {
				using frea::Vec2;
				const auto gv = [&rd](){ return frea::random::GenVec<Vec2>(rd); };
				const auto v0 = gv(),
						 v1 = gv();
				return {
					Vec2(std::min(v0.x, v1.x),
							std::min(v0.y, v1.y)),
					Vec2(std::max(v0.x, v1.x),
							std::max(v0.y, v1.y))
				};
			}
			template <class RD>
			AABB GenAABBArea(RD&& rd, const float min) {
				for(;;) {
					auto ret = GenAABB(rd);
					if(ret.bs_getArea() >= min)
						return ret;
				}
			}
		}
	}
}
