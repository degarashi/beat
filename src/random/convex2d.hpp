#pragma once
#include "../convex2d.hpp"
#include "frea/src/random/vector.hpp"

namespace beat {
	namespace g2 {
		namespace random {
			template <class RD>
			Convex GenConvex(RD&& rd, const int n) {
				return Convex::FromConcave(
					frea::random::GenVecN<Vec2>(rd, n, NEAR_THRESHOLD_SQ)
				);
			}
			template <class RD>
			Convex GenConvexArea(RD&& rd, const int n, const float min) {
				for(;;) {
					const auto ret = GenConvex(rd, n);
					const auto area = std::get<0>(ret.area_inertia_center());
					if(area >= min)
						return ret;
				}
			}
		}
	}
}
