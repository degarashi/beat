#pragma once
#include "../point2d.hpp"
#include "frea/random/vector.hpp"

namespace beat {
	namespace g2 {
		namespace random {
			template <class RD>
			Point GenPoint(RD& rd) {
				return frea::random::GenVec<Vec2>(rd);
			}
		}
	}
}
