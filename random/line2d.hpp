#pragma once
#include "../line2d.hpp"
#include "point2d.hpp"
#include "frea/random/vector.hpp"

namespace beat {
	namespace g2 {
		namespace random {
			template <class RD>
			Line GenLine(RD& rd) {
				return {
					GenPoint(rd),
					frea::random::GenVecUnit<Vec2>(rd)
				};
			}
		}
	}
}
