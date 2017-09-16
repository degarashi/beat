#pragma once
#include "../segment2d.hpp"
#include "point2d.hpp"

namespace beat {
	namespace g2 {
		namespace random {
			template <class RD>
			Segment GenSegment(RD& rd) {
				return {
					GenPoint(rd),
					GenPoint(rd)
				};
			}
		}
	}
}
