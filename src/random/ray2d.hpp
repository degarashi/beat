#pragma once
#include "line2d.hpp"
#include "../ray2d.hpp"

namespace beat {
	namespace g2 {
		namespace random {
			template <class RD>
			Ray GenRay(RD& rd) {
				const auto line = GenLine(rd);
				return {line.pos, line.dir};
			}
		}
	}
}
