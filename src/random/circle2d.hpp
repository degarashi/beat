#pragma once
#include "../circle2d.hpp"
#include "frea/src/random/vector.hpp"

namespace beat {
	namespace g2 {
		namespace random {
			template <class RDP, class RDR>
			Circle GenCircle(RDP&& rdp, RDR&& rdr) {
				return {
					frea::random::GenVec<Vec2>(rdp),
					std::abs(rdr())
				};
			}
			template <class RD>
			Circle GenCircle(RD&& rd) {
				return GenCircle(rd, rd);
			}
		}
	}
}
