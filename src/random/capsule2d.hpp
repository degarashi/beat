#pragma once
#include "segment2d.hpp"
#include "../capsule2d.hpp"

namespace beat {
	namespace g2 {
		namespace random {
			template <class RDP, class RDR>
			Capsule GenCapsule(RDP&& rdp, RDR&& rdr) {
				return {
					GenSegment(rdp),
					rdr()
				};
			}
		}
	}
}
