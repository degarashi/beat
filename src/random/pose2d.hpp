#pragma once
#include "frea/src/random/angle.hpp"
#include "frea/src/random/vector.hpp"
#include "../pose2d.hpp"

namespace beat {
	namespace g2 {
		namespace random {
			template <class RDP, class RDS>
			Pose GenPose(RDP&& rdp, RDS&& rds) {
				return Pose(
					frea::random::GenVec<frea::Vec2>(rdp),
					frea::random::GenAngle<frea::RadF>(rdp),
					frea::random::GenVec<frea::Vec2>(rds)
				);
			}
		}
	}
}
