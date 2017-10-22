#pragma once
#include "../pose2d.hpp"
#include "spine/serialization/rflag.hpp"
#include "frea/serialization/matrix.hpp"

namespace beat {
	namespace g2 {
		template <class Ar>
		void serialize(Ar& ar, Pose& pose) {
			ar(
				cereal::make_nvp("values", pose._rflag)
			);
		}
	}
}
