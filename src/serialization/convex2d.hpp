#pragma once
#include "../convex2d.hpp"
#include "frea/src/serialization/vector.hpp"
#include <cereal/types/vector.hpp>

namespace beat {
	namespace g2 {
		template <class Ar>
		void serialize(Ar& ar, Convex& c) {
			ar(
				cereal::make_nvp("point", c.point)
			);
		}
	}
}
