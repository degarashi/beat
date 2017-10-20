#pragma once
#include "../circle2d.hpp"
#include "frea/serialization/vector.hpp"

namespace beat {
	namespace g2 {
		template <class Ar>
		void serialize(Ar& ar, Circle& c) {
			ar(
				cereal::make_nvp("center", c.center),
				cereal::make_nvp("radius", c.radius)
			);
		}
	}
}
