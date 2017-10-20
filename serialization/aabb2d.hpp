#pragma once
#include "../aabb2d.hpp"
#include "frea/serialization/vector.hpp"

namespace beat {
	namespace g2 {
		template <class Ar>
		void serialize(Ar& ar, AABB& ab) {
			ar(
				cereal::make_nvp("min", ab.min),
				cereal::make_nvp("max", ab.max)
			);
		}
	}
}
