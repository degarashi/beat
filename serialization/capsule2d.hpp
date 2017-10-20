#pragma once
#include "../capsule2d.hpp"
#include "frea/serialization/vector.hpp"

namespace beat {
	namespace g2 {
		template <class Ar>
		void serialize(Ar& ar, Capsule& c) {
			ar(
				cereal::make_nvp("from", c.from),
				cereal::make_nvp("to", c.to),
				cereal::make_nvp("radius", c.radius)
			);
		}
	}
}
