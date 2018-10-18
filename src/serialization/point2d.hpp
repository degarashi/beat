#pragma once
#include "frea/src/serialization/vector.hpp"

namespace beat {
	namespace g2 {
		template <class Ar>
		void serialize(Ar& ar, Point& p) {
			ar(
				cereal::base_class<Vec2>(&p)
			);
		}
	}
}
