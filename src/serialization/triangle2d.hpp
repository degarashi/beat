#pragma once
#include "../triangle2d.hpp"
#include "frea/src/serialization/vector.hpp"

namespace beat {
	namespace g2 {
		template <class Ar>
		void serialize(Ar& ar, Triangle& t) {
			ar(
				cereal::make_nvp("pos", t.pos)
			);
		}
	}
}
