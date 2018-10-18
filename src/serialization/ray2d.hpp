#pragma once
#include "../ray2d.hpp"
#include "frea/src/serialization/vector.hpp"

namespace beat {
	namespace g2 {
		template <class Ar>
		void serialize(Ar& ar, Ray& r) {
			ar(
				cereal::make_nvp("pos", r.pos),
				cereal::make_nvp("dir", r.dir)
			);
		}
	}
}
