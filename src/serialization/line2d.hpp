#pragma once
#include "../line2d.hpp"
#include "frea/src/serialization/vector.hpp"

namespace beat {
	namespace g2 {
		template <class Ar>
		void serialize(Ar& ar, Line& line) {
			ar(
				cereal::make_nvp("pos", line.pos),
				cereal::make_nvp("dir", line.dir)
			);
		}
	}
}
