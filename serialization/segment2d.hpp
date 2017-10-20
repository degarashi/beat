#pragma once
#include "../segment2d.hpp"
#include "frea/serialization/vector.hpp"

namespace beat {
	namespace g2 {
		template <class Ar>
		void serialize(Ar& ar, Segment& s) {
			ar(
				cereal::make_nvp("from", s.from),
				cereal::make_nvp("to", s.to)
			);
		}
	}
}

