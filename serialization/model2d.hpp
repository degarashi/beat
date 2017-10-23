#pragma once

#include "serialization/point2d.hpp"
#include "serialization/line2d.hpp"
#include "serialization/ray2d.hpp"
#include "serialization/segment2d.hpp"
#include "serialization/circle2d.hpp"
#include "serialization/capsule2d.hpp"
#include "serialization/aabb2d.hpp"
#include "serialization/triangle2d.hpp"
#include "serialization/convex2d.hpp"

namespace beat {
	namespace g2 {
		template <class Ar, class T>
		void serialize(Ar& ar, Model<T>& mdl) {
			ar(
				cereal::base_class<T>(&mdl)
			);
		}
	}
}
