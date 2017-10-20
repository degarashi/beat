#include "generate.hpp"
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
		template <class T>
		struct Serialization : Generator {};
		using STypes = ::testing::Types<
			Point,
			Line,
			Ray,
			Segment,
			Circle,
			Capsule,
			AABB,
			Triangle,
			Convex
		>;
		TYPED_TEST_CASE(Serialization, STypes);

		TYPED_TEST(Serialization, General) {
			TypeParam shape;
			this->Generator::genShape(shape);
			lubee::CheckSerialization(shape);
		}
	}
}
