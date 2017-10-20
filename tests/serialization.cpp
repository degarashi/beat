#include "generate.hpp"
#include "serialization/model2d.hpp"

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
			using Mdl = Model<TypeParam>;
			Mdl shape;
			this->Generator::genShape(shape);
			lubee::CheckSerialization(shape);
		}
	}
}
