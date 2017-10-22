#include "generate.hpp"
#include "../serialization/model2d.hpp"
#include "../serialization/pose2d.hpp"

namespace beat {
	namespace g2 {
		template <class T>
		struct Serialization : Generator {};

		template <class T>
		struct SerializationG2 : Serialization<T> {};
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
		TYPED_TEST_CASE(SerializationG2, STypes);
		TYPED_TEST(SerializationG2, General) {
			using Mdl = Model<TypeParam>;
			Mdl shape;
			this->Generator::genShape(shape);
			lubee::CheckSerialization(shape);
		}

		using OtherTypes = ::testing::Types<
			Pose
		>;
		TYPED_TEST_CASE(Serialization, OtherTypes);
		TYPED_TEST(Serialization, General) {
			TypeParam obj;
			this->genShape(obj);
			lubee::CheckSerialization(obj);
		}
	}
}
