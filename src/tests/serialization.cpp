#include "model_serialization.hpp"
#include "../narrow.hpp"

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
		TYPED_TEST_SUITE(SerializationG2, STypes);
		TYPED_TEST(SerializationG2, General) {
			using Mdl = Model<TypeParam>;
			Mdl shape;
			this->Generator::genShape(shape);
			lubee::CheckSerialization(shape);
		}

		using OtherTypes = ::testing::Types<
			Pose
		>;
		TYPED_TEST_SUITE(Serialization, OtherTypes);
		TYPED_TEST(Serialization, General) {
			TypeParam obj;
			this->genShape(obj);
			lubee::CheckSerialization(obj);
		}

		struct TreeSerialization : TreeGenerator {};
		TEST_F(TreeSerialization, General) {
			using CNode = lubee::Types<Circle, AABB>;
			// Line, Rayは境界ボリュームを計算できないので除外
			using CLeaf = lubee::Types<
				Point,
				Segment,
				Circle,
				Capsule,
				AABB,
				Triangle,
				Convex
			>;
			const auto root = makeRandomTree<CNode, CLeaf>(64, 8);

			using narrow_t = g2::Types::Narrow;
			narrow_t::Initialize();
			struct Cmp {
				bool operator()(const ITf& i0, const ITf& i1) const {
					return narrow_t::EqualTree(&i0, &i1);
				}
			};
			lubee::CheckSerialization(root, Cmp());
		}
	}
}
