#include "generate.hpp"
#include "tree_generate.hpp"

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

			struct Cmp {
				bool operator()(const ITf& i0, const ITf& i1) const {
					return spi::CompareTree(i0, i1,
						[](const ITf& p0, const ITf& p1){
							if(!p0.im_isLeaf() || !p1.im_isLeaf())
								return true;
							const auto &leaf0 = dynamic_cast<const TfLeaf<>&>(p0),
										&leaf1 = dynamic_cast<const TfLeaf<>&>(p1);
							const auto mdl0 = leaf0.getModel(),
										mdl1 = leaf1.getModel();
							return mdl0->im_getCID() == mdl1->im_getCID();
						}
					);
				}
			};
			lubee::CheckSerialization(root, Cmp());
		}
	}
}
