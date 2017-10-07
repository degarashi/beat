#include "tree_generate.hpp"
#include "../narrow.hpp"
#include "../random/pose2d.hpp"

namespace beat {
	namespace g2 {
		template <class T>
		class TfNode2D : public TreeGenerator {
			protected:
				using Narrow_t = Types::Narrow;
		};
		using TfNodeTypeList = ::testing::Types<Circle,
												Segment,
												Triangle,
												Convex>;
		TYPED_TEST_CASE(TfNode2D, TfNodeTypeList);

		// 単一ノードによる姿勢変換テスト
		TYPED_TEST(TfNode2D, Transform) {
			using Shape = TypeParam;
			using ShapeM = Model<Shape>;
			// 基本の形状に姿勢変換を掛けた物(=A)と
			// 変換後の座標で直接生成した物(=B)の2種類を用意
			ShapeM s;
			this->genShape(s);

			auto& mt = this->mt();
			const auto rdf = mt.template getUniformF<float>(RangeF{1e2f});
			const auto rds = mt.template getUniformF<float>({1e-1f, 1e2f});
			auto ps = g2::random::GenPose(rdf, rds);
			auto& sc = ps.refScaling();
			sc.x = sc.y;

			const auto spA = std::make_shared<TfLeaf<>>(std::make_shared<ShapeM>(s * ps.getToWorld())),
						 spB = std::make_shared<TfLeaf<>>(std::make_shared<ShapeM>(s));
			spB->setPose(ps);

			// サポート写像が一致するか確認
			constexpr int nCheck = 10;
			for(int i=0 ; i<nCheck ; i++) {
				const auto dir = this->genDir();
				const auto p0 = spA->im_support(dir),
							p1 = spB->im_support(dir);
				ASSERT_LE(p0.distance(p1), 5e-3f);
			}
			for(int i=0 ; i<nCheck ; i++) {
				// 衝突試験用の形状を1つ用意
				ShapeM check;
				this->genShape(check);
				// 結果は同じになる筈
				using this_t = TfNode2D<TypeParam>;
				const bool bA = this_t::Narrow_t::Hit(spA.get(), &check, 0),
						 bB = this_t::Narrow_t::Hit(spB.get(), &check, 0);
				ASSERT_EQ(bA, bB);
			}
		}
	}
}
