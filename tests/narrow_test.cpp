#include "../narrow.hpp"
#include "tree_generate.hpp"

namespace beat {
	namespace g2 {
		class Narrow2D : public TreeGenerator {
			protected:
				using base_t = TreeGenerator;
				using Types_t = Types;
				using Narrow_t = Types::Narrow;
				void SetUp() override {
					base_t::SetUp();
					Narrow_t::Initialize();
				}
		};
		// 階層構造を含めたNarrowPheseテスト (2D)
		TEST_F(Narrow2D, RandomTree_Multi) {
			// ランダムに当たり判定階層構造を作る
			using CT = lubee::Types<Circle, AABB>;
			const auto c0 = makeRandomTree<CT,CT>(64, 8);
			const auto c1 = makeRandomTree<CT,CT>(64, 8);
			const auto v0 = CollectLeaf(c0),
						 v1 = CollectLeaf(c1);
			// 個別に総当たりで判定した結果 -> b0
			bool b0 = false;
			for(auto* p0 : v0) {
				for(auto* p1 : v1) {
					if((b0 |= Narrow_t::Hit(p0, p1, 0)))
						break;
				}
			}
			// ツリー構造で境界ボリュームを交えつつ判定した結果 -> b1
			const bool b1 = Narrow_t::Hit(c0.get(), c1.get(), 0);
			// 両者は一致する筈
			ASSERT_EQ(b0, b1);
		}
		TEST_F(Narrow2D, RandomTree_Single) {
			// ランダムに当たり判定階層構造を作る
			using CT = lubee::Types<Circle, AABB>;
			const auto ct = makeRandomTree<CT,CT>(64, 8);
			CircleM obj;
			this->genShape(obj);

			const auto vt = CollectLeaf(ct);
			// 個別に総当たりで判定した結果 -> b0
			bool b0 = false;
			for(auto* pt : vt) {
				if((b0 |= Narrow_t::Hit(&obj, pt, 0)))
					break;
			}
			// ツリー構造で境界ボリュームを交えつつ判定した結果 -> b1
			const bool b1 = Narrow_t::Hit(ct.get(), &obj, 0);
			// 両者は一致する筈
			ASSERT_EQ(b0, b1);
		}
	}
}
