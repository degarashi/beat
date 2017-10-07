#include "../narrow.hpp"
#include "tree_generate.hpp"
#include "../random/pose2d.hpp"

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
		TEST_F(Narrow2D, Clone) {
			// ランダムに当たり判定階層構造を作る
			using CT = lubee::Types<Circle, AABB>;
			const auto c0 = makeRandomTree<CT,CT>(64, 8);
			const auto c1 = c0->cloneTree();
			CircleM obj;
			this->genShape(obj);

			// クローンした物と結果比較
			const bool b0 = Narrow_t::Hit(c0.get(), &obj, 0),
						b1 = Narrow_t::Hit(c1.get(), &obj, 0);
			ASSERT_EQ(b0, b1);
		}
		// ツリーノードによる姿勢変換テスト
		TEST_F(Narrow2D, Transform) {
			using CTLeaf = lubee::Types<Circle, AABB>;
			using CTNode = lubee::Types<Circle>;
			const auto spA = makeRandomTree<CTNode, CTNode>(64, 8),
						spB = spA->cloneTree();
			const auto vA = CollectLeaf(spA),
						 vB = CollectLeaf(spB);

			auto& mt = this->mt();
			const auto rdf = mt.template getUniformF<float>(RangeF{1e2f});
			const auto rds = mt.template getUniformF<float>({1e-1f, 1e2f});
			// 基本の形状に姿勢変換を掛けた物(=A)と
			// 変換後の座標で直接生成した物(=B)の2種類を用意
			auto ps = g2::random::GenPose(rdf, rds);
			auto& sc = ps.refScaling();
			sc.x = sc.y;
			for(auto& vs : vA) {
				auto* vsp = dynamic_cast<TfLeaf<>*>(vs);
				vsp->setPose(ps);
				vsp->tf_setAsChanged();
			}
			const auto& toWorld = ps.getToWorld();
			for(auto& vs : vB) {
				auto* vsp = dynamic_cast<TfLeaf<>*>(vs);
				vsp->setModel(vsp->getModel()->im_clone());
				auto& mdl = *static_cast<Circle*>(vsp->getModel()->im_getCore());
				mdl = mdl * toWorld;
				vsp->tf_setAsChanged();
			}

			constexpr int nCheck = 5;
			for(int i=0 ; i<nCheck ; i++) {
				const auto t0 = makeRandomTree<CTNode, CTLeaf>(64, 4);
				const bool bA = Narrow_t::Hit(spA.get(), t0.get(), 0),
						bB = Narrow_t::Hit(spB.get(), t0.get(), 0);
				ASSERT_EQ(bA, bB);
			}
		}
	}
}
