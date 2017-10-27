#pragma once
#include "tree_generate.hpp"
#include "../broad_collision/roundrobin.hpp"
#include "../broad_collision/dualntree.hpp"
#include "../broad_collision/ntree/dim2.hpp"
#include "../broad_collision/ntree/arraymapper.hpp"
#include "../broad_collision/ntree/hashmapper.hpp"
#include "../collision_mgr.hpp"

namespace beat {
	namespace g2 {
		template <class BroadT>
		struct BroadCollision : TreeGenerator {
			using user_t = uint32_t;
			using broad_t = BroadT;
			using colmgr_t = ColMgr<broad_t, Types, user_t>;
			using narrow_t = Types::Narrow;
			constexpr static int ObjIter = 12,
								ObjDepth = 2;
			using ObjNodeT = lubee::Types<AABB>;
			using ObjLeafT = ObjNodeT;

			auto makeRandomTree() {
				return TreeGenerator::makeRandomTree<ObjNodeT, ObjLeafT>(ObjIter, ObjDepth);
			}
			//! ランダムな階層構造の形状定義
			auto makeRandomColTree(colmgr_t& dst, const int n, const CMask mask, user_t ud) {
				std::vector<typename colmgr_t::HCol> ret(n);
				for(int i=0 ; i<n ; i++) {
					const auto sp = makeRandomTree();
					ret[i] = dst.addCol(mask, sp, ud++);
				}
				return ret;
			}
			//! 形状をランダムに動かす
			/*! 動かすかどうかもランダム */
			void moveShape(TfLeaf_base* p) {
				// 50%の確率で動かす = 新たに形状を作成
				if(genInt({0,1}) == 0) {
					void* ms = p->getModel()->im_getCore();
					switch(p->im_getCID()) {
						case Circle::GetCID():
							genShape(*static_cast<Circle*>(ms));
							break;
						case AABB::GetCID():
							genShape(*static_cast<AABB*>(ms));
							break;
						default:
							AssertF("unknown shape Id");
					}
					// 動かしたらsetAsChangedを呼ぶ
					p->tf_setAsChanged();
				}
			}
			using colmgr_sp = std::shared_ptr<colmgr_t>;
			using ObjV = std::vector<typename colmgr_t::HCol>;
			struct Scene {
				colmgr_sp	cm;
				ObjV		vA,
							vB;
			};
			constexpr static int MaxObj = 32,
								IdOffsetA = 0x0000,
								IdOffsetB = 0x1000,
								IdA = 0x00000001,
								IdB = 0x80000001;
			Scene makeRandomScene() {
				Scene sc{
					std::make_shared<colmgr_t>(
						this->genFloat({1.f, 1024.f}),
						this->genFloat(RangeF{1024.f})
					),
					{}, {}
				};
				// TypeAを適当に追加
				// (MSBが0ならTypeA, 目印としてUserData=0x0000)
				sc.vA = this->makeRandomColTree(*sc.cm, this->genInt({0, MaxObj}), IdA, IdOffsetA);
				// TypeBも適当に追加
				// (MSBが1ならTypeB 目印としてUserData=0x1000)
				sc.vB = this->makeRandomColTree(*sc.cm, this->genInt({0, MaxObj}), IdB, IdOffsetB);
				return std::move(sc);
			}
		};
		using BroadTypes = ::testing::Types<
			RoundRobin<Circle>,
			RoundRobin<AABB>,
			ntree::DualNTree<ntree::g2::Dim, ntree::ArrayMapper, 5>,
			ntree::DualNTree<ntree::g2::Dim, ntree::HashMapper, 5>
		>;
		TYPED_TEST_CASE(BroadCollision, BroadTypes);
	}
}
