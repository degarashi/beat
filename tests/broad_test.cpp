#include "../broad_collision/roundrobin.hpp"
#include "../collision_mgr.hpp"
#include "tree_generate.hpp"
#include "../narrow.hpp"
#include "lubee/tuplehash.hpp"

namespace beat {
	namespace g2 {
		struct BroadCollision : TreeGenerator {
			using user_t = uint32_t;
			using volume_t = Circle;
			using colmgr_t = ColMgr<RoundRobin<volume_t>, Types, user_t>;
			using narrow_t = Types::Narrow;
			constexpr static int ObjIter = 4,
								ObjDepth = 1;
			using ObjNodeT = lubee::Types<Circle, AABB>;
			using ObjLeafT = ObjNodeT;

			auto makeRandomTree() {
				return TreeGenerator::makeRandomTree<ObjNodeT, ObjLeafT>(ObjIter, ObjDepth);
			}
			//! ランダムな階層構造の形状定義
			auto makeRandomColTree(colmgr_t& dst, const int n, const CMask mask, user_t ud) {
				std::vector<colmgr_t::HCol> ret(n);
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
		};

		namespace {
			constexpr int MaxObj = 32,
						IdOffsetA = 0x0000,
						IdOffsetB = 0x1000,
						IdA = 0x00000001,
						IdB = 0x80000001;
		}
		// 単体 -> 複数のテスト
		TEST_F(BroadCollision, Single) {
			colmgr_t cm(256, 0);
			// TypeAを適当に追加
			// (MSBが0ならTypeA, 目印としてUserData=0x0000)
			const auto vA = makeRandomColTree(cm, genInt({0, MaxObj}), IdA, IdOffsetA);
			// TypeBも適当に追加
			// (MSBが1ならTypeB 目印としてUserData=0x1000)
			const auto vB = makeRandomColTree(cm, genInt({0, MaxObj}), IdB, IdOffsetB);

			const auto spMdl = makeRandomTree();
			// -> TypeA and TypeBと判定
			using Set = std::unordered_set<user_t>;
			Set bcAB,
				bcA;
			cm.checkCollision(IdA, spMdl, [&bcAB](auto* cp){
				bcAB.insert(cp->getUserData());
			});
			// -> TypeAと判定
			cm.checkCollision(IdB, spMdl, [&bcA](auto* cp){
				bcA.insert(cp->getUserData());
			});
			// 自前で判定
			Set diyAB,
				diyA;
			for(auto& a : vA) {
				if(narrow_t::Hit(spMdl.get(), a->getModel().get(), 0)) {
					diyAB.insert(a->getUserData());
					diyA.insert(a->getUserData());
				}
			}
			for(auto& b : vB) {
				if(narrow_t::Hit(spMdl.get(), b->getModel().get(), 0)) {
					diyAB.insert(b->getUserData());
				}
			}
			// 2つの結果を比較
			ASSERT_EQ(bcAB, diyAB);
			ASSERT_EQ(bcA, diyA);
		}

		// 複数 -> 複数のテスト
		TEST_F(BroadCollision, Multi) {
			colmgr_t cm(256, 0);
			const auto v0 = makeRandomColTree(cm, genInt({0, MaxObj}), IdA, IdOffsetA);
			const auto v1 = makeRandomColTree(cm, genInt({0, MaxObj}), IdB, IdOffsetB);
			using LeafPV = std::vector<TfLeaf_base*>;
			LeafPV	leaf0, leaf1;
			{
				const auto collectLeafObj = [](LeafPV& dst, const Tf_SP& mdl){
					const auto leaf = CollectLeaf(mdl);
					const int nl = leaf.size(),
						  prev = dst.size();
					dst.resize(prev + nl);
					auto* dp = dst.data() + prev;
					for(int i=0 ; i<nl ; i++) {
						*dp++ = &dynamic_cast<TfLeaf_base&>(*leaf[i]);
					}
					D_Assert0(dp == dst.data() + dst.size());
				};
				for(auto& v : v0)
					collectLeafObj(leaf0, v->getModel());
				for(auto& v : v1)
					collectLeafObj(leaf1, v->getModel());
			}

			using HCol = colmgr_t::HCol;
			// CollisionManagerを使わずに判定した結果の格納
			const auto checkDIY = [](auto& ftCur, const auto& ftPrev, auto& ch, const HCol& hc0, const HCol& hc1, const Time_t t){
				const uint32_t id0 = hc0->getUserData(),
								id1 = hc1->getUserData();
				const uint32_t idp0 = (id0 << 16) | id1,
								idp1 = (id1 << 16) | id0;
				auto idpair = std::make_tuple(id0,id1);
				auto itr = ftPrev.find(idpair);
				ASSERT_EQ(ftCur.count(idpair), 0);
				if(narrow_t::Hit(hc0->getModel().get(), hc1->getModel().get(), t)) {
					int count = 0;
					if(itr != ftPrev.end()) {
						// 衝突カウンタ値の更新
						count = itr->second + 1;
					}
					// エントリ作成
					ftCur.emplace(idpair, count);
					ch[idp0] = count;
					ch[idp1] = count;
				} else {
					if(itr != ftPrev.end()) {
						// EndCollision
						ch[idp0] = -1;
						ch[idp1] = -1;
					}
				}
			};
			// チェック用アルゴリズムによる衝突フレーム数のカウント
			// [FromId : ToId] -> 継続フレーム数
			using FTMap = std::unordered_map<std::tuple<int,int>, int, lubee::TupleHash>;
			FTMap ftmap[2];		// 衝突履歴用に2つ使用
			int ftsw = 0;		// 現在の履歴カーソル

			// CollisionHistory-Map[MyIndex : OtherIndex]
			using CHMap = std::unordered_map<uint32_t, int>;
			// [0]=ColMgr用, [1]=チェック用アルゴリズム
			CHMap chmap[2];
			// const auto printch = [](CHMap& c) {
				// std::cout << "<" << c.size() << " items>" << std::endl;
				// for(auto& c2 : c)
					// std::cout << std::hex << c2.first << ": " << c2.second << std::endl;
			// };
			// Collision履歴にある[Id0(16bit): Id1(16bit)]に対して継続フレーム数を記録
			// (衝突が終了した場合は-1)
			const auto histToMap = [](auto& ch, const auto& vl){
				for(auto& obj : vl) {
					const uint32_t id = obj->getUserData() << 16;
					obj->getCollision([&ch, id](const auto& hist){
						const auto id2 = id | hist.hCol->getUserData();
						ASSERT_EQ(0, ch.count(id2));
						ch[id2] = hist.nFrame;
					});
					obj->getEndCollision([&ch, id](const auto& hist){
						const auto id2 = id | hist.hCol->getUserData();
						ASSERT_EQ(ch.count(id2), 0);
						ch[id2] = -1;
					});
				}
			};

			// 形状の変数値をシャッフルしながら何回か比較
			int nShuffle = genInt({1,16});
			const int nA = v0.size(),
						nB = v1.size();
			while(nShuffle-- > 0) {
				// RoundRobinによる判定
				cm.update();
				chmap[0].clear();
				histToMap(chmap[0], v0);
				histToMap(chmap[0], v1);

				// 自前判定
				{
					chmap[1].clear();
					auto& ftPrev = ftmap[ftsw];
					ftsw ^= 1;
					auto& ftCur = ftmap[ftsw];
					ftCur.clear();
					const auto time = cm.getTime();
					for(int i=0 ; i<nA ; i++) {
						// A -> A
						for(int j=i+1 ; j<nA ; j++) {
							checkDIY(ftCur, ftPrev, chmap[1], v0[i], v0[j], time);
						}
						// A -> B
						for(int j=0 ; j<nB ; j++) {
							checkDIY(ftCur, ftPrev, chmap[1], v0[i], v1[j], time);
						}
					}
				}
				// ---- コンソール出力による確認 ----
				// LogOutput("----- ChMap0 -----");
				// printch(chmap[0]);
				// LogOutput("----- ChMap1 -----");
				// printch(chmap[1]);
				ASSERT_EQ(chmap[0].size(), chmap[1].size());
				for(auto& ch0 : chmap[0]) {
					auto itr = chmap[1].find(ch0.first);
					ASSERT_NE(itr, chmap[1].end());
					ASSERT_EQ(ch0.second, itr->second);
				}

				for(auto* p : leaf0)
					moveShape(p);
				for(auto* p : leaf1)
					moveShape(p);
			}
		}
	}
}
