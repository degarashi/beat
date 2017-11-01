#include "broad_test.hpp"
#include "lubee/tuplehash.hpp"

namespace beat {
	namespace g2 {
		// 単体 -> 複数のテスト
		TYPED_TEST(BroadCollision, Single) {
			USING(user_t);
			USING(narrow_t);
			using self_t = std::decay_t<decltype(*this)>;

			const auto sc = this->makeRandomScene();
			const auto spMdl = this->makeRandomTree();
			// -> TypeA and TypeBと判定
			using Set = std::unordered_set<user_t>;
			Set bcAB,
				bcA;
			sc.cm->checkCollision(self_t::IdA, spMdl.get(), [&bcAB](auto& ud){
				bcAB.insert(ud);
			});
			// -> TypeAと判定
			sc.cm->checkCollision(self_t::IdB, spMdl.get(), [&bcA](auto& ud){
				bcA.insert(ud);
			});
			// 自前で判定
			Set diyAB,
				diyA;
			for(auto& a : sc.vA) {
				if(narrow_t::Hit(spMdl.get(), a->getModel().get(), 0)) {
					diyAB.insert(a->refUserData());
					diyA.insert(a->refUserData());
				}
			}
			for(auto& b : sc.vB) {
				if(narrow_t::Hit(spMdl.get(), b->getModel().get(), 0)) {
					diyAB.insert(b->refUserData());
				}
			}
			// 2つの結果を比較
			ASSERT_EQ(bcAB, diyAB);
			ASSERT_EQ(bcA, diyA);
		}

		// 複数 -> 複数のテスト
		TYPED_TEST(BroadCollision, Multi) {
			USING(colmgr_t);
			USING(narrow_t);

			const auto sc = this->makeRandomScene();
			using LeafPV = std::vector<TfLeaf_base*>;
			LeafPV	leaf0, leaf1;
			{
				const auto collectLeafObj = [](LeafPV& dst, const Tf_SP& mdl){
					const auto leaf = TestFixture::CollectLeaf(mdl);
					const int nl = leaf.size(),
						  prev = dst.size();
					dst.resize(prev + nl);
					auto* dp = dst.data() + prev;
					for(int i=0 ; i<nl ; i++) {
						*dp++ = &dynamic_cast<TfLeaf_base&>(*leaf[i]);
					}
					D_Assert0(dp == dst.data() + dst.size());
				};
				for(auto& v : sc.vA)
					collectLeafObj(leaf0, std::dynamic_pointer_cast<ITf>(v->getModel()));
				for(auto& v : sc.vB)
					collectLeafObj(leaf1, std::dynamic_pointer_cast<ITf>(v->getModel()));
			}

			using HCol = typename colmgr_t::HCol;
			// CollisionManagerを使わずに判定した結果の格納
			const auto checkDIY = [](auto& ftCur, const auto& ftPrev, auto& ch, const HCol& hc0, const HCol& hc1, const Time_t t){
				const uint32_t id0 = hc0->refUserData(),
								id1 = hc1->refUserData();
				const uint32_t idp0 = (id0 << 16) | id1,
								idp1 = (id1 << 16) | id0;
				const auto idpair = std::make_tuple(id0,id1);
				const auto itr = ftPrev.find(idpair);
				ASSERT_EQ(ftCur.count(idpair), 0);
				const auto bv0 = hc0->template getBVolume<typename colmgr_t::BVolume>(t),
						  bv1 = hc1->template getBVolume<typename colmgr_t::BVolume>(t);
				if(bv0.hit(bv1) && narrow_t::Hit(hc0->getModel().get(), hc1->getModel().get(), t)) {
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
					const uint32_t id = obj->refUserData() << 16;
					obj->getCollision([&ch, id](const auto& ud, const int nf){
						const auto id2 = id | ud;
						ASSERT_EQ(0, ch.count(id2));
						ch[id2] = nf;
					});
					obj->getEndCollision([&ch, id](const auto& ud, const int nf){
						const auto id2 = id | ud;
						ASSERT_EQ(0, ch.count(id2));
						ch[id2] = -1;
					});
				}
			};

			// 形状の変数値をシャッフルしながら何回か比較
			int nShuffle = this->genInt({8, 32});
			const int nA = sc.vA.size(),
						nB = sc.vB.size();
			while(nShuffle-- > 0) {
				// 当たり判定クラスによる判定
				sc.cm->update();
				chmap[0].clear();
				histToMap(chmap[0], sc.vA);
				histToMap(chmap[0], sc.vB);

				// 自前判定
				{
					chmap[1].clear();
					auto& ftPrev = ftmap[ftsw];
					ftsw ^= 1;
					auto& ftCur = ftmap[ftsw];
					ftCur.clear();
					const auto time = sc.cm->getTime();
					for(int i=0 ; i<nA ; i++) {
						// A -> A
						for(int j=i+1 ; j<nA ; j++) {
							checkDIY(ftCur, ftPrev, chmap[1], sc.vA[i], sc.vA[j], time);
						}
						// A -> B
						for(int j=0 ; j<nB ; j++) {
							checkDIY(ftCur, ftPrev, chmap[1], sc.vA[i], sc.vB[j], time);
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
					this->moveShape(p);
				for(auto* p : leaf1)
					this->moveShape(p);
			}
		}
	}
}
