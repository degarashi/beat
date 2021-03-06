#pragma once
#include "spine/src/noseq_list.hpp"
#include "getbv.hpp"
#include <unordered_set>

namespace beat {
	using CMask = uint32_t;
	//! broad-phase collision manager (round robin)
	/*!
		属性フラグが0x80000000の時はBへ、それ以外はAに登録
		A->A, A->Bでは判定が行われるが B->Bはされない
		\tparam BV	境界ボリューム
	*/
	template <class BV>
	class RoundRobin {
		public:
			enum Type {
				TypeA,
				TypeB,
				NumType
			};
			using NS_id = uint32_t;
			//! 外部(CollisionMgr)から内部Nodeを区別するId
			struct IDType {
				NS_id	id;
				Type	typ;
			};
			using BVolume = BV;
		private:
			struct Node {
				CMask			mask;	//!< 登録時に渡されたマスク値
				void*			pCol;	//!< コリジョンマネージャで使われるエントリ
				BVolume			volume;

				//! デバッグ用
				bool operator == (const Node& node) const noexcept {
					// pColは比較しない
					return mask == node.mask &&
							volume == node.volume;
				}
			};
			// 当たり判定オブジェクトを種類別に格納
			using Nodes = spi::noseq_list<Node, std::allocator<Node>, NS_id>;
			Nodes			_node[NumType];
			// 境界ボリュームを取得する為の関数
			GetBV_SP<BVolume>	_getBV;

			static Type _DetectType(const CMask m) {
				return (m & 0x80000000) ? TypeB : TypeA;
			}
		public:
			RoundRobin(const GetBV_SP<BVolume>& getbv, float /*fieldSize*/, float /*fieldOfs*/):
				_getBV(getbv)
			{}
			void clear() {
				for(auto& n : _node)
					n.clear();
			}
			//! デバッグ用
			bool operator == (const RoundRobin& r) const noexcept {
				// _getBVは比較しない
				for(int i=0 ; i<NumType ; i++) {
					if(_node[i] != r._node[i])
						return false;
				}
				return true;
			}
			// デバッグ用。オブジェクトを重複して登録してないか確認
			void selfCheck() const {
				std::unordered_set<void*> set;
				for(int i=0 ; i<NumType ; i++) {
					for(auto& n : _node[i]) {
						Assert(set.count(n.pCol)==0, "self-check failed");
						set.insert(n.pCol);
					}
				}
			}
			IDType add(void* pCol, const CMask mask) {
				const Type typ = _DetectType(mask);
				return IDType {
					_node[typ].add(Node{mask, pCol, {}}),
					typ
				};
			}
			void rem(const IDType& idt) {
				_node[idt.typ].rem(idt.id);
			}
			//! バウンディングボリュームの更新(全てのオブジェクトが対象)
			void refreshBVolume() {
				auto& bv = *_getBV;
				for(int i=0 ; i<NumType ; i++) {
					for(auto& nd : _node[i])
						nd.volume = bv(nd.pCol);
				}
			}
			//! リストに登録してある全ての物体に対して衝突判定
			/*!
				\param[in] mask		コリジョンマスク値
				\param[in] bv		判定対象のバウンディングボリューム
				\param[in] cb		コールバック関数(spn::SHandle)
				\return				衝突したペア数(境界ボリューム含まず)
			*/
			template <class CB>
			uint32_t checkCollision(const CMask mask, const BVolume& bv, CB&& cb) {
				const auto fnChk = [mask, &bv, cb=std::forward<CB>(cb)](const auto& nd) {
					for(const auto& obj : nd) {
						if(mask & obj.mask) {
							if(bv.hit(obj.volume))
								cb(obj.pCol);
						}
					}
				};
				const auto &nodeA = _node[TypeA],
							& nodeB = _node[TypeB];
				uint32_t count = nodeA.size();
				if(_DetectType(mask) == TypeA) {
					count += nodeB.size();
					// TypeBと判定
					fnChk(nodeB);
				}
				// TypeAと判定
				fnChk(nodeA);
				return count;
			}
			//! リストに溜め込まずに直接コールバックを呼ぶ
			/*!
				\param[in] ac_t		累積時間
				\param[in] cb		コールバック関数(SHandle,SHandle)
				\return				衝突したペア数(境界ボリューム含まず)
			*/
			template <class CB>
			uint32_t broadCollision(CB&& cb) {
				const auto proc = [](const Node& nd0, const Node& nd1, const auto& cb) {
					// 属性マスクによる判定
					if(nd0.mask & nd1.mask) {
						// 境界ボリュームチェック
						if(nd0.volume.hit(nd1.volume)) {
							cb(nd0.pCol, nd1.pCol);
						}
					}
				};
				uint32_t count = 0;
				const auto &nodeA = _node[TypeA],
							& nodeB = _node[TypeB];
				const uint32_t szA = nodeA.size(),
								szB = nodeB.size();
				if(szA > 0) {
					const auto itrB_a = nodeA.begin(),
								itrE_a = nodeA.end();
					{
						const auto itrE_a1 = std::prev(itrE_a, 1);
						// A -> A
						for(auto itr=itrB_a ; itr!=itrE_a1 ; ++itr) {
							const Node& nodeA = *itr;
							for(auto itr2=itr+1 ; itr2!=itrE_a ; ++itr2) {
								proc(nodeA, *itr2, cb);
							}
							count += (itrE_a - itr+1);
						}
					}
					// A -> B
					if(!nodeB.empty()) {
						count += szA * szB;
						const auto itrB_b = nodeB.begin(),
						itrE_b = nodeB.end();

						for(auto itr=itrB_a ; itr!=itrE_a ; ++itr) {
							const auto& nodeA = *itr;
							for(auto itr2=itrB_b ; itr2!=itrE_b ; ++itr2)
								proc(nodeA, *itr2, cb);
						}
					}
				}
				return count;
			}
	};
}
