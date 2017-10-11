#pragma once
#include "spine/noseq_list.hpp"

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
			};
			// 当たり判定オブジェクトを種類別に格納
			using Nodes = spi::noseq_list<Node, std::allocator<Node>, NS_id>;
			Nodes			_node[NumType];
			// 境界ボリュームを取得する為の関数
			using FGetBV = std::function<BVolume (const void*)>;
			const FGetBV 	_fGetBV;

			template <class CB>
			static bool _Proc(const Node& nd0, const Node& nd1, CB&& cb) {
				// 属性マスクによる判定
				if(nd0.mask & nd1.mask) {
					// 境界ボリュームチェック
					if(nd0.volume.hit(nd1.volume)) {
						cb(nd0.pCol, nd1.pCol);
						return true;
					}
				}
				return false;
			}
			static Type _DetectType(const CMask m) {
				return (m & 0x80000000) ? TypeB : TypeA;
			}
		public:
			RoundRobin(const FGetBV& cb, float /*fieldSize*/, float /*fieldOfs*/):
				_fGetBV(cb)
			{}
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
				for(int i=0 ; i<NumType ; i++) {
					for(auto& nd : _node[i])
						nd.volume = _fGetBV(nd.pCol);
				}
			}
			//! リストに登録してある全ての物体に対して衝突判定
			/*!
				\param[in] mask		コリジョンマスク値
				\param[in] bv		判定対象のバウンディングボリューム
				\param[in] cb		コールバック関数(spn::SHandle)
			*/
			template <class CB>
			void checkCollision(const CMask mask, const BVolume& bv, CB&& cb) {
				const auto fnChk = [mask, &bv, cb=std::forward<CB>(cb)](const auto& nd) {
					for(const auto& obj : nd) {
						if(mask & obj.mask) {
							if(bv.hit(obj.volume))
								cb(obj.pCol);
						}
					}
				};
				if(_DetectType(mask) == TypeA) {
					// TypeBと判定
					fnChk(_node[TypeB]);
				}
				// TypeAと判定
				fnChk(_node[TypeA]);
			}
			//! リストに溜め込まずに直接コールバックを呼ぶ
			/*!
				\param[in] ac_t		累積時間
				\param[in] cb		コールバック関数(SHandle,SHandle)
				\return				衝突したペア数
			*/
			template <class CB>
			int broadCollision(CB&& cb) {
				int count = 0;
				if(!_node[TypeA].empty()) {
					const auto itrB_a = _node[TypeA].begin(),
								itrE_a = _node[TypeA].end();
					{
						const auto itrE_a1 = std::prev(itrE_a, 1);
						// A -> A
						for(auto itr=itrB_a ; itr!=itrE_a1 ; ++itr) {
							const Node& nodeA = *itr;
							for(auto itr2=itr+1 ; itr2!=itrE_a ; ++itr2) {
								count += this->_Proc(nodeA, *itr2, cb);
							}
						}
					}
					// A -> B
					if(!_node[TypeB].empty()) {
						const auto itrB_b = _node[TypeB].begin(),
						itrE_b = _node[TypeB].end();

						for(auto itr=itrB_a ; itr!=itrE_a ; ++itr) {
							const auto& nodeA = *itr;
							for(auto itr2=itrB_b ; itr2!=itrE_b ; ++itr2)
								count += this->_Proc(nodeA, *itr2, cb);
						}
					}
				}
				return count;
			}
	};
}
