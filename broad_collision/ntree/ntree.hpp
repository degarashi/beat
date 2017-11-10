#pragma once
#include "spine/noseq_list.hpp"
#include "lubee/bit.hpp"
#include <cstdint>
#include <unordered_map>
#include <stack>
#include "lubee/meta/constant.hpp"
#include "../getbv.hpp"
#include <unordered_set>

namespace beat {
	namespace ntree {
		using CMask = uint32_t;
		//! broad-phase collision manager (N-tree)
		/*!
			\tparam Dim			(モートンID変換などを担当する)次元クラス(Dim2 or Dim3)
			\tparam Mapper		エントリ管理クラステンプレート(ArrayMapper or HashMapper)
			\tparam Cell		エントリーの中身(Cell or DualCell)
			\tparam NDiv		分割数
			\tparam Self		継承先のクラス
		*/
		template <
			class Dim,
			template<class,int,int> class Mapper,
			class Cell,
			int NDiv,
			class Self
		>
		class _NTree {
			public:
				using this_t = Self;
				using Dim_t = Dim;
				using Cell_t = Cell;
				constexpr static int NDim = Dim_t::N_Dim,
									N_LayerSize = Dim_t::N_LayerSize;
				using Mapper_t = Mapper<Cell_t, NDiv, NDim>;
				using BVolume = typename Dim_t::BVolume;
				using IDType = void*;

			public:
				using MortonId = typename Dim_t::MortonId;
				using Index = typename Dim_t::Index;
				using VolumeEntry = typename Dim_t::VolumeEntry;
				using NS_Id = decltype(std::declval<VolumeEntry>().id);
				struct Cache {
					void*			pObj;
					CMask			mask;
					BVolume			bvolume;
					MortonId		mortonId;
					Index			posMin,
									posMax;

					//! デバッグ用
					bool operator == (const Cache& c) const noexcept {
						// pObjは比較しない
						return mask == c.mask &&
								bvolume == c.bvolume &&
								mortonId == c.mortonId &&
								posMin == c.posMin &&
								posMax == c.posMax;
					}
				};

				Mapper_t		_mapper;
				float		_unitWidth,
							_fieldOffset;

				//! BVolumeをvoid*から計算する為にCtorで与えられるファンクタ
				GetBV_SP<BVolume> _getBV;
				using CacheNS = spi::noseq_list<Cache, std::allocator<Cache>, NS_Id>;
				CacheNS			_cache;

				using PtrToId = std::unordered_map<void*, NS_Id>;
				//! オブジェクト削除でNS_Idを検索するのに使用
				PtrToId			_ptrToId;

				//! 境界ボリューム単体との判定
				/*!
					\return コリジョン判定回数(境界ボリューム含まず
				*/
				template <class Notify>
				uint32_t iterateChk(CMask mask, const BVolume& bv, const Notify& ntf) const {
					if(getEntry(0).isEmpty())
						return 0;

					const auto mid = _toMortonMinMaxId(bv);

					VolumeEntry ve{
						mid.first.asIndex(),
						mid.second.asIndex(),
						NS_Id(0)
					};

					uint32_t count = 0;
					struct Pair {
						int		toProc;
						Index	center;
					};
					std::stack<Pair> stkId;
					// ルートノードをプッシュ
					int curWidth = this_t::Mapper_t::N_Width/2;		// 現在の走査幅 (常に2の乗数)
					stkId.push(Pair{0, Index(curWidth, curWidth)});
					while(!stkId.empty()) {
						auto p = stkId.top();
						stkId.pop();
						if(p.toProc < 0) {
							if(p.toProc == -2) {
								curWidth *= 2;
								if(curWidth == 0)
									curWidth = 1;
							}
							continue;
						}
						const auto& curEnt = getEntry(p.toProc);
						// 判定を行うかの判断 (assert含む)
						if(!curEnt.isNodeEmpty()) {
							count += static_cast<const this_t&>(*this)._doCollision(mask, bv, curEnt, ntf);
						} else {
							D_Assert0(curEnt.getLowerCount() > 0);
						}
						// 下に枝があればそれを処理
						if(curEnt.getLowerCount() > 0) {
							stkId.push(Pair{-2, Index(0,0)});
							curWidth /= 2;

							// 手持ちリストをOctave個のリストに分類(重複あり)にしてスタックに積む
							Index lowerCenterId[this_t::N_LayerSize];
							Dim_t::CalcLowerCenterId(lowerCenterId, p.center, curWidth);
							// 先に枝が有効かどうか確認
							int tcount = 0;
							for(int i=0 ; i<this_t::N_LayerSize ; i++) {
								int idx = p.toProc*this_t::N_LayerSize+1+i;		// 子ノードのインデックス
								if(hasEntry(idx)) {
									auto& ent = getEntry(idx);
									if(!ent.isEmpty()) {
										Dim_t::Classify(ve, lowerCenterId[i], [i, idx2=idx, &lowerCenterId, &stkId](const VolumeEntry& /*ve*/, int /*idx*/){
											stkId.push(Pair{idx2, lowerCenterId[i]});
										});
										++tcount;
										continue;
									}
								}
							}
							D_Assert0(tcount>0);
						}
					}
					return count;
				}
				//! コリジョン判定の為に木を巡る
				/*!
					\return コリジョン判定回数(境界ボリューム含まず
				*/
				template <class Notify>
				uint32_t iterate(const Notify& ntf) const {
					if(getEntry(0).isEmpty())
						return 0;

					uint32_t count = 0;
					typename this_t::Mapper_t::Entry::CellStack stk;		// 現在持っているオブジェクト集合

					struct Pair {
						int	toProc;			// これから処理する枝ノード. 負数は一段上に上がる
						Index center;		// 中心座標(ノードが負数の時は無効)

						Pair(const int toproc, const Index cent):
							toProc(toproc),
							center(cent)
						{}
					};
					std::stack<Pair> stkId;

					int curWidth = this_t::Mapper_t::N_Width/2;		// 現在の走査幅 (常に2の乗数)

					// ルートノードをプッシュ
					stkId.push(Pair(0, Index(curWidth, curWidth)));

					while(!stkId.empty()) {
						auto p = stkId.top();
						stkId.pop();
						if(p.toProc < 0) {
							if(p.toProc == -2) {
								stk.popBlock();		// オブジェクト集合の状態を1つ戻す
								curWidth *= 2;
								if(curWidth == 0)
									curWidth = 1;
							}
							continue;
						}

						const auto& curEnt = getEntry(p.toProc);
						// 判定を行うかの判断 (assert含む)
						if(!curEnt.isNodeEmpty()) {
							count += static_cast<const this_t&>(*this)._doCollision(stk, curEnt, ntf);
							// オブジェクト集合を加える
							stk.addBlock(curEnt, true);
						} else {
							D_Assert0(curEnt.getLowerCount() > 0);
						}

						// 下に枝があればそれを処理
						if(curEnt.getLowerCount() > 0) {
							stkId.push(Pair(-2, Index(0,0)));
							curWidth /= 2;
							// 手持ちリストをOctave個のリストに分類(重複あり)にしてスタックに積む
							Index		lowerCenterId[this_t::N_LayerSize];
							bool		bWrite[this_t::N_LayerSize] = {};
							Dim_t::CalcLowerCenterId(lowerCenterId, p.center, curWidth);
							// 先に枝が有効かどうか確認
							int tcount = 0;
							for(int i=0 ; i<this_t::N_LayerSize ; i++) {
								int idx = p.toProc*this_t::N_LayerSize+1+i;		// 子ノードのインデックス
								if(hasEntry(idx)) {
									auto& ent = getEntry(idx);
									if(!ent.isEmpty()) {
										stkId.push(Pair(idx, lowerCenterId[i]));
										++tcount;
										bWrite[i] = true;
										continue;
									}
								}
							}
							stk.template classify<Dim_t>(bWrite, p.center);
							D_Assert0(tcount>0);
						} else
							stk.popBlock();		// オブジェクト集合の状態を1つ戻す
					}
					return count;
				}

				//! 上位セルのカウンタをインクリメント
				void _incrementUpper(MortonId num) {
					while(num.value != 0) {
						num.value = (num.value-1)>>(Dim_t::N_Dim);
						D_Assert0(_mapper.refEntry(num.value).getLowerCount() >= 0);
						_mapper.increment(num.value);
					}
				}
				//! 上位セルのカウンタをデクリメント
				void _decrementUpper(MortonId num) {
					while(num.value != 0) {
						num.value = (num.value-1)>>(Dim_t::N_Dim);
						D_Assert0(_mapper.refEntry(num.value).getLowerCount() > 0);
						_mapper.decrement(num.value);
					}
				}
			protected:
				//! リスト総当たり判定
				template <class Itr0, class Itr1, class Chk, class Notify>
				static uint32_t _HitCheck(
					Itr0 itr0, const Itr0 itrE0,
					const Itr1 itr1B, const Itr1 itr1E,
					const Chk& chk, const Notify& ntf
				) {
					uint32_t count = 0;
					while(itr0 != itrE0) {
						auto& o0 = *itr0;
						for(auto itr1=itr1B ; itr1!=itr1E ; ++itr1) {
							auto& o1 = *itr1;
							if(chk(o0, o1)) {
								// 通知など
								ntf(o0, o1);
							}
						}
						++itr0;
						count += itr1E-itr1B;
					}
					return count;
				}
				//! 1つのリスト内で当たり判定
				template <class Itr0, class Chk, class Notify>
				static uint32_t _HitCheck(
					const Itr0 itr0B, const Itr0 itr0E,
					const Chk& chk, const Notify& ntf
				) {
					uint32_t count = 0;
					for(auto itr0=itr0B ; itr0!=itr0E ; ++itr0) {
						auto itr1=itr0;
						++itr1;
						for( ; itr1!=itr0E ; ++itr1) {
							++count;
							if(chk(*itr0, *itr1)) {
								// 通知など
								ntf(*itr0, *itr1);
							}
						}
					}
					return count;
				}
				template <class Notify>
				uint32_t _doCollision(const CMask mask, const BVolume& bv, const Cell_t& cur, const Notify& ntf) const {
					uint32_t count = 0;
					for(const auto& obj : cur.getObjList()) {
						const auto& c = _getCache(obj.id);
						if(mask & c.mask) {
							++count;
							if(bv.hit(c.bvolume)) {
								ntf(c.pObj);
							}
						}
					}
					return count;
				}
				template <class Notify>
				uint32_t _doCollision(const typename Cell_t::CellStack& stk, const Cell_t& cur, const Notify& ntf) const {
					const auto& ol = cur.getObjList();
					// Objリストとブロック内Objとの判定
					auto ret = stk.getObj();
					auto* bgn = std::get<0>(ret);
					auto nObj = std::get<1>(ret);
					const auto fnNtf = [this, &ntf](const VolumeEntry& v0, const VolumeEntry& v1){
						const auto &c0 = _getCache(v0.id),
									&c1 = _getCache(v1.id);
						ntf(c0.pObj, c1.pObj);
					};
					const auto fnChk = [this](const VolumeEntry& v0, const VolumeEntry& v1){
						const auto &c0 = _getCache(v0.id),
									&c1 = _getCache(v1.id);
						return (c0.mask & c1.mask) && c0.bvolume.hit(c1.bvolume);
					};
					uint32_t count = _HitCheck(bgn, bgn+nObj, ol.cbegin(), ol.cend(), fnChk, fnNtf);
					// ブロック内同士の判定
					count += _HitCheck(ol.cbegin(), ol.cend(), fnChk, fnNtf);
					return count;
				}
				const Cache& _getCache(NS_Id cid) const {
					return _cache.get(cid);
				}
			private:
				MortonId _moveObject(const IDType idt) {
					_remObject(idt, false);
					const NS_Id cid = _ptrToId.at(idt);
					const auto& cache = _cache.get(cid);
					return _addObject(cache.mask, idt, false);
				}
				/*!
					\param[in]	bAddPtr		ptrToIdに登録するか
				*/
				MortonId _addObject(const CMask mask, void* pObj, const bool bAddPtr) {
					D_Assert0(pObj);
					const auto bv = (*_getBV)(pObj);
					const auto mid = _toMortonMinMaxId(bv);
					const MortonId num = MergeMortonId(mid.first, mid.second);
					NS_Id cid;
					if(bAddPtr) {
						cid = _cache.add(Cache{pObj, mask, bv, num, mid.first.asIndex(), mid.second.asIndex()});
						D_Assert0(_ptrToId.count(pObj) == 0);
						_ptrToId[pObj] = cid;
					} else
						cid = _ptrToId.at(pObj);
					// まだエントリがリストを持っていなければ自動的に作成
					auto& entry = _mapper.refEntry(num.value);
					VolumeEntry ve{mid.first.asIndex(), mid.second.asIndex(), cid};
					entry.addObj(mask, std::move(ve));
					_incrementUpper(num);
					// LogOutput("Add(%1%) %2%", hObj.getIndex(), num);
					return num;
				}
				/*!
					\param[in]	bAddPtr		ptrToIdからも削除するか
				*/
				MortonId _remObject(const IDType idt, const bool bRemPtr) {
					D_Assert0(idt);
					const auto itr = _ptrToId.find(idt);
					D_Assert0(itr != _ptrToId.end());
					const NS_Id cid = itr->second;
					auto& cache = _cache.get(cid);
					const MortonId num = cache.mortonId;
					D_Assert0(_mapper.hasEntry(num.value));
					auto& e = _mapper.refEntry(num.value);
					// リストが空になったらエントリを削除
					if(e.remObj(cache.mask, cid))
						_mapper.remEntry(num.value);
					_decrementUpper(num);
					if(bRemPtr) {
						_cache.rem(itr->second);
						_ptrToId.erase(itr);
					}
					// LogOutput("Rem(%1%) %2%", idt.getIndex(), num);
					return num;
				}
				//! モートンIDとレベルから配列インデックスを計算
				static MortonId MortonIdtoIndex(const MortonId num, int level) {
					return static_cast<MortonId>((std::pow(uint32_t(N_LayerSize), level)-1) / (N_LayerSize-1) + num.value);
				}
				//! BBox最小、最大地点のモートンIdからエントリを格納する場所を算出
				static MortonId MergeMortonId(const MortonId num0, const MortonId num1) {
					const auto diff = num0.value ^ num1.value;
					auto num = MortonIdtoIndex(num0, Mapper_t::N_Div);
					if(diff != 0) {
						auto upLv = (lubee::bit::MSB(diff) / Dim_t::N_Dim)+1;
						do {
							num.value = (num.value-1) >> Dim_t::N_Dim;
						} while(--upLv != 0);
					}
					D_Assert0(num.value < (Mapper_t::N_Ent));
					return num;
				}
				const Cell_t& getEntry(const MortonId num) const {
					return _mapper.getEntry(num.value);
				}
				bool hasEntry(const MortonId num) const {
					return _mapper.hasEntry(num.value);
				}
				void _selfCheck(std::unordered_set<NS_Id>&, lubee::IConst<0>) const {}
				template <int L>
				void _selfCheck(std::unordered_set<NS_Id>& set, lubee::IConst<L>) const {
					constexpr int From = Mapper_t::CalcNEnt(0, lubee::IConst<L>()),
									To = Mapper_t::CalcNEnt(0, lubee::IConst<L+1>());
					const auto id2Cache = [this](const NS_Id id) -> decltype(auto) {
						return _getCache(id);
					};
					// 同じIdが二度登場しないかのチェック
					const auto chkUnique = [&set](const NS_Id id){
						Assert(set.count(id)==0, "self-check failed");
						set.insert(id);
					};
					for(int i=From ; i<To ; i++) {
						if(_mapper.hasEntry(i)) {
							auto& cell0 = _mapper.getEntry(i);
							cell0.debug_Iterate(chkUnique);
							// 正しい位置のセルに格納されてるかのチェック
							const auto chkPosition = [expect=i, &id2Cache](const NS_Id id){
								auto& c = id2Cache(id);
								Assert(expect == int(c.mortonId.value), "self-check failed");
								const auto m_id = MergeMortonId(c.posMin, c.posMax);
								Assert(expect == int(m_id.value),  "self-check failed");
							};
							cell0.debug_Iterate(chkPosition);
							for(int j=i+1 ; j<To ; j++) {
								if(_mapper.hasEntry(j)) {
									auto& cell1 = _mapper.getEntry(j);
									cell0.debug_CellCheck(id2Cache, cell1);
								}
							}
						}
					}
					_selfCheck(set, lubee::IConst<L-1>());
				}
				std::pair<MortonId, MortonId> _toMortonMinMaxId(const BVolume& bv) const {
					return Dim_t::ToMortonMinMaxId(bv, Mapper_t::N_Width, _unitWidth, _fieldOffset);
				}

			public:
				/*! \param[in] fieldSize	当たり判定対象の一片サイズ */
				_NTree(const GetBV_SP<BVolume>& f, const float fsize, const float fofs):
					_unitWidth(Mapper_t::N_Width / fsize),
					_fieldOffset(fofs),
					_getBV(f)
				{}
				void clear() {
					_mapper.clear();
					_cache.clear();
					_ptrToId.clear();
				}
				//! デバッグ用
				bool operator == (const _NTree& nt) const noexcept {
					// _ptrToId, _cacheは中身の比較をしない
					return _mapper == nt._mapper &&
							_unitWidth == nt._unitWidth &&
							_fieldOffset == nt._fieldOffset &&
							_cache.size() == nt._cache.size() &&
							_ptrToId.size() == nt._ptrToId.size();
				}
				// デバッグ用。異なるセル同士で重なりがないか確認
				void selfCheck() const {
					std::unordered_set<NS_Id>	set;
					_selfCheck(set, lubee::IConst<NDiv-1>());
				}
				//! バウンディングボリュームの更新
				void refreshBVolume() {
					const auto& bvf = *_getBV;
					for(auto& m : _ptrToId) {
						// 新しくBVolumeを計算
						const auto bv = bvf(m.first);
						const auto mid = _toMortonMinMaxId(bv);
						const auto idx0 = mid.first.asIndex(),
									idx1 = mid.second.asIndex();
						auto& cache = _cache.get(m.second);
						cache.bvolume = bv;
						if(idx0 != cache.posMin ||
							idx1 != cache.posMax)
						{
							// エントリ間の移動
							_moveObject(m.first);
							cache.posMin = idx0;
							cache.posMax = idx1;
							cache.mortonId = MergeMortonId(mid.first, mid.second);
						}
					}
				}
				IDType add(void* pObj, const CMask mask) {
					_addObject(mask, pObj, true);
					return pObj;
				}
				void rem(const IDType idt) {
					_remObject(idt, true);
				}
				template <class CB>
				uint32_t checkCollision(const CMask mask, const BVolume& bv, CB&& cb) {
					return iterateChk(mask, bv, std::forward<CB>(cb));
				}
				template <class CB>
				uint32_t broadCollision(CB&& cb) {
					return iterate(std::forward<CB>(cb));
				}
		};
	}
}
