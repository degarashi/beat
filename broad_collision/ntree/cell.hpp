#pragma once
#include "lubee/error.hpp"
#include "spine/noseq_vec.hpp"
#include <algorithm>
#include <stack>

namespace beat {
	namespace ntree {
		template <class VE>
		class Cell;
		//! 木を巡回する際に手持ちのオブジェクトを保持
		template <class VE>
		class CellStack {
			private:
				using VolumeEntry_t = VE;
				using VolV = spi::noseq_vec<VolumeEntry_t>;
				struct Ent {
					int		nPop;		//!< pop時に幾つpopするか
					int		baseIdx;
				};
				using Stack = std::stack<Ent>;

				Stack		_nstk;
				VolV		_obj;

			public:
				CellStack() {
					_nstk.push(Ent{0,0});
				}
				void addBlock(const Cell<VE>& ent, const bool bAdd) {
					addBlock(ent.getObjList(), bAdd);
				}
				void addBlock(const VolV& ol, const bool bAdd) {
					const int nAdd = ol.size();
					if(bAdd)
						_nstk.top().nPop += nAdd;
					else
						_nstk.push(Ent{nAdd, int(_obj.size())});

					int cur = _obj.size();
					_obj.resize(cur + nAdd);
					for(auto& obj : ol)
						_obj[cur++] = obj;
				}

				/*! \param[in] bWirte 下層レイヤーのマス目有効フラグ
					\param[in] centerId */
				template <class DIM>
				void classify(const bool (&bWrite)[DIM::N_LayerSize], const typename DIM::Index centerId) {
					auto cur = getObj();
					const int stride = std::get<1>(cur);
					// 処理途中で配列が再アロケートされることを防ぐ
					int wcur = _obj.size();		// 実際に書き込まれたオブジェクトを示すカーソル
					_obj.resize(wcur + stride * DIM::N_LayerSize);
					// 振り分けるポインタを一本の配列に一時的に格納
					const std::unique_ptr<const VolumeEntry_t*>	da(new const VolumeEntry_t*[stride * DIM::N_LayerSize]);
					const VolumeEntry_t** daP[DIM::N_LayerSize];
					for(int i=0 ; i<DIM::N_LayerSize ; i++)
						daP[i] = &da.get()[i*stride];

					// オブジェクトを振り分け
					cur = getObj();
					DIM::Classify(daP, std::get<0>(cur), stride, centerId);

					// 枝の有効リストを見ながらVolumeEntry_t実体をコピー, タグ付け
					for(int i=0 ; i<DIM::N_LayerSize ; i++) {
						if(bWrite[i]) {
							auto base = da.get() + i*stride;

							// daPに幾つオブジェクトが割り振られたか
							const int nObj = daP[i] - base;
							_nstk.push(Ent{nObj, wcur});
							for(int j=0 ; j<nObj ; j++)
								_obj[wcur++] = *(*base++);
						}
					}
					// 本来の配列の長さに設定
					D_Assert0(wcur <= static_cast<int>(_obj.size()));
					_obj.resize(wcur);
				}
				bool isTopEmpty() const {
					return int(_obj.size()) <= _nstk.top().baseIdx;
				}
				void popBlock() {
					const auto top = _nstk.top();
					_nstk.pop();

					_obj.erase(_obj.end()-top.nPop, _obj.end());
				}
				//! スタックトップのVolEntryリストを受け取る
				std::tuple<const VolumeEntry_t*, int> getObj() const {
					if(_obj.empty())
						return std::make_tuple((const VolumeEntry_t*)nullptr, 0);

					const int bi = _nstk.top().baseIdx;
					return std::make_tuple(&_obj[0] + bi, int(_obj.size())-bi);
				}
		};

		using CMask = uint32_t;
		using VolumeId = uint32_t;
		//! シングルセルNTreeエントリ
		template <class VE>
		class Cell {
			private:
				using VolumeEntry_t = VE;
				using VolV = spi::noseq_vec<VolumeEntry_t>;
				VolV		_olist;			//!< セルに内包されたオブジェクトリスト
				int			_nLower;		//!< このセルより下に幾つオブジェクトが存在するか
			protected:
				void _remObj(VolV& v, const VolumeId cid) {
					const auto itr = std::find_if(v.begin(), v.end(),
								[cid](const auto& e){ return e.id == cid; });
					D_Assert0(itr != v.cend());
					v.erase(itr);
				}
			public:
				//! 巡回時にはこのスタックを使う
				using CellStack = CellStack<VolumeEntry_t>;
				Cell():
					_nLower(0)
				{}
				// NTreeから呼ばれる。隣接セルとの重複チェック
				template <class IdToCache>
				void debug_CellCheck(const IdToCache& id2c, const Cell& cell) const {
					const auto	&ent0 = getObjList(),
								&ent1 = cell.getObjList();
					for(auto& e0 : ent0) {
						const auto& c0 = id2c(e0.id);
						for(auto& e1 : ent1) {
							const auto& c1 = id2c(e1.id);
							Assert(!c0.bvolume.hit(c1.bvolume), "self-check failed");
						}
					}
				}
				template <class CB>
				void debug_Iterate(CB& cb) const {
					for(auto& e0 : getObjList())
						cb(e0.id);
				}
				void clear() {
					_olist.clear();
					_nLower = 0;
				}
				void addObj(CMask /*mask*/, const VolumeEntry_t& ve) {
					_olist.emplace_back(ve);
				}
				bool remObj(CMask /*mask*/, const VolumeId cid) {
					_remObj(_olist, cid);
					return isEmpty();
				}
				//! このノード単体が空か？
				bool isNodeEmpty() const {
					return _olist.empty();
				}
				//! 下位ノードを含めて空か？
				bool isEmpty() const {
					return isNodeEmpty() && _nLower==0;
				}
				const VolV& getObjList() const {
					return _olist;
				}
				int getLowerCount() const {
					return _nLower;
				}
				void incrementLowerCount() {
					++_nLower;
					D_Assert0(_nLower > 0);
				}
				void decrementLowerCount() {
					--_nLower;
					D_Assert0(_nLower >= 0);
				}
		};
	}
}
