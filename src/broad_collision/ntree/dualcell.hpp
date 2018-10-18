#pragma once
#include "cell.hpp"

namespace beat {
	namespace ntree {
		template <class VE>
		class DualCell;

		template <class VE>
		class DualCellStack {
			private:
				using VolumeEntry_t = VE;
				CellStack<VE> _stk,
							  _stkM;
			public:
				void addBlock(const DualCell<VolumeEntry_t>& ent, const bool bAdd) {
					_stk.addBlock(ent.getObjList(), bAdd);
					_stkM.addBlock(ent.getMapList(), bAdd);
				}
				void popBlock() {
					_stk.popBlock();
					_stkM.popBlock();
				}
				std::tuple<const VolumeEntry_t*, int> getObj() const {
					return _stk.getObj();
				}
				std::tuple<const VolumeEntry_t*, int> getMap() const {
					return _stkM.getObj();
				}
				bool isTopEmpty() const {
					return _stk.isTopEmpty() && _stkM.isTopEmpty();
				}
				template <class DIM>
				void classify(const bool (&bWrite)[DIM::N_LayerSize], const typename DIM::Index centerId) {
					_stk.template classify<DIM>(bWrite, centerId),
					_stkM.template classify<DIM>(bWrite, centerId);
				}
		};

		//! マスク値でオブジェクトを2種類に分ける
		template <class VE>
		class DualCell: public Cell<VE> {
			private:
				using base_t = Cell<VE>;
				using VolumeEntry_t = VE;
				using VolV = spi::noseq_vec<VolumeEntry_t>;
				VolV	_mlist;
			public:
				using CellStack = DualCellStack<VolumeEntry_t>;
				static bool IsTypeB(const CMask mask) {
					return mask & 0x80000000;
				}
				bool operator == (const DualCell& c) const noexcept {
					return static_cast<const base_t&>(*this) == static_cast<const base_t&>(c) &&
							Debug_ArrayCompare(_mlist, c._mlist);
				}
				bool operator != (const DualCell& c) const noexcept {
					return !(this->operator == (c));
				}
				// NTreeから呼ばれる。隣接セルとの重複チェック
				template <class IdToCache>
				void debug_CellCheck(const IdToCache& id2c, const DualCell& cell) const {
					base_t::debug_CellCheck(id2c, cell);

					const auto	&ent0 = getMapList(),
								&ent1 = cell.getMapList();
					for(auto& e0 : ent0) {
						const auto& c0 = id2c(e0.id);
						for(auto& e1 : ent1) {
							const auto& c1 = id2c(e1.id);
							Assert(!c0.bvolume.hit(c1.bvolume), "self-check failed");
						}
					}
				}
				template <class CB>
				void debug_Iterate(CB&& cb) const {
					base_t::debug_Iterate(cb);
					for(auto& e0 : getMapList())
						cb(e0.id);
				}
				void addObj(const CMask mask, const VolumeEntry_t& v) {
					if(IsTypeB(mask))
						_mlist.emplace_back(v);
					else
						base_t::addObj(mask, v);
				}
				bool remObj(const CMask mask, const VolumeId cid) {
					if(IsTypeB(mask))
						this->_remObj(_mlist, cid);
					else
						base_t::remObj(mask, cid);
					return isEmpty();
				}
				void clear() {
					_mlist.clear();
					base_t::clear();
				}
				bool isNodeEmpty() const {
					return base_t::isNodeEmpty() && _mlist.empty();
				}
				bool isEmpty() const {
					return base_t::isEmpty() && _mlist.empty();
				}
				const VolV& getMapList() const {
					return _mlist;
				}
		};
	}
}
