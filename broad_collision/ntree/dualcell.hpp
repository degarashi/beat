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
