#pragma once
#include "mapperbase.hpp"
#include "lubee/src/meta/countof.hpp"
#include "lubee/src/error.hpp"

namespace beat {
	namespace ntree {
		using Morton_t = uint32_t;
		//! 配列によるマッパー実装
		/*!
			\tparam Ent		エントリクラス
			\tparam NDiv	分割度
			\tparam Dim		次元数(2 or 3)
		*/
		template <class Ent, int NDiv, int Dim>
		class ArrayMapper : public MapperBase<NDiv, Dim> {
			public:
				using base_t = MapperBase<NDiv, Dim>;
				using Entry = Ent;
			private:
				Entry	_ent[base_t::N_Ent];	//!< 線形木
			public:
				bool operator == (const ArrayMapper& a) const noexcept {
					for(int i=0 ; i<base_t::N_Ent ; i++) {
						if(_ent[i] != a._ent[i])
							return false;
					}
					return true;
				}
#ifdef DEBUG
				bool hasEntry(const Morton_t n) const {
					D_Assert0(n<countof(_ent));
#else
				bool hasEntry(const Morton_t /*n*/) const {
#endif
					// 配列なので常にエントリは存在する
					return true;
				}
				const Entry& getEntry(const Morton_t n) const {
					D_Assert0(n<countof(_ent));
					return _ent[n];
				}
				Entry& refEntry(const Morton_t n) {
					D_Assert0(n<countof(_ent));
					return _ent[n];
				}
#ifdef DEBUG
				void remEntry(const Morton_t n) {
					// 固定配列なので削除しない
					D_Assert0(_ent[n].isEmpty());
#else
				void remEntry(const Morton_t /*n*/) {
#endif
				}
				void increment(const Morton_t num) {
					D_Assert0(_ent[num].getLowerCount() >= 0);
					_ent[num].incrementLowerCount();
				}
				void decrement(const Morton_t num) {
					// カウンタが0になっても何もしない
					D_Assert0(_ent[num].getLowerCount() > 0);
					_ent[num].decrementLowerCount();
				}
				void clear() {
					for(auto& e : _ent)
						e.clear();
				}
		};
	}
}

