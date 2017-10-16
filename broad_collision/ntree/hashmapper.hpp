#pragma once
#include "mapperbase.hpp"
#include "lubee/meta/countof.hpp"
#include "lubee/error.hpp"
#include <unordered_map>

namespace beat {
	namespace ntree {
		using Morton_t = uint32_t;
		//! ハッシュマップによるエントリ実装
		/*!
			\tparam Ent		エントリクラス
			\tparam NDiv	分割度
			\tparam NL		次元数(2 or 3)
		*/
		template <class Ent, int NDiv, int Dim>
		class HashMapper : public MapperBase<NDiv,Dim> {
			public:
				using Entry = Ent;
			private:
				using ObjHash = std::unordered_map<Morton_t, Entry>;
				ObjHash		_ent;
			public:
				HashMapper() {
					// ルートノードだけは作成しておく
					_ent[0];
				}
				bool hasEntry(const Morton_t n) const {
					return _ent.count(n) == 1;
				}
				const Entry& getEntry(const Morton_t n) const {
					return _ent.at(n);
				}
				Entry& refEntry(const Morton_t n) {
					return _ent[n];
				}
				void remEntry(const Morton_t n) {
					const auto itr = _ent.find(n);
					D_Assert0(itr->second.isEmpty());
					if(n != 0)
						_ent.erase(itr);
				}
				void increment(const Morton_t num) {
					D_Assert0(_ent[num].getLowerCount() >= 0);
					_ent[num].incrementLowerCount();
				}
				void decrement(const Morton_t num) {
					// カウンタが0になったらエントリを削除 (ルートは消さない)
					const auto itr = _ent.find(num);
					D_Assert0(itr!=_ent.end() && itr->second.getLowerCount()>0);
					itr->second.decrementLowerCount();
					if(itr->second.isEmpty() && num!=0) {
						D_Assert0(itr->second.getObjList().empty());
						_ent.erase(itr);
					}
				}
				void clear() {
					_ent.clear();
				}
		};
	}
}

