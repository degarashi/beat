#pragma once
#include "ntree/ntree.hpp"
#include "ntree/dualcell.hpp"

namespace beat {
	namespace ntree {
		//! マップ用判定と通常オブジェクトを分けたコリジョンクラス
		/*!
			属性フラグが0x80000000の時はBへ、それ以外はAに登録
			A->A, A->Bでは判定が行われるが B->Bはされない
			\tparam Dim			(モートンID変換などを担当する)次元クラス(Dim2 or Dim3)
			\tparam Mapper		エントリ管理クラステンプレート(ArrayMapper or HashMapper)
			\tparam NDiv		分割数
		*/
		/*! マップ用判定同士は判定しない */
		template <
			class Dim,
			template <class,int,int> class Mapper,
			int NDiv
		>
		class DualNTree : public _NTree<
									Dim,
									Mapper,
									DualCell<typename Dim::VolumeEntry>,
									NDiv,
									DualNTree<Dim, Mapper, NDiv>
								>
		{
			private:
				template <class, template<class,int,int> class, class, int, class>
				friend class _NTree;
				using base_t = _NTree<Dim, Mapper, DualCell<typename Dim::VolumeEntry>, NDiv, DualNTree<Dim, Mapper, NDiv>>;
				using cell_t = typename base_t::Cell_t;

			protected:
				template <class Notify>
				int _doCollision(const CMask mask, const typename base_t::BVolume& bv, const cell_t& cur, const Notify& ntf) const {
					int count = 0;
					const auto fnChk = [&](const auto& v){
						for(auto& obj : v) {
							const auto& c = base_t::_getCache(obj.id);
							if((mask & c.mask) &&
								bv.hit(c.bvolume))
							{
								ntf(c.pObj);
								++count;
							}
						}
					};
					fnChk(cur.getObjList());
					if(!cell_t::IsTypeB(mask))
						fnChk(cur.getMapList());
					return count;
				}
				template <class Notify>
				int _doCollision(const typename cell_t::CellStack& stk, const cell_t& cur, const Notify& ntf) const {
					const auto fnChk = [this](const auto& v0, const auto& v1){
						const auto	&c0 = base_t::_getCache(v0.id),
									&c1 = base_t::_getCache(v1.id);
						return (c0.mask & c1.mask) && c0.bvolume.hit(c1.bvolume);
					};
					const auto fnNtf = [this, &ntf](const auto& v0, const auto& v1){
						const auto	&c0 = base_t::_getCache(v0.id),
									&c1 = base_t::_getCache(v1.id);
						ntf(c0.pObj, c1.pObj);
					};
					const auto &ol = cur.getObjList(),
								&ml = cur.getMapList();
					int count = 0;
					{
						const auto obj = stk.getObj();
						auto* from = std::get<0>(obj);
						auto* to = from + std::get<1>(obj);
						// Objリストとブロック内Objとの判定
						count += base_t::_HitCheck(from, to, ol, fnChk, fnNtf);
						// ObjリストとマップObjとの判定
						count += base_t::_HitCheck(from, to, ml, fnChk, fnNtf);
					}
					{
						const auto obj = stk.getMap();
						auto* from = std::get<0>(obj);
						auto* to = from + std::get<1>(obj);
						count += base_t::_HitCheck(from, to, ol, fnChk, fnNtf);
					}
					if(!ol.empty() && !ml.empty())
						count += base_t::_HitCheck(ol.cbegin(), ol.cend(), ml, fnChk, fnNtf);
					// ブロック内Obj同士の判定
					count += base_t::_HitCheck(ol, fnChk, fnNtf);
					return count;
				}
			public:
				using base_t::base_t;
		};
	}
}
