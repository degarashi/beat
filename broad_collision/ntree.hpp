#pragma once
#include "ntree/ntree.hpp"
#include "ntree/cell.hpp"

namespace beat {
	namespace ntree {
		/*!
			\tparam Dim			(モートンID変換などを担当する)次元クラス(Dim2 or Dim3)
			\tparam Mapper		エントリ管理クラステンプレート(ArrayMapper or HashMapper)
			\tparam NDiv		分割数
		*/
		template <
			class Dim,
			template<class,int,int> class Mapper,
			int NDiv
		>
		class NTree : public _NTree<
								Dim,
								Mapper,
								Cell<typename Dim::VolumeEntry>,
								NDiv,
								NTree<Dim, Mapper, NDiv>
							>
		{
			private:
				using base_t = _NTree<Dim, Mapper, Cell<typename Dim::VolumeEntry>, NDiv, NTree<Dim, Mapper, NDiv>>;
			public:
				using base_t::base_t;
		};
	}
}
