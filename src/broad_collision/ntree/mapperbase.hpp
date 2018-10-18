#pragma once
#include "lubee/src/meta/constant.hpp"

namespace beat {
	namespace ntree {
		// -------- <<マッパー実装テンプレートクラス>> --------
		/*!
			\tparam NDiv	分割度
			\tparam Dim		次元数(2 or 3)
		*/
		template <int NDiv, int Dim>
		class MapperBase {
			public:
				constexpr static int CalcNEnt(const int sum, lubee::IConst<0>) {
					return sum + 1;
				}
				template <int Cur>
				constexpr static int CalcNEnt(const int sum, lubee::IConst<Cur>) {
					return CalcNEnt(sum+lubee::ConstantPow<Cur>(N_LayerSize), lubee::IConst<Cur-1>());
				}
				constexpr static int N_Div = NDiv,
									N_Width = (1 << N_Div),
									N_LayerSize = (1 << Dim),
									N_Ent = CalcNEnt(0, lubee::IConst<N_Div>());
			protected:
				MapperBase() {
					// 2D木なら4, 3D木なら8. それ以外はエラー
					static_assert(N_LayerSize==4||N_LayerSize==8, "invalid dimension");
				}
		};
	}
}
