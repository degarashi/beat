#pragma once
#include "lubee/bit.hpp"
#include "gjk2d.hpp"

namespace beat {
	using ColFunc = bool (*)(const void*, const void*);
	using Time_t = uint64_t;
	namespace {
		namespace narrow {
			template <class Types>
			struct Init0 {
				using CTGeo = typename Types::CTGeo;
				using GJK = typename Types::GJK;
				using IModel = typename Types::IModel;

				template <class T0, class T1>
				static bool _BranchIfGJK(const IModel* m0, const IModel* m1, std::true_type) {
					// 専用アルゴリズムでの当たり判定
					const auto* t0 = reinterpret_cast<const T0*>(m0->im_getCore());
					const auto* t1 = reinterpret_cast<const T1*>(m1->im_getCore());
					return t0->hit(*t1);
				}
				template <class T0, class T1>
				static bool _BranchIfGJK(const IModel* t0, const IModel* t1, std::false_type) {
					//GJKアルゴリズムでの当たり判定
					return GJK(*t0,*t1).getResult();
				}
				template <class T0, class T1>
				constexpr static bool HasAlg = !std::is_same<lubee::none_t, decltype(std::declval<T0>().hit(std::declval<T1>()))>::value;
				template <class T0, class T1>
				using HasAlg_t = lubee::BConst<HasAlg<T0,T1>>;
				template <class T0, class T1>
				static bool _FlipObj(const IModel* t0, const IModel* t1, std::true_type) {
					return _BranchIfGJK<T1,T0>(t1, t0, HasAlg_t<T1,T0>());
				}
				template <class T0, class T1>
				static bool _FlipObj(const IModel* t0, const IModel* t1, std::false_type) {
					return _BranchIfGJK<T0,T1>(t0, t1, HasAlg_t<T0,T1>());
				}

				static bool FalseFunc[[noreturn]](const void* /*m0*/, const void* /*m1*/) {
					AssertF0();
				}
				template <int Id0, int Id1>
				static ColFunc GetFunc(std::false_type) {
					return [](const void*, const void*){ return false; };
				}
				template <int Id0, int Id1>
				static ColFunc GetFunc(std::true_type) {
					static_assert(Id0 < CTGeo::size, "invalid Id0");
					static_assert(Id1 < CTGeo::size, "invalid Id1");
					using T0 = typename CTGeo::template At<Id0>;
					using T1 = typename CTGeo::template At<Id1>;
					using Flip = typename lubee::ArithmeticT<Id0, Id1>::great;
					// リスト登録用関数ラッパ
					return [](const void* p0, const void* p1){
						return _FlipObj<T0,T1>(
							reinterpret_cast<const IModel*>(p0),
							reinterpret_cast<const IModel*>(p1),
							Flip()
						);
					};
				}
			};

			// 内側ループ
			template <class Types, int Id0, int Id1>
			struct InitB {
				static void Proc(ColFunc*& dst) {
					using CTGeo = typename Types::CTGeo;
					*dst-- =
						Init0<Types>::template GetFunc<Id0,Id1>(
							lubee::BConst<
								lubee::Arithmetic<Id0, CTGeo::size>::lesser &&
								lubee::Arithmetic<Id1, CTGeo::size>::lesser
							>()
						);
					InitB<Types, Id0, Id1-1>::Proc(dst);
				}
			};
			template <class Types, int Id0>
			struct InitB<Types, Id0, -1> {
				static void Proc(ColFunc*& /*dst*/) {}
			};

			// 外側ループ
			template <class Types, int Cur, int Width>
			struct InitA {
				static void Proc(ColFunc*& dst) {
					InitB<Types, Cur, Width-1>::Proc(dst);
					InitA<Types, Cur-1, Width>::Proc(dst);
				}
			};
			template <class Types, int Width>
			struct InitA<Types, -1, Width> {
				static void Proc(ColFunc*& /*dst*/) {}
			};
		}
	}
	//! Narrow-phase 判定関数群
	template <class Types>
	class Narrow {
		private:
			using CTGeo = typename Types::CTGeo;
			using IModel = typename Types::IModel;
			using GJK = typename Types::GJK;

			constexpr static int WideBits = lubee::bit::MSB(CTGeo::size) + 1,
								ArraySize = 1<<(WideBits*2);
			static ColFunc cs_cfunc[ArraySize];

			template <class T, class IM>
			struct IModelWrap : IM {
				const T& source;
				IModelWrap(const T& src):
					source(src)
				{}
				using Vec_t = decltype(std::declval<IM>().im_getCenter());
				Vec_t im_support(const Vec_t& v) const override {
					return source.support(v);
				}
				const void* getCore() const override {
					return static_cast<const T*>(&source);
				}
			};
			static bool _HitSingle(const IModel* mdl0, const IModel* mdl1) {
				if(mdl0->im_canCacheShape() & mdl1->im_canCacheShape()) {
					// 変換後のキャッシュされた形状で判定
					return GetCFunc(mdl0->im_getCID(), mdl1->im_getCID())(mdl0, mdl1);
				} else {
					// GJKで判定
					return GJK(*mdl0, *mdl1).getResult();
				}
			}
			//! mdl0を展開したものとmdl1を当たり判定
			/*!
				mdl1は既にrefreshをかけてある前提
				\param[in] mdl0 展開する方のインタフェース
				\param[in] mdl1 展開されない方のインタフェース
			*/
			static bool _LExpandCheck(const IModel* mdl0, const IModel* mdl1, const bool bSwap, const Time_t t) {
				mdl0->im_refresh(t);
				mdl1->im_refresh(t);
				auto in = mdl0->im_getInner();
				if(in) {
					do {
						if(in.get()->im_refresh(t)) {
							if(_HitSingle(in.get(), mdl1)) {
								if(_LExpandCheck(mdl1, in.get(), false, t))
									return true;
							}
						}
					} while(++in);
					return false;
				} else {
					if(!bSwap)
						return _LExpandCheck(mdl1, mdl0, true, t);
					return true;
				}
			}
		public:
			//! 当たり判定を行う関数をリストにセットする
			static void Initialize() {
				constexpr int WideM = (1<<WideBits);
				ColFunc* cfp = cs_cfunc+ArraySize-1;
				narrow::InitA<Types, WideM-1, WideM>::Proc(cfp);
			}
			//! 当たり判定を行う関数ポインタを取得
			static ColFunc GetCFunc(const int id0, const int id1) {
				return cs_cfunc[(id0 << WideBits) | id1];
			}
			//! IModelインタフェースを介さず直接判定
			template <class T0, class T1,
				ENABLE_IF(!std::is_pointer<T0>{} && !std::is_pointer<T1>{})>
			static bool Hit(const T0& t0, const T1& t1) {
				constexpr int id0 = CTGeo::template Find<T0>::result,
							id1 = CTGeo::template Find<T1>::result;
				// GJKで判定されるかもしれないのでIModelラッパーをつける
				IModelWrap<T0, IModel> tmp0(t0);
				IModelWrap<T1, IModel> tmp1(t1);
				return GetCFunc(id0, id1)(&tmp0, &tmp1);
			}
			//! 2つの物体(階層構造可)を当たり判定
			static bool Hit(const IModel* mdl0, const IModel* mdl1, const Time_t t) {
				if(mdl0->im_refresh(t) && mdl1->im_refresh(t)) {
					if(_HitSingle(mdl0, mdl1)) {
						if(mdl0->im_hasInner() | mdl1->im_hasInner())
							return _LExpandCheck(mdl0, mdl1, false, t);
						return true;
					}
				}
				return false;
			}
	};
	template <class Types>
	ColFunc Narrow<Types>::cs_cfunc[ArraySize];

	namespace narrow {}
}
