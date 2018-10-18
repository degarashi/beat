#pragma once
#include "lubee/src/meta/typelist.hpp"
#include "lubee/src/error.hpp"

namespace beat {
	//! コリジョンID取得関数を付加
	/*! \tparam T	コリジョン構造体
		\tparam CT	コリジョン型リスト(CType) */
	template <class T, class CT>
	struct CID_base {
		static constexpr uint32_t GetCID() {
			return CT::template Find<T>;
		}
	};

	//! サポートされていない関数を読んだ時の実行時エラーを送出
	#define INVOKE_ERROR AssertF("not supported function: %1%", __func__);

	using Time_t = uint64_t;
	/*! \tparam T	コリジョン構造体 (CID_base)
		\tparam MDL	Modelインタフェース */
	template <class T, class MDL>
	struct Model_base : virtual MDL, T {
		using T::T;
		Model_base() = default;
		Model_base(T&& t):
			T(std::move(t))
		{}
		Model_base(const T& t):
			T(t)
		{}
		virtual ~Model_base() {}
		virtual bool im_refresh(Time_t /*t*/) const override {
			return true;
		}
		virtual bool im_canCacheShape() const override {
			return true;
		}
		bool im_isLeaf() const override {
			return false;
		}
		//! モデルの実体へのポインタ
		void* im_getCore() override {
			return static_cast<T*>(this);
		}
		const void* im_getCore() const override {
			return static_cast<const T*>(this);
		}
		//! 最寄りのユーザーデータを取得
		/*! このノードが持っていればそれを返し、無ければ親を遡って探す */
		void* im_getUserData() const override {
			return nullptr;
		}
		uint32_t im_getCID() const override {
			return T::GetCID();
		}
		std::ostream& im_print(std::ostream& os) const override {
			return os << static_cast<const T&>(*this);
		}
	};
}
