#pragma once
#include <cstdint>

namespace beat {
	//! aがb以上だったらaからsizeを引いた値を返す
	inline int32_t CndSub(const int32_t a, const int32_t b, const int32_t size) noexcept {
		return a - (size & ((b - a - 1) >> 31));
	}
	//! aがb以上だったらaからbを引いた値を返す
	inline int32_t CndSub(const int32_t a, const int32_t b) noexcept {
		return CndSub(a, b, b);
	}
	//! aがbより小さかったらaにsizeを足した値を返す
	inline int32_t CndAdd(const int32_t a, const int32_t b, const int32_t size) noexcept {
		return a + (size & ((a - b) >> 31));
	}
	//! aが0より小さかったらaにsizeを足した値を返す
	inline int32_t CndAdd(const int32_t a, const int32_t size) noexcept {
		return CndAdd(a, 0, size);
	}
	//! aがlowerより小さかったらsizeを足し、upper以上だったらsizeを引く
	inline int32_t CndRange(const int32_t a, const int32_t lower, const int32_t upper, const int32_t size) noexcept {
		return a + (size & ((a - lower) >> 31))
					- (size & ((upper - a - 1) >> 31));
	}
	//! aが0より小さかったらsizeを足し、size以上だったらsizeを引く
	inline int32_t CndRange(const int32_t a, const int32_t size) noexcept {
		return CndRange(a, 0, size, size);
	}
}
