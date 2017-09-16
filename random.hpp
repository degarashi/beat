#pragma once
#include "lubee/error.hpp"
#include <utility>
#include <vector>

namespace beat {
	template <class RD, class V=decltype(std::declval<RD>()())>
	auto GenerateCoeff(RD&& rd, const int n, V cremain=1) {
		D_Assert0(cremain >= 0);
		std::vector<V> ret(n);
		auto* dst = ret.data();
		const auto rand = [&rd](const auto r){
			return rd({0, r});
		};
		for(int i=0 ; i<n-1 ; i++) {
			auto c = rand(cremain);
			cremain -= c;
			*dst++ = c;
		}
		*dst++ = cremain;
		return ret;
	}
	template <class RD, class T>
	auto GenerateCoeffMin(RD&& rd, const int n, const T min, const T cremain) {
		auto ret = GenerateCoeff(rd, n, cremain-min*n);
		for(auto& r : ret)
			r += min;
		return ret;
	}
}
