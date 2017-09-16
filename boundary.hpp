#pragma once
#include "imodel.hpp"
#include "spine/optional.hpp"

namespace beat {
	template <class S, class IM>
	spi::Optional<S> MakeBoundary(const IM** p, size_t n, const Time_t t) {
		S s;
		auto fp = &S::setBoundary;
		while(n-- > 0) {
			if((*p)->im_refresh(t)) {
				(s.*fp)(*p);
				fp = &S::appendBoundary;
			}
			++p;
		}
		if(fp == &S::setBoundary)
			return spi::none;
		return s;
	}
	template <class S, class IM>
	spi::Optional<S> MakeBoundaryPtr(const void* pp, size_t n, size_t stride, const Time_t t) {
		S s;
		auto fp = &S::setBoundary;
		auto pv = reinterpret_cast<uintptr_t>(pp);
		while(n-- > 0) {
			auto* p = reinterpret_cast<const IM*>(pv);
			if(p->im_refresh(t)) {
				(s.*fp)(p);
				fp = &S::appendBoundary;
			}
			pv += stride;
		}
		if(fp == &S::setBoundary)
			return spi::none;
		return s;
	}
}

