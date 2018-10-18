#pragma once
#include "tfleaf2d.hpp"
#include "../tfleaf_cached.hpp"

namespace beat {
	namespace g2 {
		template <class Ar, class S, class UD>
		void save(Ar& ar, const TfLeafCached<S,UD>& leaf) {
			ar(
				cereal::base_class<TfLeaf<UD>>(&leaf)
			);
		}
		template <class Ar, class S, class UD>
		void load(Ar& ar, TfLeafCached<S,UD>& leaf) {
			save(ar, leaf);
			leaf._baseAccum = spi::none;
		}
	}
}
namespace cereal {
	template <class Ar, class S, class UD>
	struct specialize<Ar, beat::g2::TfLeafCached<S,UD>, cereal::specialization::non_member_load_save> {};
}
