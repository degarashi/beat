#pragma once
#include "tfnode_base2d.hpp"
#include "pose2d.hpp"
#include "../tfleaf2d.hpp"

namespace beat {
	namespace g2 {
		template <class Ar>
		void serialize(Ar& ar, TfLeaf_base& leaf) {
			ar(
				cereal::base_class<ITf>(&leaf),
				cereal::make_nvp("rflag", leaf._rflag)
			);
		}
		template <class Ar, class UD>
		void serialize(Ar& ar, TfLeaf<UD>& leaf) {
			ar(
				cereal::base_class<TfLeaf_base>(&leaf),
				cereal::make_nvp("udata", leaf._udata)
			);
		}
	}
}
