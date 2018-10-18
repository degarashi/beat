#pragma once
#include "tfnode_base2d.hpp"
#include "../tfnode_static2d.hpp"

namespace beat {
	namespace g2 {
		template <class Ar, class B, class UD>
		void serialize(Ar& ar, TfNode_Static<B,UD>& node) {
			ar(
				cereal::make_nvp("changed", node._bChanged),
				cereal::make_nvp("valid", node._bValid),
				cereal::base_class<TfNode_base<B,UD>>(&node)
			);
		}
	}
}
