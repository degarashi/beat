#pragma once
#include "../tfnode_base2d.hpp"
#include "tfnode_static2d.hpp"
#include "tfleaf_cached.hpp"
#include "model2d.hpp"
#include "pose2d.hpp"
#include "spine/serialization/treenode.hpp"

namespace beat {
	namespace g2 {
		template <class Ar>
		void serialize(Ar& ar, ITf& itf) {
			ar(
				cereal::base_class<spi::TreeNode<ITf>>(&itf)
			);
		}
		template <class Ar, class B, class UD>
		void serialize(Ar& ar, TfNode_base<B,UD>& node) {
			ar(
				cereal::base_class<ITf>(&node),
				cereal::base_class<typename TfNode_base<B,UD>::model_t>(&node),
				cereal::make_nvp("udata", node._udata)
			);
		}
	}
}
