#pragma once

#include "point2d.hpp"
#include "line2d.hpp"
#include "ray2d.hpp"
#include "segment2d.hpp"
#include "circle2d.hpp"
#include "capsule2d.hpp"
#include "aabb2d.hpp"
#include "triangle2d.hpp"
#include "convex2d.hpp"

namespace beat {
	namespace g2 {
		template <class Ar, class T>
		void serialize(Ar& ar, Model<T>& mdl) {
			ar(
				cereal::base_class<T>(&mdl)
			);
		}
	}
}

#include <cereal/types/polymorphic.hpp>
#include <cereal/archives/binary.hpp>
#include <cereal/archives/json.hpp>

CEREAL_REGISTER_TYPE(beat::g2::PointM)
CEREAL_REGISTER_TYPE(beat::g2::LineM)
CEREAL_REGISTER_TYPE(beat::g2::RayM)
CEREAL_REGISTER_TYPE(beat::g2::SegmentM)
CEREAL_REGISTER_TYPE(beat::g2::AABBM)
CEREAL_REGISTER_TYPE(beat::g2::TriangleM)
CEREAL_REGISTER_TYPE(beat::g2::CircleM)
CEREAL_REGISTER_TYPE(beat::g2::ConvexM)
CEREAL_REGISTER_TYPE(beat::g2::CapsuleM)

CEREAL_REGISTER_POLYMORPHIC_RELATION(beat::g2::IModel, beat::g2::PointM)
CEREAL_REGISTER_POLYMORPHIC_RELATION(beat::g2::IModel, beat::g2::LineM)
CEREAL_REGISTER_POLYMORPHIC_RELATION(beat::g2::IModel, beat::g2::RayM)
CEREAL_REGISTER_POLYMORPHIC_RELATION(beat::g2::IModel, beat::g2::SegmentM)
CEREAL_REGISTER_POLYMORPHIC_RELATION(beat::g2::IModel, beat::g2::AABBM)
CEREAL_REGISTER_POLYMORPHIC_RELATION(beat::g2::IModel, beat::g2::TriangleM)
CEREAL_REGISTER_POLYMORPHIC_RELATION(beat::g2::IModel, beat::g2::CircleM)
CEREAL_REGISTER_POLYMORPHIC_RELATION(beat::g2::IModel, beat::g2::ConvexM)
CEREAL_REGISTER_POLYMORPHIC_RELATION(beat::g2::IModel, beat::g2::CapsuleM)
