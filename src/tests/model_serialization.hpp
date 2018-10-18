#pragma once
#include "tree_generate.hpp"
#include "../serialization/tfleaf_cached.hpp"

#include <cereal/types/polymorphic.hpp>
#include <cereal/archives/binary.hpp>
#include <cereal/archives/json.hpp>

CEREAL_REGISTER_TYPE_WITH_NAME(beat::g2::TreeGenerator::AABBNode, "AABBNode")
CEREAL_REGISTER_TYPE_WITH_NAME(beat::g2::TreeGenerator::CircleNode, "CircleNode")

CEREAL_REGISTER_TYPE_WITH_NAME(beat::g2::TreeGenerator::PointLeaf, "PointLeaf")
CEREAL_REGISTER_TYPE_WITH_NAME(beat::g2::TreeGenerator::LineLeaf, "LineLeaf")
CEREAL_REGISTER_TYPE_WITH_NAME(beat::g2::TreeGenerator::RayLeaf, "RayLeaf")
CEREAL_REGISTER_TYPE_WITH_NAME(beat::g2::TreeGenerator::SegmentLeaf, "SegmentLeaf")
CEREAL_REGISTER_TYPE_WITH_NAME(beat::g2::TreeGenerator::AABBLeaf, "AABBLeaf")
CEREAL_REGISTER_TYPE_WITH_NAME(beat::g2::TreeGenerator::TriangleLeaf, "TriangleLeaf")
CEREAL_REGISTER_TYPE_WITH_NAME(beat::g2::TreeGenerator::CircleLeaf, "CircleLeaf")
CEREAL_REGISTER_TYPE_WITH_NAME(beat::g2::TreeGenerator::ConvexLeaf, "ConvexLeaf")
CEREAL_REGISTER_TYPE_WITH_NAME(beat::g2::TreeGenerator::CapsuleLeaf, "CapsuleLeaf")

CEREAL_REGISTER_POLYMORPHIC_RELATION(beat::g2::ITf, beat::g2::TreeGenerator::AABBNode)
CEREAL_REGISTER_POLYMORPHIC_RELATION(beat::g2::ITf, beat::g2::TreeGenerator::CircleNode)

CEREAL_REGISTER_POLYMORPHIC_RELATION(beat::g2::ITf, beat::g2::TreeGenerator::PointLeaf)
CEREAL_REGISTER_POLYMORPHIC_RELATION(beat::g2::ITf, beat::g2::TreeGenerator::LineLeaf)
CEREAL_REGISTER_POLYMORPHIC_RELATION(beat::g2::ITf, beat::g2::TreeGenerator::RayLeaf)
CEREAL_REGISTER_POLYMORPHIC_RELATION(beat::g2::ITf, beat::g2::TreeGenerator::SegmentLeaf)
CEREAL_REGISTER_POLYMORPHIC_RELATION(beat::g2::ITf, beat::g2::TreeGenerator::AABBLeaf)
CEREAL_REGISTER_POLYMORPHIC_RELATION(beat::g2::ITf, beat::g2::TreeGenerator::TriangleLeaf)
CEREAL_REGISTER_POLYMORPHIC_RELATION(beat::g2::ITf, beat::g2::TreeGenerator::CircleLeaf)
CEREAL_REGISTER_POLYMORPHIC_RELATION(beat::g2::ITf, beat::g2::TreeGenerator::ConvexLeaf)
CEREAL_REGISTER_POLYMORPHIC_RELATION(beat::g2::ITf, beat::g2::TreeGenerator::CapsuleLeaf)
