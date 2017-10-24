#pragma once
#include <cstdint>
#include <algorithm>

namespace beat {
	namespace ntree {
		using Morton_t = uint32_t;
		template <class Idx>
		struct MortonId {
			Morton_t	value;

			MortonId() = default;
			MortonId(const Morton_t m):
				value(m)
			{}
			operator Idx () const {
				return asIndex();
			}
			Idx asIndex() const {
				return {value};
			}
			bool operator == (const MortonId& m) const noexcept {
				return value == m.value;
			}
		};
		template <class Idx>
		using MortonPair = std::pair<MortonId<Idx>, MortonId<Idx>>;

		using VolumeId = uint32_t;
		template <class Idx>
		struct VolumeEntry {
			Idx			min,
						max;
			VolumeId	id;

			//! デバッグ用
			bool operator == (const VolumeEntry& v) const noexcept {
				// idは比較しない
				return min == v.min &&
						max == v.max;
			}
		};
	}
}
namespace std {
	template <class Idx>
	struct hash<beat::ntree::VolumeEntry<Idx>> {
		std::size_t operator ()(const beat::ntree::VolumeEntry<Idx>& ve) const noexcept {
			return (ve.min.value + 0xdeadbeef) ^ ve.max.value;
		}
	};
}
