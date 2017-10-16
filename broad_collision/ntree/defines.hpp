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
		};
		template <class Idx>
		using MortonPair = std::pair<MortonId<Idx>, MortonId<Idx>>;

		using VolumeId = uint32_t;
		template <class Idx>
		struct VolumeEntry {
			Idx			min,
						max;
			VolumeId	id;
		};
	}
}
