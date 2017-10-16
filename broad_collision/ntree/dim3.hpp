#pragma once
#include "defines.hpp"
#include <functional>

namespace beat {
	namespace g3 {
		class AABB;
	}
	namespace ntree {
		namespace g3 {
			class Index;
			using MortonId = MortonId<Index>;
			class Index {
				private:
					//! 10bitまでの数値を2つ飛びに変換する
					static Morton_t _SeparateBits2(Morton_t x);
					//! 32bitの2つ飛び10bit数値を元に戻す
					static Morton_t _ComposeBits2(Morton_t x);
				public:
					union {
						struct {
							Morton_t x:10,
									 y:10,
									 z:10;
						};
						Morton_t	value;
					};
					Index() = default;
					Index(uint32_t x, uint32_t y, uint32_t z);
					Index(MortonId m);
					operator MortonId() const;
					MortonId asMortonId() const;
					bool operator == (const Index& idx) const;
					bool operator != (const Index& idx) const;
			};
		}
	}
}
