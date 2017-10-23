#pragma once
#include "tfleaf2d.hpp"

namespace beat {
	namespace g2 {
		//! キャッシュ有りのTfLeaf
		template <class Shape, class Ud=spi::none_t>
		class TfLeafCached : public TfLeaf<Ud> {
			private:
				using base_t = TfLeaf<Ud>;

				using Ac_OP = spi::Optional<uint32_t>;
				mutable Ac_OP	_baseAccum;
				mutable Shape	_tfShape;
				const Shape& _getTfShape() const {
					const auto accum = base_t::getAccum();
					if(!_baseAccum || *_baseAccum != accum) {
						_baseAccum = accum;
						this->getModel()->im_transform(&_tfShape, base_t::getGlobal());
					}
					return _tfShape;
				}
				template <class Ar, class S, class UD>
				friend void save(Ar&, const TfLeafCached<S,UD>&);
				template <class Ar, class S, class UD>
				friend void load(Ar&, TfLeafCached<S,UD>&);
			public:
				using base_t::base_t;
				bool im_canCacheShape() const override {
					return true;
				}
				Vec2 im_support(const Vec2& dir) const override {
					// 予め変換しておいた形状でサポート写像
					return _getTfShape().support(dir);
				}
				void im_getBVolume(Circle& c) const override {
					c = _getTfShape().bs_getBCircle();
				}
				void im_getBVolume(AABB& a) const override {
					a = _getTfShape().bs_getBBox();
				}
				const void* im_getCore() const override {
					return &_getTfShape();
				}
				void* im_getCore() override {
					_getTfShape();
					// データが改変されるかも知れないので更新フラグを立てる
					_baseAccum = spi::none;
					return &_tfShape;
				}
		};
	}
}
