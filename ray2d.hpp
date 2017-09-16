#pragma once
#include "imodel2d.hpp"

namespace beat {
	namespace g2 {
		using Vec2x2 = std::pair<Vec2, Vec2>;
		struct Line;
		//! 半直線
		struct Ray : CID<Ray> {
			Vec2	pos, dir;

			Ray() = default;
			Ray(const Vec2& p, const Vec2& d);
			const Line& asLine() const;
			Vec2x2 nearest(const Ray& r) const;
			Vec2 nearest(const Vec2& p) const;

			// -----------------------------
			float bs_getArea() const;
			float bs_getInertia() const;
			const Vec2& bs_getCenter() const;
			const Circle& bs_getBCircle() const;
			const AABB& bs_getBBox() const;

			Vec2 support(const Vec2& dir) const;
			Ray operator * (const AMat32& m) const;
			friend std::ostream& operator << (std::ostream& os, const Ray& r);
			// -----------------------------

			spi::none_t hit(...) const;
			bool hit(const Vec2& p) const;
		};
		using RayM = Model<Ray>;
	}
}
