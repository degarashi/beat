#pragma once
#include "imodel2d.hpp"

namespace beat {
	namespace g2 {
		struct Circle;
		class AABB;
		struct Point : Vec2, CID<Point> {
			using Vec2::Vec2;
			using Vec2::distance;

			// ----------------------------------------
			float bs_getArea() const;
			float bs_getInertia() const;
			const Vec2& bs_getCenter() const;
			Circle bs_getBCircle() const;
			AABB bs_getBBox() const;

			const Vec2& support(const Vec2& dir) const;
			Point operator * (const AMat32& m) const;
			friend std::ostream& operator << (std::ostream& os, const Point& p);
			// -----------------------------
			spi::none_t hit(...) const;
			bool hit(const Vec2& p) const;
		};
		using PointM = Model<Point>;
	}
}
