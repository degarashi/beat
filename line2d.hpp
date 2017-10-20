#pragma once
#include "imodel2d.hpp"
#include "constant.hpp"
#include "g2.hpp"

namespace beat {
	using Vec2_OP = spi::Optional<Vec2>;
	using Vec2x2 = std::pair<Vec2, Vec2>;
	namespace g2 {
		struct Line : CID<Line> {
			Vec2	pos, dir;

			Line() = default;
			Line(const Vec2& p, const Vec2& d) noexcept;
			Vec2_OP crossPoint(const Line& st) const noexcept;
			Vec2 nearest(const Vec2& p) const noexcept;
			float distance(const Vec2& p) const noexcept;
			Vec2 perpendicularDir() const noexcept;
			Line perpendicular() const noexcept;
			LineDivision checkSide(const Vec2& p, float t=DOT_THRESHOLD) const;

			// -----------------------------
			float bs_getArea() const;
			float bs_getInertia() const;
			const Vec2& bs_getCenter() const;
			const Circle& bs_getBCircle() const;
			const AABB& bs_getBBox() const;

			Vec2 support(const Vec2& dir) const noexcept;
			Line operator * (const AMat32& m) const;
			bool operator == (const Line& l) const;
			bool operator != (const Line& l) const;
			friend std::ostream& operator << (std::ostream& os, const Line& l);
			// -----------------------------

			spi::none_t hit(...) const;
			bool hit(const Vec2& p) const;
		};
		using LineM = Model<Line>;
	}
}
