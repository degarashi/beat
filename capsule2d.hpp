#pragma once
#include "imodel2d.hpp"

namespace beat {
	namespace g2 {
		struct Segment;
		struct Capsule : CID<Capsule> {
			Vec2	from, to;
			float	radius;

			Capsule() = default;
			Capsule(const Vec2& f, const Vec2& t, float r);
			Capsule(const Segment& s, float r);
			Vec2 getDir() const;
			const Segment& asSegment() const;

			// -----------------------------
			float bs_getArea() const;
			float bs_getInertia() const;
			Vec2 bs_getCenter() const;
			Circle bs_getBCircle() const;
			AABB bs_getBBox() const;

			Vec2 support(const Vec2& dir) const;
			Capsule operator * (const AMat32& m) const;
			Capsule& operator += (const Vec2& ofs);
			friend std::ostream& operator << (std::ostream& os, const Capsule& c);
			// -----------------------------

			spi::none_t hit(...) const;
			bool hit(const Vec2& p) const;
			bool hit(const Segment& s) const;
			bool hit(const Capsule& c) const;
		};
		using CapsuleM = Model<Capsule>;
	}
}
