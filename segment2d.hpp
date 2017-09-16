#pragma once
#include "imodel2d.hpp"
#include "g.hpp"
#include "g2.hpp"

namespace beat {
	namespace g2 {
		//! 線分
		struct Segment : CID<Segment> {
			Vec2	from, to;

			Segment() = default;
			Segment(const Vec2& v0, const Vec2& v1);
			float distance(const Segment& l) const;
			float length() const;
			float len_sq() const;
			/*! \return Vec2(最寄り座標),LINEPOS(最寄り線分位置) */
			LNear nearest(const Vec2& p) const;
			//! 線分が交差する位置を調べる
			/*! \return first: 交差座標
						second: 交差しているライン位置 (ONLINE or NOHIT) */
			Vec2_OP crossPoint(const Segment& l) const;
			Vec2_OP crossPoint(const Line& l) const;
			Vec2 getDir() const;
			Line asLine() const;
			float ratio(const Vec2& p) const;
			bool online(const Vec2& p) const;

			// -----------------------------
			float bs_getArea() const;
			float bs_getInertia() const;
			Vec2 bs_getCenter() const;
			Circle bs_getBCircle() const;
			AABB bs_getBBox() const;

			Vec2 support(const Vec2& dir) const;
			Segment operator * (const AMat32& m) const;
			friend std::ostream& operator << (std::ostream& os, const Segment& s);
			// -----------------------------

			spi::none_t hit(...) const;
			bool hit(const Vec2& p) const;
			bool hit(const Segment& s) const;
		};
		using SegmentM = Model<Segment>;
	}
}
