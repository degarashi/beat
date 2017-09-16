#pragma once
#include "g2.hpp"
#include "constant.hpp"
#include "imodel2d.hpp"

namespace beat {
	namespace g2 {
		class AABB;
		struct Circle;
		struct Triangle : CID<Triangle> {
			Vec2	pos[3];

			Triangle() = default;
			Triangle(const Vec2& p0, const Vec2& p1, const Vec2& p2) noexcept;
			static Triangle Encapsule(const AABB& a);
			static Triangle Encapsule(const Circle& c);
			//! 鈍角を探す
			/*! \return 鈍角の番号 (負数は該当なし) */
			int getObtuseCorner() const;
			//! 頂点の並びを反転
			void flip();
			//! 頂点の並びが時計回りかを判定
			bool isClockwise() const;
			void clockwise();
			static float CalcArea(const Vec2& v0, const Vec2& v1);
			static float CalcArea(const Vec2& v0, const Vec2& v1, const Vec2& v2);
			float area() const;
			Vec2 onArea(const float c0, const float c1, const float c2) const;
			Vec2 support(const Vec2& dir) const;
			Vec2 barycentricCoord(const Vec2& p) const;
			LineDivision checkSide(const Line& l, float t=DOT_THRESHOLD) const;
			Triangle& operator += (const Vec2& ofs);
			void distend(float r);

			// -----------------------------
			float bs_getArea() const;
			float bs_getInertia() const;
			Vec2 bs_getCenter() const;
			Circle bs_getBCircle() const;
			AABB bs_getBBox() const;

			Triangle operator * (const AMat32& m) const;
			friend std::ostream& operator << (std::ostream& os, const Triangle& p);
			// -----------------------------

			spi::none_t hit(...) const;
			bool hit(const Vec2& p) const;
			bool hit(const Segment& s) const;
			bool hit(const Triangle& t) const;
		};
		using TriangleM = Model<Triangle>;
	}
}
