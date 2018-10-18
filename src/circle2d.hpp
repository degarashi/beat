#pragma once
#include "imodel2d.hpp"

namespace beat {
	namespace g2 {
		class AABB;
		struct Segment;
		struct Circle : CID<Circle> {
			Vec2	center;
			float	radius;
			struct NoValidCircle : std::invalid_argument {
				using std::invalid_argument::invalid_argument;
			};

			Circle() = default;
			Circle(const Vec2& c, float r);
			static Vec2 CircumscribingCenter(const Vec2& v0, const Vec2& v1, const Vec2& v2);
			static Circle FromEquationCoeff(const float l, const float m, const float n);
			static Circle Circumscribing(const Vec2& v0, const Vec2& v1, const Vec2& v2);
			static Circle CircumscribingM(const Vec2& v0, const Vec2& v1, const Vec2& v2);
			static Circle Encapsule(const AABB& a);
			void distend(float r);
			Circle operator * (float s) const;
			Circle& operator += (const Vec2& ofs);

			// -----------------------------
			float bs_getArea() const;
			float bs_getInertia() const;
			const Vec2& bs_getCenter() const;
			const Circle& bs_getBCircle() const;
			AABB bs_getBBox() const;

			Vec2 support(const Vec2& dir) const;
			Circle operator * (const AMat32& m) const;
			bool operator == (const Circle& c) const;
			bool operator != (const Circle& c) const;
			friend std::ostream& operator << (std::ostream& os, const Circle& c);
			// -----------------------------

			spi::none_t hit(...) const;
			bool hit(const Vec2& p) const;
			bool hit(const Segment& s) const;
			bool hit(const Circle& c) const;

			// ---- for MakeBoundary ----
			void setBoundary(const IModel* p);
			void appendBoundary(const IModel* p);
		};
		using CircleM = Model<Circle>;
	}
}
