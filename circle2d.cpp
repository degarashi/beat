#include "circle2d.hpp"
#include "frea/constant.hpp"
#include "frea/ulps.hpp"
#include "segment2d.hpp"
#include "aabb2d.hpp"
#include "constant.hpp"

namespace beat {
	namespace g2 {
		using frea::Square;
		Circle::Circle(const Vec2& c, const float r):
			center(c), radius(r)
		{
			D_Assert0(r >= 0);
		}
		Vec2 Circle::CircumscribingCenter(const Vec2& v0, const Vec2& v1, const Vec2& v2) {
			const Vec2 s0 = Square(v0),
						 s1 = Square(v1),
						 s2 = Square(v2);
			const float C = 2 * ((v0.y - v2.y) * (v0.x - v1.x) - (v0.y - v1.y) * (v0.x - v2.x));
			if(std::abs(C) <= 1e-5f)
				throw NoValidCircle("Circle::CircumscribingCenter");
			const float x = ((s0.x - s1.x + s0.y - s1.y) * (v0.y - v2.y) - (s0.x - s2.x + s0.y - s2.y) * (v0.y - v1.y)) / C;
			const float y = ((s0.x - s2.x + s0.y - s2.y) * (v0.x - v1.x) - (s0.x - s1.x + s0.y - s1.y) * (v0.x - v2.x)) / C;
			return {x, y};
		}
		Circle Circle::FromEquationCoeff(const float l, const float m, const float n) {
			const Vec2 c{-l/2, -m/2};
			const float r = std::sqrt(-n + Square(c.x) + Square(c.y));
			return Circle{c, r};
		}
		Circle Circle::Circumscribing(const Vec2& v0, const Vec2& v1, const Vec2& v2) {
			const Vec2 c = CircumscribingCenter(v0, v1, v2);
			return Circle{c, c.distance(v0)};
		}
		Circle Circle::Encapsule(const AABB& a) {
			const float rd = std::sqrt(Square(a.width()) + Square(a.height()))/2;
			return Circle{a.bs_getCenter(), frea::ulps::Move(rd,64)};
		}
		void Circle::distend(const float r) {
			radius *= r;
		}

		// 半径のみ倍率をかける
		Circle Circle::operator * (const float s) const {
			return Circle(center, radius*s);
		}
		Circle& Circle::operator += (const Vec2& ofs) {
			center += ofs;
			return *this;
		}
		std::ostream& operator << (std::ostream& os, const Circle& c) {
			return os << "Circle(2d) [ center: " << c.center << ", radius: " << c.radius << ']';
		}
		float Circle::bs_getArea() const {
			AssertF("not implemented yet");
		}
		float Circle::bs_getInertia() const {
			AssertF("not implemented yet");
		}
		const Vec2& Circle::bs_getCenter() const {
			return center;
		}
		const Circle& Circle::bs_getBCircle() const {
			return *this;
		}
		AABB Circle::bs_getBBox() const {
			Vec2 d(radius);
			return AABB(center - d,
						center + d);
		}
		Circle Circle::operator * (const AMat32& m) const {
			auto& m2 = reinterpret_cast<const frea::AMat2&>(m);
			Vec2 tx(radius,0),
				ty(0,radius),
				ori(center);
			ori = ori.convertI<3,2>(1) * m;
			tx = tx * m2;
			ty = ty * m2;
			return Circle(ori,
						std::sqrt(std::max(tx.len_sq(), ty.len_sq())));
		}

		Vec2 Circle::support(const Vec2& dir) const {
			return dir * radius + center;
		}
		bool Circle::hit(const Vec2& p) const {
			return center.dist_sq(p) <= Square(radius);
		}
		bool Circle::hit(const Circle& c) const {
			return center.dist_sq(c.center) <= Square(radius + c.radius);
		}
		bool Circle::hit(const Segment& s) const {
			const Vec2 np = s.nearest(center).first;
			return np.dist_sq(center) <= Square(radius);
		}
		using Mat34 = frea::Mat_t<float, 3, 4, false>;
		Circle Circle::CircumscribingM(const Vec2& v0, const Vec2& v1, const Vec2& v2) {
			Mat34 mat;
			const auto set = [&mat](const int n, const auto& v) {
				mat.m[n][0] = v.x;
				mat.m[n][1] = v.y;
				mat.m[n][2] = 1;
				mat.m[n][3] = -Square(v.x) -Square(v.y);
			};
			set(0, v0);
			set(1, v1);
			set(2, v2);
			mat.rowReduce(1e-5f);
			return Circle::FromEquationCoeff(mat.m[0][3], mat.m[1][3], mat.m[2][3]);
		}
		void Circle::setBoundary(const IModel* p) {
			p->im_getBVolume(*this);
		}
		void Circle::appendBoundary(const IModel* p) {
			Circle c2;
			p->im_getBVolume(c2);
			Vec2 toC2 = c2.center - center;
			const float lensq = toC2.len_sq();
			if(lensq >= ZEROVEC_LENGTH_SQ) {
				toC2 *= 1.0f / std::sqrt(lensq);
				const Vec2 tv(support(toC2) - c2.support(-toC2));
				const float r_min = std::min(radius, c2.radius);
				if(tv.dot(toC2) < 0 ||
					tv.len_sq() < frea::Square(r_min*2))
				{
					// 新たな円を算出
					const Vec2 tv0(center - toC2*radius),
								 tv1(c2.center + toC2*c2.radius);
					radius = tv0.distance(tv1) * .5f;
					center = (tv0+tv1) * .5f;
				} else {
					// 円が内包されている
					if(radius < c2.radius) {
						radius = c2.radius;
						center = c2.center;
					}
				}
			} else {
				// 円の中心が同じ位置にある
				radius = std::max(c2.radius, radius);
			}
		}
	}
}
