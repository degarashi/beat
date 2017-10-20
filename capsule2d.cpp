#include "capsule2d.hpp"
#include "circle2d.hpp"
#include "aabb2d.hpp"
#include "segment2d.hpp"
#include "frea/ulps.hpp"

namespace beat {
	namespace g2 {
		Capsule::Capsule(const Vec2& f, const Vec2& t, const float r):
			from(f), to(t), radius(r)
		{
			D_Assert0(r >= 0);
		}
		Capsule::Capsule(const Segment& s, const float r):
			Capsule(s.from, s.to, r)
		{}

		float Capsule::bs_getArea() const {
			INVOKE_ERROR
		}
		float Capsule::bs_getInertia() const {
			INVOKE_ERROR
		}
		Vec2 Capsule::bs_getCenter() const {
			return (from+to) * 0.5f;
		}
		Vec2 Capsule::getDir() const {
			return (to - from).normalization();
		}
		const Segment& Capsule::asSegment() const {
			return reinterpret_cast<const Segment&>(*this);
		}
		Circle Capsule::bs_getBCircle() const {
			return Circle(bs_getCenter(),
							from.distance(to) + radius*2);
		}
		AABB Capsule::bs_getBBox() const {
			const auto dir = (to-from).normalization();
			return AABB(from - dir*radius,
						to + dir*radius);
		}
		Vec2 Capsule::support(const Vec2& dir) const {
			const auto d = frea::ulps::Move(radius, -(1<<14));
			if(from.dot(dir) > to.dot(dir)) {
				return from + dir*d;
			}
			return to + dir*d;
		}
		Capsule Capsule::operator * (const AMat32& m) const {
			Capsule c;
			c.from = from.convertI<3,2>(1) * m;
			c.to = to.convertI<3,2>(1) * m;
			c.radius = radius * m.convert<2,2>().calcDeterminant();
			return c;
		}
		bool Capsule::operator == (const Capsule& c) const {
			return from == c.from &&
					to == c.to &&
					radius == c.radius;
		}
		bool Capsule::operator != (const Capsule& c) const {
			return !(this->operator ==(c));
		}
		Capsule& Capsule::operator += (const Vec2& ofs) {
			from += ofs;
			to += ofs;
			return *this;
		}
		bool Capsule::hit(const Vec2& p) const {
			return Segment(from, to).nearest(p).first.distance(p) <= radius;
		}
		bool Capsule::hit(const Segment& s) const {
			Segment s0(from, to);
			return s0.distance(s) <= radius;
		}
		bool Capsule::hit(const Capsule& c) const {
			Segment s0(from, to),
					s1(c.from, c.to);
			return s0.distance(s1) <= radius+c.radius;
		}
		std::ostream& operator << (std::ostream& os, const Capsule& c) {
			return os << "Capsule(2d) [ from: " << c.from
				<< ", to: " << c.to
				<< ", radius: " << c.radius << ']';
		}
	}
}

