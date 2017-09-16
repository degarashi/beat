#include "segment2d.hpp"
#include "line2d.hpp"
#include "lubee/compare.hpp"
#include "constant.hpp"
#include "point2d.hpp"
#include "aabb2d.hpp"
#include "circle2d.hpp"

namespace beat {
	namespace g2 {
		Segment::Segment(const Vec2& v0, const Vec2& v1):
			from(v0), to(v1)
		{}
		float Segment::distance(const Segment& ls) const {
			if(hit(ls))
				return 0;

			auto ln = nearest(ls.from);
			float distsq = ls.from.dist_sq(ln.first);
			ln = nearest(ls.to);
			distsq = std::min(distsq, ls.to.dist_sq(ln.first));
			ln = ls.nearest(from);
			distsq = std::min(distsq, from.dist_sq(ln.first));
			ln = ls.nearest(to);
			distsq = std::min(distsq, to.dist_sq(ln.first));
			return std::sqrt(distsq);
		}
		float Segment::length() const {
			return from.distance(to);
		}
		float Segment::len_sq() const {
			return from.dist_sq(to);
		}
		LNear Segment::nearest(const Vec2& p) const {
			Vec2 toP(p-from),
				toV1(to-from);
			if(toV1.len_sq() < ZEROVEC_LENGTH_SQ)
				return LNear(from, LinePos::Begin);
			const float lenV1 = toV1.normalize();
			const float d = toV1.dot(toP);
			if(d <= 0)
				return LNear(from, LinePos::Begin);
			else if(d >= lenV1)
				return LNear(to, LinePos::End);
			else {
				const float t = d / lenV1;
				return LNear(from*(1-t) + to*t, LinePos::OnLine);
			}
		}
		Vec2_OP Segment::crossPoint(const Segment& s) const {
			if(auto cp = crossPoint(s.asLine())) {
				if(s.online(*cp))
					return cp;
			}
			if(auto cp = s.crossPoint(asLine())) {
				if(online(*cp))
					return cp;
			}
			return spi::none;
		}
		Vec2_OP Segment::crossPoint(const Line& l) const {
			float c0 = l.dir.dot(from - l.pos),
					c1 = l.dir.dot(to - l.pos);
			if(c0*c1 <= 0) {
				Vec2 diff(to-from);
				c0 = std::abs(c0);
				c1 = std::abs(c1);
				const float d = c0 / (c0+c1);
				return from + diff*d;
			}
			return spi::none;
		}
		Vec2 Segment::getDir() const {
			return (to-from).normalization();
		}
		Line Segment::asLine() const {
			return Line(from, (to-from).normalization());
		}
		float Segment::ratio(const Vec2& p) const {
			Vec2 toP(p-from),
				toV1(to-from);
			const float len = toV1.length();
			return toV1.dot(toP) / len;
		}
		bool Segment::online(const Vec2& p) const {
			Vec2 toV1(to-from),
				toP(p-from);
			const float len = toV1.normalize();
			const float d = toV1.dot(toP);
			return lubee::IsInRange(d, 0.f, len+ZEROVEC_LENGTH);
		}

		float Segment::bs_getArea() const {
			INVOKE_ERROR
		}
		float Segment::bs_getInertia() const {
			INVOKE_ERROR
		}
		Vec2 Segment::bs_getCenter() const {
			return (from + to) * 0.5f;
		}
		Circle Segment::bs_getBCircle() const {
			return Circle((from + to) * 0.5f,
							from.distance(to));
		}
		AABB Segment::bs_getBBox() const {
			AABB ab(from, from);
			ab.min.selectMin(to);
			ab.max.selectMax(to);
			return ab;
		}

		Vec2 Segment::support(const Vec2& dir) const {
			const float d[2] = {dir.dot(from), dir.dot(to)};
			if(d[0] > d[1])
				return from;
			return to;
		}
		Segment Segment::operator * (const AMat32& m) const {
			return Segment{
				from.convertI<3,2>(1)*m,
				to.convertI<3,2>(1)*m
			};
		}
		std::ostream& operator << (std::ostream& os, const Segment& s) {
			return os << "Segment(2d) [ from: " << s.from << ", to: " << s.to << ']';
		}

		bool Segment::hit(const Vec2& p) const {
			Vec2 v = p - from,
				 dir = to-from;
			const float len = dir.normalize();
			if(len < ZEROVEC_LENGTH)
				return Point(from).hit(p);

			constexpr float Th = 1e-3f;
			const float d = v.dot(dir);
			if(d < -Th)
				return false;
			if(d >= len+Th)
				return false;
			v -= dir*d;
			return v.len_sq() <= NEAR_THRESHOLD_SQ;
		}
		bool Segment::hit(const Segment& l) const {
			return IsCrossing(asLine(), l.asLine(), length(), l.length(), 0);
		}
	}
}
