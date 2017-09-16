#include "line2d.hpp"
#include "g2.hpp"
#include "constant.hpp"

namespace beat {
	namespace g2 {
		Line::Line(const Vec2& p, const Vec2& d) noexcept:
			pos(p), dir(d)
		{
			D_Assert0(std::abs(d.length()-1.f) < 1e-4f);
		}
		Vec2_OP Line::crossPoint(const Line& st) const noexcept {
			const auto fn = [](const auto v){ return v; };
			const auto res = NearestPoint(*this, st, fn, fn);
			if(!res)
				return spi::none;
			return res->first;
		}
		Vec2 Line::nearest(const Vec2& p) const noexcept {
			return pos + dir * ((p-pos).dot(dir));
		}
		float Line::distance(const Vec2& p) const noexcept {
			return std::abs(dir.dot(p - pos));
		}
		Vec2 Line::perpendicularDir() const noexcept {
			return {dir.y, -dir.x};
		}
		Line Line::perpendicular() const noexcept {
			return {pos, perpendicularDir()};
		}
		LineDivision Line::checkSide(const Vec2& p, const float t) const {
			const auto p2 = p - pos;
			const float d = dir.cw(p2);
			if(d < -t)
				return LineDivision::Ccw;
			if(d > t)
				return LineDivision::Cw;
			return LineDivision::OnLine;
		}

		float Line::bs_getArea() const {
			INVOKE_ERROR
		}
		float Line::bs_getInertia() const {
			INVOKE_ERROR
		}
		const Vec2& Line::bs_getCenter() const {
			return pos;
		}
		const Circle& Line::bs_getBCircle() const {
			INVOKE_ERROR
		}
		const AABB& Line::bs_getBBox() const {
			INVOKE_ERROR
		}
		Vec2 Line::support(const Vec2& dir) const noexcept {
			constexpr auto inf = std::numeric_limits<float>::infinity();
			if(this->dir.dot(dir) > 0)
				return pos + dir*inf;
			return pos + this->dir*-inf;
		}
		Line Line::operator * (const AMat32& m) const {
			return Line{
				pos.convertI<3,2>(1)*m,
				dir.convert<3>()*m
			};
		}
		std::ostream& operator << (std::ostream& os, const Line& l) {
			return os << "Line [ pos: " << l.pos << ", dir: " << l.dir << ']';
		}

		bool Line::hit(const Vec2& p) const {
			const Vec2 v = p - pos;
			const float d = std::abs(v.dot(dir));
			return (pos + dir*d).dist_sq(p) <= NEAR_THRESHOLD_SQ;
		}
	}
}
