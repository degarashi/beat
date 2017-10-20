#include "triangle2d.hpp"
#include "circle2d.hpp"
#include "line2d.hpp"
#include "aabb2d.hpp"
#include "conditional.hpp"
#include "segment2d.hpp"

namespace beat {
	namespace g2 {
		using frea::AVec2;
		Triangle::Triangle(const Vec2& p0, const Vec2& p1, const Vec2& p2) noexcept:
			pos{p0, p1, p2}
		{}
		float Triangle::bs_getArea() const {
			return area();
		}
		float Triangle::bs_getInertia() const {
			return (1.0f/18) * (pos[0].dot(pos[0])
									+ pos[0].dot(pos[0])
									+ pos[0].dot(pos[0])
									- pos[1].dot(pos[2])
									- pos[2].dot(pos[0])
									- pos[0].dot(pos[1]));
		}
		Circle Triangle::bs_getBCircle() const {
			int id = getObtuseCorner();
			if(id >= 0) {
				// 鈍角を持っていれば直径を使う
				const Vec2 &v0 = pos[CndAdd(id-1, 3)],
							&v1 = pos[CndSub(id+1, 3)];
				return Circle((v0+v1)*0.5f, v0.distance(v1));
			} else {
				// なければ3点の外接円
				Line line0((pos[1]+pos[0]) * 0.5f,
							((pos[1]-pos[0]) * cs_mRot90[0]).normalization()),
					line1((pos[2]+pos[0]) * 0.5f,
							((pos[2]-pos[0]) * cs_mRot90[0]).normalization());
				const auto vp = *line0.crossPoint(line1);
				return Circle(vp,
							vp.distance(pos[0]));
			}
		}
		AABB Triangle::bs_getBBox() const {
			AABB ab(pos[0], pos[0]);
			for(int i=1 ; i<3 ; i++) {
				ab.min.selectMin(pos[i]);
				ab.max.selectMax(pos[i]);
			}
			return ab;
		}
		int Triangle::getObtuseCorner() const {
			AVec2 v01(pos[1]-pos[0]),
				v02(pos[2]-pos[0]),
				v12(pos[2]-pos[1]);
			if(v01.dot(v02) < 0)
				return 0;

			v01 *= -1;
			if(v01.dot(v12) < 0)
				return 1;

			v12 *= -1;
			v02 *= -1;
			if(v02.dot(v12) < 0)
				return 2;
			return -1;
		}
		Triangle Triangle::operator * (const AMat32& m) const {
			Triangle ret;
			for(int i=0 ; i<3 ; i++)
				ret.pos[i] = pos[i].convertI<3,2>(1) * m;
			return ret;
		}
		bool Triangle::operator == (const Triangle& t) const {
			return pos[0] == t.pos[0] &&
					pos[1] == t.pos[1] &&
					pos[2] == t.pos[2];
		}
		bool Triangle::operator != (const Triangle& t) const {
			return !(this->operator ==(t));
		}
		Triangle Triangle::Encapsule(const AABB& a) {
			return Encapsule(Circle::Encapsule(a));
		}
		Triangle Triangle::Encapsule(const Circle& c) {
			const auto& ce = c.center;
			const auto r = c.radius;
			const auto dx = 2*r,
						dy0 = r,
						dy1 = Sqrt3*r;
			return {
				{ce.x - dx, ce.y - dy0},
				{ce.x, ce.y + dy1},
				{ce.x + dx, ce.y - dy0}
			};
		}
		float Triangle::CalcArea(const Vec2& v0, const Vec2& v1) {
			return std::abs(v0.cw(v1)) / 2;
		}
		float Triangle::CalcArea(const Vec2& v0, const Vec2& v1, const Vec2& v2) {
			return (v1-v0).cw(v2-v0) / 2;
		}
		float Triangle::area() const {
			return CalcArea(pos[0], pos[1], pos[2]);
		}
		Vec2 Triangle::onArea(const float c0, const float c1, const float c2) const {
			return pos[0]*c0 + pos[1]*c1 + pos[2]*c2;
		}
		void Triangle::flip() {
			std::swap(pos[1], pos[2]);
		}
		bool Triangle::isClockwise() const {
			return area() >= 0;
		}
		void Triangle::clockwise() {
			if(!isClockwise())
				flip();
		}
		bool Triangle::hit(const Vec2& p) const {
			return (pos[1] - pos[0]).cw(p - pos[0]) >= 0 &&
					(pos[2] - pos[1]).cw(p - pos[1]) >= 0 &&
					(pos[0] - pos[2]).cw(p - pos[2]) >= 0;
		}
		bool Triangle::hit(const Segment& s) const {
			for(int i=0 ; i<3 ; i++) {
				if(Segment{pos[i], pos[(i+1)%3]}.hit(s))
					return true;
			}
			return hit(s.from);
		}
		bool Triangle::hit(const Triangle& t) const {
			for(int i=0 ; i<3 ; i++) {
				const Segment s0(pos[i], pos[(i+1)%3]);
				for(int j=0 ; j<3 ; j++) {
					if(Segment(t.pos[j], t.pos[(j+1)%3]).hit(s0))
						return true;
				}
			}
			return hit(t.pos[0]) || t.hit(pos[0]);
		}
		Vec2 Triangle::barycentricCoord(const Vec2& p) const {
			const Vec2 	toV1(pos[1]-pos[0]),
						toV2(pos[2]-pos[0]),
						toVT(p-pos[0]);
			const float det = CramerDet(toV1, toV2);
			return CramersRule(toV1, toV2, toVT, 1.f/det);
		}
		Vec2 Triangle::support(const Vec2& dir) const {
			int index = -1;
			float d = std::numeric_limits<float>::lowest();
			for(int i=0 ; i<3 ; i++) {
				const float td = pos[i].dot(dir);
				if(td > d) {
					d = td;
					index = i;
				}
			}
			return pos[index];
		}
		LineDivision Triangle::checkSide(const Line& l, const float t) const {
			uint32_t res = 0;
			for(int i=0 ; i<3 ; i++)
				res |= l.checkSide(pos[i], t);
			return static_cast<LineDivision>(res);
		}
		Vec2 Triangle::bs_getCenter() const {
			return (pos[0] + pos[1] + pos[2]) * (1.0f/3);
		}
		Triangle& Triangle::operator += (const Vec2& ofs) {
			for(auto& v : pos)
				v += ofs;
			return *this;
		}
		void Triangle::distend(const float r) {
			const Vec2 c = bs_getCenter();
			for(auto& v : pos) {
				auto dir = v - c;
				float dist = dir.normalize();
				dist *= r;
				v = c.l_intp(v, r);
			}
		}
		std::ostream& operator << (std::ostream& os, const Triangle& p) {
			return os << "Triangle(2d) [ 0: " << p.pos[0]
						<< ", 1: " << p.pos[1]
						<< ", 2: " << p.pos[2] << ']';
		}
	}
}
