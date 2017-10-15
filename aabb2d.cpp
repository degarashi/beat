#include "aabb2d.hpp"
#include "lubee/compare.hpp"
#include "circle2d.hpp"
#include "segment2d.hpp"

namespace beat {
	namespace g2 {
		int AABB::_getAreaNumX(const float p) const {
			if(p < min.x)
				return 0x00;
			if(p < max.x)
				return 0x01;
			return 0x02;
		}
		int AABB::_getAreaNumY(const float p) const {
			if(p < min.y)
				return 0x00;
			if(p < max.y)
				return 0x01;
			return 0x02;
		}
		void AABB::_makeSegmentX(Segment& s, const int num) const {
			s.from.x = s.to.x = ((num==0x01) ? min.x : max.x);
			s.from.y = min.y;
			s.to.y = max.y;
		}
		void AABB::_makeSegmentY(Segment& s, const int num) const {
			s.from.y = s.to.y = ((num==0x01) ? min.y : max.y);
			s.from.x = min.x;
			s.to.x = max.x;
		}
		bool AABB::_checkHitX(const Segment& s0, int from, int to) const {
			if(to < from)
				std::swap(from, to);
			Segment s1;
			while(from != to) {
				_makeSegmentX(s1, ++from);
				if(s0.hit(s1))
					return true;
			}
			return false;
		}
		bool AABB::_checkHitY(const Segment& s0, int from, int to) const {
			if(to < from)
				std::swap(from, to);
			Segment s1;
			while(from != to) {
				_makeSegmentY(s1, ++from);
				if(s0.hit(s1))
					return true;
			}
			return false;
		}
		std::pair<int,int> AABB::_getAreaNum(const Vec2& p) const {
			return std::make_pair(_getAreaNumX(p.x),
									_getAreaNumY(p.y));
		}

		AABB::AABB(const Vec2& min_v, const Vec2& max_v):
			min(min_v), max(max_v)
		{
			D_Assert0(min.x <= max.x && min.y <= max.y);
		}
		AABB::AABB(const lubee::RectF& r):
			min(r.x0, r.y0),
			max(r.x1, r.y1)
		{}
		float AABB::width() const {
			return max.x - min.x;
		}
		float AABB::height() const {
			return max.y - min.y;
		}
		Vec2 AABB::nearest(const Vec2& pos) const {
			return pos.getMax(min).getMin(max);
		}
		void AABB::distend(const float r) {
			D_Assert0(r >= 0);
			const Vec2 center = (min + max) * .5f;
			Vec2 diff = max - min;
			diff *= r * .5f;
			min = center - diff;
			max = center + diff;
		}
		void AABB::encapsule(const Vec2& v) {
			min.x = std::min(min.x, v.x);
			max.x = std::max(max.x, v.x);
			min.y = std::min(min.y, v.y);
			max.y = std::max(max.y, v.y);
		}

		float AABB::bs_getInertia() const {
			throw std::domain_error("not implemented yet");
		}
		float AABB::bs_getArea() const {
			const Vec2 sz = max - min;
			return sz.x * sz.y;
		}
		Vec2 AABB::bs_getCenter() const {
			return (min + max) * 0.5f;
		}
		Circle AABB::bs_getBCircle() const {
			// 対角線 = 直径
			return Circle((min + max) * .5f,
							min.distance(max) * .5f);
		}
		const AABB& AABB::bs_getBBox() const {
			return *this;
		}

		Vec2 AABB::support(const Vec2& dir) const {
			return {(dir.x > 0 ? max.x : min.x),
					(dir.y > 0 ? max.y : min.y)};
		}
		AABB& AABB::operator += (const Vec2& ofs) {
			min += ofs;
			max += ofs;
			return *this;
		}
		AABB AABB::operator * (const AMat32& m) const {
			using frea::Vec3;
			AABB ret;
			// AABBを座標変換した物を包むAABB
			const Vec2 v[] = {
				min.convertI<3,2>(1) * m,
				Vec3(min.x, max.y, 1) * m,
				Vec3(max.x, min.y, 1) * m,
				max.convertI<3,2>(1) * m
			};
			ret.min = ret.max = v[0];
			for(int i=1 ; i<4 ; i++) {
				ret.min.selectMin(v[i]);
				ret.max.selectMax(v[i]);
			}
			return ret;
		}
		bool AABB::operator == (const AABB& a) const {
			return min == a.min
				&& max == a.max;
		}
		bool AABB::operator != (const AABB& a) const {
			return !(this->operator == (a));
		}
		std::ostream& operator << (std::ostream& os, const AABB& a) {
			return os << "AABB(2d) [ min: " << a.min << ", max: " << a.max << ']';
		}

		bool AABB::hit(const Vec2& pos) const {
			return lubee::IsInRange(pos.x, min.x, max.x) &&
					lubee::IsInRange(pos.y, min.y, max.y);
		}
		bool AABB::hit(const Segment& l) const {
			const auto f0 = _getAreaNum(l.from),
						f1 = _getAreaNum(l.to);
			const auto diffX = std::abs(f0.first - f1.first);
			const auto diffY = std::abs(f0.second - f1.second);
			// 片方の点が中央位置にあればヒットしている
			if(f0.first==1 && f0.second==1)
				return true;
			if(f1.first==1 && f1.second==1)
				return true;
			// 片方の軸に変化が無ければ領域のみで判定可
			if(diffX==0)
				return f0.first==0x01 && diffY==2;
			if(diffY==0)
				return f0.second==0x01 && diffX==2;

			// X軸について判定
			if(_checkHitX(l, f0.first, f1.first))
				return true;
			// Y軸について判定
			return _checkHitY(l, f0.second, f1.second);
		}
		bool AABB::hit(const AABB& ab) const {
			if(ab.max.x < min.x)
				return false;
			if(ab.min.x > max.x)
				return false;
			if(ab.max.y < min.y)
				return false;
			if(ab.min.y > max.y)
				return false;
			return true;
		}

		void AABB::setBoundary(const IModel* p) {
			min.x = p->im_support(Vec2(-1,0)).x;
			max.x = p->im_support(Vec2(1,0)).x;
			min.y = p->im_support(Vec2(0,-1)).y;
			max.y = p->im_support(Vec2(0,1)).y;
		}
		void AABB::appendBoundary(const IModel* p) {
			min.x = std::min(min.x, p->im_support(Vec2(-1,0)).x);
			max.x = std::max(max.x, p->im_support(Vec2(1,0)).x);
			min.y = std::min(min.y, p->im_support(Vec2(0,-1)).y);
			max.y = std::max(max.y, p->im_support(Vec2(0,1)).y);
		}
	}
}
