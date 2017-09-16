#include "point2d.hpp"
#include "circle2d.hpp"
#include "aabb2d.hpp"
#include "constant.hpp"

namespace beat {
	namespace g2 {
		float Point::bs_getArea() const {
			INVOKE_ERROR
		}
		float Point::bs_getInertia() const {
			INVOKE_ERROR
		}
		const Vec2& Point::bs_getCenter() const {
			return *this;
		}
		Circle Point::bs_getBCircle() const {
			// 円の半径が0だと点同士の時にヒットしないので微量含める
			return Circle(*this, NEAR_THRESHOLD);
		}
		AABB Point::bs_getBBox() const {
			const Vec2 d(NEAR_THRESHOLD);
			return AABB(*this - d,
						*this + d);
		}
		const Vec2& Point::support(const Vec2& /*dir*/) const {
			return *this;
		}
		Point Point::operator * (const AMat32& m) const {
			return Point(convertI<3,2>(1) * m);
		}
		std::ostream& operator << (std::ostream& os, const Point& c) {
			return os << "Point(2d) [ pos: " << static_cast<const Vec2&>(c) << ']';
		}
		bool Point::hit(const Vec2& p) const {
			return dist_sq(p) <= NEAR_THRESHOLD_SQ;
		}
	}
}
