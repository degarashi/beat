#include "ray2d.hpp"
#include "g2.hpp"
#include "constant.hpp"

namespace beat {
	namespace g2 {
		Ray::Ray(const Vec2& p, const Vec2& d):
			pos(p), dir(d)
		{}
		const Line& Ray::asLine() const {
			return *reinterpret_cast<const Line*>(this);
		}
		Vec2x2 Ray::nearest(const Ray& r) const {
			const auto fn = [](const float v) { return std::max(0.f, v); };
			if(auto v2 = NearestPoint(asLine(), r.asLine(), fn, fn))
				return *v2;
			auto p = nearest(r.pos);
			return {p, r.pos};
		}
		Vec2 Ray::nearest(const Vec2& p) const {
			return NearestPoint(asLine(), p, [](const float f){ return std::max(0.f,f); });
		}

		float Ray::bs_getArea() const {
			INVOKE_ERROR
		}
		float Ray::bs_getInertia() const {
			INVOKE_ERROR
		}
		const Vec2& Ray::bs_getCenter() const {
			return pos;
		}
		const Circle& Ray::bs_getBCircle() const {
			INVOKE_ERROR
		}
		const AABB& Ray::bs_getBBox() const {
			INVOKE_ERROR
		}

		Vec2 Ray::support(const Vec2& dir) const {
			constexpr auto inf = std::numeric_limits<float>::infinity();
			if(dir.dot(dir) > 0)
				return pos + dir*inf;
			return pos;
		}
		Ray Ray::operator * (const AMat32& m) const {
			return Ray{
				pos.convertI<3,2>(1)*m,
				dir.convert<3>()*m
			};
		}
		std::ostream& operator << (std::ostream& os, const Ray& r) {
			return os << "Ray [ pos: " << r.pos << ", dir: " << r.dir << ']';
		}

		bool Ray::hit(const Vec2& p) const {
			const Vec2 v = p - pos;
			const float d = v.dot(dir);
			if(d < 0)
				return false;
			return (pos + dir*d).dist_sq(p) <= NEAR_THRESHOLD_SQ;
		}
	}
}
