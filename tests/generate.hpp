#pragma once
#include "lubee/tests/test.hpp"
#include "../random/convex2d.hpp"
#include "../random/point2d.hpp"
#include "../random/segment2d.hpp"
#include "../random/circle2d.hpp"
#include "../random/aabb2d.hpp"
#include "../random/capsule2d.hpp"
#include "../random/line2d.hpp"
#include "../random/ray2d.hpp"
#include "../random/triangle2d.hpp"

namespace beat {
	namespace g2 {
		using lubee::RangeF;
		class Generator : public lubee::test::Random {
			private:
				using RInt = std::function<int (const RangeF&)>;
				RInt	_ri;
				using RFloat = decltype(std::declval<lubee::test::Random>().mt().getUniformF<float>());
				RFloat	_rp,
						_rcirr,
						_rcapr;
			public:
				void setPointRange(const RangeF& r) {
					_rp = mt().getUniformF<float>(r);
				}
				void setCircleRadius(const RangeF& r) {
					_rcirr = mt().getUniformF<float>(r);
				}
				void setCapsuleRadius(const RangeF& r) {
					_rcapr = mt().getUniformF<float>(r);
				}
				Generator():
					_ri(mt().getUniformF<int>()),
					_rp(mt().getUniformF<float>(RangeF{1e2f})),
					_rcirr(mt().getUniformF<float>(RangeF{0, 1e1f})),
					_rcapr(mt().getUniformF<float>(RangeF{0, 1e1f}))
				{}
				void genShape(PointM& dst) { dst = genPoint(); }
				void genShape(SegmentM& dst) { dst = genSegment(); }
				void genShape(LineM& dst) { dst = genLine(); }
				void genShape(RayM& dst) { dst = genRay(); }
				void genShape(AABBM& dst) { dst = genAABB(); }
				void genShape(TriangleM& dst) { dst = genTriangleArea(); }
				void genShape(CapsuleM& dst) { dst = genCapsule(); }
				void genShape(CircleM& dst) { dst = genCircle(); }
				void genShape(ConvexM& dst) { dst = genConvex(); }

				Point genPoint() {
					return random::GenPoint(_rp);
				}
				Line genLine() {
					return random::GenLine(_rp);
				}
				Ray genRay() {
					return random::GenRay(_rp);
				}
				Segment genSegment() {
					return random::GenSegment(_rp);
				}
				Circle genCircle() {
					return random::GenCircle(_rp, _rcirr);
				}
				AABB genAABB() {
					return random::GenAABBArea(_rp, 1e-1f);
				}
				Capsule genCapsule() {
					return random::GenCapsule(_rp, _rcapr);
				}
				Vec2 genDir() {
					return frea::random::GenVecUnit<Vec2>(mt().getUniformF<float>());
				}
				Triangle genTriangleArea() {
					return random::GenTriangleArea(_rp, 1e-1f);
				}	
				Triangle genTriangle() {
					return random::GenTriangle(_rp);
				}	
				Convex genConvex(int n=-1) {
					auto& mt = this->mt();
					if(n < 0) {
						n = mt.getUniform<int>({3, 32+1});
					}
					return random::GenConvexArea(_rp, n, 1e-1f);
				}
				int genInt(const RangeF& r) {
					return _ri(r);
				}
		};
	}
}
