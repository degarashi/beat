#include "random.hpp"
#include "frea/ulps.hpp"
#include "../gjk2d.hpp"
#include "generate.hpp"

namespace beat {
	namespace g2 {
		using lubee::RangeF;
		using frea::Vec2;
		struct Triangle2D : Generator {};
		TEST_F(Triangle2D, Encapsule_AABB) {
			const auto ab = genAABB();
			const auto t = Triangle::Encapsule(ab);
			ASSERT_TRUE(t.hit(Vec2{ab.min.x, ab.min.y}));
			ASSERT_TRUE(t.hit(Vec2{ab.max.x, ab.min.y}));
			ASSERT_TRUE(t.hit(Vec2{ab.min.x, ab.max.y}));
			ASSERT_TRUE(t.hit(Vec2{ab.max.x, ab.max.y}));
		}
		TEST_F(Triangle2D, Hit_Point_GJK) {
			const TriangleM tri = genTriangleArea();
			const PointM pt = genPoint();
			const auto b0 = tri.hit(pt),
						b1 = GSimplex(tri, pt).getResult();
			ASSERT_EQ(b0, b1);
		}
		TEST_F(Triangle2D, Hit_Segment_GJK) {
			TriangleM tri = genTriangleArea();
			const SegmentM seg = genSegment();
			auto b0 = tri.hit(seg),
				b1 = GSimplex(tri, seg).getResult();
			if(b0 != b1) {
				tri.distend(1.f-1e-3f);
				b0 = tri.hit(seg);
				b1 = GSimplex(tri, seg).getResult();
			}
			ASSERT_EQ(b0, b1);
		}
		TEST_F(Triangle2D, Hit_Triangle_GJK) {
			TriangleM t0 = genTriangleArea(),
						t1 = genTriangleArea();
			bool b0 = t0.hit(t1),
				b1 = GSimplex(t0, t1).getResult();
			if(b0 != b1) {
				t0.distend(1.f-1e-3f);
				b0 = t0.hit(t1);
				b1 = GSimplex(t0, t1).getResult();
			}
			ASSERT_EQ(b0, b1);
		}
		TEST_F(Triangle2D, Encapsule_Circle) {
			const auto c = genCircle();
			const auto t = Triangle::Encapsule(c);
			const Vec2 toHit = c.center + genDir() * frea::ulps::Move(c.radius, -(1<<12));
			ASSERT_TRUE(t.hit(toHit));
		}
		TEST_F(Triangle2D, Clockwise) {
			auto t = genTriangleArea();
			const bool b0 = t.isClockwise();
			t.flip();
			const bool b1 = t.isClockwise();
			ASSERT_TRUE(b0 ^ b1);
		}
		TEST_F(Triangle2D, OnArea) {
			const auto t = genTriangleArea();
			auto& mt = this->mt();
			auto rdp = mt.getUniformF<float>(RangeF{1e2f});
			const auto c = GenerateCoeff(rdp, 3);
			const auto p = t.onArea(c[0], c[1], c[2]);
			ASSERT_TRUE(t.hit(p));
		}
		TEST_F(Triangle2D, BVolume) {
			const TriangleM p(genTriangle());
			const Circle c = p.bs_getBCircle();
			for(auto& p2 : p.pos) {
				ASSERT_LE(c.center.distance(p2), c.radius+1e-3f);
			}
		}
		TEST_F(Triangle2D, Triangle) {
			TriangleM t(genTriangleArea());
			// 3頂点を合計1になるような係数で合成した座標は必ず三角形の中に入る
			auto& mt = this->mt();
			auto rdi = mt.getUniformF<int>();
			const auto c = GenerateCoeff(rdi, 3);
			ASSERT_NEAR(c[0]+c[1]+c[2], 1.f, 1e-5f);
			const Vec2 pos = t.onArea(c[0], c[1], c[2]);
			EXPECT_TRUE(t.hit(pos));

			// 頂点順が逆ならば結果も逆になる(辺上の場合は常にflaseだから問題なし)
			t.flip();
			EXPECT_FALSE(t.hit(pos));
		}
		TEST_F(Triangle2D, Support) {
			const auto tri = genTriangleArea();
			const auto dir = genDir();
			const auto pos = tri.support(dir);
			ASSERT_TRUE(tri.hit(pos));
		}
	}
}
