#include "../gjk2d.hpp"
#include "generate.hpp"
#include "frea/ulps.hpp"

namespace beat {
	namespace g2 {
		using lubee::RangeF;
		struct Circle2D : Generator {
			Circle2D() {
				setCircleRadius({1e-2f, 1e1f});
			}
		};
		TEST_F(Circle2D, Hit_Point) {
			const auto c = genCircle();
			const auto dir = genDir();
			const Vec2 noHit = c.center + dir * frea::ulps::Move(c.radius, 1<<14);
			ASSERT_FALSE(c.hit(noHit));
			const Vec2 mustHit = c.center + dir * frea::ulps::Move(c.radius, -(1<<14));
			ASSERT_TRUE(c.hit(mustHit));
		}
		TEST_F(Circle2D, Hit_Point_GJK) {
			const CircleM c = genCircle();
			const PointM p = genPoint();
			const bool b0 = c.hit(p);
			const bool b1 = GSimplex(c, p).getResult();
			ASSERT_EQ(b0, b1);
		}
		TEST_F(Circle2D, Hit_Circle_GJK) {
			const CircleM c0 = genCircle();
			const CircleM c1 = genCircle();
			const bool b0 = c0.hit(c1);
			const bool b1 = GSimplex(c0, c1).getResult();
			ASSERT_EQ(b0, b1);
		}
		TEST_F(Circle2D, Hit_Segment_GJK) {
			auto& mt = this->mt();
			auto rdp = mt.getUniformF<float>(RangeF{1e2f});
			const CircleM c = genCircle();
			const SegmentM s = random::GenSegment(rdp);
			const bool b0 = c.hit(s);
			const bool b1 = GSimplex(c, s).getResult();
			ASSERT_EQ(b0, b1);
		}
		TEST_F(Circle2D, Support) {
			const CircleM c = genCircle();
			const PointM dir = genDir();
			const Vec2 sv = c.support(dir);
			// 中心座標からの距離は全てradiusと等しい筈
			EXPECT_NEAR((sv - c.center).length(), c.radius, 1e-3f);
			// 方向ベクトルとの内積は限りなくradiusに近い筈
			EXPECT_NEAR((sv - c.center).dot(dir), c.radius, 1e-3);
		}
		TEST_F(Circle2D, Encapsule_Rect) {
			auto& mt = this->mt();
			auto rdp = mt.getUniformF<float>(RangeF{1e2f});
			const auto ab = random::GenAABB(rdp);
			Circle c = Circle::Encapsule(ab);
			ASSERT_TRUE(c.hit(Vec2{ab.min.x, ab.min.y}));
			ASSERT_TRUE(c.hit(Vec2{ab.max.x, ab.min.y}));
			ASSERT_TRUE(c.hit(Vec2{ab.min.x, ab.max.y}));
			ASSERT_TRUE(c.hit(Vec2{ab.max.x, ab.max.y}));

			c.radius = frea::ulps::Move(c.radius, -(1<<10));
			ASSERT_FALSE(c.hit(Vec2{ab.min.x, ab.min.y}));
			ASSERT_FALSE(c.hit(Vec2{ab.max.x, ab.min.y}));
			ASSERT_FALSE(c.hit(Vec2{ab.min.x, ab.max.y}));
			ASSERT_FALSE(c.hit(Vec2{ab.max.x, ab.max.y}));
		}
		TEST_F(Circle2D, Circumscribing) {
			auto& mt = this->mt();
			auto rdp = mt.getUniformF<float>(RangeF{1e2f});
			const auto pts = frea::random::GenVecN<Vec2>(rdp, 3, 0);
			try {
				const auto c = Circle::Circumscribing(pts[0], pts[1], pts[2]);
				const auto Th = frea::ulps::Move(c.radius, 1<<10)-c.radius;
				for(int i=0 ; i<3 ; i++)
					ASSERT_NEAR(c.radius, c.center.distance(pts[i]), Th);
			} catch(const Circle::NoValidCircle&) {
				const float dx = std::abs(pts[0].x - pts[1].x) + std::abs(pts[1].x - pts[2].x),
							dy = std::abs(pts[0].y - pts[1].y) + std::abs(pts[1].y - pts[2].y);
				const float dist = std::min(pts[0].distance(pts[1]), std::min(pts[1].distance(pts[2]), pts[2].distance(pts[0])));
				ASSERT_LT(std::min(dist, std::min(dx, dy)), 1e-3f);
			}
		}
		TEST_F(Circle2D, Boundary) {
			auto& mt = this->mt();
			const int n = mt.template getUniform<int>({1,64});
			std::vector<CircleM> cv;
			for(int i=0 ; i<n ; i++)
				cv.push_back(genCircle());
			// ランダムな個数の円で境界(代表)円を算出
			const CircleM cbound(*MakeBoundaryPtr<Circle, IModel>(cv.data(), n, sizeof(CircleM), 0));
			for(int i=0 ; i<0x100 ; i++) {
				const CircleM ctest = genCircle();
				const bool b0 = ctest.hit(cbound);
				bool b1 = false;
				for(int j=0 ; j<n ; j++) {
					if((b1 = ctest.hit(cv[j])))
						break;
				}
				// 0b00: 境界円がFalseなら個別判定もFalse
				// 0b01: 不正
				// 0b10: 境界円がTrueで個別判定がFalseはあり得る
				// 0b11: 個別判定がTrueなら境界判定もTrue
				const uint32_t flag = (b0 ? 0b10 : 0b00) | (b1 ? 0b01 : 0b00);
				ASSERT_NE(flag, 0b01);
			}
		}
	}
}
