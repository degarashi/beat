#include "../gjk2d.hpp"
#include "generate.hpp"

namespace beat {
	namespace g2 {
		using lubee::RangeF;
		struct AABB2D : Generator {
			AABB2D() {
				setCircleRadius({1e-2f, 1e1f});
			}
		};
		TEST_F(AABB2D, Encapsule_Point) {
			const int N = genInt({1, 1024});
			std::vector<Vec2> pts(N);
			for(int i=0 ; i<N ; i++) {
				pts[i] = genPoint();
			}
			const auto r = AABB::Encapsule(pts.begin(), pts.end());
			for(int i=0 ; i<N ; i++) {
				ASSERT_TRUE(r.hit(pts[i]));
			}
		}
		TEST_F(AABB2D, Hit_Point_GJK) {
			const PointM p(genPoint());
			const AABBM ab(genAABB());
			ASSERT_LE(ab.min.x, ab.max.x);
			ASSERT_LE(ab.min.y, ab.max.y);
			// AABB -> Point のHit関数をGJK関数で結果を比較
			const bool b0 = ab.hit(p);
			GSimplex gs(ab, p);
			const bool b1 = gs.getResult();
			ASSERT_EQ(b0, b1);
		}
		TEST_F(AABB2D, Hit_Segment_GJK) {
			const AABBM ab(genAABB());
			const SegmentM s(genSegment());
			const bool b0 = ab.hit(s);
			const bool b1 = GSimplex(ab, s).getResult();
			ASSERT_EQ(b0, b1);
		}
		TEST_F(AABB2D, Hit_AABB_GJK) {
			const AABBM ab0(genAABB()),
						  ab1(genAABB());
			const bool b0 = ab0.hit(ab1);
			const bool b1 = GSimplex(ab0, ab1).getResult();
			ASSERT_EQ(b0, b1);
		}
		TEST_F(AABB2D, Support) {
			const auto ab = genAABB();
			const auto dir = genDir();
			const auto pos = ab.support(dir);
			ASSERT_TRUE(ab.hit(pos));
		}
		TEST_F(AABB2D, Hit_Circle_GJK) {
			const AABBM ab(genAABB());
			CircleM c(genCircle());
			bool b0 = false;
			// 内包判定(Circle in AABB)
			auto &amin = ab.min,
				&amax = ab.max;
			if(c.support({1,0}).x <= amax.x
				&& c.support({-1,0}).x >= amin.x
				&& c.support({0,1}).y <= amax.y
				&& c.support({0,-1}).y >= amin.y)
				b0 = true;
			// 内包判定(AABB in Circle)
			if(ab.support({1,0}).x <= c.support({1,0}).x
				&& ab.support({-1,0}).x >= c.support({-1,0}).x
				&& ab.support({0,1}).y <= c.support({0,1}).y
				&& ab.support({0,-1}).y >= c.support({0,-1}).y)
				b0 = true;
			// 4辺をセグメント判定
			if(c.hit(SegmentM(amin, Vec2{amin.x, amax.y}))
				|| c.hit(SegmentM(Vec2{amin.x, amax.y}, amax))
				|| c.hit(SegmentM(amax, Vec2{amax.x, amin.y}))
				|| c.hit(SegmentM(Vec2{amax.x, amin.y}, amin)))
				b0 = true;
			bool b1 = GSimplex(ab, c).getResult();
			if(b0 && !b1) {
				c.radius += 5e-3f;
				b1 = GSimplex(ab, c).getResult();
			}
			ASSERT_EQ(b0, b1);
		}
		TEST_F(AABB2D, Boundary) {
			auto& mt = this->mt();
			const int n = mt.template getUniform<int>({1,64});
			std::vector<AABBM> av;
			for(int i=0 ; i<n ; i++)
				av.push_back(genAABB());
			// ランダムな個数のAABBで境界(代表)AABBを算出
			AABBM abound(*MakeBoundaryPtr<AABB, IModel>(av.data(), n, sizeof(AABBM), 0));

			for(int i=0 ; i<0x100 ; i++) {
				const AABBM atest = genAABB();
				bool b0 = atest.hit(abound),
					b1 = false;
				for(int j=0 ; j<n ; j++) {
					if((b1 = atest.hit(av[j])))
						break;
				}
				// 0b00: 境界AABBがFalseなら個別判定もFalse
				// 0b01: 不正
				// 0b10: 境界AABBがTrueで個別判定がFalseはあり得る
				// 0b11: 個別判定がTrueなら境界判定もTrue
				const uint32_t flag = (b0 ? 0b10 : 0b00) | (b1 ? 0b01 : 0b00);
				ASSERT_NE(flag, 0b01);
			}
		}
	}
}
