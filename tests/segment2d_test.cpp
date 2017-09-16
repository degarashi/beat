#include "../random.hpp"
#include "../gjk2d.hpp"
#include "generate.hpp"

namespace beat {
	namespace g2 {
		using lubee::RangeF;
		struct Segment2D : Generator {
			Segment2D() {
				setPointRange(RangeF{1e3f});
			}
		};
		TEST_F(Segment2D, Hit_Point_GJK) {
			SegmentM s(genSegment());
			PointM v(genPoint());
			// 点との判定がGJKと一致するか
			const auto chk = [&s,&v](){
				const bool b0 = s.hit(v);
				GSimplex gs(s,v);
				const bool b1 = gs.getResult();
				EXPECT_EQ(b0, b1);
			};
			chk();

			// 両端の点が同じケース
			s.to = s.from;
			chk();

			// 線分上の点は必ずヒットする
			auto& mt = this->mt();
			const auto rdf = mt.template getUniformF<float>();
			const auto c = GenerateCoeff(rdf, 2);
			if(s.from.dist_sq(s.to) < NEAR_THRESHOLD_SQ)
				static_cast<Vec2&>(v) = s.from;
			else
				static_cast<Vec2&>(v) = s.from*c[0] + s.to*c[1];
			EXPECT_TRUE(s.hit(v));
		}
		TEST_F(Segment2D, Nearest_Point) {
			SegmentM s(genSegment());
			PointM p(genPoint());
			constexpr auto NS = NEAR_THRESHOLD_SQ;
			// 最近傍点がきちんと線分上に乗っているかのテスト
			const auto res = s.nearest(p);
			switch(res.second) {
				case LinePos::OnLine:
					EXPECT_TRUE(s.hit(res.first));
					break;
				case LinePos::Begin:
					EXPECT_LE(s.from.dist_sq(res.first), NS);
					break;
				case LinePos::End:
					EXPECT_LE(s.to.dist_sq(res.first), NS);
					break;
				default:
					EXPECT_TRUE(false);
			}
		}
		TEST_F(Segment2D, Hit_Segment_GJK) {
			// 線分との判定結果をGJKと比較
			const SegmentM s0(genSegment()),
								 s1(genSegment());
			const bool b0 = s0.hit(s1);
			GSimplex gs(s0, s1);
			const bool b1 = gs.getResult();
			if(b0 != b1) {
				const bool b0 = s0.hit(s1);
				GSimplex gs(s0, s1);
				const bool b1 = gs.getResult();
				ASSERT_EQ(b0, b1);
			}
		}
		TEST_F(Segment2D, Support) {
			const SegmentM s(genSegment());
			const auto dir = genDir();
			const auto sv = s.support(dir);
			const float dist0 = sv.dist_sq(s.from),
						  dist1 = sv.dist_sq(s.to),
						  dot0 = dir.dot(s.from),
						  dot1 = dir.dot(s.to);

			// サポート座標は線分の両端のどちらかと一致している筈
			if(dist0 < NEAR_THRESHOLD_SQ) {
				// 更に両端の内、内積が大きい方と一致しているか
				ASSERT_GE(dot0, dot1);
			} else if(dist1 < NEAR_THRESHOLD_SQ) {
				ASSERT_GE(dot1, dot0);
			} else {
				ASSERT_TRUE(false);
			}
			ASSERT_TRUE(s.hit(sv));
		}
	}
}
