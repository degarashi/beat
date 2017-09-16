#include "../gjk2d.hpp"
#include "generate.hpp"

namespace beat {
	namespace g2 {
		using lubee::RangeF;
		struct Point2D : Generator {};
		TEST_F(Point2D, Hit_Point_GJK) {
			const PointM p0(genPoint()),
					   p1(genPoint());
			// Point -> Point の、Hit関数とGJK関数で結果が一致するかチェック
			const bool b0 = p0.hit(p1);
			GSimplex gs(p0, p1);
			const bool b1 = gs.getResult();
			ASSERT_EQ(b0, b1);
		}
		TEST_F(Point2D, Support) {
			const auto p0 = genPoint();
			const auto dir = genDir();
			const auto p1 = p0.support(dir);
			ASSERT_TRUE(p0.hit(p1));
		}
	}
}
