#include "../gjk2d.hpp"
#include "generate.hpp"

namespace beat {
	namespace g2 {
		using lubee::RangeF;
		struct Capsule2D : Generator {
			Capsule2D() {
				setCircleRadius({1e-2f, 1e1f});
				setCapsuleRadius({0, 1e1f});
			}
		};
		TEST_F(Capsule2D, Hit_Point_GJK) {
			const CapsuleM c(genCapsule());
			const PointM p(genPoint());
			const bool b0 = c.hit(p);
			const bool b1 = GSimplex(c, p).getResult();
			ASSERT_EQ(b0, b1);
			const bool b2 = c.im_hit(p);
			ASSERT_EQ(b0, b2);
		}
		TEST_F(Capsule2D, Hit_Capsule_GJK) {
			const CapsuleM c0(genCapsule()),
							 c1(genCapsule());
			const bool b0 = c0.hit(c1);
			const bool b1 = GSimplex(c0, c1).getResult();
			const Segment s0(c0.from, c0.to),
						s1(c1.from, c1.to);
			if(b0 != b1) {
				const bool b0 = c0.hit(c1);
				const bool b1 = GSimplex(c0, c1).getResult();
				ASSERT_EQ(b0, b1);
			}
		}
		TEST_F(Capsule2D, Support) {
			const auto c = genCapsule();
			const auto dir = genDir();
			const Vec2 pos = c.support(dir);
			ASSERT_TRUE(c.hit(pos));
		}
	}
}
