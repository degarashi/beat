#include "generate.hpp"
#include "gjk2d.hpp"

namespace beat {
	namespace g2 {
		template <class T>
		struct Epa2D : public Generator {
			Epa2D() {
				setPointRange(RangeF{1e1f});
				setCircleRadius(RangeF{1e-2f, 1e1f});
			}
		};
		template <class T>
		float GetHitPointSize(T*) { return 5e-3f; }
		float GetHitPointSize(Convex*) { return 5e-2f; }

		using Epa2DTypeList = ::testing::Types<Circle,
												AABB,
												Triangle,
												Convex>;
		TYPED_TEST_CASE(Epa2D, Epa2DTypeList);
		TYPED_TEST(Epa2D, Epa) {
			using ShapeM = Model<TypeParam>;
			ShapeM c[2];
			this->genShape(c[0]);
			this->genShape(c[1]);
			const float EvadeDist = 1e-2f,
						HitPointSize = GetHitPointSize((TypeParam*)0);
			const GEpa gepa(c[0], c[1]);
			if(gepa.getResult()) {
				// pv = (first=最深点, second=回避ベクトル)
				auto& pv = gepa.getPVector();
				// 回避ベクトル始点は物体Aの内部にある筈
				ASSERT_TRUE(c[0].im_hit(pv.first) ||
						GSimplex(c[0], CircleM(Circle{pv.first, HitPointSize})).getResult());
				if(pv.second.isZero(ZEROVEC_LENGTH)) {
					// 点衝突の場合はここで終わり
					return;
				}
				// 左辺に指定した物体を衝突回避ベクトル分移動させたら衝突回避出来る筈
				c[0] += pv.second + pv.second.normalization() * EvadeDist;
				ASSERT_FALSE(GSimplex(c[0], c[1]).getResult());
			} else {
				const auto np = gepa.getNearestPair();
				// 最近傍点Aは物体Aの内部にある筈
				{
					const CircleM hp(Circle{np.first, HitPointSize});
					ASSERT_TRUE(c[0].im_hit(np.first) || GSimplex(c[0], hp).getResult());
				}
				// 最近傍点Bは物体Bの内部にある筈
				{
					const CircleM hp(Circle{np.second, HitPointSize});
					ASSERT_TRUE(c[1].im_hit(np.second) || GSimplex(c[1], hp).getResult());
				}
				// 左辺の物体を最近傍対ベクトル分(p1 - p0)移動させたら衝突する筈
				const auto pv = (np.second - np.first);
				bool bSuccess = false;
				for(float cd = 0.f ; cd < 1.f ; cd += .05f) {
					auto c0 = c[0];
					c0 += pv + pv.normalization()*cd;
					if(!GSimplex(c0, c[1]).getResult()) {
						continue;
					}
					bSuccess = true;
					break;
				}
				ASSERT_TRUE(bSuccess);
			}
		}
	}
}
