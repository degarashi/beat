#include "generate.hpp"
#include "../narrow.hpp"
#include "../tfnode_base2d.hpp"

namespace beat {
	namespace g2 {
		template <class T>
		class BVolume2D : public Generator {
			protected:
				using base_t = Generator;
				using Narrow_t = Types::Narrow;
				void SetUp() override {
					base_t::SetUp();
					Narrow_t::Initialize();
				}
		};
		using BVolume2DTL = ::testing::Types<Point,
											Segment,
											AABB,
											Triangle,
											Convex>;
		TYPED_TEST_SUITE(BVolume2D, BVolume2DTL);
		// BVolume, BBoxのテスト
		TYPED_TEST(BVolume2D, General) {
			using Shape = TypeParam;
			using ShapeM = Model<Shape>;

			ShapeM s;
			this->genShape(s);
			CircleM bv;
			s.im_getBVolume(bv);
			AABBM ab;
			s.im_getBVolume(ab);

			// 元形状とBVolume, BBox形状の衝突判定比較
			using this_t = BVolume2D<TypeParam>;
			constexpr int nCheck = 10;
			for(int i=0 ; i<nCheck ; i++) {
				// 比較対象はPolygon
				const auto rp = TriangleM{this->genTriangle()};

				const bool b_base = this_t::Narrow_t::Hit(&s, &rp, 0),
						 b_bv = this_t::Narrow_t::Hit(&bv, &rp, 0),
						 b_ab = this_t::Narrow_t::Hit(&ab, &rp, 0);
				// 元形状が衝突しているならBVolume, BBoxも衝突している
				if(b_base) {
					ASSERT_TRUE(b_bv);
					ASSERT_TRUE(b_ab);
				}
				// BVolumeが衝突していないなら元形状もなし
				if(!b_bv) {
					ASSERT_FALSE(b_base);
				}
				// BBoxが衝突していないなら元形状もなし
				if(!b_ab) {
					ASSERT_FALSE(b_base);
				}
			}
		}
	}
}
