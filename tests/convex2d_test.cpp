#include "../convex2d.hpp"
#include "../random.hpp"
#include "../gjk2d.hpp"
#include "../concave2d.hpp"
#include "generate.hpp"

namespace beat {
	namespace g2 {
		using lubee::RangeF;
		struct Convex2D : Generator {};
		TEST_F(Convex2D, BVolume) {
			const ConvexM cv = genConvex();
			const Circle c = cv.bs_getBCircle();
			// 全ての点が円の中に入ってるかチェック
			for(const auto& p : cv.points()) {
				ASSERT_LE(c.center.distance(p), c.radius+ NEAR_THRESHOLD);
			}
		}
		TEST_F(Convex2D, Support) {
			const auto cnv = genConvex();
			const auto dir = genDir();
			const auto pos = cnv.support(dir);
			ASSERT_TRUE(cnv.hit(pos));
		}
		TEST_F(Convex2D, CheckPosition_0) {
			auto& mt = this->mt();
			const auto rcf = mt.getUniformF<float>();

			int np = mt.template getUniform<int>({3, 64});
			ConvexM cv = genConvex(np);
			np = cv.points().size();
			ASSERT_TRUE(cv.isClockwise());
			const auto& pts = cv.points();
			const Vec2 center = (pts[0] + pts[1] + pts[2]) / 3.f;
			for(int i=0 ; i<np ; i++) {
				// ポリゴンを構成する3頂点から係数を合計1で算出した座標は必ずcheckPositionでその位置を返す
				const Triangle tri{
					center,
					pts[i],
					pts[(i+1)%np]
				};
				ASSERT_TRUE(tri.isClockwise());
				const auto c = GenerateCoeffMin(rcf, 3, 5e-2f, 1.f);
				const Vec2 tmp = tri.onArea(c[0], c[1], c[2]);
				const auto res = cv.checkPosition(tmp, 0);
				ASSERT_NE(ConvexPos::Outer, res.first);
				ASSERT_TRUE(i==res.second || (i+1)%np==res.second || i==(res.second+1)%np);
			}
		}
		TEST_F(Convex2D, CheckPosition_1) {
			const ConvexM c = genConvex();
			const PointM p = genPoint();
			const auto res = c.checkPosition(p).first;
			if(c.hit(p)) {
				ASSERT_NE(ConvexPos::Outer, res);
			} else {
				ASSERT_EQ(ConvexPos::Outer, res);
			}
		}
		TEST_F(Convex2D, Hit_Point_GJK) {
			ConvexM c = genConvex();
			ASSERT_TRUE(c.isClockwise());
			const PointM p = genPoint();
			bool b0 = c.hit(p);
			bool b1 = GSimplex(c,p).getResult();
			if(b0 != b1) {
				c.distend(1.f-1e-3f);
				b0 = c.hit(p);
				b1 = GSimplex(c,p).getResult();
			}
			ASSERT_EQ(b0, b1);
		}
		TEST_F(Convex2D, Hit_Convex_GJK) {
			ConvexM c0 = genConvex(),
					c1 = genConvex();
			const int nc0 = c0.points().size(),
					nc1 = c1.points().size();
			// ポリゴンを総当りで地道に(確実に)判定
			const bool b0 = [=,&c0,&c1](){
				for(int i=0 ; i<nc0-2 ; i++) {
					const Triangle poly0{c0.points()[0], c0.points()[i+1], c0.points()[i+2]};
					for(int j=0 ; j<nc1-2 ; j++) {
						const Triangle poly1{c1.points()[0], c1.points()[j+1], c1.points()[j+2]};
						if(poly0.hit(poly1))
							return true;
					}
				}
				return false;
			}();
			// GJKにて判定
			bool b1 = GSimplex(c0, c1).getResult();
			if(b0 && !b1) {
				b1 = GSimplex(c0, c1).getResult();
			}
			// 結果を比較
			ASSERT_EQ(b0, b1);
		}
		TEST_F(Convex2D, AddPoint) {
			ConvexM c = genConvex();
			const Point p = genPoint();
			if(c.addPoint(p)) {
				ASSERT_TRUE(Convex(c.points()).isClockwise());
			}
		}
		TEST_F(Convex2D, MonotoneToTriangle) {
			auto& mt = this->mt();
			const auto rff = mt.template getUniformF<float>();
			const auto rfi = mt.template getUniformF<int>();
			const auto mp = MonoPolygon::Random(rff, rfi,
													{-1e2f, 1e2f},
													{1e-1f, 1e1f},
													rfi({3,32}));
			auto& dir = mp.getDir();
			const auto pts = mp.getPoints();
			ASSERT_TRUE(MonoPolygon::IsMonotone(pts, dir));

			const auto tri = Convex::MonotoneToTriangle(pts, dir);
			for(auto& t : tri)
				ASSERT_TRUE(t.toTriangle(pts).isClockwise());

			// 生成されたConvex群の判定結果とMonotonePolygonの判定結果を比べる
			// Convexの集合とMonotoneが同じ形状かを判定
			constexpr int NCheck = 0x100;
			for(int i=0 ; i<NCheck ; i++) {
				auto p = genPoint();
				bool b0 = mp.hit(p),
					 b1 = false;
				for(auto& t : tri) {
					if((b1 = t.hit(pts, p)))
						break;
				}
				ASSERT_EQ(b0, b1);
			}
		}
		TEST_F(Convex2D, ConcaveToMonotone) {
			// constexpr float d = 1e-2f;
			// PointL pts = {{0,0}, {0+d,0.1}, {0.9+d,0.1}, {0.9,0.9}, {0+2*d,0.9}, {0+3*d,1}, {1+d,1}, {1,0}};
			auto& mt = this->mt();
			const auto rff = mt.template getUniformF<float>();
			const auto rfi = mt.template getUniformF<int>();
			const auto cc = ConcavePolygon::Random(rff, rfi, {-1e1f, 1e1f}, 10);
			const auto pts = cc.getPoints();
			const auto axis = PCA(pts).first;
			ASSERT_TRUE(ConcavePolygon::IsConcave(pts));

			std::vector<Triangle> tri;
			Convex::ConcaveToMonotone(pts, axis, [dir=axis, &tri](PointL&& pl){
				// ちゃんとMonotoneになってるかチェック
				ASSERT_TRUE(MonoPolygon::IsMonotone(pl, dir));
				// Convexのテストは他でやってるのでここではしない
				const auto t2 = Convex::MonotoneToTriangle(pl, dir);
				for(auto& t : t2) {
					tri.emplace_back(t.toTriangle(pl));
					ASSERT_TRUE(tri.back().isClockwise());
				}
			});

			constexpr int NCheck = 0x100;
			for(int i=0 ; i<NCheck ; i++) {
				const auto p = genPoint();
				bool b0 = cc.hit(p),
					 b1 = false;
				for(auto& t : tri) {
					if((b1 = t.hit(p)))
						break;
				}
				ASSERT_EQ(b0, b1);
			}
		}
	}
}
