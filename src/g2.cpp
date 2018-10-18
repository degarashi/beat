#include "g2.hpp"
#include "line2d.hpp"
#include "lubee/src/compare.hpp"
#include "frea/src/constant.hpp"
#include "frea/src/matrix.hpp"

namespace beat {
	namespace g2 {
		const AMat2 cs_mRot90[2] = {
			{frea::COS90, -frea::SIN90,
			frea::SIN90, frea::COS90},
			{frea::COS90, frea::SIN90,
			-frea::SIN90, frea::COS90}
		};
		bool IsCW(const PointL& pts) {
			const int nP = pts.size();
			if(nP < 3)
				return false;
			for(int i=0 ; i<nP-2 ; i++) {
				if((pts[i+1] - pts[i]).cw(pts[i+2] - pts[i+1]) < 0)
					return false;
			}
			return (pts[0] - pts[nP-1]).cw(pts[1] - pts[0]) >= 0;
		}
		bool IsCrossing(const Line& ls0, const Line& ls1, const float len0, const float len1, const float t) {
			bool res0 = false,
				 res1 = false;
			const auto fn0 = [&res0, len0, t](const float f){
				res0 = lubee::IsInRange(f, -t, len0+t);
				return f;
			};
			const auto fn1 = [&res1, len1, t](const float f){
				res1 = lubee::IsInRange(f, -t, len1+t);
				return f;
			};
			NearestPoint(ls0, ls1, fn0, fn1);
			return res0 & res1;
		}
		Vec2 NearestPoint(const Line& ls, const Vec2& p, const ClipF& clip) {
			const Vec2 toP = p - ls.pos;
			const float d = ls.dir.dot(toP);
			return ls.pos + ls.dir * clip(d);
		}
		Vec2x2_OP NearestPoint(const Line& ls0, const Line& ls1, const ClipF& clip0, const ClipF& clip1) {
			const float st0d = ls0.dir.len_sq(),
						st1d = ls1.dir.len_sq(),
						st01d = ls0.dir.dot(ls1.dir);
			const float d = st0d * st1d - frea::Square(st01d);
			if(std::abs(d) < 1e-5f) {
				// 2つの直線は平行
				return lubee::none;
			}
			const frea::Mat2 m0(st1d, st01d,
								st01d, st0d);
			Vec2	m1((ls1.pos - ls0.pos).dot(ls0.dir),
						(ls0.pos - ls1.pos).dot(ls1.dir));
			m1 = m0 * m1;
			m1 /= d;
			return Vec2x2(
				ls0.pos + ls0.dir * clip0(m1.x),
				ls1.pos + ls1.dir * clip1(m1.y)
			);
		}
		float CramerDet(const Vec2& v0, const Vec2& v1) {
			return v0.x * v1.y - v1.x * v0.y;
		}
		Vec2 CramersRule(const Vec2& v0, const Vec2& v1, const Vec2& a0, const float detInv) {
			return Vec2(CramerDet(a0, v1) * detInv,
						CramerDet(v0, a0) * detInv);
		}
		Vec2 MinkowskiSub(const IModel& m0, const IModel& m1, const Vec2& dir) {
			return m0.im_support(dir) - m1.im_support(-dir);
		}
		Vec2 MixVector(const float (&cf)[3], const Vec2& p0, const Vec2& p1, const Vec2& p2) {
			return p0 * cf[0] +
					p1 * cf[1] +
					p2 * cf[2];
		}
		void BarycentricCoord(float ret[3], const Vec2& p0, const Vec2& p1, const Vec2& p2, const Vec2& pos) {
			const Vec2 pt0 = p0 - p2,
						pt1 = p1 - p2,
						ptp = pos - p2;

			AMat2 m;
			m.setColumn<0>(pt0);
			m.setColumn<1>(pt1);
			const float det = m.calcDeterminant();

			m.setColumn<0>(ptp);
			ret[0] = m.calcDeterminant();

			m.setColumn<0>(pt0);
			m.setColumn<1>(ptp);
			ret[1] = m.calcDeterminant();
			if(det != 0) {
				ret[0] /= det;
				ret[1] /= det;
			}
			ret[2] = 1.f - ret[0] - ret[1];
		}
		namespace {
			constexpr float MIN_DUAL2DIST = 1e-5f;
		}
		Vec2 Dual2(const Vec2& v0, const Vec2& v1) {
			const Vec2 dir = (v1-v0).normalization();
			float dist = dir.cw(-v0);
			dist = std::max(MIN_DUAL2DIST, dist);
			return dir / dist;
		}
		Line Dual(const Vec2& v) {
			Vec2 t0(0,-v.y),
				t1(1, v.x-v.y);
			t1 = (t1-t0).normalization();
			return Line(t0, t1);
		}
		Vec2 Dual(const Line& ls) {
			const Vec2& t0 = ls.pos;
			const Vec2 t1 = ls.dir + t0;
			return Vec2((t0.y-t1.y) / (t0.x-t1.x),
						-t0.ccw(t1) / (t1.x-t0.x));
		}
		PArray MakeIndexArray(const PointL& pts, const Vec2& dir) {
			const int sz = pts.size();
			PArray ret(sz);
			for(int i=0 ; i<sz ; i++)
				ret[i] = std::make_pair(i, pts[i].dot(dir));
			std::sort(ret.begin(), ret.end(), [](const auto& p0, const auto& p1){
				return p0.second < p1.second;
			});
			return ret;
		}
		namespace {
			std::pair<float,float> QuadraticFormula(const float f0, const float f1, const float f2) {
				const float A = 2 * f0,
							B = std::sqrt(f1*f1 - 4*f0*f2);
				return std::make_pair(
						(-f1 + B) / A,
						(-f1 - B) / A
				);
			}
		}
		Vec2x2 PCA(const PointL& pts) {
			const int N = pts.size();
			D_Assert0(N >= 2);
			Vec2 gv(0);
			for(auto& p : pts)
				gv += p;
			gv /= N;
			AMat2 m{};
			for(int k=0 ; k<N ; k++) {
				const Vec2 d = pts[k] - gv;
				for(int i=0 ; i<2 ; i++) {
					for(int j=0 ; j<2 ; j++) {
						m.m[i][j] += d[i]*d[j];
					}
				}
			}
			for(int i=0 ; i<2 ; i++) {
				for(int j=0 ; j<2 ; j++) {
					m.m[i][j] /= N;
				}
			}
			if(std::abs(m.m[0][0]) < 1e-6f) {
				return {Vec2{0,1}, Vec2{1,0}};
			}
			if(std::abs(m.m[1][1]) < 1e-6f) {
				return {Vec2{1,0}, Vec2{0,1}};
			}

			const float f2 = m.m[0][0] * m.m[1][1] - (m.m[0][1] * m.m[1][0]),
						f1 = -m.m[0][0] - m.m[1][1],
						f0 = 1.f;
			const auto qf = QuadraticFormula(f0, f1, f2);

			const auto fn = [&m](auto& dst, const float q) {
				frea::Mat_t<float, 2, 3, true> m2;
				m2.setRow<0>({m.m[0][0] - q, m.m[0][1], 1.f});
				m2.setRow<1>({m.m[1][0], m.m[1][1] - q, 1.f});
				D_Assert0(m2.rowReduce(0.f) == 0);
				if(m2.m[1][1] == 0) {
					D_Assert0(m2.m[0][2] == 0);
					dst = Vec2{1, m2.m[0][1]}.normalization();
					return false;
				}
				dst = Vec2{m2.m[0][2], m2.m[1][2]}.normalization();
				return true;
			};
			Vec2x2 ret;
			if(!fn(ret.first, qf.first)) {
				if(fn(ret.second, qf.second)) {
					ret.first = ret.second * g2::cs_mRot90[0];
				} else {
					std::cout << "" << std::endl;
				}
			} else
				ret.second = ret.first * g2::cs_mRot90[0];
			return ret;
		}
	}
}
