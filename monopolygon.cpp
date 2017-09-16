#include "concave2d.hpp"
#include "frea/random/vector.hpp"
#include "frea/interpolation.hpp"
#include "lubee/compare.hpp"
#include "lubee/cyclevalue.hpp"
#include "segment2d.hpp"

namespace beat {
	namespace g2 {
		namespace {
			struct IdxCursor {
				lubee::CyInt	cw,
								ccw;
			};
		}
		bool MonoPolygon::IsMonotone(const PointL& pts, const Vec2& dir) {
			// dir方向についてソート
			const int sz = pts.size();
			D_Assert0(sz >= 3);
			if(sz < 4)
				return true;
			const auto ia = MakeIndexArray(pts, dir);
			const auto fnInt = lubee::CyInt::MakeValueF(sz);
			IdxCursor cur{fnInt(ia[0].first),
							fnInt(ia[0].first)};
			// 最初の点を基準にする
			for(int idx=1 ; idx<sz-1 ; idx++) {
				const auto& p = ia[idx];
				// 何れかのカーソルと隣接してなければMonotoneではない
				if((cur.cw+1) == p.first) {
					cur.cw = p.first;
				} else if((cur.ccw-1) == p.first) {
					cur.ccw = p.first;
				} else
					return false;
			}
			return ConcavePolygon::IsConcave(pts);
		}
		MonoPolygon::MonoPolygon(DistV&& dL, DistV&& dR, const Vec2& ori, const Vec2& dir, const float wofs):
			_vOrigin(ori),
			_vDir(dir),
			_distL(std::move(dL)),
			_distR(std::move(dR)),
			_widthOffset(wofs)
		{}
		MonoPolygon MonoPolygon::Random(const FRandF& rff, const FRandI& rfi, const RangeF& rV, const RangeF& rLen, int nV) {
			nV = std::max(3, nV);
			// ランダムにスイープ方向を決める
			const auto dir = frea::random::GenVecUnit<Vec2>(rff);
			// 原点座標
			const auto rdp = [&](){ return rff(rV); };
			const auto origin = frea::random::GenVec<Vec2>(rdp);
			// 長さ
			const float length = rff(rLen);

			const int nLeft = rfi({1, nV-2}),
					nRight = rfi({0, nV-2-nLeft});
			const auto fnMakeLengthList = [&rff](const int n, const float len) {
				std::vector<Vec2>	distL(n+1);
				float sumL = 0;
				for(auto& d : distL) {
					const float num = rff({1e-2f, 1e1f});
					sumL += num;
					d.x = num;
					d.y = rff({1e-3f, 1.f});
				}
				distL.back().y = 0;
				for(auto& d : distL) {
					d.x /= sumL;
					d.x *= len;
				}
				return distL;
			};
			const float width_offset = rff(rLen * 1e-1f);
			auto distL = fnMakeLengthList(nLeft, length),
				distR = fnMakeLengthList(nRight, length);
			return MonoPolygon(std::move(distL),
								std::move(distR),
								origin,
								dir,
								width_offset);
		}
		PointL MonoPolygon::getPoints() const {
			int nLeft = _distL.size()-1,
				nRight = _distR.size()-1;
			PointL pts(2 + nLeft + nRight);
			auto* ptr = pts.data();
			Vec2 dir90{-_vDir.y, _vDir.x};
			// 始点を追加
			*ptr++ = _vOrigin + dir90*_widthOffset;
			float cur = 0;
			// 左側にランダムな数の頂点(最低1)
			for(int i=0 ; i<nLeft ; i++) {
				auto& dist = _distL[i];
				cur += dist.x;
				*ptr++ = _vOrigin + _vDir*cur + dir90*(dist.y + _widthOffset);
			}
			cur += _distL.back().x;
			// 終点に頂点を配置
			*ptr++ = _vOrigin + _vDir*cur + dir90*_widthOffset;
			// 右側にランダムな数の頂点
			cur -= _distR.back().x;
			for(int i=nRight-1 ; i>=0 ; i--) {
				auto& dist = _distR[i];
				*ptr++ = _vOrigin + _vDir*cur - dir90*(dist.y - _widthOffset);
				cur -= dist.x;
			}
			D_Assert0(pts.data()+pts.size() == ptr);
			return pts;
		}
		const Vec2& MonoPolygon::getDir() const {
			return _vDir;
		}
		bool MonoPolygon::hit(const Vec2& p) const {
			const Vec2 dir90(-_vDir.y, _vDir.x);
			const auto toP = p - (_vOrigin + dir90*_widthOffset);
			float d_vert = _vDir.dot(toP),
				d_horz = std::sqrt(toP.len_sq() - frea::Square(d_vert));
			if(d_vert < 0)
				return false;
			const auto fnGetHDist = [](const auto& distV, float d_vert, const float invalid){
				int nL = distV.size();
				float cur_y = 0;
				for(int i=0 ; i<nL ; i++) {
					auto &dist = distV[i];
					if(d_vert <= dist.x)
						return frea::Lerp(cur_y, dist.y, d_vert / dist.x);
					d_vert -= dist.x;
					cur_y = dist.y;
				}
				return invalid;
			};
			if(dir90.dot(toP) < 0)
				d_horz *= -1;
			float dL = fnGetHDist(_distL, d_vert, -1.f),
					dR = fnGetHDist(_distR, d_vert, -1.f);
			return lubee::IsInRange(d_horz, -dR, dL);
		}
	}
}
