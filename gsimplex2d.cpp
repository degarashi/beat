#include "gjk2d.hpp"
#include "g.hpp"
#include "segment2d.hpp"
#include "circle2d.hpp"
#include "aabb2d.hpp"
#include "point2d.hpp"

namespace beat {
	namespace g2 {
		float GetRatio(const Vec2& v0, const Vec2& v1, const Vec2& pos) {
			if(std::abs(v0.x - v1.x) > std::abs(v0.y - v1.y))
				return (pos.x - v0.x) / (v1.x - v0.x);
			return (pos.y - v0.y) / (v1.y - v0.y);
		}

		void GSimplex::_minkowskiSub(const Vec2& dir, const int n) {
			_posA[n] = _m0.im_support(dir);
			_tri.pos[n] = _posA[n] - _m1.im_support(-dir);
		}
		void GSimplex::_setAsHit(const int nv, const Vec2& inner) {
			_nVtx = nv;
			_bHit = true;
			_inner = inner;
		}
		void GSimplex::_setAsNotHit(const int nv) {
			_nVtx = nv;
			_bHit = false;
		}
		GSimplex::GSimplex(const IModel& m0, const IModel& m1):
			_m0(m0),
			_m1(m1)
		{
			_gjkMethod();
		}
		bool GSimplex::getResult() const {
			return _bHit;
		}
		const Vec2& GSimplex::getInner() const {
			return _inner;
		}
		namespace {
			// [中心となる頂点] -> [その前の頂点, 現在の頂点, 次の頂点]
			constexpr int NeighborIndex[3][3] = {
				{2,0,1},
				{0,1,2},
				{1,2,0}
			};
		}
		void GSimplex::_gjkMethod() {
			// とりあえず物体の中心位置でサポートベクトルを決める
			Vec2 dir(_m1.im_getCenter() - _m0.im_getCenter());
			float lens = dir.len_sq();
			if(lens < NEAR_THRESHOLD_SQ)
				dir = Vec2(1,0);
			else
				dir /= std::sqrt(lens);

			_minkowskiSub(dir, 0);
			if(dir.dot(_tri.pos[0]) < 0)
				return _setAsNotHit(1);

			// 原点と重なっていたら終了 = 内部点
			lens = _tri.pos[0].len_sq();
			if(lens < NEAR_THRESHOLD_SQ)
				return _setAsHit(1, _posA[0]);

			dir = _tri.pos[0] / std::sqrt(lens) * -1.f;
			_minkowskiSub(dir, 1);

			// 2つのミンコフスキー頂点から次の方向を決定(Segmentの最短距離)
			{
				Vec2 sdir = _tri.pos[1] - _tri.pos[0];
				const float dirLen = sdir.normalize();
				const float r = sdir.dot(-_tri.pos[0]);
				D_Assert0(r >= 0);
				if(r > dirLen)
					return _setAsNotHit(2);
				if(r >= dirLen-1e-5f) {
					std::swap(_posA[0], _posA[1]);
					std::swap(_tri.pos[0], _tri.pos[1]);
					return _setAsHit(1, _posA[0]);
				}
				Vec2 cp = _tri.pos[0] + sdir*r;
				const float len = cp.normalize();
				if(len < ZEROVEC_LENGTH) {
					// ライン上に原点がある
					// 90度回転
					Vec2 ldir = Vec2{sdir.y, -sdir.x};
					const Vec2 v0 = _m0.im_support(ldir) - _m1.im_support(-ldir);
					// 更に反転(-90)
					ldir *= -1;
					const Vec2 v1 = _m0.im_support(ldir) - _m1.im_support(-ldir);
					if((v1-_tri.pos[0]).cw(v0-_tri.pos[0]) >= 0) {
						const Vec2 pos = _posA[0].l_intp(_posA[1], r / dirLen);
						return _setAsHit(2, pos);
					}
					return _setAsNotHit(2);
				}
				dir = cp;
			}
			// 次に探索する方向
			LNear res(dir, LinePos::OnLine);
			// 今までの最短距離
			constexpr float DistMax = std::numeric_limits<float>::max();
			float minDist = DistMax;
			// 次の探索結果を書き込むインデックス
			int idx = 2;
			for(;;) {
				// 新たな頂点を追加
				dir = res.first.normalization() * -1.f;
				_minkowskiSub(dir, idx);
				if(dir.dot(_tri.pos[idx]) < 0)
					return _setAsNotHit(3);

				// 既存の頂点と同じ座標だったらこれ以上進展はないということで、終了
				const auto& nei = NeighborIndex[idx];
				auto &rV0 = _tri.pos[nei[0]],
						&rV1 = _tri.pos[nei[1]],
						&rV2 = _tri.pos[nei[2]];
				{
					const float d1 = rV1.dot(dir),
							  d0 = rV0.dot(dir),
							  d2 = rV2.dot(dir);
					if(d1-NEAR_THRESHOLD < d0 || d1-NEAR_THRESHOLD < d2)
						return _setAsNotHit(3);
				}

				const Vec2 ori(0);
				// 現時点で三角形が原点を含んでいるか
				{
					bool bIn;
					if(_tri.isClockwise())
						bIn = _tri.hit({0,0});
					else {
						_tri.flip();
						bIn = _tri.hit({0,0});
						_tri.flip();
					}
					if(bIn) {
						// 交差領域の1点を算出
						float cf[3];
						BarycentricCoord(cf, _tri.pos[0], _tri.pos[1], _tri.pos[2], ori);
						const auto v = MixVector(cf, _posA[0], _posA[1], _posA[2]);
						return _setAsHit(3, v);
					}
				}
				float dist = DistMax;
				idx = -1;
				// nei[0], nei[1], dist&, idx&, res&, rV0, rV1 -> bool
				res = Segment(_tri.pos[nei[0]], _tri.pos[nei[1]]).nearest(ori);
				if(res.second == LinePos::OnLine) {
					const float td = res.first.len_sq();
					if(td < 1e-7f) {
						const float r = GetRatio(rV0, rV1, res.first);
						const auto in = _posA[nei[0]]*(1-r) + _posA[nei[1]]*r;
						return _setAsHit(3, in);
					}
					if(dist > td) {
						dist = td;
						// この辺について調べる
						idx = nei[2];
					}
				}
				{
					auto res2 = Segment(_tri.pos[nei[1]], _tri.pos[nei[2]]).nearest(ori);
					if(res2.second == LinePos::OnLine) {
						const float td = res2.first.len_sq();
						if(td < 1e-7f) {
							const float r = GetRatio(rV1, rV2, res.first);
							const auto in = _posA[nei[1]]*(1-r) + _posA[nei[2]]*r;
							return _setAsHit(3, in);
						}
						if(dist > td) {
							dist = td;
							// この辺について調べる
							idx = nei[0];
							res = res2;
						}
					}
				}
				if(dist > minDist)
					return _setAsNotHit(3);
				minDist = dist;
				if(idx < 0)
					return _setAsNotHit(3);
			}
		}
	}
}
