#include "gjk2d.hpp"
#include "segment2d.hpp"
#include "lubee/src/compare.hpp"

namespace beat {
	namespace g2 {
		bool LmLen::operator < (const LmLen& len) const {
			return dist < len.dist;
		}
		LmLen LmLen::AsLine(const float dist, const Vec2& dir, const int i0, const int i1) {
			D_Assert0(std::abs(1.f - dir.length()) < ZEROVEC_LENGTH);
			return LmLen{dist, dir, {i0,i1}};
		}
		LmS LmS::AsPoint(const float dist, const Vec2& dir, const MVert& vtx) {
			D_Assert0(std::abs(1.f - dir.length()) < ZEROVEC_LENGTH);
			return LmS{true, dist, dir, {vtx}};
		}
		LmS LmS::AsPoint(const MVert& vtx) {
			Vec2 dir = vtx.mpos;
			const float len = dir.normalize();
			return AsPoint(len, dir, vtx);
		}
		LmS LmS::AsLine(const float dist, const Vec2& dir, const MVert& v0, const MVert& v1) {
			D_Assert0(std::abs(1.f - dir.length()) < ZEROVEC_LENGTH);
			return LmS{false, dist, dir, {v0, v1}};
		}
		LmS LmS::AsLine(Vec2 cp, const MVert& v0, const MVert& v1) {
			const float len = cp.normalize();
			return AsLine(len, cp, v0, v1);
		}

		// デバッグ用Print関数
		namespace {
			void PrintV(std::ostream& os, const Vec2& v) {
				os << '[' << v.x << ',' << v.y << ']';
			}
			void PrintV(std::ostream& os, const MVert& v) {
				PrintV(os, v.mpos);
				PrintV(os, v.posA);
			}
		}
		void GEpa::_printLmLen(std::ostream& os) const {
			const int nV = _vl.size();
			for(int i=0 ; i<nV ; i++) {
				PrintV(os, _vl[i]);
				os << std::endl;
			}
			for(auto itr=_lmLen.cbegin() ; itr!=_lmLen.cend() ; itr++) {
				os << itr->dist << ':';
				PrintV(os, itr->dir);
				os << itr->id[0] << ',' << itr->id[1] << std::endl;
			}
		}

		MVert& GEpa::_allocVert() {
			_vl.resize(_vl.size()+1);
			return _vl.back();
		}
		const MVert& GEpa::_minkowskiSub(const Vec2& dir) {
			MVert& v = _allocVert();
			_minkowskiSub(v, dir);
			return v;
		}
		void GEpa::_minkowskiSub(MVert& dst, const Vec2& dir) {
			dst.posA = _m0.im_support(dir);
			dst.mpos = dst.posA - _m1.im_support(-dir);
		}
		void GEpa::_epaMethodOnHit() {
			D_Assert0(!_lmLen.empty());
			// 探索候補が空か候補の中で一番近い線分がこれまでの最短より遠ければ計算終了
			while(!_lmLen.empty()) {
				const auto fr = _lmLen.pop_frontR();
				// v0,v2 = 前からあった頂点
				// v1 = 新しく追加される頂点
				const int i1 = _vl.size(),
							i0 = fr.id[0],
							i2 = fr.id[1];
				const MVert &v1 = _minkowskiSub(fr.dir),
							&v0 = _vl[i0],
							&v2 = _vl[i2];
				const float d1 = v1.mpos.dot(fr.dir),
							vd0 = v0.mpos.dist_sq(v1.mpos),
							vd2 = v2.mpos.dist_sq(v1.mpos);
				// 前回の結果からdirベクトル方向の距離がほぼ変わらなかった場合は終了
				if(vd0 < NEAR_THRESHOLD_SQ ||
					vd2 < NEAR_THRESHOLD_SQ ||
					d1 < fr.dist + NEAR_THRESHOLD)
				{
					if(fr.dist < 0) {
						// on2Hits関数からの初期値
						_result.second = Vec2(0,0);
					} else {
						_result.second = fr.dir * -fr.dist;
					}
					float r = std::max(0.f, fr.dist);
					D_Assert0(!lubee::IsOutstanding(r));
					r = GetRatio(v0.mpos, v2.mpos, fr.dir * r);
					D_Assert0(!lubee::IsOutstanding(r));
					_result.first = v0.posA.l_intp(v2.posA, r);
					D_Assert0(!_result.first.isNaN());
					return;
				}
				// 新しく追加された頂点を交えて線分候補を作成
				const auto add = [this, &fr](const auto& v0, const auto& v1, const int i0, const int i1){
					auto res = Segment(v0.mpos, v1.mpos).nearest(Vec2{0});
					if(res.second == LinePos::OnLine) {
						if(res.first.len_sq() < 1e-6f) {
							// 最近傍点が原点と重なっている
							const float r = GetRatio(v0.mpos, v1.mpos, res.first);
							_result.first = _vl[fr.id[0]].posA.l_intp(_vl[fr.id[1]].posA, r);
							_result.second = res.first.normalization() * -1;
							D_Assert0(!_result.first.isNaN());
							return true;
						}
						const float len = res.first.normalize();
						_lmLen.insert(LmLen::AsLine(len, res.first, i0, i1));
					}
					return false;
				};
				if(add(v0, v1, i0, i1))
					return;
				if(add(v1, v2, i1, i2))
					return;
			}
			AssertF0();
		}
		// 無衝突時: 最近傍対を求める
		void GEpa::_epaMethodNoHit() {
			MVert mv;
			for(;;) {
				const auto fr = _lms;
				const auto &mv0 = fr.vtx[0],
							&mv2 = fr.vtx[1];
				_minkowskiSub(mv, -fr.dir);
				if(fr.bPoint) {
					const auto &v0 = mv0.mpos,
								&v1 = mv.mpos;
					if(v0 == v1) {
						_result.first = mv0.posA;
						_result.second = _result.first - v0;
						return;
					}
					const Vec2 dir = (v1 - v0).normalization();
					const float r = dir.dot(-v0);
					if(r < 1e-5f) {
						_result.first = mv0.posA;
						_result.second = _result.first - v0;
						return;
					}
					if(r >= 1.f-1e-5f) {
						_lms = LmS::AsPoint(mv);
						continue;
					}
					_lms = LmS::AsLine(v0 + dir*r, mv0, mv);
				} else {
					const auto &v0 = mv0.mpos,
								&v1 = mv.mpos,
								&v2 = mv2.mpos;
					const auto f0 = fr.dist,
								f1 = fr.dir.dot(mv.mpos);
					if(f1 < 1e-5f ||
						f1 >= f0-1e-4f ||
						v0.dist_sq(v1) < 1e-6f ||
						v2.dist_sq(v1) < 1e-6f)
					{
						const Vec2 cp = fr.dir * fr.dist;
						const float r = GetRatio(v0, v2, cp);
						_result.first = mv0.posA.l_intp(mv2.posA, r);
						_result.second = _result.first - cp;
						return;
					}

					auto sn = Segment(v0, v1).nearest(Vec2{0});
					if(sn.second != LinePos::End) {
						_lms = LmS::AsLine(sn.first, mv0, mv);
						continue;
					}
					sn = Segment(v1, v2).nearest(Vec2{0});
					if(sn.second != LinePos::End) {
						_lms = LmS::AsLine(sn.first, mv, mv2);
						continue;
					}
					_lms = LmS::AsPoint(mv);
				}
			}
		}
		void GEpa::_adjustClockwise() {
			// 頂点数が3の時専用
			D_Assert0(int(_vl.size()) == 3);
			const Vec2 v01(_vl[1].mpos- _vl[0].mpos),
						v02(_vl[2].mpos - _vl[0].mpos);
			if(v01.cw(v02) < 0)
				std::swap(_vl[0], _vl[1]);
		}
		void GEpa::_makeLmLenOnHit() {
			D_Assert0(getResult());
			const int nV = _vl.size();
			D_Assert0(nV >= 3);

			for(int i=0 ; i<nV ; i++) {
				const auto &p0 = _vl[i],
							&p1 = _vl[(i+1)%nV];
				auto res = Segment(p1.mpos, p0.mpos).nearest(Vec2{0});
				const float len = res.first.length();
				if(len < NEAR_THRESHOLD) {
					// 線分上に原点があるので少し小細工
					// 線分方向90度回転かつ凸包に関わっていない頂点側
					Vec2 dir(p1.mpos - p0.mpos);
					dir.normalize();
					dir *= cs_mRot90[0];
					if(dir.dot(_vl[(i+2)%nV].mpos - p0.mpos) > 0.f)
						dir *= -1.f;
					res.first = dir;
				} else
					res.first /= len;
				_lmLen.insert(LmLen::AsLine(len, res.first, i, (i+1)%nV));
			}
		}
		void GEpa::_makeLmLenOnNoHit() {
			D_Assert0(!getResult());
			const int nV = _vl.size();
			D_Assert0(nV >= 3);
			// 最近傍辺
			for(int i=0 ; i<nV ; i++) {
				const auto &p0 = _vl[i],
							&p1 = _vl[(i+1)%nV];
				Vec2 dir = p1.mpos - p0.mpos;
				const auto len = dir.normalize();
				const auto d = dir.dot(-p0.mpos);
				constexpr float Th = 1e-3f;
				if(lubee::IsInRange<float>(d, Th, len-Th)) {
					Vec2 cp = p0.mpos + dir*d;
					const auto cplen = cp.normalize();
					_lms = LmS::AsLine(cplen, cp, p0, p1);
					return;
				}
			}

			// 最近傍点
			MVert mv;
			float dist = std::numeric_limits<float>::max();
			for(auto& p : _vl) {
				const float d = p.mpos.len_sq();
				if(d < dist) {
					dist = d;
					mv = p;
				}
			}
			_lms = LmS::AsPoint(mv);
		}
		void GEpa::_on2NoHits() {
			const float d0 = _tri.pos[0].length(),
						d1 = _tri.pos[1].length();
			if(d0 > d1)
				_lms = LmS::AsPoint(d1, _tri.pos[1]/d1, MVert{_tri.pos[1], _posA[1]});
			else
				_lms = LmS::AsPoint(d0, _tri.pos[0]/d0, MVert{_tri.pos[0], _posA[0]});
		}
		void GEpa::_on2Hits() {
			Vec2 dir = _tri.pos[0]-_tri.pos[1];
			dir.normalize();
			dir *= cs_mRot90[0];
			// 距離に負数をセットして先に検索されるようにする
			_lmLen.insert(LmLen::AsLine(-1, dir, 0, 1));
			// 反対側も登録
			_lmLen.insert(LmLen::AsLine(-1, -dir, 1, 0));
		}

		GEpa::GEpa(const IModel& m0, const IModel& m1):
			GSimplex(m0,m1)
		{
			const int nV = _nVtx;
			_vl.resize(nV);
			for(int i=0 ; i<nV ; i++) {
				auto& res = _vl[i];
				res.mpos = _tri.pos[i];
				res.posA = _posA[i];
			}

			if(getResult()) {
				if(nV == 1) {
					// 点衝突
					_result.first = _posA[0];
					_result.second = Vec2(0,0);
					return;
				} else if(nV == 2)
					_on2Hits();
				else {
					_adjustClockwise();
					_makeLmLenOnHit();
				}
				_epaMethodOnHit();
			} else {
				if(nV == 1) {
					const Vec2& v = _minkowskiSub(-_tri.pos[0].normalization()).mpos;
					auto res = Segment(_tri.pos[0], v).nearest(Vec2{0});
					const float len = res.first.normalize();
					if(res.second == LinePos::OnLine) {
						_lms = LmS::AsLine(len, res.first, _vl[0], _vl[1]);
					} else if( res.second == LinePos::End) {
						_lms = LmS::AsPoint(len, res.first, _vl[1]);
					} else {
						_result.first = _posA[0];
						_result.second = _result.first - _tri.pos[0];
						return;
					}
				} else if(nV == 2)
					_on2NoHits();
				else {
					_adjustClockwise();
					_makeLmLenOnNoHit();
				}
				_epaMethodNoHit();
			}
		}
		const Vec2x2& GEpa::getNearestPair() const {
			D_Assert0(!getResult());
			D_Assert0(!_result.first.isNaN() && !_result.second.isNaN());
			return _result;
		}
		const Vec2x2& GEpa::getPVector() const {
			D_Assert0(getResult());
			D_Assert0(!_result.first.isNaN() && !_result.second.isNaN());
			return _result;
		}
	}
}
