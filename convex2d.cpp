#include "convex2d.hpp"
#include "triangle2d.hpp"
#include "frea/angle_func.hpp"
#include "aabb2d.hpp"
#include "circle2d.hpp"
#include "line2d.hpp"
#include "segment2d.hpp"
#include "conditional.hpp"
#include "lubee/cyclevalue.hpp"
#include <unordered_set>
#include <unordered_map>
#include <deque>

namespace beat {
	namespace g2 {
		Convex::Convex(const PointL& pl):
			point(pl)
		{
			D_Assert0(int(pl.size()) >= 3);
		}
		Convex::Convex(PointL&& pl) noexcept:
			point(std::move(pl))
		{
			D_Assert0(int(point.size()) >= 3);
		}
		Convex::Convex(const std::initializer_list<Vec2>& v):
			point(v.size())
		{
			D_Assert0(int(v.size()) >= 3);
			auto itrD = point.begin();
			auto itr = v.begin();
			while(itr != v.end())
				*itrD++ = *itr++;
		}

		namespace {
			bool Back(const Vec2& v0, const Vec2& v1, const Vec2& v2) {
				const Vec2 to0(v0 - v1),
							to2(v2 - v1);
				const float c = to0.cw(to2),
							d = to0.dot(to2);
				return c >= 0 || (c > -1e-3f && d > 1e-3f);
			}
		}
		Convex Convex::FromConcave(const PointL& src) {
			const int nV = src.size();
			D_Assert0(nV >= 3);

			// X軸についてソート
			PointL tsrc(src);
			std::sort(tsrc.begin(), tsrc.end(), [](const Vec2& v0, const Vec2& v1){ return v0.x < v1.x; });

			PointL pts(nV+1);
			Vec2* pDst = &pts[0];
			*pDst++ = tsrc[0];
			*pDst++ = tsrc[1];
			for(int rc=2 ; rc<nV ; rc++) {
				while(Back(pDst[-2], pDst[-1], tsrc[rc])) {
					if(--pDst == &pts[1])
						break;
				}
				*pDst++ = tsrc[rc];
			}
			*pDst++ = tsrc[nV-2];
			Vec2* pTgt = pDst-1;
			for(int rc=nV-3 ; rc>=0 ; rc--) {
				while(Back(pDst[-2], pDst[-1], tsrc[rc])) {
					if(--pDst == pTgt)
						break;
				}
				*pDst++ = tsrc[rc];
			}
			// 末尾がダブる為、削る
			--pDst;

			D_Assert0(pDst <= &pts[0]+nV);
			pts.resize(pDst - &pts[0]);
			return Convex(std::move(pts));
		}
		Vec2 Convex::bs_getCenter() const {
			return std::get<2>(area_inertia_center());
		}
		float Convex::bs_getInertia() const {
			return std::get<1>(area_inertia_center());
		}
		AABB Convex::bs_getBBox() const {
			D_Assert0(!point.empty());
			AABB ab(point[0], point[0]);
			const int nV = point.size();
			for(int i=1 ; i<nV ; i++) {
				ab.min.selectMin(point[i]);
				ab.max.selectMax(point[i]);
			}
			return ab;
		}
		Circle Convex::bs_getBCircle() const {
			D_Assert0(!point.empty());
			// 多分遅いアルゴリズムだが、今はこれで我慢
			// 全ての3点の組み合わせを調べる
			constexpr float fmin = std::numeric_limits<float>::lowest();
			constexpr float fmax = std::numeric_limits<float>::max();
			Vec2 vmin(fmax),
				 vmax(fmin);
			for(auto& p : point) {
				vmin.selectMin(p);
				vmax.selectMax(p);
			}
			AABBM ab(vmin, vmax);
			Circle c;
			c.setBoundary(&ab);
			return c;
		}
		namespace {
			struct AreaSum {
				float result;

				AreaSum():
					result(0)
				{}
				void operator()(int /*n*/, const Vec2& p0, const Vec2& p1) {
					result += Triangle::CalcArea(p0,p1);
				}
			};
			using AreaL = std::vector<float>;
			struct AreaList {
				AreaL	areaL;
				float	sum;

				AreaList(int n):
					areaL(n),
					sum(0)
				{}
				void operator()(const int n, const Vec2& p0, const Vec2& p1) {
					const float a = Triangle::CalcArea(p0,p1);
					areaL[n] = a;
					sum += a;
				}
			};
			template <class CB>
			void IterateEdge(const Convex& c, CB&& cb) {
				// 頂点数は3つ以上
				const auto& pts = c.points();
				const int nL = pts.size();
				D_Assert0(nL > 2);

				// 先にブリッジの箇所を処理
				cb(nL-1, pts.back(), pts.front());
				for(int i=0 ; i<nL-1 ; i++)
					cb(i, pts[i], pts[i+1]);
			}
		}
		float Convex::bs_getArea() const {
			AreaSum as;
			IterateEdge(*this, as);
			return as.result;
		}
		Convex& Convex::operator += (const Vec2& ofs) {
			for(auto& p : point)
				p += ofs;
			return *this;
		}
		Convex Convex::operator * (const AMat32& m) const {
			const int nP = point.size();
			PointL pl(nP);
			for(int i=0 ; i<nP ; i++)
				pl[i] = point[i].convertI<3,2>(1) * m;
			return pl;
		}
		bool Convex::operator == (const Convex& c) const {
			return point == c.point;
		}
		bool Convex::operator != (const Convex& c) const {
			return !(this->operator ==(c));
		}
		Convex& Convex::operator *= (const AMat32& m) {
			for(auto& p : point)
				p = p.convertI<3,2>(1) * m;
			return *this;
		}
		bool Convex::isClockwise() const {
			const int nV = point.size();
			if(nV < 3)
				return false;
			if(nV == 3)
				return true;
			const Vec2 c = (point[0] + point[1] + point[2]) * (1.f/3);
			for(int i=0 ; i<nV-1 ; i++) {
				if((point[i+1] - point[i]).cw(c-point[i]) < 0)
					return false;
			}
			return (point[0] - point[nV-1]).cw(c-point[0]) >= 0;
		}
		std::tuple<float,float,Vec2> Convex::area_inertia_center() const {
			const int nL = point.size();
			AreaList al(nL);
			IterateEdge(*this, std::ref(al));
			const float invarea = 1.f / al.sum;
			const float areaInv3 = invarea * (1.f/3),
						areaInv6 = invarea * (1.f/6);
			Vec2 center(0,0);
			float inertia = 0;
			IterateEdge(*this, [&, areaInv3, areaInv6](const int n, const Vec2& p0, const Vec2& p1) {
				center += (p0 + p1) * al.areaL[n] * areaInv3;
				inertia += al.areaL[n] * areaInv6 * (p0.dot(p0) + p0.dot(p1) + p1.dot(p1));
			});
			inertia -= center.len_sq();
			return std::make_tuple(al.sum, inertia, center);
		}
		void Convex::adjustLoop() {
			// point全部が凸包に使用される頂点と仮定
			const Vec2 c = (point[0] + point[1] + point[2]) * (1.f/3);
			const int nV = point.size();
			using AngPair = std::pair<frea::RadF, Vec2>;
			std::vector<AngPair> tmp(nV);
			for(int i=0 ; i<nV ; i++)
				tmp[i] = AngPair{frea::AngleValue(Vec2(point[i]-c)), point[i]};
			std::sort(tmp.begin(), tmp.end(),
				[](const AngPair& a0, const AngPair& a1) {
					return a0.first < a1.first;
			});
			for(int i=0 ; i<nV ; i++)
				point[i] = tmp[i].second;
		}
		Vec2 Convex::support(const Vec2& dir) const {
			Vec2 result = point[0];
			float dMax = point[0].dot(dir);
			const int nV = point.size();
			for(int i=1 ; i<nV ; i++) {
				const float d = point[i].dot(dir);
				if(d > dMax) {
					result = point[i];
					dMax = d;
				}
			}
			return result;
		}
		const PointL& Convex::points() const noexcept {
			return point;
		}
		bool Convex::hit(const Vec2& p) const {
			const int sz = point.size();
			for(int i=0 ; i<sz ; i++) {
				const float d = (point[(i+1)%sz] - point[i]).cw(p - point[i]);
				if(d < 0)
					return false;
			}
			return true;
		}
		Convex2 Convex::splitTwo(const Line& ls) const {
			const int nV = point.size();
			PointL pt0(nV*2), pt1(nV*2);		// 最初に最大容量確保しておき、後で縮める
			Vec2* pDst[2] = {&pt0[0], &pt1[0]};

			constexpr float DOT_THRESHOLD = 1e-5f;
			// 右側は0, ライン上は前回値, 左側は1を返す
			auto fcheck = [&ls](int prev, const Vec2& pt) -> int {
				float c = ls.dir.ccw(pt - ls.pos);
				if(c >= DOT_THRESHOLD)
					return 0;
				if(c <= -DOT_THRESHOLD)
					return 1;
				return prev;
			};
			// 走査の始点はライン上でない頂点とする
			int cur = 0;
			int flag;
			while(cur != nV) {
				const auto& p = point[cur++];
				flag = fcheck(-1, p);
				if(flag != -1)
					break;
				// とりあえず片方のリストに貯めておく
				*pDst[0]++ = p;
			}
			// 2点以上がライン上なら分割なし
			if(cur > 2) {
				if(flag == 0x01) {
					// すべて右側
					return Convex2({}, point);
				} else {
					// すべて左側
					return Convex2(point, {});
				}
			}
			if(flag == 0x01) {
				std::swap(pt0, pt1);
				std::swap(pDst[0], pDst[1]);
			}
			int prev = flag;

			const auto fadd = [&pDst, &ls](const Vec2& pPrev, const Vec2& pCur, const int flg) {
				switch(flg) {
					case 0x03:		// Left -> Left
						*pDst[1]++ = pPrev;
						break;
					case 0x02: {	// Left -> Right
						auto res = Segment(pPrev, pCur).crossPoint(ls);
						*pDst[1]++ = pPrev;
						if(!res)
							*res = pCur;
						if(pPrev.dist_sq(*res) >= 1e-6f)
							*pDst[1]++ = *res;
						*pDst[0]++ = *res;
						break; }
					case 0x01: {	// Right -> Left
						auto res = Segment(pPrev, pCur).crossPoint(ls);
						if(!res)
							*res = pCur;
						*pDst[0]++ = pPrev;
						if(pPrev.dist_sq(*res) >= 1e-6f)
							*pDst[0]++ = *res;
						*pDst[1]++ = *res;
						break; }
					case 0x00:		// Right -> Right
						*pDst[0]++ = pPrev;
						break;
				}
			};
			for(int i=cur ; i<nV ; i++) {
				// 正数が左側、負数は右側
				const auto& p = point[i];
				prev = fcheck(prev, p);
				flag = ((flag<<1) | prev) & 0x03;
				fadd(point[i-1], p, flag);
			}
			prev = fcheck(prev, point[0]);
			flag = ((flag<<1) | prev) & 0x03;
			fadd(point[nV-1], point[0], flag);

			if(pDst[0] - &pt0[0] < 2)
				pDst[0] = &pt0[0];
			else if(pDst[1] - &pt1[0] < 2)
				pDst[1] = &pt1[0];
			else {
				if(cur == 2) {
					// 最初の1点がライン上だったので頂点を複製
					if((flag & 1) == 1)
						*pDst[1]++ = pt0[0];
					else
						*pDst[0]++ = pt1[0];
				}
			}

			pt0.resize(pDst[0] - &pt0[0]);
			pt1.resize(pDst[1] - &pt1[0]);
			return Convex2(std::move(pt0), std::move(pt1));
		}
		Convex Convex::split(const Line& ls) {
			auto res = splitTwo(ls);
			std::swap(res.first, *this);
			return std::move(res.second);
		}
		void Convex::splitThis(const Line& l) {
			split(l);
		}
		std::tuple<bool,Vec2,Vec2> Convex::checkCrossingLine(const Line& ls) const {
			//TODO: 余裕があったら2分探索にする
			Vec2 pt[2];
			Vec2* ppt = pt;
			const int nV = point.size();
			for(int i=0 ; i<nV ; i++) {
				auto res = Segment(point[i], point[CndSub(i+1, nV)]).crossPoint(ls);
				if(res) {
					*ppt++ = *res;
					if(ppt == pt+2)
						break;
				}
			}
			return std::make_tuple(ppt == pt+2, pt[0], pt[1]);
		}
		Segment Convex::getOuterSegment(const int n) const {
			return Segment(point[(n+1)%point.size()], point[n]);
		}
		Line Convex::getOuterLine(const int n) const {
			return Line(point[n], (point[CndSub(n+1, point.size())] - point[n]).normalization());
		}
		void Convex::distend(const float r) {
			const int nP = point.size();
			PointL tmp(nP);
			// 単純に重心からの方向ベクトルで算出
			const auto center = bs_getCenter();
			for(int i=0 ; i<nP ; i++) {
				const Vec2 dir = (point[i] - center) * r;
				tmp[i] = center + dir;
			}
		}
		bool Convex::addPoint(const Vec2& p) {
			const int sz = point.size();
			D_Assert0(sz >= 3);
			const auto res = checkPosition(p);
			if(res.first == ConvexPos::Inner) {
				// pが凸包内部にある場合は追加出来ない
				return false;
			}
			bool bResult = false;
			if(res.first == ConvexPos::OnLine) {
				// エッジ上にあるならOk
				bResult = true;
			} else if(res.first == ConvexPos::Outer) {
				// 左側のエッジの右側で
				if(getOuterLine(CndSub(res.second+1, sz, sz)).checkSide(p) != LineDivision::Ccw) {
					// 右側のエッジの左側にあれば
					if(getOuterLine(CndAdd(res.second-1, 0, sz)).checkSide(p) != LineDivision::Ccw) {
						// 凸形状を保てるのでOk
						bResult = true;
					}
				}
			}
			if(bResult) {
				point.emplace(point.begin()+res.second+1, p);
				return true;
			}
			return false;
		}
		std::pair<ConvexPos,int> Convex::checkPosition(const Vec2& pos, const float threshold) const {
			// 適当に内部点を算出
			const Vec2 inner = (point[0] + point[1] + point[2]) * (1.f/3);
			const Vec2 toP(pos - inner);
			if(toP.len_sq() < NEAR_THRESHOLD_SQ) {
				// 重心がちょうどposと重なってしまったら少しずらす
				inner.l_intp(point[0], 0.5f);
			}

			// 内部のどの三角形に該当するか2分探索
			const int nV = point.size();
			struct Tmp {
				Vec2	vec;
				int		index;
				float	cw;
			};
			// tmpA = point[0]へのベクトル
			Tmp tmpA, tmpB, tmpC;
			tmpA.vec = point[0]-inner;
			tmpA.index = 0;
			tmpA.cw = tmpA.vec.cw(toP);
			// tmpB = point[nV]へのベクトル
			tmpB.index = nV;
			tmpB.vec = tmpA.vec;
			tmpB.cw = tmpA.cw;

			while(tmpA.index+1 < tmpB.index) {
				// 捜査対象範囲の半分で区切る => tmpC
				tmpC.index = (tmpA.index+tmpB.index) / 2;
				tmpC.vec = point[tmpC.index] - inner;
				tmpC.cw = tmpC.vec.cw(toP);
				const float crAC = tmpA.vec.cw(tmpC.vec);

				if(tmpA.cw >= 0) {
					// posはtmpAの右側
					if(crAC >= 0) {
						// tmpCはtmpAの右側
						if(tmpC.cw <= 0) {
							// posはtmpCの左側
							tmpB = tmpC;
						} else
							tmpA = tmpC;
					} else
						tmpB = tmpC;
				} else {
					// posはtmpAの左側
					if(crAC >= 0) {
						// tmpCはtmpAの右側
						tmpA = tmpC;
					} else {
						if(tmpC.cw >= 0) {
							// posはtmpCの右側
							tmpA = tmpC;
						} else
							tmpB = tmpC;
					}
				}
			}
			tmpB.index = CndSub(tmpB.index, nV);
			const float d = (point[tmpB.index] - point[tmpA.index]).cw(pos - point[tmpA.index]);
			ConvexPos posf;
			if(d > threshold)
				posf = ConvexPos::Inner;
			else if(d < -threshold)
				posf = ConvexPos::Outer;
			else
				posf = ConvexPos::OnLine;
			return std::make_pair(posf, tmpA.index);
		}
		Convex Convex::GetOverlappingConvex(const Convex& m0, const Convex& m1, const Vec2& inner) {
			D_Assert(m0.hit(inner) && m1.hit(inner), "invalid inner point");

			// DualTransformで直線から点に変換
			const auto &v0 = m0.points(),
						&v1 = m1.points();
			const int nV0 = v0.size(),
						nV1 = v1.size();
			std::vector<Vec2> vTmp(std::max(nV0,nV1));
			std::vector<Vec2> v2(nV0+nV1);
			for(int i=0 ; i<nV0 ; i++)
				vTmp[i] = v0[i] - inner;
			for(int i=0 ; i<nV0 ; i++)
				v2[i] = Dual2(vTmp[i], vTmp[CndSub(i+1, nV0)]);

			for(int i=0 ; i<nV1 ; i++)
				vTmp[i] = v1[i] - inner;
			for(int i=0 ; i<nV1 ; i++)
				v2[i+nV0] = Dual2(vTmp[i], vTmp[CndSub(i+1, nV1)]);

			// 凸包を求める
			Convex cc = Convex::FromConcave(v2);
			// DualTransformで直線から点に変換
			const int nVD = cc.point.size();
			v2.resize(nVD);
			for(int i=0 ; i<nVD ; i++)
				v2[i] = -Dual2(cc.point[i], cc.point[CndSub(i+1, nVD)]) + inner;

			return v2;
		}
		Convex::ITriangleV Convex::MonotoneToTriangle(const PointL& pts, const Vec2& dir) {
			const int sz = pts.size();
			D_Assert0(sz>=3);

			const auto fnIsCW = [&pts](const int i0, const int i1, const int i2) {
				const auto& pt0 = pts[i0];
				return (pts[i1] - pt0).cw(pts[i2] - pt0) > 0;
			};
			if(sz < 4) {
				if(fnIsCW(0,1,2))
					return {ITriangle_t(0,1,2)};
				return {ITriangle_t(0,2,1)};
			}
			ITriangleV ret;
			const auto fnOutputTriangle = [&ret, &fnIsCW](const int i0, const int i1, const int i2){
				if(fnIsCW(i0,i1,i2)) {
					ret.emplace_back(i0, i1, i2);
					return true;
				}
				return false;
			};
			const auto ia = MakeIndexArray(pts, dir);
			const auto fnInt = lubee::CyInt::MakeValueF(sz);
			// MonotonePolygonの左辺(idx_cw=上辺カーソル、idx_ccw=下辺カーソル)
			auto idx_cw = fnInt(1),
				idx_ccw = fnInt(0);
			if(fnInt(ia[idx_cw].first-1) != ia[idx_ccw].first)
				std::swap(idx_cw, idx_ccw);
			idx_cw.set(ia[idx_cw].first);
			idx_ccw.set(ia[idx_ccw].first);

			std::deque<int> rope;
			rope.push_back(idx_cw);
			rope.push_front(idx_ccw);
			for(int cur=2 ; cur<sz ; cur++) {
				const auto& a = ia[cur];
				Convex cnv;
				if(a.first == (rope.back()+1)%sz) {
					while(rope.size() > 1) {
						if(fnOutputTriangle(*(rope.end()-2), rope.back(), a.first)) {
							rope.pop_back();
						} else
							break;
					}
					rope.push_back(a.first);
				} else {
					while(rope.size() > 1) {
						if(fnOutputTriangle(*(rope.begin()+1), a.first, rope.front())) {
							rope.pop_front();
						} else
							break;
					}
					rope.push_front(a.first);
				}
			}
			return ret;
		}
		namespace {
			int BinSearch(const PointL& point, const int nV, const Convex& mdl, int a, int b, const bool flip) {
				for(;;) {
					if(a+1 >= b)
						break;
					const int c = (a+b)/2;
					if(mdl.hit(point[CndRange(c, nV)]) ^ flip)
						b = c;
					else
						a = c;
				}
				return CndRange(a, nV);
			}
		}
		std::pair<bool,PointL> Convex::getOverlappingPoints(const Convex& mdl, const Vec2& inner) const {
			const auto res = checkPosition(inner);
			if(res.first != ConvexPos::Outer) {
				const int nV = point.size();
				int a,b, begI;
				// 2分探索で衝突が始まる地点を探す
				if(mdl.hit(point[res.second])) {
					a = res.second - nV;
					b = res.second;
					// 衝突開始インデックス(これ自体は衝突していない)
					begI = BinSearch(point, nV, mdl, a,b,false);
				} else {
					begI = res.second;
					if(!mdl.hit(point[CndSub(res.second+1, nV)]))
						return std::make_pair(false, PointL());
				}

				// 衝突が終わる地点を探す
				a = begI + 1;
				b = a + nV;
				// 衝突終了インデックス(これ自体衝突している)
				int endI = BinSearch(point,nV,mdl,a,b,true);
				if(begI == endI) {
					// 全部出力
					return std::make_pair(true, point);
				}
				endI = CndSub(endI+1, nV);
				endI = CndAdd(endI, begI, nV)+1;
				PointL pts(endI - begI);
				auto* pDst = &pts[0];
				// 開始地点から終了地点までを出力
				while(begI != endI) {
					*pDst++ = point[CndSub(begI,nV)];
					++begI;
				}
				return std::make_pair(false, std::move(pts));
			}
			return std::make_pair(false, PointL());
		}
		namespace {
			bool CheckCrossingEdge(const PointL& pts, const Edge& e) {
				const Segment s0(pts[e.first], pts[e.second]);
				const int sz = pts.size();
				for(int i=0 ; i<sz ; i++) {
					const int i0 = i,
							i1 = (i+1)%sz;
					if(i0 == e.first ||
						i0 == e.second)
						continue;
					if(i1 == e.first ||
						i1 == e.second)
						continue;
					if(Segment(pts[i0], pts[i1]).hit(s0))
						return true;
				}
				return false;
			}
			enum class CType {
				Merge,
				Split,
				None
			};
			using lubee::CyInt;
			using IArray = std::vector<int>;
			// base = pa基準のインデックス
			CType CornerType(const PointL& pts, const PArray& pa, const IArray& idxToI, const CyInt base) {
				const CyInt pts_id(pa[base].first, base.limit);
				const auto &pbase = pts[pts_id],
							&pnext = pts[pts_id+1],
							&pprev = pts[pts_id-1];
				const float x = pa[base].second,
							dx_n = pa[idxToI[pts_id+1]].second - x,
							dx_p = pa[idxToI[pts_id-1]].second - x;
				if(dx_p * dx_n < 0)
					return CType::None;
				const float cw = (pprev - pbase).cw(pnext - pbase);
				if(cw > 0) {
					if(dx_p > 0) {
						// 両方共ラインの右
						return CType::Split;
					} else {
						// 両方共ラインの左
						return CType::Merge;
					}
				}
				return CType::None;
			}
			struct Link {
				private:
					CyInt _baseId;
					struct L {
						int	id,
							priority;
						L(const int i):
							id(i)
						{}
					};
					using LV = std::vector<L>;
					LV	_lv;

				public:
					Link(const CyInt base):
						_baseId(base)
					{
						_lv.emplace_back(base+1);
					}
					void add(const int id) {
						if(std::find_if(_lv.begin(), _lv.end(), [id](const auto& l){ return l.id == id; }) == _lv.end())
							_lv.emplace_back(id);
					}
					void sort(const int sz) {
						for(auto& l : _lv) {
							l.priority = l.id;
							if(l.id < _baseId)
								l.priority += sz;
						}
						std::sort(_lv.begin(), _lv.end(), [](const auto& l0, const auto& l1){
							return l0.priority < l1.priority;
						});
					}
					int baseId() const {
						return _baseId;
					}
					const LV& link() const {
						return _lv;
					}
					int getNextLink(const int id) const {
						if(id == _baseId-1)
							return _lv.back().id;
						auto itr = std::find_if(_lv.begin(), _lv.end(), [id](const auto& l){
							return l.id == id;
						});
						D_Assert0(itr != _lv.end() && itr != _lv.begin());
						--itr;
						return itr->id;
					}
			};
		}
		void Convex::ConcaveToMonotone(const PointL& pts, const Vec2& dir, const CBPoints& cb) {
			const int sz = pts.size();
			D_Assert0(sz >= 3);
			if(sz < 4) {
				cb(PointL(pts));
				return;
			}
			std::unordered_map<int, Link> link;
			// Edge=pts基準のインデックス
			const auto addLink = [sz, &link](const Edge& e){
				const auto proc = [sz, &link](const Edge& e){
					if(link.count(e.first) == 0)
						link.emplace(e.first, Link(CyInt{e.first, sz}));
					link.at(e.first).add(e.second);
				};
				proc(e);
				proc(e.flip());
			};
			// Dir方向でソートする
			const auto ia = MakeIndexArray(pts, dir);
			IArray idxToI(sz);
			for(int i=0 ; i<sz ; i++) {
				idxToI[ia[i].first] = i;
			}
			// merge point, split point を探す
			for(int i=0 ; i<sz ; i++) {
				auto& pt = ia[i];
				switch(CornerType(pts, ia, idxToI, CyInt(i, sz, CyInt::AsInRange))) {
					case CType::Merge:
						[&](){
							// 右の最寄り頂点があれば処理
							if(i < sz-1) {
								int cur = i+1;
								while(pt.second >= ia[cur].second || CheckCrossingEdge(pts, Edge{pt.first, ia[cur].first})) {
									if(++cur == sz)
										return;
								}
								// リンクを設定(元の頂点インデックスを指定)
								addLink({pt.first, ia[cur].first});
							}
						}();
						break;
					case CType::Split:
						[&](){
							// 左の最寄り頂点があれば処理
							if(i > 0) {
								int cur = i-1;
								while(pt.second <= ia[cur].second || CheckCrossingEdge(pts, Edge{pt.first, ia[cur].first})) {
									if(--cur == -1)
										return;
								}
								// リンクを設定(元の頂点インデックスを指定)
								addLink({pt.first, ia[cur].first});
							}
						}();
						break;
					case CType::None:
						break;
				}
			}
			if(link.empty()) {
				cb(PointL(pts));
				return;
			}
			std::unordered_set<Edge, Edge> cand;
			for(auto& l : link) {
				l.second.sort(sz);
				const auto base = l.second.baseId();
				for(const auto& l : l.second.link()) {
					cand.emplace(base, l.id);
				}
			}

			while(!cand.empty()) {
				const auto e = *cand.begin();
				cand.erase(cand.begin());

				PointL pl = {pts[e.first]};
				int prev = e.first;
				CyInt cur(e.second, sz);
				// 一周するまでループ
				do {
					pl.push_back(pts[cur]);
					// 今いる頂点から分岐が存在するか？
					const auto itr = link.find(cur);
					if(itr != link.end()) {
						const int next = itr->second.getNextLink(prev);
						const auto itr2 = cand.find(Edge{cur, next});
						if(itr2 != cand.end())
							cand.erase(itr2);
						prev = cur;
						cur = next;
					} else {
						prev = cur;
						cur += 1;
					}
				} while(cur != e.first);

				// コールバック関数に送出
				cb(std::move(pl));
			}
		}
		std::ostream& operator << (std::ostream& os, const Convex& c) {
			os << "Convex(2d) [";
			int idx=0;
			bool bF = true;
			for(auto& p : c.point) {
				if(!bF)
					os << ", ";
				os << idx++ << ": " << p;
				bF = false;
			}
			return os << ']';
		}
	}
}
