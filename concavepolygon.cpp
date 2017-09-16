#include "concave2d.hpp"
#include "frea/random/vector.hpp"
#include "segment2d.hpp"
#include "capsule2d.hpp"
#include "circle2d.hpp"
#include "aabb2d.hpp"
#include "gjk2d.hpp"

namespace beat {
	namespace g2 {
		bool ConcavePolygon::IsConcave(const PointL& pts) {
			const int sz = pts.size();
			for(int i=0 ; i<sz ; i++) {
				const Segment s0(pts[i], pts[(i+1)%sz]);
				for(int j=i+2 ; j<sz-1 ; j++) {
					const Segment s1(pts[j], pts[(j+1)%sz]);
					if(s0.hit(s1))
						return false;
				}
			}
			return true;
		}
		g2::PointL ConcavePolygon::getPoints() const {
			// 始点を探す -> どれが一辺でも隣接ポリゴンを持ってない物
			int polyId = -1,
				edgeId;
			const int nPoly = _poly.size();
			for(int i=0 ; i<nPoly ; i++) {
				auto* p = _poly[i];
				for(int j=0 ; j<3 ; j++) {
					if(!p->data[j].pNeighbor) {
						polyId = i;
						edgeId = j;
						break;
					}
				}
				if(polyId >= 0)
					break;
			}
			D_Assert0(polyId >= 0);

			std::vector<bool> chk0(_vtx.size());
			// 時計回りに外周をたどってリスト出力
			std::vector<int> index;
			const auto* pFirst = _poly[polyId];
			index.push_back(pFirst->idx[edgeId]);
			chk0[index.back()] = true;
			// 次のポリゴンへ
			const auto* pe = pFirst->data.data() + ((edgeId+1)%3);
			if(!pe->pNeighbor) {
				// 2辺が外周
				index.push_back(pFirst->idx[(edgeId+1)%3]);
				chk0[index.back()] = true;
				pe = pFirst->data.data() + ((edgeId+2)%3);
			}
			const  auto* p = pe->pNeighbor;
			edgeId = pe->edgeId;
			if(p) {
				for(;;) {
					// 隣にポリゴンを持っていたらそのまま進む
					const auto* pe = p->data.data() + ((edgeId+1)%3);
					if(pe->pNeighbor) {}
					else {
						// なければ最初の頂点を追加した後に次へ
						index.push_back(p->idx[(edgeId+1)%3]);
						if(chk0[index.back()]) {
							index.pop_back();
							break;
						}
						chk0[index.back()] = true;
						pe = p->data.data() + ((edgeId+2)%3);
						if(!pe->pNeighbor) {
							// 2辺が外周
							index.push_back(p->idx[(edgeId+2)%3]);
							if(chk0[index.back()]) {
								index.pop_back();
								break;
							}
							chk0[index.back()] = true;
							pe = p->data.data() + edgeId;
						}
					}
					p = pe->pNeighbor;
					edgeId = pe->edgeId;
				}
			} else {
				D_Assert0(int(_vtx.size())==3);
				index.push_back(pFirst->idx[(edgeId+2)%3]);
			}
			// 外周 == 頂点サイズ
			D_Assert0(index.size() == _vtx.size());
			const int nv = _vtx.size();
			std::vector<bool> chk(nv);
			for(auto& idx : index) {
				D_Assert0(!chk[idx]);
				chk[idx] = true;
			}

			g2::PointL pl(nv);
			for(int i=0 ; i<nv ; i++)
				pl[i] = _vtx[index[i]];
			return pl;
		}
		bool ConcavePolygon::hit(const Vec2& p) const {
			// 中身は単なる三角形ポリゴンのリストなので個別に判定
			for(auto& pl : _poly) {
				if(Triangle(_vtx[pl->idx[0]],
							_vtx[pl->idx[1]],
							_vtx[pl->idx[2]]).hit(p))
					return true;
			}
			return false;
		}
		ConcavePolygon ConcavePolygon::Random(const FRandF& rff, const FRandI& rfi, const RangeF& rV, int nPoly) {
			// ランダムに3角ポリゴンを繋げていく
			nPoly = std::max(1, nPoly);

			ConcavePolygon res;
			auto& vtx = res._vtx;
			auto& poly = res._poly;
			// 最初のポリゴンを生成
			{
				const auto rdp = [&](){ return rff(rV); };
				vtx = frea::random::GenVecN<Vec2>(rdp, 3, 1e1f);
				int idx[3] = {0,1,2};
				if(!g2::Triangle(vtx[0], vtx[1], vtx[2]).isClockwise())
					std::swap(idx[1], idx[2]);
				poly.emplace_back(new IdxT({idx[0], idx[1], idx[2]}, {}));
			}
			--nPoly;

			while(nPoly > 0) {
				// ランダムでポリゴンを選び、隣に何も繋がっていない辺を選ぶ
				int polyId,
					edgeId = -1;
				for(;;) {
					polyId = rfi({0, int(poly.size()-1)});
					auto* p = poly[polyId];
					for(int i=0 ; i<3 ; i++) {
						if(!p->data[i].pNeighbor) {
							edgeId = i;
							break;
						}
					}
					if(edgeId >= 0)
						break;
				}
				auto* p = poly[polyId];
				g2::Triangle tpoly(vtx[p->idx[0]], vtx[p->idx[1]], vtx[p->idx[2]]);
				auto &v0 = tpoly.pos[edgeId],
					&v1 = tpoly.pos[(edgeId+1)%3];
				Vec2 dir = (v1 - v0).normalization();
				dir = Vec2(-dir.y, dir.x);
				const auto edgeCenter = (v0 + v1)/2;

				// 外側に変異
				// 変異量を特に制限しない
				const float moveDist = rff({1e-2f, 1e1f});
				const auto newVtx = edgeCenter + dir * moveDist;
				// 新たに増えるポリゴン線分
				const int i0 = p->idx[edgeId],
							i1 = p->idx[(edgeId+1)%3];
				CapsuleM cap[2] = {
					{vtx[i0], newVtx, 1e-5f},
					{vtx[i1], newVtx, 1e-5f}
				};
				for(int k=0 ; k<2 ; k++) {
					auto& c = cap[k];
					const auto dir = c.getDir();
					c.from += dir*1e-3f;
				}
				// 他のポリゴンと重なっていたらやり直し
				if(
					![&vtx, &poly, cap](){
						for(auto* p : poly) {
							const TriangleM tri = p->toTriangle(vtx);
							if(GSimplex(tri, cap[0]).getResult() ||
								GSimplex(tri, cap[1]).getResult())
								return false;
						}
						return true;
					}()
				)
					continue;

				// 新しい頂点を登録
				const int newVid = vtx.size();
				vtx.push_back(newVtx);
				// ポリゴンを形成
				poly.push_back(new IdxT({i0, newVid, i1},
										{{Neighbor<IdxT>{nullptr, 0}, {nullptr,0}, {p, edgeId}}}));
				auto& pe = p->data[edgeId];
				pe.pNeighbor = poly.back();
				pe.edgeId = 2;
				--nPoly;
			}
			return res;
		}
	}
}
