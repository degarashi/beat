#include "../g.hpp"
#include "../circle2d.hpp"
#include "../triangle2d.hpp"
#include "../aabb2d.hpp"
#include "frea/src/random/vector.hpp"
#include "frea/src/ulps.hpp"
#include "lubee/src/random.hpp"
#include "lubee/src/error.hpp"
#include "lubee/src/tests/test.hpp"
#include <unordered_map>
#include <gtest/gtest.h>

namespace beat {
	struct EdgeTo {
		int		polyId, side;
	};
	using frea::Vec2;
	using Vec2V = std::vector<Vec2>;
	using g2::Triangle;
	using g2::Circle;
	using g2::AABB;
	using ITriangle_t = ITriangle<Triangle, Vec2>;
	struct DTriangle : ITriangle_t {
		Circle	circle;

		DTriangle() = default;
		DTriangle(const Vec2V& pts, const int i0, const int i1, const int i2):
			ITriangle_t(i0, i1, i2)
		{
			refreshCircle(pts);
		}
		void refreshCircle(const Vec2V& pts) {
			const auto t = toTriangle(pts);
			try {
				circle = Circle::Circumscribing(t.pos[0], t.pos[1], t.pos[2]);
			} catch(const Circle::NoValidCircle&) {
				circle.center = Vec2{0,0};
				circle.radius = std::numeric_limits<float>::max();
			}
		}
	};
	class Delaunay {
		private:
			using EdgeMap = std::unordered_map<Edge, EdgeTo, Edge>;
			EdgeMap		_edge;
			using DTV = std::vector<DTriangle>;
			DTV			_dt;
			Vec2V		_pts;

		public:
			Delaunay(const AABB& ab):
				_dt(1),
				_pts(3)
			{
				const auto super = Triangle::Encapsule(ab);
				for(int i=0 ; i<3 ; i++)
					_pts[i] = super.pos[i];
				_dt[0] = DTriangle(_pts, 0, 1, 2);
				_edge.emplace(Edge{0,1}, EdgeTo{0, 0});
				_edge.emplace(Edge{1,2}, EdgeTo{0, 1});
				_edge.emplace(Edge{2,0}, EdgeTo{0, 2});
			}
			void addPoint(const Vec2& p) {
				const int prevNV = _pts.size();
				_pts.emplace_back(p);

				// find triangle
				int idx = -1;
				const int NT = _dt.size();
				for(int j=0 ; j<NT ; j++) {
					if(_dt[j].hit(_pts, p)) {
						idx = j;
						break;
					}
				}
				D_Assert0(idx >= 0);

				_dt.resize(NT+2);
				const int n_id0 = NT,
						n_id1 = NT+1;
				auto& tri = _dt[idx];
				auto& id = tri.idx;
				// BCD
				_dt[n_id0] = DTriangle{_pts, id[1], id[2], prevNV};
				_edge[Edge{id[1], id[2]}] = EdgeTo{n_id0, 0};
				_edge[Edge{id[2], prevNV}] = EdgeTo{n_id0, 1};
				_edge[Edge{prevNV, id[1]}] = EdgeTo{n_id0, 2};
				// CAD
				_dt[n_id1] = DTriangle{_pts, id[2], id[0], prevNV};
				_edge[Edge{id[2], id[0]}] = EdgeTo{n_id1, 0};
				_edge[Edge{id[0], prevNV}] = EdgeTo{n_id1, 1};
				_edge[Edge{prevNV, id[2]}] = EdgeTo{n_id1, 2};

				using EdgeStack = std::vector<Edge>;
				EdgeStack stk;
				stk.emplace_back(Edge{id[1], id[0]});
				stk.emplace_back(Edge{id[2], id[1]});
				stk.emplace_back(Edge{id[0], id[2]});

				// ABD
				id[2] = prevNV;
				_edge[Edge{id[0], id[1]}] = EdgeTo{idx, 0};
				_edge[Edge{id[1], prevNV}] = EdgeTo{idx, 1};
				_edge[Edge{prevNV, id[0]}] = EdgeTo{idx, 2};
				tri.refreshCircle(_pts);

				while(!stk.empty()) {
					const auto e = stk.back();
					stk.pop_back();

					const auto itr0 = _edge.find(e);
					if(itr0 == _edge.end())
						continue;

					const auto ef = e.flip();
					const auto itr1 = _edge.find(ef);
					D_Assert0(itr1 != _edge.end());
					auto &dt0 = _dt[itr0->second.polyId],
						&dt1 = _dt[itr1->second.polyId];
					const auto dt0side = itr0->second.side;
					const auto dt1side = itr1->second.side;
					const int dt0other = dt0.idx[(dt0side+2)%3];
					const int dt1other = dt1.idx[(dt1side+2)%3];
					if(dt1.circle.hit(_pts[dt0other])) {
						_edge.erase(itr0);
						_edge.erase(itr1);

						dt0.idx[dt0side] = dt1other;
						dt1.idx[dt1side] = dt0other;

						for(int k=0 ; k<3 ; k++) {
							_edge[dt0.getEdge(k)] = EdgeTo{int(&dt0 - &_dt[0]), k};
							_edge[dt1.getEdge(k)] = EdgeTo{int(&dt1 - &_dt[0]), k};
						}
						dt0.refreshCircle(_pts);
						dt1.refreshCircle(_pts);
						for(int k=0 ; k<3 ; k++) {
							if(k != (dt0side+2)%3)
								stk.emplace_back(dt0.getEdge(k).flip());
							if(k != (dt1side+2)%3)
								stk.emplace_back(dt1.getEdge(k).flip());
						}
					}
				}
			}
			const Vec2V& points() const noexcept {
				return _pts;
			}
			using TriangleV = std::vector<ITriangle_t>;
			TriangleV result() const {
				const int NT = _dt.size();
				TriangleV ret(NT);
				auto* dst = ret.data();
				// 外郭三角形を消す
				int DT = _dt.size();
				for(int i=0 ; i<DT ; i++) {
					auto& d = _dt[i];
					bool bInv = false;
					for(int j=0 ; j<3 ; j++) {
						if(d.idx[j] < 3) {
							bInv = true;
							break;
						}
					}
					if(!bInv) {
						*dst++ = _dt[i];
					}
				}
				ret.resize(dst - ret.data());
				return ret;
			}
	};
	namespace test {
		using lubee::RangeF;
		struct Delaunay : lubee::test::Random {};
		TEST_F(Delaunay, Triangles) {
			auto& mt = this->mt();
			const int NV = mt.getUniform<int>({3,100});
			auto rdp = mt.getUniformF<float>(RangeF{1e2f});
			const auto pts0 = frea::random::GenVecN<Vec2>(rdp, NV, 5e-2f);
			const auto ab = AABB::Encapsule(pts0.begin(), pts0.end());
			::beat::Delaunay d(ab);
			for(int i=0 ; i<NV ; i++)
				d.addPoint(pts0[i]);

			const auto& pts = d.points();
			const auto tv = d.result();
			for(auto& t : tv) {
				ASSERT_GE(t.toTriangle(pts).area(), 0);
				for(int j=0 ; j<NV ; j++) {
					if(t.idx[0] == j ||
						t.idx[1] == j ||
						t.idx[2] == j)
						continue;

					auto tri = t.toTriangle(pts);
					auto c = Circle::Circumscribing(tri.pos[0], tri.pos[1], tri.pos[2]);
					c.radius = frea::ulps::Move(c.radius, -(1<<14));
					ASSERT_FALSE(c.hit(pts[j]));
				}
			}
		}
	}
}

