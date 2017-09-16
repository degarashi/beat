#pragma once
#include "frea/vector.hpp"

namespace beat {
	struct Edge : std::pair<int,int> {
		using base_t = std::pair<int,int>;
		using base_t::base_t;

		std::size_t operator() (const Edge& e) const noexcept {
			const auto h = std::hash<int>();
			return h(e.first) ^ h(e.second);
		}
		Edge flip() const {
			return {second, first};
		}
	};
	template <class T, class V>
	struct ITriangle {
		using triangle_t = T;
		using vertex_t = V;
		using varray_t = std::vector<vertex_t>;
		int		idx[3];

		ITriangle() = default;
		ITriangle(const int i0, const int i1, const int i2):
			idx{i0, i1, i2}
		{}
		triangle_t toTriangle(const varray_t& pts) const {
			return triangle_t(pts[idx[0]], pts[idx[1]], pts[idx[2]]);
		}
		Edge getEdge(const int n) const {
			return {idx[n], idx[(n+1)%3]};
		}
		//! Edgeに属さない頂点番号を取得
		int getNonEdgePoint(const int eId) const {
			constexpr int lst[] = {2, 0, 1};
			return idx[lst[eId]];
		}
		bool hit(const varray_t& pts, const vertex_t& p) const {
			return toTriangle(pts).hit(p);
		}
	};
	template <class D, class T, class V>
	struct ITriangleData : ITriangle<T,V> {
		using base_t = ITriangle<T,V>;
		using data_t = D;
		data_t	data;

		ITriangleData() = default;
		ITriangleData(const base_t& base, const data_t& ta):
			base_t(base),
			data(ta)
		{}
	};
	template <template <class> class D, class T, class V>
	struct ITriangleDataR : ITriangle<T, V> {
		using base_t = ITriangle<T, V>;
		using Data_t = D<ITriangleDataR<D,T,V>>;
		Data_t	data;

		ITriangleDataR() = default;
		ITriangleDataR(const base_t& base, const Data_t& ta):
			base_t(base),
			data(ta)
		{}
	};
	//! ライン上の位置を示す
	enum class LinePos {
		Begin,		//!< 始点
		End,		//!< 終点
		OnLine,		//!< ライン上
		NoContact
	};
	using frea::Vec2;
	using LNear = std::pair<Vec2, LinePos>;
}
