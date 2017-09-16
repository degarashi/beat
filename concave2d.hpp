#pragma once
#include "lubee/range.hpp"
#include "frea/vector.hpp"
#include "g.hpp"
#include "triangle2d.hpp"

namespace beat {
	namespace g2 {
		using frea::Vec2;
		using PointL = std::vector<Vec2>;
		using lubee::RangeF;
		template <class T>
		using FRand = std::function<T (const lubee::Range<T>&)>;
		using FRandF = FRand<float>;
		using FRandI = FRand<int>;
		// Monotoneの内部距離をとっとけば点との判定可能
		class MonoPolygon {
			using DistV = std::vector<Vec2>;
			private:
				Vec2		_vOrigin,
							_vDir;		//!< 軸方向
				// 主軸で左右に分けた場合の頂点数と原点からの距離
				DistV			_distL,
								_distR;
				float			_widthOffset;

				MonoPolygon(DistV&& dL, DistV&& dR, const Vec2& ori, const Vec2& dir, float wofs);
			public:
				PointL getPoints() const;
				const Vec2& getDir() const;
				bool hit(const Vec2& p) const;
				static MonoPolygon Random(const FRandF& rff, const FRandI& rfi, const RangeF& rV, const RangeF& rLen, int nMaxV);
				static bool IsMonotone(const PointL& pts, const Vec2& dir);
		};
		/*! 3角ポリゴンを1つずつ加えていって最後に外周をとればConcaveになる
			3角ポリゴンの集合体なので
			当たり判定のチェックは容易に出来る */
		class ConcavePolygon {
			private:
				template <class T>
				struct Neighbor {
					T*	pNeighbor;
					int	edgeId;		//!< pNeighbor側のエッジ
				};
				template <class T>
				using PtrT = std::array<Neighbor<T>, 3>;
				using IdxT = ITriangleDataR<PtrT, Triangle, Vec2>;
				using PolyPV = std::vector<IdxT*>;

				PointL		_vtx;
				PolyPV		_poly;

				ConcavePolygon() = default;
			public:
				PointL getPoints() const;
				bool hit(const Vec2& p) const;
				static ConcavePolygon Random(const FRandF& rff, const FRandI& rfi, const RangeF& rV, int nPoly);
				static bool IsConcave(const PointL& pts);
		};
	}
}
