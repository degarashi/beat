#pragma once
#include "imodel2d.hpp"
#include "constant.hpp"
#include "g.hpp"

namespace beat {
	namespace g2 {
		//! 凸包の位置を示す
		enum class ConvexPos {
			Inner,
			OnLine,
			Outer
		};
		using PointL = std::vector<Vec2>;
		using Convex2 = std::pair<Convex, Convex>;
		//! 多角形基本クラス
		class Convex : public CID<Convex> {
			private:
				PointL	point;

			public:
				Convex() = default;
				/*! \param[in] v 凸包と分かっている頂点 */
				Convex(const std::initializer_list<Vec2>& v);
				Convex(const PointL& pl);
				Convex(PointL&& pl) noexcept;

				//! 頂点の並びを時計回りに修正
				void adjustLoop();
				//! 頂点が時計回りになっているか
				bool isClockwise() const;
				using CBPoints = std::function<void (PointL&&)>;
				static void ConcaveToMonotone(const PointL& pts, const Vec2& dir, const CBPoints& cb);
				using ITriangle_t = ::beat::ITriangle<Triangle, Vec2>;
				using ITriangleV = std::vector<ITriangle_t>;
				static ITriangleV MonotoneToTriangle(const PointL& pts, const Vec2& dir);

				std::pair<bool,PointL> getOverlappingPoints(const Convex& mdl, const Vec2& inner) const;
				static Convex GetOverlappingConvex(const Convex& m0, const Convex& m1, const Vec2& inner);
				/*! 同時に求めると少し効率が良い為 */
				std::tuple<float,float,Vec2> area_inertia_center() const;
				//! 凹ポリゴンを内包するような凸形状を求める
				static Convex FromConcave(const PointL& src);
				void distend(float r);
				const PointL& points() const noexcept;
				//! 2つに分割
				/*! \param[out] c0 線分の進行方向左側
					\param[out] c1 線分の進行方向右側 */
				Convex2 splitTwo(const Line& l) const;
				//! 2つに分割して左側を自身に適用
				Convex split(const Line& l);
				//! 2つに分割して右側は捨てる
				void splitThis(const Line& l);
				//! 凸包が直線と交差している箇所を2点計算
				std::tuple<bool,Vec2,Vec2> checkCrossingLine(const Line& l) const;
				//! 内部的な通し番号における外郭ライン
				Segment getOuterSegment(int n) const;
				Line getOuterLine(int n) const;
				bool addPoint(const Vec2& p);
				//! 指定ポイントの内部的な領域IDと内外位置を取得
				/*! \return first=内外判定
							second=領域ID */
				std::pair<ConvexPos, int> checkPosition(const Vec2& pos, float threshold=DOT_THRESHOLD) const;

				// ----------------------------------------
				float bs_getArea() const;
				Circle bs_getBCircle() const;
				Vec2 bs_getCenter() const;
				float bs_getInertia() const;
				AABB bs_getBBox() const;

				Vec2 support(const Vec2& dir) const;
				Convex operator * (const AMat32& m) const;
				Convex& operator *= (const AMat32& m);
				Convex& operator += (const Vec2& ofs);
				bool operator == (const Convex& c) const;
				bool operator != (const Convex& c) const;
				friend std::ostream& operator << (std::ostream& os, const Convex& c);
				// ----------------------------------------

				spi::none_t hit(...) const;
				bool hit(const Vec2& p) const;
		};
		using ConvexM = Model<Convex>;
	}
}
