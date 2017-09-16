#pragma once
#include "spine/optional.hpp"
#include "frea/matrix.hpp"

namespace beat {
	using frea::Vec2;
	using frea::AMat2;
	using Vec2_OP = spi::Optional<Vec2>;
	using Vec2x2 = std::pair<Vec2, Vec2>;
	using Vec2x2_OP = spi::Optional<Vec2x2>;
	using PointL = std::vector<Vec2>;

	template <class T, class V>
	struct ITriangle;
	template <class D, class T, class V>
	struct ITriangleData;

	namespace g2 {
		struct IModel;
		struct Line;
		struct Triangle;
		using ITriangle = ::beat::ITriangle<Triangle, Vec2>;
		template <class D>
		using ITriangleData = ::beat::ITriangleData<D, Triangle, Vec2>;

		//! 90度回転行列(2D)
		extern const AMat2 cs_mRot90[2];
		//! 直線との位置関係
		enum LineDivision {
			OnLine = 0x00,
			Cw = 0x01,
			Ccw = 0x02,
			Bridge = 0x03
		};

		using ClipF = std::function<float (float)>;

		bool IsCW(const PointL& pts);
		bool IsCrossing(const Line& ls0, const Line& ls1, float len0, float len1, float t);
		Vec2 NearestPoint(const Line& ls, const Vec2& p, const ClipF& clip);
		Vec2x2_OP NearestPoint(const Line& ls0, const Line& ls1, const ClipF& clip0, const ClipF& clip1);
		//! クラメルの公式で使う行列式を計算
		float CramerDet(const Vec2& v0, const Vec2& v1);
		//! 2元1次方程式を計算
		Vec2 CramersRule(const Vec2& v0, const Vec2& v1, const Vec2& a0, float detInv);
		//! 2次元ベクトルを係数で混ぜ合わせる
		Vec2 MixVector(const float (&cf)[3], const Vec2& p0, const Vec2& p1, const Vec2& p2);
		//! 重心座標を計算
		void BarycentricCoord(float ret[3], const Vec2& p0, const Vec2& p1, const Vec2& p2, const Vec2& pos);

		//! ミンコフスキー差を求める
		Vec2 MinkowskiSub(const IModel& m0, const IModel& m1, const Vec2& dir);
		//! DualTransform (point2D -> line2D)
		Line Dual(const Vec2& v);
		//! DualTransform (line2D -> point2D)
		Vec2 Dual(const Line& ls);
		Vec2 Dual2(const Vec2& v0, const Vec2& v1);
		//! 主成分解析
		Vec2x2 PCA(const PointL& pts);
		using PArray = std::vector<std::pair<int,float>>;
		PArray MakeIndexArray(const PointL& pts, const Vec2& dir);
	}
}
