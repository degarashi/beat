#pragma once
#include "triangle2d.hpp"
#include "spine/object_pool.hpp"
#include "lubee/assoc.hpp"

namespace beat {
	namespace g2 {
		float GetRatio(const Vec2& v0, const Vec2& v1, const Vec2& pos);
		//! GJK法による衝突判定(2D)
		/*! ヒットチェックのみ。衝突時は内部点を出力可 */
		class GSimplex {
			protected:
				const IModel	&_m0, &_m1;
				Triangle _tri;			//!< 凸包を構成する三角形
				Vec2	_posA[3],		//!< vtx(A-B)を求める時に使ったA側の座標
						_inner;			//!< 内部点
				bool	_bHit;			//!< 衝突の有無
				int		_nVtx;			//!< 使用された頂点の数(min=1, max=3)
			private:
				void _minkowskiSub(const Vec2& dir, int n);
				void _gjkMethod();
				void _setAsHit(int nv, const Vec2& inner);
				void _setAsNotHit(int nv);
			public:
				//! 初期化 = GJKによる判定(ヒットチェックだけ)
				GSimplex(const IModel& m0, const IModel& m1);
				bool getResult() const;
				//! 衝突時: 内部点を取得
				const Vec2& getInner() const;
		};

		struct MVert {
			Vec2	mpos,
					posA;
		};
		//! for onHit
		struct LmLen {
			float			dist;
			Vec2			dir;
			//! 辺に使用された頂点番号
			int				id[2];

			static LmLen AsLine(float dist, const Vec2& dir, int i0, int i1);

			bool operator < (const LmLen& len) const;
		};
		//! for onNotHit
		struct LmS {
			bool			bPoint;
			float			dist;
			Vec2			dir;
			MVert			vtx[2];

			static LmS AsPoint(float dist, const Vec2& dir, const MVert& vtx);
			static LmS AsPoint(const MVert& vtx);
			static LmS AsLine(float dist, const Vec2& dir, const MVert& v0, const MVert& v1);
			static LmS AsLine(Vec2 cp, const MVert& v0, const MVert& v1);
		};
		//! GJKで最近傍対を求める
		/*! 常に頂点リストを時計回りに保つ */
		class GEpa : public GSimplex {
			private:
				//! ミンコフスキー差の凸形状を構成する頂点
				using VList = std::vector<MVert>;
				VList			_vl;
				//! 新しく頂点メモリを確保
				MVert& _allocVert();

				//! 最短距離リスト(for onHit)
				lubee::AssocVec<LmLen>	_lmLen;
				//! 最短距離情報(for onNotHit)
				LmS						_lms;
				//! デバッグ用
				void _printLmLen(std::ostream& os) const;

				Vec2x2	_result;
				//! v0.firstとv1.firstからなる線分候補をリストに追加
				/*!	\return 最近傍点が原点と重なっていれば0x02, 線分候補が追加されれば0x01, それ以外は0x00 */
				int _addLmLen(const MVert& v0, const MVert& v1, Vec2& cp);

				//! 指定方向へのミンコフスキー差
				const MVert& _minkowskiSub(const Vec2& dir);
				void _minkowskiSub(MVert& dst, const Vec2& dir);
				//! Hit時の脱出ベクトル
				/*! 最低でも3頂点以上持っている前提 */
				void _epaMethodOnHit();
				//! NoHit時の最短ベクトル
				void _epaMethodNoHit();

				//! NoHit: 頂点が2つしか無い時の補正
				void _on2NoHits();
				//! Hit: 頂点が2つしか無い時の補正
				void _on2Hits();
				void _on3Hits();
				//! 頂点の並び順を時計回りに修正
				void _adjustClockwise();
				//! 頂点リスト(3つ以上)から最短距離リストを生成
				void _makeLmLenOnHit();
				void _makeLmLenOnNoHit();

			public:
				GEpa(const IModel& m0, const IModel& m1);
				/*! 非衝突時に有効
					\return A側の最近傍点, B側の最近傍点 */
				const Vec2x2& getNearestPair() const;
				/*! 衝突時にそれを回避するための最短移動ベクトル(A側)
					\return first=A側の最深点 second=A側の回避ベクトル */
				const Vec2x2& getPVector() const;
		};
	}
}
