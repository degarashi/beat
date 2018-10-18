#pragma once
#include "spine/src/rflag.hpp"
#include "lubee/src/alignedalloc.hpp"
#include "frea/src/matrix.hpp"
#include "frea/src/angle.hpp"
#include "frea/src/quaternion.hpp"
#include "spine/src/optional.hpp"

namespace beat {
	namespace g3 {
		//! 3次元姿勢クラス
		/*! 変換適用順序は[拡縮][回転][平行移動] */
		class Pose : public lubee::CheckAlign<Pose> {
			private:
				constexpr static float TURN_THRESHOLD = 1e-5f;
				using AQuat = frea::AQuat;
				using Vec3 = frea::Vec3;
				using AVec3 = frea::AVec3;
				using AMat43 = frea::Mat_t<float, 4, 3, true>;
				using Mat4 = frea::Mat4;
				using RadF = frea::RadF;
				using Ac_OP = spi::Optional<uint32_t>;
				#define SEQ \
					((Offset)(AVec3)) \
					((Rotation)(AQuat)) \
					((Scaling)(AVec3)) \
					((Accum)(Ac_OP)(Offset)(Rotation)(Scaling)) \
					((ToWorld)(AMat43)(Offset)(Rotation)(Scaling)) \
					((ToLocal)(AMat43)(ToWorld))
				RFLAG_DEFINE(Pose, SEQ)

				template <class Ar>
				friend void serialize(Ar&, Pose&);

			public:
				RFLAG_SETMETHOD_DEFINE(SEQ)
				RFLAG_GETMETHOD_DEFINE(SEQ)
				RFLAG_REFMETHOD_DEFINE(SEQ)
				RFLAG_SETMETHOD(Accum)
				#undef SEQ

				Pose() = default;
				Pose(const Pose& p) = default;
				Pose(const AVec3& pos, const AQuat& rot, const AVec3& sc);
				Pose(const AMat43& m);
				Pose(const Mat4& m);
				void identity();

				AVec3 getUp() const;
				AVec3 getRight() const;
				AVec3 getDir() const;
				void setAll(const AVec3& ofs, const AQuat& q, const AVec3& sc);

				// Rotationに変更を加える
				void addAxisRotation(const AVec3& axis, RadF radf);
				// Positionに変更を加える
				void addOffset(const AVec3& ad);

				// ---- helper function ----
				//! 前方への移動(XZ平面限定)
				void moveFwd2D(float speed);
				//! サイド移動(XZ平面限定)
				void moveSide2D(float speed);
				//! 上方移動(XZ平面限定)
				void moveUp2D(float speed);
				//! 前方への移動(軸フリー)
				void moveFwd3D(float speed);
				//! サイド移動(軸フリー)
				void moveSide3D(float speed);
				//! 上方移動(軸フリー)
				void moveUp3D(float speed);

				//! 方向転換(軸指定)
				void turnAxis(const AVec3& axis, RadF rad);
				//! Yaw Pitch Roll指定の回転
				void turnYPR(RadF yaw, RadF pitch, RadF roll);
				//! 差分入力
				void addRotation(const AQuat& q);
				//! 補間付き回転
				/*! 3軸の目標距離を合計した物が閾値以下ならtrueを返す */
				bool lerpTurn(const AQuat& q_tgt, float t, float threshold=TURN_THRESHOLD);
				//! Upベクトルをrollが0になるよう補正
				void adjustNoRoll();

				Pose& operator = (const Pose& ps) = default;
				Pose& operator = (const AMat43& m);
				// ---- compare method ----
				bool operator == (const Pose& ps) const noexcept;
				bool operator != (const Pose& ps) const noexcept;

				Pose lerp(const Pose& p1, float t) const;
				friend std::ostream& operator << (std::ostream&, const Pose&);

				// ---- Lua互換用メソッド ----
				bool equal(const Pose& p) const noexcept;
				std::string toString() const;
		};
		std::ostream& operator << (std::ostream& os, const Pose& ps);
	}
}

