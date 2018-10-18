#pragma once
#include "spine/src/rflag.hpp"
#include "lubee/src/alignedalloc.hpp"
#include "frea/src/matrix.hpp"
#include "frea/src/angle.hpp"
#include "spine/src/optional.hpp"

namespace beat {
	namespace g2 {
		//! 2次元姿勢クラス
		/*! 変換適用順序は[拡縮][回転][平行移動] */
		class Pose : public lubee::CheckAlign<Pose> {
			private:
				using Vec2 = frea::Vec2;
				using AVec2 = frea::AVec2;
				using AMat32 = frea::Mat_t<float, 3, 2, true>;
				using RadF = frea::RadF;
				using Ac_OP = spi::Optional<uint32_t>;
				#define SEQ \
					((Offset)(AVec2)) \
					((Rotation)(RadF)) \
					((Scaling)(AVec2)) \
					((Accum)(Ac_OP)(Offset)(Rotation)(Scaling)) \
					((ToWorld)(AMat32)(Offset)(Rotation)(Scaling)) \
					((ToLocal)(AMat32)(ToWorld))
				RFLAG_DEFINE(Pose, SEQ)

				template <class Ar>
				friend void serialize(Ar&, Pose&);

			public:
				Pose() = default;
				Pose(const Vec2& pos, RadF ang, const Vec2& sc);
				void identity();

				RFLAG_SETMETHOD_DEFINE(SEQ)
				RFLAG_GETMETHOD_DEFINE(SEQ)
				RFLAG_REFMETHOD_DEFINE(SEQ)
				RFLAG_SETMETHOD(Accum)
				#undef SEQ

				void setAll(const Vec2& ofs, RadF ang, const Vec2& sc);
				// ---- helper function ----
				void moveUp(float speed);
				void moveDown(float speed);
				void moveLeft(float speed);
				void moveRight(float speed);
				Vec2 getUp() const;
				Vec2 getRight() const;
				void setUp(const Vec2& up);

				Pose lerp(const Pose& p1, float t) const;
				Pose& operator = (const Pose& ps) = default;
				bool operator == (const Pose& ps) const noexcept;
				bool operator != (const Pose& ps) const noexcept;
				friend std::ostream& operator << (std::ostream&, const Pose&);

				// ---- Lua互換用メソッド ----
				bool equal(const Pose& ps) const noexcept;
				std::string toString() const;
		};
		std::ostream& operator << (std::ostream& os, const Pose& ps);
	}
}
