#pragma once
#include "lubee/alignedalloc.hpp"
#include "pose2d.hpp"
#include "spine/rflag.hpp"
#include "tfnode_base2d.hpp"
#include "circle2d.hpp"
#include "aabb2d.hpp"

namespace beat {
	namespace g2 {
		//! 座標変換ありのモデル基底
		/*! スケーリングはX,Yとも同じ比率のみ許可 */
		class TfLeaf_base : public lubee::CheckAlign<TfLeaf_base>,
						public ITf
		{
			private:
				using Pose_t = ::beat::g2::Pose;
				struct Pose;
				struct Getter : spi::RFlag_Getter<uint32_t> {
					using RFlag_Getter::operator ();
					counter_t operator()(const Pose_t& p, Pose*, const TfLeaf_base&) const {
						return p.getAccum();
					}
				};
				using Global_t = spi::AcCheck<AMat32, Getter>;
				#define SEQ \
					((Pose)(Pose_t)) \
					((Global)(Global_t)(Pose)) \
					((Local)(AMat32)(Global)) \
					((Determinant)(float)(Global)) \
					((Model)(Model_SP)) \
					((Inertia)(float)(Model)(Determinant)) \
					((Area)(float)(Model)(Determinant)) \
					((Center)(Vec2)(Model)(Global)) \
					((BCircle)(Circle)(Model)(Pose)) \
					((BBox)(AABB)(Model)(Pose)) \
					((Accum)(uint32_t)(Model)(Pose))
				RFLAG_DEFINE(TfLeaf_base, SEQ)

				template <class Ar>
				friend void serialize(Ar&, TfLeaf_base&);
			protected:
				TfLeaf_base() = default;

			public:
				RFLAG_GETMETHOD_DEFINE(SEQ)
				RFLAG_SETMETHOD_DEFINE(SEQ)
				RFLAG_REFMETHOD_DEFINE(SEQ)
				#undef SEQ

				TfLeaf_base(const Model_SP& m);
				Vec2 im_getCenter() const override;
				float im_getArea() const override;
				float im_getInertia() const override;
				void im_getBVolume(Circle& c) const override;
				void im_getBVolume(AABB& a) const override;
				void* im_getCore() override;
				const void* im_getCore() const override;
				Vec2 im_toLocal(const Vec2& v) const override;
				Vec2 im_toLocalDir(const Vec2& v) const override;
				Vec2 im_toWorld(const Vec2& v) const override;
				Vec2 im_toWorldDir(const Vec2& v) const override;
				const AMat32& im_getToLocal() const override;
				const AMat32& im_getToWorld() const override;
				Vec2 im_support(const Vec2& dir) const override;
				uint32_t im_getCID() const override;
				void im_transform(void* dst, const AMat32& m) const override;
				bool im_hit(const Vec2& p) const override;

				void tf_setAsChanged() override;
				Pose_t& tf_refPose() override;
				const Pose_t& tf_getPose() const override;

				bool im_isLeaf() const override;

				std::ostream& im_print(std::ostream& os) const override;
				friend std::ostream& operator << (std::ostream&, const TfLeaf_base&);
		};
		//! 形状キャッシュ無しのTfLeaf
		template <class Ud=spi::none_t>
		class TfLeaf : public TfLeaf_base {
			private:
				using base_t = TfLeaf_base ;
				Ud	_udata;
				template <class Ar, class UD>
				friend void serialize(Ar&, TfLeaf<UD>&);

			public:
				using base_t::base_t;
				/*! ユーザーデータがvoidの時は親ノードのデータを返す */
				void* im_getUserData() const override {
					return _getUserData(&_udata, std::is_same<spi::none_t, Ud>());
				}
				bool im_canCacheShape() const override {
					return false;
				}
				Model_SP im_clone() const override {
					return std::make_shared<TfLeaf>(*this);
				}
		};
	}
}
