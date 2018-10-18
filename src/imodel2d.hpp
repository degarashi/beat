#pragma once
#include "imodel.hpp"
#include "frea/src/matrix.hpp"
#include "spine/src/optional.hpp"
#include "spine/src/treenode.hpp"
#include "boundary.hpp"

namespace beat {
	template <class Types>
	class Narrow;

	namespace g2 {
		struct Point;
		struct Line;
		struct Ray;
		struct Segment;
		struct Circle;
		struct Capsule;
		class AABB;
		struct Triangle;
		class Convex;
		using CTGeo = lubee::Types<Convex, Triangle, AABB, Capsule, Circle, Segment, Ray, Line, Point>;

		template <class T>
		using CID = CID_base<T, CTGeo>;

		using frea::Vec2;
		using AMat32 = frea::Mat_t<float, 3,2, true>;

		struct MdlItr;
		struct IModel;
		using Model_SP = std::shared_ptr<IModel>;
		struct IModel {
			virtual Vec2 im_getCenter() const = 0;
			virtual float im_getArea() const = 0;
			virtual float im_getInertia() const = 0;
			virtual void im_getBVolume(Circle& c) const = 0;
			virtual void im_getBVolume(AABB& a) const = 0;
			virtual void* im_getCore() = 0;
			virtual bool im_isLeaf() const = 0;
			virtual const void* im_getCore() const = 0;
			virtual bool im_refresh(Time_t t) const;
			virtual bool im_canCacheShape() const = 0;
			virtual void* im_getUserData() const = 0;
			//! サポート射像
			/*! 均等でないスケーリングは対応しない、移動は後でオフセット、回転はdirを逆にすれば代用可
				・・との理由で行列変換後の物体に対する射像は無し */
			virtual Vec2 im_support(const Vec2& dir) const = 0;
			//! 図形と点の判定
			virtual bool im_hit(const Vec2& p) const = 0;
			virtual uint32_t im_getCID() const = 0;
			virtual Model_SP im_clone() const = 0;
			virtual Vec2 im_toLocal(const Vec2& v) const;
			virtual Vec2 im_toLocalDir(const Vec2& v) const;
			virtual Vec2 im_toWorld(const Vec2& v) const;
			virtual Vec2 im_toWorldDir(const Vec2& v) const;
			virtual const AMat32& im_getToLocal() const;
			virtual const AMat32& im_getToWorld() const;
			const static AMat32 cs_idMat;
			virtual MdlItr im_getInner() const;
			virtual void im_selfCheck(Time_t t) const;
			bool im_hasInner() const;

			virtual void im_transform(void* dst, const AMat32& m) const = 0;
			virtual std::ostream& im_print(std::ostream& os) const = 0;
			virtual ~IModel() {}
		};

		//! TにIModelインタフェースを付加
		template <class T>
		struct Model : Model_base<T, IModel> {
			using base_t = Model_base<T, IModel>;
			using base_t::base_t;

			Model() = default;
			Model(const T& t):
				base_t(t)
			{}
			Model(T&& t):
				base_t(std::move(t))
			{}
			// ------- 各種ルーチンの中継 -------
			Model_SP im_clone() const override {
				return std::make_shared<Model<T>>(static_cast<const T&>(*this));
			}
			void im_transform(void* dst, const AMat32& m) const override {
				*reinterpret_cast<T*>(dst) = *this * m;
			}
			void im_getBVolume(Circle& c) const override {
				c = T::bs_getBCircle();
			}
			void im_getBVolume(AABB& a) const override {
				a = T::bs_getBBox();
			}
			float im_getInertia() const override {
				return T::bs_getInertia();
			}
			float im_getArea() const override {
				return T::bs_getArea();
			}
			Vec2 im_getCenter() const override {
				return T::bs_getCenter();
			}
			Vec2 im_support(const Vec2& dir) const override {
				return T::support(dir);
			}
			bool im_hit(const Vec2& pos) const override {
				return T::hit(pos);
			}
		};

		class ITf;
		class GSimplex;
		class TfLeaf_base;
		struct Types {
			using CTGeo = CTGeo;
			using IModel = IModel;
			using ITf = ITf;
			using TfLeaf_base = TfLeaf_base;
			using GJK = GSimplex;
			using Narrow = ::beat::Narrow<Types>;
		};
	}
}
