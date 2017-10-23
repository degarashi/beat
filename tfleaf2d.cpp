#include "tfleaf2d.hpp"

namespace beat {
	namespace g2 {
		TfLeaf_base::TfLeaf_base(const Model_SP& m) {
			setModel(m);
			refPose().identity();
		}
		Vec2 TfLeaf_base::im_getCenter() const {
			return getCenter();
		}
		float TfLeaf_base::im_getArea() const {
			return getArea();
		}
		float TfLeaf_base::im_getInertia() const {
			return getInertia();
		}
		void TfLeaf_base::im_getBVolume(Circle& c) const {
			c = getBCircle();
		}
		void TfLeaf_base::im_getBVolume(AABB& a) const {
			a = getBBox();
		}
		void* TfLeaf_base::im_getCore() {
			return refModel()->im_getCore();
		}
		const void* TfLeaf_base::im_getCore() const {
			return getModel()->im_getCore();
		}
		Vec2 TfLeaf_base::im_toLocal(const Vec2& v) const {
			return v.convertI<3,2>(1) * getLocal();
		}
		Vec2 TfLeaf_base::im_toLocalDir(const Vec2& v) const {
			return (v.convert<3>() * getLocal()).normalization();
		}
		Vec2 TfLeaf_base::im_toWorld(const Vec2& v) const {
			return v.convertI<3,2>(1) * getGlobal();
		}
		Vec2 TfLeaf_base::im_toWorldDir(const Vec2& v) const {
			return (v.convert<3>() * getGlobal()).normalization();
		}
		const AMat32& TfLeaf_base::im_getToLocal() const {
			return getLocal();
		}
		const AMat32& TfLeaf_base::im_getToWorld() const {
			return getGlobal();
		}
		Vec2 TfLeaf_base::im_support(const Vec2& dir) const {
			auto& sc = getPose().getScaling();
			D_Assert0(sc.x == sc.y);
			return im_toWorld(getModel()->im_support(im_toLocalDir(dir)));
		}
		uint32_t TfLeaf_base::im_getCID() const {
			return getModel()->im_getCID();
		}
		void TfLeaf_base::im_transform(void* dst, const AMat32& m) const {
			auto m2 = getPose().getToWorld().convertI<3,3>(1) * m;
			getModel()->im_transform(dst, m2);
		}
		bool TfLeaf_base::im_hit(const Vec2& p) const {
			return getModel()->im_hit(im_toLocal(p));
		}
		void TfLeaf_base::tf_setAsChanged() {
			refModel();
			ITf::tf_setAsChanged();
		}
		TfLeaf_base::Pose_t& TfLeaf_base::tf_refPose() {
			return refPose();
		}
		const TfLeaf_base::Pose_t& TfLeaf_base::tf_getPose() const {
			return getPose();
		}
		std::ostream& TfLeaf_base::im_print(std::ostream& os) const {
			return getModel()->im_print(os);
		}
		std::ostream& operator << (std::ostream& os, const TfLeaf_base& node) {
			return os << "TfLeaf2D [ pose: " << node.getPose()
						<< ", node accum: " << node.getAccum() << ']';
		}

		bool TfLeaf_base::_refresh(Global::value_t& m, Global*) const {
			const auto ret = _rflag.getWithCheck(this, m);
			if(ret.flag) {
				auto& ps = *std::get<0>(ret);
				auto& sc = ps.getScaling();
				D_Assert0(sc.x == sc.y);
				m = ps.getToWorld().convert<3,2>();
				return true;
			}
			return false;
		}
		bool TfLeaf_base::_refresh(Local::value_t& m, Local*) const {
			frea::AMat3 tm = getGlobal().convertI<3,3>(1).inversion();
			m = tm.convert<3,2>();
			return true;
		}
		bool TfLeaf_base::_refresh(Determinant::value_t& d, Determinant*) const {
			d = getGlobal().convert<2,2>().calcDeterminant();
			return true;
		}
		bool TfLeaf_base::_refresh(Inertia::value_t& f, Inertia*) const {
			f = getModel()->im_getInertia() * getDeterminant();
			return true;
		}
		bool TfLeaf_base::_refresh(Area::value_t& f, Area*) const {
			f = getModel()->im_getArea() * getDeterminant();
			return true;
		}
		bool TfLeaf_base::_refresh(Center::value_t& v, Center*) const {
			v = im_toWorld(getModel()->im_getCenter());
			return true;
		}
		bool TfLeaf_base::_refresh(BCircle::value_t& c, BCircle*) const {
			getModel()->im_getBVolume(c);
			c = c * getPose().getToWorld().convert<3,2>();
			return true;
		}
		bool TfLeaf_base::_refresh(BBox::value_t& a, BBox*) const {
			const auto& mdl = *getModel();
			a.max.x = im_toWorld(mdl.im_support(im_toLocalDir({1,0}))).x;
			a.max.y = im_toWorld(mdl.im_support(im_toLocalDir({0,1}))).y;
			a.min.x = im_toWorld(mdl.im_support(im_toLocalDir({-1,0}))).x;
			a.min.y = im_toWorld(mdl.im_support(im_toLocalDir({0,-1}))).y;
			return true;
		}
		bool TfLeaf_base::_refresh(Accum::value_t& a, Accum*) const {
			if(a)
				++*a;
			else
				a = 0;
			getModel();
			getPose();
			return true;
		}
		bool TfLeaf_base::im_isLeaf() const {
			return true;
		}
	}
}
