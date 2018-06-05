#include "pose3d.hpp"
#include "frea/affine_parts.hpp"

namespace beat {
	namespace g3 {
		bool Pose::_refresh(Accum::value_t& dst, Accum*) const {
			getOffset();
			getRotation();
			getScaling();
			if(dst)
				++(*dst);
			else
				dst = 0;
			return true;
		}
		bool Pose::_refresh(ToWorld::value_t& dst, ToWorld*) const {
			using frea::AMat4;
			auto& sc = getScaling();
			AMat4 m = AMat4::Scaling({sc.x, sc.y, sc.z, 1});
			m *= getRotation().asMat44();
			m.setRow<3>(getOffset().convertI<4,3>(1));
			dst = m.convert<4,3>();
			return true;
		}
		bool Pose::_refresh(ToLocal::value_t& dst, ToLocal*) const {
			dst = (getToWorld().convertI<4,4>(1).inversion()).convert<4,3>();
			return true;
		}
		Pose::Pose(const AVec3& pos, const AQuat& rot, const AVec3& sc) {
			setAll(pos, rot, sc);
		}
		Pose::Pose(const Mat4& m):
			Pose(frea::AMat4(m).convert<4,3>())
		{}
		Pose::Pose(const AMat43& m) {
			const auto ap = frea::AffineParts<float>::Decomp(m);
			setAll(ap.offset, ap.rotation, ap.scale);
		}
		void Pose::identity() {
			setAll(AVec3(0), AQuat::Identity(), AVec3(1));
		}
		frea::AVec3 Pose::getUp() const {
			return getRotation().getUp();
		}
		frea::AVec3 Pose::getRight() const {
			return getRotation().getRight();
		}
		frea::AVec3 Pose::getDir() const {
			return getRotation().getDir();
		}
		void Pose::setAll(const AVec3& ofs, const AQuat& q, const AVec3& sc) {
			setOffset(ofs);
			setRotation(q);
			setScaling(sc);
			setAccum(spi::none);
		}
		void Pose::addAxisRotation(const AVec3& axis, const RadF radf) {
			setRotation(getRotation().rotation(axis, radf));
		}
		void Pose::addOffset(const AVec3& ad) {
			const AVec3& ofs = getOffset();
			setOffset(ad + ofs);
		}
		void Pose::moveFwd2D(const float speed) {
			Vec3 vZ = getDir();
			vZ.y = 0;
			vZ.normalize();
			addOffset(vZ * speed);
		}
		void Pose::moveSide2D(const float speed) {
			Vec3 vX = getRight();
			vX.y = 0;
			vX.normalize();
			addOffset(vX * speed);
		}
		void Pose::moveUp2D(const float speed) {
			addOffset(Vec3(0,speed,0));
		}
		void Pose::moveFwd3D(const float speed) {
			addOffset(getDir() * speed);
		}
		void Pose::moveSide3D(const float speed) {
			addOffset(getRight() * speed);
		}
		void Pose::moveUp3D(const float speed) {
			addOffset(getUp() * speed);
		}
		void Pose::turnAxis(const AVec3& axis, const RadF ang) {
			auto q = getRotation();
			q.rotate(axis, ang);
			setRotation(q);
		}
		void Pose::turnYPR(const RadF yaw, const RadF pitch, const RadF roll) {
			auto q = getRotation();
			q *= AQuat::RotationYPR(yaw, pitch, roll);
			setRotation(q);
		}
		void Pose::addRotation(const AQuat& q) {
			auto q0 = getRotation();
			q0 *= q;
			setRotation(q0);
		}
		bool Pose::lerpTurn(const AQuat& q_tgt, const float t, const float threshold) {
			auto& q = refRotation();
			q.slerp(q_tgt, t);
			return q.distance(q_tgt) < threshold;
		}
		void Pose::adjustNoRoll() {
			// X軸のY値が0になればいい

			// 回転を一旦行列に直して軸を再計算
			const auto& q = getRotation();
			auto rm = q.asMat33();
			// Zはそのままに，X軸のY値を0にしてY軸を復元
			AVec3 zA = rm.getRow<2>(),
				xA = rm.getRow<0>();
			xA.y = 0;
			if(xA.len_sq() < 1e-5f) {
				// Xが真上か真下を向いている
				frea::DegF ang;
				if(rm.m[0][1] > 0) {
					// 真上 = Z軸周りに右へ90度回転
					ang = frea::DegF(90);
				} else {
					// 真下 = 左へ90度
					ang = frea::DegF(-90);
				}
				setRotation(AQuat::RotationZ(ang) * q);
			} else {
				xA.normalize();
				AVec3 yA = zA % xA;
				setRotation(AQuat::FromAxis(xA, yA, zA));
			}
		}
		Pose& Pose::operator = (const AMat43& m) {
			return *this = Pose(m);
		}
		bool Pose::operator == (const Pose& ps) const noexcept {
			return getOffset() == ps.getOffset() &&
					getRotation() == ps.getRotation() &&
					getScaling() == ps.getScaling();
		}
		bool Pose::operator != (const Pose& ps) const noexcept {
			return !(this->operator == (ps));
		}
		Pose Pose::lerp(const Pose& p1, const float t) const {
			return {
				getOffset().l_intp(p1.getOffset(), t),
				getRotation().slerp(p1.getRotation(), t),
				getScaling().l_intp(p1.getScaling(), t)
			};
		}
		bool Pose::equal(const Pose& p) const noexcept {
			return *this == p;
		}
		std::string Pose::toString() const {
			return lubee::ToString(*this);
		}
		std::ostream& operator << (std::ostream& os, const Pose& ps) {
			return os << "Pose [ offset:" << ps.getOffset()
					<< ", rotation: " << ps.getRotation()
					<< ", scale: " << ps.getScaling() << ']';
		}
	}
}
