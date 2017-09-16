#include "pose2d.hpp"
#include "lubee/tostring.hpp"
#include "frea/angle_func.hpp"

namespace beat {
	namespace g2 {
		bool Pose::_refresh(Accum::value_t& dst, Accum*) const {
			getOffset();
			getRotation();
			getScaling();
			++dst;
			return true;
		}
		bool Pose::_refresh(ToWorld::value_t& dst, ToWorld*) const {
			dst = AMat3::Scaling(getScaling().convert<3>());
			dst *= frea::AMat2::Rotation(getRotation()).convertI<3,3,2>(1);
			dst.setRow<2>(getOffset().convertI<3,2>(1));
			return true;
		}
		bool Pose::_refresh(ToLocal::value_t& dst, ToLocal*) const {
			dst = getToWorld().inversion();
			return true;
		}
		Pose::Pose(const Vec2& pos, const RadF ang, const Vec2& sc) {
			setAll(pos, ang, sc);
			setAccum(std::rand());
		}
		void Pose::identity() {
			setAll(Vec2(0), RadF(0), Vec2(1));
		}
		void Pose::setAll(const Vec2& ofs, const RadF ang, const Vec2& sc) {
			setOffset(ofs);
			setRotation(ang);
			setScaling(sc);
		}
		void Pose::moveUp(const float speed) {
			refOffset() += getUp() * speed;
		}
		void Pose::moveDown(const float speed) {
			moveUp(-speed);
		}
		void Pose::moveRight(const float speed) {
			refOffset() += getRight() * speed;
		}
		void Pose::moveLeft(const float speed) {
			moveRight(-speed);
		}
		frea::Vec2 Pose::getUp() const {
			const float rd = (getRotation() + frea::DegF(90)).get();
			return {std::cos(rd), std::sin(rd)};
		}
		frea::Vec2 Pose::getRight() const {
			const float rd = getRotation().get();
			return {std::cos(rd), std::sin(rd)};
		}
		void Pose::setUp(const Vec2& up) {
			setRotation(AngleValue(up));
		}
		Pose Pose::lerp(const Pose& p1, const float t) const {
			Pose ret;
			ret.setOffset(getOffset().l_intp(p1.getOffset(), t));
			ret.setRotation((p1.getRotation() - getRotation()) * t + getRotation());
			ret.setScaling(getScaling().l_intp(p1.getScaling(), t));
			ret.setAccum(getAccum()-1);
			return ret;
		}
		bool Pose::operator == (const Pose& ps) const noexcept {
			return getOffset() == ps.getOffset() &&
					getRotation() == ps.getRotation() &&
					getScaling() == ps.getScaling();
		}
		bool Pose::operator != (const Pose& ps) const noexcept {
			return !(this->operator == (ps));
		}
		bool Pose::equal(const Pose& p) const noexcept {
			return *this == p;
		}
		std::string Pose::toString() const {
			return lubee::ToString(*this);
		}
		std::ostream& operator << (std::ostream& os, const Pose& ps) {
			return os << "Pose [ offset: " << ps.getOffset()
					<< ", angle: " << ps.getRotation()
					<< ", scale: " << ps.getScaling() << ']';
		}
	}
}
