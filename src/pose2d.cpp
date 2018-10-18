#include "pose2d.hpp"
#include "lubee/src/tostring.hpp"
#include "frea/src/angle_func.hpp"

namespace beat {
	namespace g2 {
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
			auto tmp = frea::AMat3::Scaling(getScaling().convert<3>());
			tmp *= frea::AMat2::Rotation(getRotation()).convertI<3,3>(1);
			tmp.setRow<2>(getOffset().convertI<3,2>(1));
			dst = tmp.convert<3,2>();
			return true;
		}
		bool Pose::_refresh(ToLocal::value_t& dst, ToLocal*) const {
			dst = (getToWorld().convertI<3,3>(1).inversion()).convert<3,2>();
			return true;
		}
		Pose::Pose(const Vec2& pos, const RadF ang, const Vec2& sc) {
			setAll(pos, ang, sc);
		}
		void Pose::identity() {
			setAll(Vec2(0), RadF(0), Vec2(1));
		}
		void Pose::setAll(const Vec2& ofs, const RadF ang, const Vec2& sc) {
			setOffset(ofs);
			setRotation(ang);
			setScaling(sc);
			setAccum(spi::none);
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
			return {
				getOffset().l_intp(p1.getOffset(), t),
				(p1.getRotation() - getRotation()) * t + getRotation(),
				getScaling().l_intp(p1.getScaling(), t)
			};
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
