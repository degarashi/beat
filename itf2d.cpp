#include "tfnode_base2d.hpp"

namespace beat {
	namespace g2 {
		void ITf::tf_setAsChanged() {
			if(const auto sp = getParent())
				sp->tf_setAsChanged();
		}
		void* ITf::_getUserData(const void*, std::true_type) const {
			if(const auto sp = getParent())
				return sp->im_getUserData();
			return nullptr;
		}
		void* ITf::_getUserData(const void* udata, std::false_type) const {
			return const_cast<void*>(udata);
		}
		MdlItr ITf::im_getInner() const {
			return MdlItr(getChild());
		}
		Pose& ITf::tf_refPose() {
			AssertF("invalid function call");
		}
		const Pose& ITf::tf_getPose() const {
			return const_cast<ITf*>(this)->tf_getPose();
		}
		Model_SP ITf::clone() const {
			return im_clone();
		}
	}
}
