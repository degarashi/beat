#include "tfnode_base.hpp"

namespace beat {
	namespace g2 {
		void ITf::_setAsChanged() {}
		void ITf::tf_setAsChanged() {
			_setAsChanged();
			if(const auto sp = getParent())
				sp->tf_setAsChanged();
		}
		void* ITf::_getUserData(void*, std::true_type) const {
			if(auto sp = getParent())
				return sp->im_getUserData();
			return nullptr;
		}
		void* ITf::_getUserData(void* udata, std::false_type) const {
			return udata;
		}
		MdlItr ITf::im_getInner() const {
			return MdlItr(getChild());
		}
		bool ITf::im_hasInner() const {
			return static_cast<bool>(getChild());
		}
		Pose& ITf::tf_refPose() {
			AssertF("invalid function call");
		}
		const Pose& ITf::tf_getPose() const {
			return const_cast<ITf*>(this)->tf_getPose();
		}
	}
}
