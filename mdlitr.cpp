#include "tfnode_base.hpp"

namespace beat {
	namespace g2 {
		MdlItr::MdlItr(const Tf_SP& sp):
			_sp(sp)
		{}
		MdlItr& MdlItr::operator ++ () {
			_sp = _sp->getSibling();
			return *this;
		}
		bool MdlItr::operator == (const MdlItr& m) const {
			return _sp == m._sp;
		}
		bool MdlItr::operator != (const MdlItr& m) const {
			return !(operator == (m));
		}
		MdlItr::operator bool () const {
			return static_cast<bool>(_sp);
		}
		const ITf* MdlItr::get() const {
			return _sp.get();
		}
	}
}
