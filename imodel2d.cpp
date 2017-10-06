#include "imodel2d.hpp"
#include "tfnode_base2d.hpp"

namespace beat {
	namespace g2 {
		Vec2 IModel::im_toLocal(const Vec2& v) const {
			return v;
		}
		Vec2 IModel::im_toLocalDir(const Vec2& v) const {
			return v;
		}
		Vec2 IModel::im_toWorld(const Vec2& v) const {
			return v;
		}
		Vec2 IModel::im_toWorldDir(const Vec2& v) const {
			return v;
		}
		const AMat32& IModel::im_getToLocal() const {
			return cs_idMat;
		}
		const AMat32& IModel::im_getToWorld() const {
			return cs_idMat;
		}
		bool IModel::im_refresh(Time_t /*t*/) const {
			return true;
		}
		MdlItr IModel::im_getInner() const {
			return MdlItr();
		}
		bool IModel::im_hasInner() const {
			return static_cast<bool>(im_getInner());
		}

		const AMat32 IModel::cs_idMat = AMat32::Identity();
		std::ostream& operator << (std::ostream& os, const IModel& mdl) {
			return mdl.im_print(os);
		}
	}
}
