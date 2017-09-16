#include "imodel2d.hpp"
#include "tfnode_base.hpp"

namespace beat {
	namespace g2 {
		MdlItr IModel::im_getInner() const {
			return MdlItr();
		}
		bool IModel::im_hasInner() const {
			return false;
		}

		const AMat32 IModel::cs_idMat = AMat32::Identity();
		const AMat32& IModel::im_getToLocalI() const {
			const auto op = im_getToLocal();
			if(op)
				return *op;
			return cs_idMat;
		}
		const AMat32& IModel::im_getToWorldI() const {
			const auto op = im_getToWorld();
			if(op)
				return *op;
			return cs_idMat;
		}
		std::ostream& operator << (std::ostream& os, const IModel& mdl) {
			return mdl.im_print(os);
		}
	}
}
