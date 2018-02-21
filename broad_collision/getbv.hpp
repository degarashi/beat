#pragma once
#include <memory>

namespace beat {
	template <class BV>
	struct IGetBV {
		virtual BV operator()(const void* p) const = 0;
		virtual ~IGetBV() {}
	};
	template <class BV>
	using GetBV_SP = std::shared_ptr<IGetBV<BV>>;
}
