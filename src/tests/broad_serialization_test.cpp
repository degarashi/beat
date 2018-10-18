#include "broad_test.hpp"
#include "../serialization/collision_mgr.hpp"
#include "model_serialization.hpp"

namespace beat {
	namespace g2 {
		TYPED_TEST(BroadCollision, Serialization) {
			const auto sc = this->makeRandomScene();
			lubee::CheckSerialization(sc.cm);
		}
	}
}
