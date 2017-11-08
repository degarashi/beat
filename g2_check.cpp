#include "g2.hpp"
#include "tfnode_base2d.hpp"
#include "lubee/random.hpp"
#include "frea/random/vector.hpp"

namespace beat {
	namespace g2 {
		void CheckBoundary(const IModel* a, const IModel* b) {
			auto mt = lubee::RandomMT::Make<4>();
			auto rf = mt.getUniformF<float>();
			constexpr int N = 512;
			for(int i=0 ; i<N ; i++) {
				const auto dir = frea::random::GenVecUnit<Vec2>(rf);
				const auto d0 = dir.dot(a->im_support(dir)),
							d1 = dir.dot(b->im_support(dir));
				Assert0(d0 >= d1);
			}
		}
		void CheckBoundaryTree(const IModel* node, const Time_t t) {
			// 子系ノードをリストアップ
			std::vector<const IModel*> pm;
			auto in = node->im_getInner();
			while(in) {
				auto* c = in.get();
				if(c->im_refresh(t)) {
					CheckBoundary(node, c);
					CheckBoundaryTree(c, t);
				}
				++in;
			}
		}
	}
}
