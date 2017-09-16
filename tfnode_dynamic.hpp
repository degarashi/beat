#pragma once
#include "tfnode_base.hpp"

namespace beat {
	namespace g2 {
		template <class Boundary, class Ud=spi::none_t>
		class TfNode_Dynamic : public TfNode_base<Boundary, Ud> {
			private:
				using base_t = TfNode_base<Boundary, Ud>;
				using SP = typename base_t::SP;
				using Time_OP = spi::Optional<Time_t>;
				mutable Time_OP		_opTime = 0;
				mutable bool		_bValid;
			protected:
				static void OnChildAdded(typename base_t::pointer* self, const SP& /*node*/) {
					static_cast<TfNode_Dynamic*>(self)->_opTime = spi::none;
				}
				static void OnChildRemove(typename base_t::pointer* self, const SP& /*node*/) {
					static_cast<TfNode_Dynamic*>(self)->_opTime = spi::none;
				}
			public:
				SP clone() const override {
					return std::make_shared<TfNode_Dynamic>(*this);
				}
				bool imn_refresh(Time_t t) const override {
					// 更新時刻が古い時のみ処理を行う
					if(!_opTime || *_opTime < t) {
						_opTime = t;

						std::vector<const IModel*> pm;
						// 子ノードをリストアップ
						this->template iterateDepthFirst<false>([&pm](auto& node, int depth){
							if(depth == 0)
								return spi::Iterate::StepIn;
							pm.push_back(&node);
							return spi::Iterate::Next;
						});
						if((_bValid = !pm.empty())) {
							// 境界ボリュームの更新
							auto* core = const_cast<Boundary*>(reinterpret_cast<const Boundary*>(base_t::getCore()));
							auto op = MakeBoundary<Boundary>(&pm[0], pm.size(), t);
							if((_bValid = op))
								*core = *op;
						}
						// 子ノードが無い場合は常にヒットしない
					}
					return _bValid;
				}
		};
	}
}
