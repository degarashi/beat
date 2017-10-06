#pragma once
#include "tfnode_base2d.hpp"

namespace beat {
	namespace g2 {
		//! フレーム経過で内部構造が変化しないノード(ユーザーが自分でフラグをセットする)
		template <class Boundary, class Ud=spi::none_t>
		class TfNode_Static : public TfNode_base<Boundary, Ud> {
			private:
				using base_t = TfNode_base<Boundary, Ud>;
				using SP = typename base_t::SP;
				mutable bool	_bChanged = true,
								_bValid;
			protected:
				static void OnChildAdded(typename base_t::pointer* self, const SP& /*node*/) {
					static_cast<TfNode_Static*>(self)->_setAsChanged();
				}
				static void OnChildRemove(typename base_t::pointer* self, const SP& /*node*/) {
					static_cast<TfNode_Static*>(self)->_setAsChanged();
				}
			public:
				void tf_setAsChanged() override {
					_bChanged = true;
					base_t::tf_setAsChanged();
				}
				Model_SP im_clone() const override {
					return std::make_shared<TfNode_Static>(*this);
				}
				bool im_refresh(Time_t t) const override {
					// 更新条件はフラグ
					if(_bChanged) {
						_bChanged = false;
						std::vector<const IModel*> pm;
						// 直下の子ノードをリストアップ
						this->template iterateDepthFirst<false>([&pm](auto& node, int depth){
							if(depth == 0)
								return spi::Iterate::StepIn;
							pm.push_back(&node);
							return spi::Iterate::Next;
						});
						if((_bValid = !pm.empty())) {
							// 境界ボリュームの更新
							auto* core = const_cast<Boundary*>(reinterpret_cast<const Boundary*>(base_t::im_getCore()));
							auto op = MakeBoundary<Boundary>(&pm[0], pm.size(), t);
							if((_bValid = static_cast<bool>(op)))
								*core = *op;
						}
						// 子ノードが無い場合は常にヒットしない
					}
					return _bValid;
				}
		};
	}
}
