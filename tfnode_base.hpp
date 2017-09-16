#pragma once
#include "imodel2d.hpp"

namespace beat {
	namespace g2 {
		class ITf;
		using Tf_SP = std::shared_ptr<ITf>;
		//! ITfの子ノードイテレータ
		struct MdlItr {
			Tf_SP	_sp;

			MdlItr() = default;
			MdlItr(const Tf_SP& sp);

			MdlItr& operator ++ ();
			bool operator == (const MdlItr& m) const;
			bool operator != (const MdlItr& m) const;
			explicit operator bool () const;
			const ITf* get() const;
		};

		class Pose;
		class ITf : public spi::TreeNode<ITf>,
					virtual public IModel
		{
			private:
				friend class spi::TreeNode<ITf>;

			protected:
				void* _getUserData(void*, std::true_type) const;
				void* _getUserData(void* udata, std::false_type) const;
				virtual void _setAsChanged();

			public:
				//! 子ノードの取得
				MdlItr im_getInner() const override;
				bool im_hasInner() const override;
				//! 下層オブジェクトの形状を変更した時に手動で呼ぶ
				virtual void tf_setAsChanged();
				virtual Pose& tf_refPose[[noreturn]]();
				virtual const Pose& tf_getPose() const;
		};
		template <class Boundary, class Ud>
		class TfNode_base : public ITf,
							public Model<Boundary>
		{
			protected:
				using model_t = Model<Boundary>;
				Ud					_udata;
			public:
				using model_t::model_t;
				/*! ユーザーデータがvoidの時は親ノードのデータを返す */
				void* im_getUserData() const override {
					return _getUserData(&_udata, std::is_same<spi::none_t, Ud>());
				}
		};
	}
}
