#pragma once
#include "spine/resmgr.hpp"
#include "spine/optional.hpp"
#include "broad_collision/getbv.hpp"
#include <vector>

namespace cereal {
	template <class T>
	class construct;
	class access;
}
namespace beat {
	using CMask = uint32_t;
	using Time_t = uint64_t;
	using Int_OP = spi::Optional<int>;

	struct ColMem_Index {
		Time_t	time;	//!< indexが指している累積時間
		Int_OP	front,	//!< 線形リスト先頭インデックス
				last;	//!< 線形リスト末尾インデックス

		ColMem_Index() = default;
		//! リストに何もつないでない状態に初期化
		ColMem_Index(const Time_t tm):
			time(tm)
		{}
		//! リスト末尾に新しくインデックスを加える
		/*!	\param[in] idx 新しく加えるインデックス値
			\return 直前の先頭インデックス値 */
		Int_OP setLastIndex(const int idx) {
			const Int_OP ret = last;
			if(!front)
				front = last = idx;
			else
				last = idx;
			return ret;
		}

		bool operator == (const ColMem_Index& idx) const noexcept {
			return time == idx.time &&
					front == idx.front &&
					last == idx.last;
		}
	};
	template <class BCId, class Hist>
	struct IColMgr {
		using CBHist = std::function<void (const Hist&)>;
		virtual Time_t getTime() const noexcept = 0;
		virtual void iterateHistCur(int idx, const CBHist& cb) const = 0;
		virtual void iterateHistPrev(int idx, const CBHist& cb) const = 0;
		virtual void deleteBCID(BCId id) = 0;
	};
	// 単方向リスト要素
	template <class HC>
	struct ColMgr_Hist {
		HC		hCol;
		int		nFrame;				//!< 衝突継続したフレーム数
		int		nextOffset;			//!< 次のエントリへのバイトオフセット

		ColMgr_Hist() = default;
		ColMgr_Hist(const HC& hc, const int nf):
			hCol(hc),
			nFrame(nf),
			nextOffset(0)
		{}

		//! デバッグ用
		bool operator == (const ColMgr_Hist& h) const noexcept {
			// hColは比較しない
			return nFrame == h.nFrame &&
					nextOffset == h.nextOffset;
		}
	};
	/*! コリジョン情報を纏めた構造体
		\tparam MDL		モデルインタフェース(shared_ptr)
		\tparam BCID	BroadCollisionクラスのオブジェクトを特定できるようなデータ型
		\tparam UD		任意のユーザーデータ型
	*/
	template <class MDL, class BCID, class UD>
	class ColMem : public std::enable_shared_from_this<ColMem<MDL,BCID,UD>> {
		private:
			using ICM = IColMgr<BCID, ColMgr_Hist<std::shared_ptr<ColMem>>>;
			ICM*		_cmgr;
			//! 当たり判定対象フラグ
			CMask		_mask;
			using Index_OP = spi::Optional<ColMem_Index>;
			Index_OP	_cur,
						_prev;
			MDL			_spMdl;
			BCID		_bcid;
			UD			_udata;

			template <class Ar, class M, class B, class U>
			friend void serialize(Ar&, ColMem<M,B,U>&);
		public:
			//! デバッグ用
			bool operator == (const ColMem& c) const noexcept {
				return _mask == c._mask &&
						_cur == c._cur &&
						_prev == c._prev &&
						_udata == c._udata;
			}
			ColMem() = default;
			template <class UD2>
			ColMem(const CMask ms, const MDL& mdl, UD2&& ud):
				_mask(ms),
				_spMdl(mdl),
				_udata(std::forward<UD2>(ud))
			{}
			~ColMem() {
				_cmgr->deleteBCID(_bcid);
			}
			void setBCID(ICM* cm, const BCID& ns) {
				_cmgr = cm;
				_bcid = ns;
			}
			template <class BV>
			BV getBVolume(const Time_t t) const {
				BV bv;
				D_Assert0(_spMdl->im_refresh(t));
				_spMdl->im_getBVolume(bv);
				return bv;
			}
			const MDL& getModel() const {
				return _spMdl;
			}
			CMask getMask() const {
				return _mask;
			}
			UD& refUserData() {
				return _udata;
			}
			const UD& getUserData() const {
				return _udata;
			}
			//! 衝突判定履歴を巡回 (コリジョン開始 / 継続中の判定)
			/*! \param cb[in,out] コールバック(CMgr::Hist) */
			template <class CB>
			void getCollision(CB&& cb) const {
				const Time_t tm = _cmgr->getTime();
				if(_cur) {
					auto& cur = *_cur;
					// 現在の時刻と等しい時に巡回
					if(cur.time == tm) {
						_cmgr->iterateHistCur(*cur.front, cb);
					}
				}
			}
			//! 衝突判定結果を参照 (コリジョン終了判定)
			/*! \param cb[in,out] コールバック(CMgr::Hist) */
			template <class CB>
			void getEndCollision(CB&& cb) const {
				const Time_t tm = _cmgr->getTime();
				if(_prev && _prev->time == tm-1) {
					if(_cur->time == tm) {
						// 前回何かと衝突していて前回も判定済の場合
						// _prevにあって_curに無い物が対象
						const int prev = *(_prev->front),
									cur = *(_cur->front);
						_cmgr->iterateHistPrev(prev, [cur, cm=_cmgr, &cb](const auto& hPrev){
							bool bFound = false;
							cm->iterateHistCur(cur, [&bFound, &hPrev](const auto& hCur){
								if(hCur.hCol == hPrev.hCol) {
									bFound = true;
								}
							});
							if(!bFound) {
								cb(hPrev);
							}
						});
					}
				}
				if(_cur && _cur->time == tm-1) {
					// 今回何とも衝突しなかった場合
					// _cur のリスト全部対象。ただし履歴はColMgr上では過去のものになってるのでPrevを参照
					_cmgr->iterateHistPrev(*_cur->front, cb);
				}
			}
			//! 引数の時刻-1と合致する方のリストインデックス先頭を取得
			/*! \return どちらも無効ならspi::none */
			Int_OP getPrevIndex(const Time_t tm) const {
				if(_cur && _cur->time == tm-1)
					return _cur->front;
				if(_prev && _prev->time == tm-1)
					return _prev->front;
				return spi::none;
			}
			//! カレントリスト末尾に新しくインデックスを加える
			/*! \return 直前の先頭インデックス値 */
			Int_OP setLastIndex(const Time_t tm, const int idx) {
				// もし新しい時刻の設定だったらカーソルの新旧入れ替え
				if(_cur) {
					if(_cur->time < tm) {
						_prev = _cur;
						_cur = spi::construct(tm);
					}
				} else
					_cur = spi::construct(tm);
				return _cur->setLastIndex(idx);
			}
	};

	struct ColMgr_Preamble {
		float	fsize,
				fofs;

		bool operator == (const ColMgr_Preamble& p) const noexcept {
			return fsize == p.fsize &&
					fofs == p.fofs;
		}
	};
	/*!
		\tparam	BC		BroadCollision
		\tparam	Types	g2::Types or g3::Types
		\tparam UD		userdata type
	*/
	template <class BC, class Types, class UD>
	class ColMgr : public IColMgr<
							typename BC::IDType,
							ColMgr_Hist<
								std::shared_ptr<
									ColMem<
										std::shared_ptr<typename Types::ITf>,
										typename BC::IDType,
										UD
									>
								>
							>
						>
	{
		public:
			using user_t = UD;														//!< Userdata
			using broad_t = BC;														//!< BroadCollision class
			using ITf = typename Types::ITf;										//!< ITf interface
			using IModel = typename Types::IModel;
			using BVolume = typename broad_t::BVolume;								//!< Bounding-volume
			using BCId = typename broad_t::IDType;									//!< BroadCollision dependant Id
			// ---- model interface type ----
			using Mdl_SP = std::shared_ptr<ITf>;
			using CMem = ColMem<Mdl_SP, BCId, user_t>;								//!< Collision information structure
			using HCol = std::shared_ptr<CMem>;
			using WCol = std::weak_ptr<CMem>;
			using resmgr_t = spi::ResMgr<CMem>;

		private:
			using Narrow = typename Types::Narrow;
			using Hist = ColMgr_Hist<std::shared_ptr<CMem>>;
			using HistV = std::vector<Hist>;
			using Hist_OP = spi::Optional<const Hist&>;
			using CBHist = std::function<void (const Hist&)>;

			ColMgr_Preamble		_preamble;
			broad_t				_broad;
			resmgr_t			_resmgr;
			Time_t				_time = 0;
			//! history, removelistの現在のインデックス
			int					_swHist = 0;
			HistV				_hist[2];

			HistV& _getCurHist() {
				return _hist[_swHist];
			}
			HistV& _getPrevHist() {
				return _hist[_swHist^1];
			}
			const HistV& _getCurHist() const {
				return _hist[_swHist];
			}
			const HistV& _getPrevHist() const {
				return _hist[_swHist^1];
			}
			//! hc0のリストにhc1のエントリが存在するかチェック
			/*	\return hc1が見つかればその参照を返す */
			Hist_OP _hasPrevCollision(const CMem* cm0, const CMem* cm1) const {
				Hist_OP ret;
				if(const auto pidx = cm0->getPrevIndex(_time)) {
					iterateHistPrev(*pidx, [cm1,&ret](const Hist& h){
						if(h.hCol.get() == cm1)
							ret = h;
					});
				}
				return ret;
			}
			/*!
				\param hv	_hist[0] or _hist[1]
				\param idx	巡回を開始するインデックス
			*/
			static void _IterateHist(const HistV& hv, const int idx, const CBHist& cb) {
				D_Assert0(idx>=0);
				auto* hst = &hv[idx];
				for(;;) {
					cb(*hst);
					const int np = hst->nextOffset;
					if(np != 0)
						hst = reinterpret_cast<decltype(hst)>(reinterpret_cast<uintptr_t>(hst) + np);
					else
						break;
				}
			}
			HistV& _switchHist() {
				_swHist ^= 1;
				auto& cur = _getCurHist();
				cur.clear();
				return cur;
			}
			void _makeHist(const HCol& c0, const HCol& c1) {
				const auto fn = [this, &hist=_getCurHist()](const HCol& hc0, const HCol& hc1) {
					// 前のフレームでの継続フレームリストを探索
					int nf = 0;
					if(auto hist = _hasPrevCollision(hc0.get(), hc1.get()))
						nf = hist->nFrame + 1;
					const int nextId = hist.size();
					const Int_OP lastId = hc0->setLastIndex(_time, nextId);
					if(lastId)
						hist[*lastId].nextOffset = (nextId - *lastId) * sizeof(Hist);
					hist.push_back(Hist(hc1, nf));
				};
				fn(c0, c1);
				fn(c1, c0);
			}
			GetBV_SP<BVolume> _makeGetBV() const {
				struct GetBV : IGetBV<BVolume> {
					const ColMgr* _self;

					GetBV(const ColMgr* s):
						_self(s)
					{}
					BVolume operator()(const void* p) const override {
						auto* cm = static_cast<const CMem*>(p);
						return cm->template getBVolume<BVolume>(_self->_time);
					}
				};
				return GetBV_SP<BVolume>(new GetBV(this));
			}
			template <class Ar, class B, class T, class UD2>
			friend void serialize(Ar&, ColMgr<B,T,UD2>&);
			friend class cereal::access;
			template <class Ar>
			static void load_and_construct(Ar&, cereal::construct<ColMgr>&);

			ColMgr(const ColMgr_Preamble& p):
				_preamble(p),
				_broad(_makeGetBV(), p.fsize, p.fofs)
			{
				Narrow::Initialize();
			}

		public:
			ColMgr(const float fieldSize, const float fieldOfs):
				ColMgr(ColMgr_Preamble{fieldSize, fieldOfs})
			{}
			void cleanBackup() {
				_resmgr.cleanBackup();
			}
			// デバッグ用
			bool operator == (const ColMgr& cm) const noexcept {
				// _resmgrはサイズだけ比較
				return _preamble == cm._preamble &&
						_broad == cm._broad &&
						_resmgr.size() == cm._resmgr.size() &&
						_time == cm._time &&
						_swHist == cm._swHist &&
						_hist[0] == cm._hist[0] &&
						_hist[1] == cm._hist[1];
			}
			// デバッグ用
			void selfCheck() const {
				_broad.selfCheck();
			}
			Time_t getTime() const noexcept override {
				return _time;
			}
			void iterateHistCur(const int idx, const CBHist& cb) const override {
				_IterateHist(_getCurHist(), idx, cb);
			}
			void iterateHistPrev(const int idx, const CBHist& cb) const override {
				_IterateHist(_getPrevHist(), idx, cb);
			}
			//! 当たり判定対象を追加
			/*! \param mask[in] 当たり判定マスク */
			template <class UD2>
			HCol addCol(const CMask mask, const Mdl_SP& spMdl, UD2&& ud=user_t()) {
				HCol hlC = _resmgr.emplace(mask, spMdl, std::forward<UD2>(ud));
				hlC->setBCID(this, _broad.add(hlC.get(), mask));
				D_Assert(spMdl->im_refresh(_time), "empty object detected");
				return hlC;
			}
			void deleteBCID(const BCId id) override {
				_broad.rem(id);
			}
			//! 単体での当たり判定処理
			/*!
				\param[in] mask		コリジョンマスク値
				\param[in] mp		判定対象のモデルポインタ
				\param[in] cb		コールバック関数(HCol)
			*/
			template <class CB>
			void checkCollision(const CMask mask, const IModel* mp, CB&& cb) {
				_broad.refreshBVolume();
				if(!mp->im_refresh(_time))
					return;
				BVolume bv;
				mp->im_getBVolume(bv);
				_broad.checkCollision(mask, bv,
					[time=_time, pMdl=mp, cb=std::forward<CB>(cb)](const void* p) {
						auto* m = static_cast<const CMem*>(p);
						// 詳細判定
						if(Narrow::Hit(pMdl, m->getModel().get(), time))
							cb(m);
					}
				);
			}
			template <class CB>
			void checkCollision(const CMask mask, const Mdl_SP& spMdl, CB&& cb) {
				checkCollision(mask, spMdl.get(), std::forward<CB>(cb));
			}
			//! 全てのオブジェクトを相互に当たり判定
			/*!	時間を1進め、衝突履歴の更新
				A->Bの組み合わせでコールバックを呼び、B->Aは呼ばない
				\param[in] cb		衝突が検出される度に呼ばれるコールバック関数(CMem*, CMem*)
				\param[in] bAdvance	trueの時に時刻を進め、新旧の履歴を切り替える
			*/
			void update() {
				_switchHist();
				++_time;
				_broad.refreshBVolume();
				// advance時, フレームヒットリストを再構築
				_broad.broadCollision([this](void* p0, void* p1){
					// 同じオブジェクトという事はあり得ない筈
					D_Assert0(p0 != p1);
					auto* cm0 = static_cast<CMem*>(p0);
					auto* cm1 = static_cast<CMem*>(p1);

					// 詳細判定
					if(Narrow::Hit(cm0->getModel().get(), cm1->getModel().get(), _time)) {
						_makeHist(cm0->shared_from_this(), cm1->shared_from_this());
					}
				});
			}
	};
}
