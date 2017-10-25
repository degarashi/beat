#pragma once
#include "../collision_mgr.hpp"
#include "spine/serialization/resmgr.hpp"
#include "spine/serialization/optional.hpp"

namespace beat {
	template <class Ar>
	void serialize(Ar& ar, ColMgr_Preamble& p) {
		ar(
			cereal::make_nvp("fieldSize", p.fsize),
			cereal::make_nvp("fieldOfs", p.fofs)
		);
	}
	template <class Ar>
	void serialize(Ar& ar, ColMem_Index& idx) {
		ar(
			cereal::make_nvp("time", idx.time),
			cereal::make_nvp("front", idx.front),
			cereal::make_nvp("last", idx.last)
		);
	}
	template <class Ar, class HC>
	void serialize(Ar& ar, ColMgr_Hist<HC>& h) {
		ar(
			cereal::make_nvp("col", h.hCol),
			cereal::make_nvp("nFrame", h.nFrame),
			cereal::make_nvp("nextOffset", h.nextOffset)
		);
	}

	template <class Ar, class M, class B, class U>
	void serialize(Ar& ar, ColMem<M,B,U>& cm) {
		// _cmgr, _bcid は、シリアライズしないでロード後に再設定する
		ar(
			cereal::make_nvp("mask", cm._mask),
			cereal::make_nvp("cur", cm._cur),
			cereal::make_nvp("prev", cm._prev),
			cereal::make_nvp("model", cm._spMdl),
			cereal::make_nvp("udata", cm._udata)
		);
	}
	template <class Ar, class B, class T, class UD>
	void serialize(Ar& ar, ColMgr<B,T,UD>& c) {
		ar(
			cereal::make_nvp("preamble", c._preamble),
			cereal::make_nvp("res", c._resmgr),
			cereal::make_nvp("time", c._time),
			cereal::make_nvp("swHist", c._swHist),
			cereal::make_nvp("hist", c._hist)
		);
	}
	template <class B, class T, class UD>
	template <class Ar>
	void ColMgr<B,T,UD>::load_and_construct(Ar& ar, cereal::construct<ColMgr>& c) {
		ColMgr_Preamble pre;
		ar(cereal::make_nvp("preamble", pre));
		c(pre);
		ar(
			cereal::make_nvp("res", c->_resmgr),
			cereal::make_nvp("time", c->_time),
			cereal::make_nvp("swHist", c->_swHist),
			cereal::make_nvp("hist", c->_hist)
		);
		for(const auto& r : c->_resmgr) {
			r->setBCID(c.ptr(), c->_broad.add(r.get(), r->getMask()));
		}
	}
}
