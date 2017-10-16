#include "dim2.hpp"
#include "lubee/error.hpp"
#include "lubee/compare.hpp"
#include "../../aabb2d.hpp"

namespace beat {
	namespace ntree {
		namespace g2 {
			// ---------------------- Index ----------------------
			Morton_t Index::SeparateBits1(Morton_t x) {
				D_Assert0(x < (1<<16));
				x = (x | (x<<8)) & 0x00ff00ff;
				x = (x | (x<<4)) & 0x0f0f0f0f;
				x = (x | (x<<2)) & 0x33333333;
				x = (x | (x<<1)) & 0x55555555;
				return x;
			}
			Morton_t Index::ComposeBits1(Morton_t x) {
				x = (x | (x>>1)) & 0x33333333;
				x = (x | (x>>2)) & 0x0f0f0f0f;
				x = (x | (x>>4)) & 0x00ff00ff;
				x = (x | (x>>8)) & 0x0000ffff;
				return x;
			}
			Index::Index(const uint32_t x, const uint32_t y):
				x(x),
				y(y)
			{}
			Index::Index(const MortonId m):
				x(ComposeBits1(m.value & 0x55555555)),
				y(ComposeBits1((m.value & 0xaaaaaaaa)>>1))
			{}
			Index::operator MortonId() const {
				return asMortonId();
			}
			MortonId Index::asMortonId() const {
				return SeparateBits1(x) | (SeparateBits1(y) << 1);
			}
			bool Index::operator == (const Index& idx) const {
				return value == idx.value;
			}
			bool Index::operator != (const Index& idx) const {
				return !(this->operator == (idx));
			}

			// ---------------------- Dim ----------------------
			std::pair<MortonId,MortonId> Dim::ToMortonMinMaxId(const BVolume& bv, const int nwidth, const float unit, const float ofs) {
				const auto sat = [nwidth,unit,ofs](const float w) -> uint32_t {
					return lubee::Saturate<int>((w+ofs)*unit, 0, nwidth-1);
				};
				return std::make_pair(Index(sat(bv.min.x), sat(bv.min.y)),
										Index(sat(bv.max.x), sat(bv.max.y)));
			}
			int Dim::Classify(const VolumeEntry& ve, const Index centerId, const CBClassify& cb) {
				const uint32_t cx = centerId.x,
								cy = centerId.y;
				const Index pmin = ve.min,
							pmax = ve.max;
				D_Assert0(pmin.x <= pmax.x && pmin.y <= pmax.y);
				int count = 0;
				if(pmin.x < cx) {
					// 左リストに登録
					if(pmin.y < cy) {
						// 左上
						cb(ve, 0);
						++count;
					}
					if(pmax.y >= cy) {
						// 左下
						cb(ve, 2);
						++count;
					}
				}
				if(pmax.x >= cx) {
					// 右リストに登録
					if(pmin.y < cy) {
						// 右上
						cb(ve, 1);
						++count;
					}
					if(pmax.y >= cy) {
						// 右下
						cb(ve, 3);
						++count;
					}
				}
				return count;
			}
			void Dim::Classify(const VolumeEntry** (&dst)[N_LayerSize], const VolumeEntry* src, const int nSrc, const Index centerId) {
				const auto cb =[&dst](const VolumeEntry& ve, const int idx){
					*dst[idx]++ = &ve;
				};
				// オブジェクト振り分け
				for(int i=0 ; i<nSrc ; i++) {
					const int count = Classify(src[i], centerId, cb);
					D_Assert0(count > 0);
				}
			}
			void Dim::CalcLowerCenterId(Index (&dst)[N_LayerSize], const Index id, const int width) {
				const uint32_t cx = id.x,
								cy = id.y;
				// 中心座標計算
				dst[0] = Index(cx - width, cy - width);
				dst[1] = Index(cx + width, cy - width);
				dst[2] = Index(cx - width, cy + width);
				dst[3] = Index(cx + width, cy + width);
			}
		}
	}
}
