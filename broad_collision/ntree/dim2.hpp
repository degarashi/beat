#pragma once
#include "defines.hpp"
#include <functional>

namespace beat {
	namespace g2 {
		class AABB;
	}
	namespace ntree {
		namespace g2 {
			struct Index;
			using MortonId = MortonId<Index>;
			struct Index {
				private:
					//! 16bitの数値ビットを1つ飛びに変換する
					static Morton_t SeparateBits1(Morton_t x);
					//! 32bitの1つ飛び16bit数値を元に戻す
					static Morton_t ComposeBits1(Morton_t x);
				public:
					union {
						struct {
							Morton_t x:16,
									 y:16;
						};
						Morton_t	value;
					};
					Index() = default;
					Index(uint32_t x, uint32_t y);
					//! モートンIDから軸別インデックス値へ変換
					Index(MortonId m);
					//! 軸別インデックス値からモートンIDに変換
					operator MortonId() const;
					MortonId asMortonId() const;
					bool operator == (const Index& idx) const;
					bool operator != (const Index& idx) const;
			};
			using VolumeEntry = VolumeEntry<Index>;
			// -------- <<コリジョンツリー次元ポリシークラス>> --------
			//! 4分木用
			struct Dim {
				using MortonId = MortonId;
				using Index = Index;
				using VolumeEntry = VolumeEntry;
				constexpr static int N_Dim = 2,
									N_LayerSize = (1 << N_Dim);
				using BVolume = ::beat::g2::AABB;

				//! geo2d::IModelからモートンIdを算出
				/*!
					\return first=Min(XY)のモートンId
							second=Max(XY)のモートンId
				*/
				static std::pair<MortonId,MortonId> ToMortonMinMaxId(const BVolume& bv, int nwidth, float unit, float ofs);
				//! オブジェクトの軸分類
				/*!
					\param[out] dst			オブジェクトの振り分け先 (4マス) 重複あり
					\param[in] src			振り分けるオブジェクト配列
					\param[in] nSrc			srcの要素数
					\param[in] centerId		振り分け中心点(軸別のマス目インデックス)
				*/
				static void Classify(const VolumeEntry** (&dst)[N_LayerSize], const VolumeEntry* src, int nSrc, Index centerId);
				using CBClassify = std::function<void (const VolumeEntry&, int)>;
				static int Classify(const VolumeEntry& ve, Index centerIdx, const CBClassify& cb);
				//! 1つ下の階層の中心インデックス値を算出
				/*!
					\param[out] idDst	1つ下の階層の中心座標インデックス
					\param[in] id		現在の階層の中心座標インデックス
					\param[in] width	現在の階層のマス幅(width*2で全体)
				*/
				static void CalcLowerCenterId(Index (&dst)[N_LayerSize], Index id, int width);
			};
		}
	}
}
