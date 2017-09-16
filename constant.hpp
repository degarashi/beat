#pragma once

namespace beat {
	constexpr float Sqrt2 = 1.41421356237f,
					Sqrt3 = 1.73205080757f;
	constexpr static float DOT_THRESHOLD = 1e-4f,		//!< 内積による表裏判定基準
							NEAR_THRESHOLD = 1e-4f,		//!< 同じ座標と判断する基準
							ZEROVEC_LENGTH = 1e-5f,
							SQUARE_RATIO = 1e-2f,
							DOT_THRESHOLD_SQ = DOT_THRESHOLD * SQUARE_RATIO,
							NEAR_THRESHOLD_SQ = NEAR_THRESHOLD * SQUARE_RATIO,
							ZEROVEC_LENGTH_SQ = ZEROVEC_LENGTH * SQUARE_RATIO,
							INFINITY_LENGTH = 1e6f;		//!< 無限遠にあるとみなす距離
}
