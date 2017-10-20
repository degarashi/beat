#pragma once
#include "imodel2d.hpp"
#include "lubee/rect.hpp"

namespace beat {
	namespace g2 {
		struct Segment;
		//! AxisAlignedBox
		class AABB : public CID<AABB> {
			private:
				int _getAreaNumX(float p) const;
				int _getAreaNumY(float p) const;
				void _makeSegmentX(Segment& s, int num) const;
				void _makeSegmentY(Segment& s, int num) const;
				bool _checkHitX(const Segment& s0, int from, int to) const;
				bool _checkHitY(const Segment& s0, int from, int to) const;
				std::pair<int,int> _getAreaNum(const Vec2& p) const;

				template <class Ar>
				friend void serialize(Ar&, AABB&);

			public:
				Vec2 min, max;

				AABB() = default;
				AABB(const Vec2& min_v, const Vec2& max_v);
				AABB(const lubee::RectF& rect);
				Vec2 nearest(const Vec2& pos) const;
				float width() const;
				float height() const;
				void distend(float r);
				void encapsule(const Vec2& v);
				template <class Itr>
				static AABB Encapsule(Itr itr, const Itr itrE) {
					AABB ret(*itr, *itr);
					while(++itr != itrE) {
						ret.encapsule(*itr);
					}
					return ret;
				}

				// ----------------------------------------
				float bs_getArea() const;
				float bs_getInertia() const;
				Vec2 bs_getCenter() const;
				Circle bs_getBCircle() const;
				const AABB& bs_getBBox() const;

				Vec2 support(const Vec2& dir) const;
				AABB& operator += (const Vec2& ofs);
				AABB operator * (const AMat32& m) const;
				bool operator == (const AABB& a) const;
				bool operator != (const AABB& a) const;
				friend std::ostream& operator << (std::ostream& os, const AABB& a);
				// ----------------------------------------

				spi::none_t hit(...) const;
				bool hit(const Vec2& p) const;
				bool hit(const Segment& l) const;
				bool hit(const AABB& ab) const;

				// ---- for MakeBoundary ----
				void setBoundary(const IModel* p);
				void appendBoundary(const IModel* p);
		};
		using AABBM = Model<AABB>;
	}
}
