#pragma once
#include "generate.hpp"
#include "../g2objects.hpp"
#include "../tfnode_static2d.hpp"
#include "../tfleaf_cached.hpp"

namespace beat {
	namespace g2 {
		namespace {
			namespace tree {
				template <template <class...> class CT>
				void SetCid(int* /*dst*/, CT<>*) {}
				template <template <class...> class CT,
							class T0, class... Ts>
				void SetCid(int* dst, CT<T0, Ts...>*) {
					*dst = T0::GetCID();
					SetCid(dst+1, (CT<Ts...>*)nullptr);
				}
			}
		}
		using lubee::RangeI;
		class TreeGenerator : public Generator {
			public:
				template <class T>
				using Node_t = TfNode_Static<T>;
				using AABBNode = Node_t<AABB>;
				using CircleNode = Node_t<Circle>;

				template <class T>
				using Leaf_t = TfLeafCached<T>;
				using PointLeaf = Leaf_t<Point>;
				using LineLeaf = Leaf_t<Line>;
				using RayLeaf = Leaf_t<Ray>;
				using SegmentLeaf = Leaf_t<Segment>;
				using AABBLeaf = Leaf_t<AABB>;
				using TriangleLeaf = Leaf_t<Triangle>;
				using CircleLeaf = Leaf_t<Circle>;
				using ConvexLeaf = Leaf_t<Convex>;
				using CapsuleLeaf = Leaf_t<Capsule>;

			public:
				template <class T>
				Tf_SP makeAsLeaf(T&& src) {
					return std::make_shared<Leaf_t<T>>(
						std::make_shared<Model<T>>(
							std::forward<T>(src)
						)
					);
				}
				template <class T>
				Tf_SP makeAsNode() {
					return std::make_shared<Node_t<T>>();
				}
				Tf_SP genLeaf(const int cid) {
					#define _MAKELEAF(typ, gtyp)	case typ::GetCID(): return makeAsLeaf<typ>(gen##gtyp());
					#define MAKELEAF(typ)	_MAKELEAF(typ, typ)
					switch(cid) {
						MAKELEAF(Point)
						MAKELEAF(Line)
						MAKELEAF(Ray)
						MAKELEAF(Segment)
						MAKELEAF(AABB)
						_MAKELEAF(Triangle, TriangleArea)
						MAKELEAF(Circle)
						MAKELEAF(Convex)
						MAKELEAF(Capsule)
					}
					#undef MAKELEAF
					AssertF("unknown collision-id");
				}
				Tf_SP genNode(const int cid) {
					#define MAKENODE(typ)	case typ::GetCID(): return makeAsNode<typ>();
					switch(cid) {
						MAKENODE(AABB)
						MAKENODE(Circle)
					}
					#undef MAKENODE
					AssertF("unknown collision-id");
				}
				template <class T, class D=decltype(std::declval<T>().get())>
				static auto CollectLeaf(const T& spRoot, D=nullptr) {
					std::vector<D> v;
					spRoot->template iterateDepthFirst<false>([&v](auto& node, int /*depth*/){
						if(node.im_isLeaf())
							v.push_back(static_cast<D>(&node));
						return spi::Iterate::StepIn;
					});
					return v;
				}
				template <class CTNode, class CTLeaf>
				Tf_SP makeRandomTree(const int nIteration, const int maxDepth) {
					Assert0(nIteration>0 && maxDepth >= 0);
					Tf_SP spRoot;
					do {
						enum Manipulation {
							MNP_Add,			//!< 現在の階層にリーフノードを加える
							MNP_Up,				//!< 階層を上がる
							MNP_MakeChild,		//!< 子ノードを作ってそこにカーソルを移動
							N_Manipulation
						};
						constexpr int NLeaf = CTLeaf::size,
										NNode = CTNode::size;
						int CidId_Leaf[NLeaf],
							CidId_Node[NNode];
						tree::SetCid(CidId_Leaf, (CTLeaf*)nullptr);
						tree::SetCid(CidId_Node, (CTNode*)nullptr);

						spRoot = genNode(CidId_Node[genInt({0,NNode-1})]);
						auto spCursor = spRoot;
						int cursorDepth = 0;
						const int nIter = genInt({0, nIteration});
						for(int i=0 ; i<nIter ; i++) {
							const int m = genInt({0, N_Manipulation-1});
							switch(m) {
								case MNP_Add: {
									spCursor->addChild(genLeaf(CidId_Leaf[genInt({0,NLeaf-1})]));
									break; }
								case MNP_Up:
									// 深度が0の時は何もしない
									if(cursorDepth > 0) {
										spCursor = spCursor->getParent();
										--cursorDepth;
									}
									break;
								case MNP_MakeChild:
									// 最大深度を超えている時は何もしない
									if(cursorDepth < maxDepth) {
										auto c = genNode(CidId_Node[genInt({0,NNode-1})]);
										spCursor->addChild(c);
										spCursor = c;
										++cursorDepth;
									}
									break;
							}
							Assert0(cursorDepth <= maxDepth);
						}
					} while(!spRoot->im_refresh(0));
					return spRoot;
				}
		};
	}
}
