/******************************************************************************
 * Copyright (c) 2013, NVIDIA CORPORATION.  All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the NVIDIA CORPORATION nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
 * ARE DISCLAIMED. IN NO EVENT SHALL NVIDIA CORPORATION BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************/

/******************************************************************************
 *
 * Code and text by Sean Baxter, NVIDIA Research
 * See http://nvlabs.github.io/moderngpu for repository and documentation.
 *
 ******************************************************************************/

#pragma once

#include "mgpudevice.cuh"
#include "util/mgpucontext.h"

namespace mgpu {

////////////////////////////////////////////////////////////////////////////////
// kernels/reduce.cuh

// Reduce input and return variable in device memory or host memory, or both.
// Provide a non-null pointer to retrieve data.
template<typename InputIt, typename T, typename Op>
MGPU_HOST void Reduce(InputIt data_global, int count, T identity, Op op,
	T* reduce_global, T* reduce_host, CudaContext& context);

// T = std::iterator_traits<InputIt>::value_type.
// Reduce with identity = 0 and op = mgpu::plus<T>.
// Returns the value in host memory.
template<typename InputIt>
MGPU_HOST typename std::iterator_traits<InputIt>::value_type
Reduce(InputIt data_global, int count, CudaContext& context);


////////////////////////////////////////////////////////////////////////////////
// kernels/scan.cuh

// Scan inputs in device memory.
// MgpuScanType may be:
//		MgpuScanTypeExc (exclusive) or
//		MgpuScanTypeInc (inclusive).
// Return the total in device memory, host memory, or both.
template<MgpuScanType Type, typename DataIt, typename T, typename Op,
	typename DestIt>
MGPU_HOST void Scan(DataIt data_global, int count, T identity, Op op,
	T* reduce_global, T* reduce_host, DestIt dest_global, 
	CudaContext& context);

// Exclusive scan with identity = 0 and op = mgpu::plus<T>.
// Returns the total in host memory.
template<typename InputIt, typename TotalType>
MGPU_HOST void ScanExc(InputIt data_global, int count, TotalType* total,
	CudaContext& context);

// Like above, but don't return the total.
template<typename InputIt>
MGPU_HOST void ScanExc(InputIt data_global, int count, CudaContext& context);

////////////////////////////////////////////////////////////////////////////////
// kernels/bulkremove.cuh

// Compact the elements in source_global by removing elements identified by
// indices_global. indices_global must be unique, sorted, and range between 0
// and sourceCount - 1. The number of outputs is sourceCount - indicesCount.

// IndicesIt should resolve to an integer type. iterators like step_iterator
// are supported.

// If sourceCount = 10, indicesCount = 6, and indices = (1, 3, 4, 5, 7, 8), then
// dest = A0 A2 A6 A9. (All indices between 0 and sourceCount - 1 except those
// in indices_global).
template<typename InputIt, typename IndicesIt, typename OutputIt>
MGPU_HOST void BulkRemove(InputIt source_global, int sourceCount,
	IndicesIt indices_global, int indicesCount, OutputIt dest_global,
	CudaContext& context);


////////////////////////////////////////////////////////////////////////////////
// kernels/bulkinsert.cuh

// Combine aCount elements in a_global with bCount elements in b_global.
// Each element a_global[i] is inserted before position indices_global[i] and
// stored to dest_global. The insertion indices are relative to the B array,
// not the output. Indices must be sorted but not necessarily unique. 

// If aCount = 5, bCount = 3, and indices = (1, 1, 2, 3, 3), the output is:
// B0 A0 A1 B1 A2 B2 A3 A4.
template<typename InputIt1, typename IndicesIt, typename InputIt2,
	typename OutputIt>
MGPU_HOST void BulkInsert(InputIt1 a_global, IndicesIt indices_global, 
	int aCount, InputIt2 b_global, int bCount, OutputIt dest_global,
	CudaContext& context);


////////////////////////////////////////////////////////////////////////////////
// kernels/merge.cuh

// MergeKeys merges two arrays of sorted inputs with C++-comparison semantics.
// aCount items from aKeys_global and bCount items from bKeys_global are merged
// into aCount + bCount items in keys_global.
// Comp is a comparator type supporting strict weak ordering.
// If !comp(b, a), then a is placed before b in the output.
template<typename KeysIt1, typename KeysIt2, typename KeysIt3, typename Comp>
MGPU_HOST void MergeKeys(KeysIt1 aKeys_global, int aCount, KeysIt2 bKeys_global,
	int bCount, KeysIt3 keys_global, Comp comp, CudaContext& context);

// MergeKeys specialized with Comp = mgpu::less<T>.
template<typename KeysIt1, typename KeysIt2, typename KeysIt3>
MGPU_HOST void MergeKeys(KeysIt1 aKeys_global, int aCount, KeysIt2 bKeys_global,
	int bCount, KeysIt3 keys_global, CudaContext& context);

// MergePairs merges two arrays of sorted inputs by key and copies values.
// If !comp(bKey, aKey), then aKey is placed before bKey in the output, and
// the corresponding aData is placed before bData. This corresponds to *_by_key
// functions in Thrust.
template<typename KeysIt1, typename KeysIt2, typename KeysIt3, typename ValsIt1,
	typename ValsIt2, typename ValsIt3, typename Comp>
MGPU_HOST void MergePairs(KeysIt1 aKeys_global, ValsIt1 aVals_global, 
	int aCount, KeysIt2 bKeys_global, ValsIt2 bVals_global, int bCount,
	KeysIt3 keys_global, ValsIt3 vals_global, Comp comp, CudaContext& context);

// MergePairs specialized with Comp = mgpu::less<T>.
template<typename KeysIt1, typename KeysIt2, typename KeysIt3, typename ValsIt1,
	typename ValsIt2, typename ValsIt3>
MGPU_HOST void MergePairs(KeysIt1 aKeys_global, ValsIt1 aVals_global, 
	int aCount, KeysIt2 bKeys_global, ValsIt2 bVals_global, int bCount,
	KeysIt3 keys_global, ValsIt3 vals_global, CudaContext& context);


////////////////////////////////////////////////////////////////////////////////
// kernels/mergesort.cuh

// MergesortKeys sorts data_global using comparator Comp.
// If !comp(b, a), then a comes before b in the output. The data is sorted
// in-place.
template<typename T, typename Comp>
MGPU_HOST void MergesortKeys(T* data_global, int count, Comp comp,
	CudaContext& context);

// MergesortKeys specialized with Comp = mgpu::less<T>.
template<typename T>
MGPU_HOST void MergesortKeys(T* data_global, int count, CudaContext& context);

// MergesortPairs sorts data by key, copying data. This corresponds to 
// sort_by_key in Thrust.
template<typename KeyType, typename ValType, typename Comp>
MGPU_HOST void MergesortPairs(KeyType* keys_global, ValType* values_global,
	int count, Comp comp, CudaContext& context);

// MergesortPairs specialized with Comp = mgpu::less<KeyType>.
template<typename KeyType, typename ValType>
MGPU_HOST void MergesortPairs(KeyType* keys_global, ValType* values_global,
	int count, CudaContext& context);

// MergesortIndices is like MergesortPairs where values_global is treated as
// if initialized with integers (0 ... count - 1). 
template<typename KeyType, typename Comp>
MGPU_HOST void MergesortIndices(KeyType* keys_global, int* values_global,
	int count, Comp comp, CudaContext& context);

// MergesortIndices specialized with Comp = mgpu::less<KeyType>.
template<typename KeyType>
MGPU_HOST void MergesortIndices(KeyType* keys_global, int* values_global,
	int count, CudaContext& context);


////////////////////////////////////////////////////////////////////////////////
// kernels/segmentedsort.cuh

// Mergesort count items in-place in data_global. Keys are compared with Comp
// (as they are in MergesortKeys), however keys remain inside the segments 
// defined by flags_global. 

// flags_global is a bitfield cast to uint*. Each bit in flags_global is a 
// segment head flag. Only keys between segment head flags (inclusive on the
// left and exclusive on the right) may be exchanged. The first element is 
// assumed to start a segment, regardless of the value of bit 0.

// Passing verbose=true causes the function to print mergepass statistics to the
// console. This may be helpful for developers to understand the performance 
// characteristics of the function and how effectively it early-exits merge
// operations.
template<typename T, typename Comp>
MGPU_HOST void SegSortKeysFromFlags(T* data_global, int count,
	const uint* flags_global, CudaContext& context, Comp comp,
	bool verbose = false);

// SegSortKeysFromFlags specialized with Comp = mgpu::less<T>.
template<typename T>
MGPU_HOST void SegSortKeysFromFlags(T* data_global, int count,
	const uint* flags_global, CudaContext& context, bool verbose = false);

// Segmented sort using head flags and supporting value exchange.
template<typename KeyType, typename ValType, typename Comp>
MGPU_HOST void SegSortPairsFromFlags(KeyType* keys_global, 
	ValType* values_global, int count, const uint* flags_global, 
	CudaContext& context, Comp comp, bool verbose = false);

// SegSortPairsFromFlags specialized with Comp = mgpu::less<T>.
template<typename KeyType, typename ValType>
MGPU_HOST void SegSortPairsFromFlags(KeyType* keys_global, 
	ValType* values_global, int count, const uint* flags_global, 
	CudaContext& context, bool verbose = false);

// Segmented sort using segment indices rather than head flags. indices_global
// is a sorted and unique list of indicesCount segment start locations. These
// indices correspond to the set bits in the flags_global field. A segment
// head index for position 0 may be omitted.
template<typename T, typename Comp>
MGPU_HOST void SegSortKeysFromIndices(T* data_global, int count,
	const int* indices_global, int indicesCount, CudaContext& context,
	Comp comp, bool verbose = false);

// SegSortKeysFromIndices specialized with Comp = mgpu::less<T>.
template<typename T>
MGPU_HOST void SegSortKeysFromIndices(T* data_global, int count,
	const int* indices_global, int indicesCount, CudaContext& context,
	bool verbose = false);

// Segmented sort using segment indices and supporting value exchange.
template<typename KeyType, typename ValType, typename Comp>
MGPU_HOST void SegSortPairsFromIndices(KeyType* keys_global, 
	ValType* values_global, int count, const int* indices_global, 
	int indicesCount, CudaContext& context, Comp comp, bool verbose = false);

// SegSortPairsFromIndices specialized with Comp = mgpu::less<KeyType>.
template<typename KeyType, typename ValType>
MGPU_HOST void SegSortPairsFromIndices(KeyType* keys_global, 
	ValType* values_global, int count, const int* indices_global, 
	int indicesCount, CudaContext& context, bool verbose = false);


////////////////////////////////////////////////////////////////////////////////
// kernels/localitysort.cuh

// LocalitySortKeys is a version of MergesortKeys optimized for non-uniformly 
// random input arrays. If the keys begin close to their sorted destinations,
// this function may exploit the structure to early-exit merge passes.

// Passing verbose=true causes the function to print mergepass statistics to the
// console. This may be helpful for developers to understand the performance 
// characteristics of the function and how effectively it early-exits merge
// operations.
template<typename T, typename Comp>
MGPU_HOST void LocalitySortKeys(T* data_global, int count, CudaContext& context,
	Comp comp, bool verbose = false);

// LocalitySortKeys specialized with Comp = mgpu::less<T>.
template<typename T>
MGPU_HOST void LocalitySortKeys(T* data_global, int count, CudaContext& context,
	bool verbose = false);

// Locality sort supporting value exchange.
template<typename KeyType, typename ValType, typename Comp>
MGPU_HOST void LocalitySortPairs(KeyType* keys_global, ValType* values_global,
	int count, CudaContext& context, Comp comp, bool verbose = false);

// LocalitySortPairs specialized with Comp = mpgu::less<T>.
template<typename KeyType, typename ValType>
MGPU_HOST void LocalitySortPairs(KeyType* keys_global, ValType* values_global,
	int count, CudaContext& context, bool verbose = false);


////////////////////////////////////////////////////////////////////////////////
// kernels/sortedsearch.cuh

// Vectorized sorted search. This is the most general form of the function.
// Executes two simultaneous linear searches on two sorted inputs.

// Bounds:
//		MgpuBoundsLower - 
//			lower-bound search of A into B.
//			upper-bound search of B into A.
//		MgpuBoundsUpper - 
//			upper-bound search of A into B.
//			lower-bound search of B into A.
// Type[A|B]:
//		MgpuSearchTypeNone - no output for this input.
//		MgpuSearchTypeIndex - return search indices as integers.
//		MgpuSearchTypeMatch - return 0 (no match) or 1 (match).
//			For TypeA, returns 1 if there is at least 1 matching element in B
//				for element in A.
//			For TypeB, returns 1 if there is at least 1 matching element in A
//				for element in B.
//		MgpuSearchTypeIndexMatch - return search indices as integers. Most
//			significant bit is match bit.
//	aMatchCount, bMatchCount - If Type is Match or IndexMatch, return the total 
//		number of elements in A or B with matches in B or A, if the pointer is
//		not null. This generates a synchronous cudaMemcpyDeviceToHost call that
//		callers using streams should be aware of.
template<MgpuBounds Bounds, MgpuSearchType TypeA, MgpuSearchType TypeB,
	typename InputIt1, typename InputIt2, typename OutputIt1, 
	typename OutputIt2, typename Comp>
MGPU_HOST void SortedSearch(InputIt1 a_global, int aCount, InputIt2 b_global,
	int bCount, OutputIt1 aIndices_global, OutputIt2 bIndices_global,
	Comp comp, CudaContext& context, int* aMatchCount = 0, 
	int* bMatchCount = 0);

// SortedSearch specialized with Comp = mgpu::less<T>.
template<MgpuBounds Bounds, MgpuSearchType TypeA, MgpuSearchType TypeB,
	typename InputIt1, typename InputIt2, typename OutputIt1, 
	typename OutputIt2>
MGPU_HOST void SortedSearch(InputIt1 a_global, int aCount, InputIt2 b_global,
	int bCount, OutputIt1 aIndices_global, OutputIt2 bIndices_global,
	CudaContext& context, int* aMatchCount = 0, int* bMatchCount = 0);

// SortedSearch specialized with
// TypeA = MgpuSearchTypeIndex
// TypeB = MgpuSearchTypeNone
// aMatchCount = bMatchCount = 0.
template<MgpuBounds Bounds, typename InputIt1, typename InputIt2,
	typename OutputIt, typename Comp>
MGPU_HOST void SortedSearch(InputIt1 a_global, int aCount, InputIt2 b_global,
	int bCount, OutputIt aIndices_global, Comp comp, CudaContext& context);

// SortedSearch specialized with Comp = mgpu::less<T>.
template<MgpuBounds Bounds, typename InputIt1, typename InputIt2,
	typename OutputIt>
MGPU_HOST void SortedSearch(InputIt1 a_global, int aCount, InputIt2 b_global,
	int bCount, OutputIt aIndices_global, CudaContext& context);

// SortedEqualityCount returns the difference between upper-bound (computed by
// this function) and lower-bound (passed as an argument). That is, it computes
// the number of occurences of a key in B that match each key in A.

// The provided operator must have a method:
//		int operator()(int lb, int ub) const;
// It returns the count given the lower- and upper-bound indices.
// 
// An object SortedEqualityOp is provided:
//		struct SortedEqualityOp {
//			MGPU_HOST_DEVICE int operator()(int lb, int ub) const {
//				return ub - lb;
//			}
//		};
template<typename InputIt1, typename InputIt2, typename InputIt3,
	typename OutputIt, typename Comp, typename Op>
MGPU_HOST void SortedEqualityCount(InputIt1 a_global, int aCount,
	InputIt2 b_global, int bCount, InputIt3 lb_global, OutputIt counts_global, 
	Comp comp, Op op, CudaContext& context);

// Specialization of SortedEqualityCount with Comp = mgpu::less<T>.
template<typename InputIt1, typename InputIt2, typename InputIt3,
	typename OutputIt, typename Op>
MGPU_HOST void SortedEqualityCount(InputIt1 a_global, int aCount,
	InputIt2 b_global, int bCount, InputIt3 lb_global, OutputIt counts_global, 
	Op op, CudaContext& context);

////////////////////////////////////////////////////////////////////////////////
// kernels/loadbalance.cuh

// LoadBalanceSearch is a special vectorized sorted search. Consider bCount
// objects that generate a variable number of work items, with aCount work
// items in total. The caller computes an exclusive scan of the work-item counts
// into b_global.

// indices_global has aCount outputs. indices_global[i] is the index of the 
// object that generated the i'th work item.
// Eg:
// work-item counts:  2,  5,  3,  0,  1.
// scan counts:       0,  2,  7, 10, 10.   aCount = 11.
// 
// LoadBalanceSearch computes the upper-bound of counting_iterator<int>(0) with
// the scan of the work-item counts and subtracts 1:
// LBS: 0, 0, 1, 1, 1, 1, 1, 2, 2, 2, 4.

// This is equivalent to expanding the index of each object by the object's
// work-item count.


template<typename InputIt>
MGPU_HOST void LoadBalanceSearch(int aCount, InputIt b_global, int bCount,
	int* indices_global, CudaContext& context);

////////////////////////////////////////////////////////////////////////////////
// kernels/intervalmove.cuh

// IntervalExpand duplicates intervalCount items in values_global.
// indices_global is an intervalCount-sized array filled with the scan of item
// expand counts. moveCount is the total number of outputs (sum of expand 
// counts).

// Eg:
//		values  =  0,  1,  2,  3,  4,  5,  6,  7,  8
//		counts  =  1,  2,  1,  0,  4,  2,  3,  0,  2
//		indices =  0,  1,  3,  4,  4,  8, 10, 13, 13 (moveCount = 15).
// Expand values[i] by counts[i]:
// output  =  0, 1, 1, 2, 4, 4, 4, 4, 5, 5, 6, 6, 6, 8, 8 
template<typename IndicesIt, typename ValuesIt, typename OutputIt>
MGPU_HOST void IntervalExpand(int moveCount, IndicesIt indices_global, 
	ValuesIt values_global, int intervalCount, OutputIt output_global,
	CudaContext& context);

// IntervalMove is a load-balanced and vectorized device memcpy.
// It copies intervalCount variable-length intervals from user-defined sources
// to user-defined destinations. If destination intervals overlap, results are
// undefined.

// Eg:
// Interval counts:
//    0:     3    9    1    9    8    5   10    2    5    2
//   10:     8    6    5    2    4    0    8    2    5    6
// Scan of interval counts (indices_global):
//    0:     0    3   12   13   22   30   35   45   47   52
//   10:    54   62   68   73   75   79   79   87   89   94  (moveCount = 100).
// Interval gather (gather_global):
//    0:    75   86   17    2   67   24   37   11   95   35
//   10:    52   18   47    0   13   75   78   60   62   29
// Interval scatter (scatter_global):
//    0:    10   80   99   27   41   71   15    0   36   13
//   10:    89   49   66   97   76   76    2   25   61   55

// This vectorizes into 20 independent memcpy operations which are load-balanced
// across CTAs:
// move 0: (75, 78)->(10, 13)       move 10: (52, 60)->(10, 18)
// move 1: (86, 95)->(80, 89)       move 11: (18, 24)->(49, 55)
// move 2: (17, 18)->(99,100)       move 12: (47, 52)->(66, 71)
// move 3: ( 2, 11)->(27, 36)       move 13: ( 0,  2)->(97, 99)
// move 4: (67, 75)->(41, 49)       move 14: (13, 17)->(76, 80)
// move 5: (24, 29)->(71, 76)       move 15: (75, 75)->(76, 76)
// move 6: (37, 47)->(15, 25)       move 16: (78, 86)->( 2, 10)
// move 7: (11, 13)->( 0,  3)       move 17: (60, 62)->(25, 27)
// move 8: (95,100)->(36, 41)       move 18: (62, 67)->(61, 66)
// move 9: (35, 37)->(13, 15)       move 19: (29, 35)->(55, 61)
template<typename GatherIt, typename ScatterIt, typename IndicesIt, 
	typename InputIt, typename OutputIt>
MGPU_HOST void IntervalMove(int moveCount, GatherIt gather_global, 
	ScatterIt scatter_global, IndicesIt indices_global, int intervalCount, 
	InputIt input_global, OutputIt output_global, CudaContext& context);

// IntervalGather is a specialization of IntervalMove that stores output data
// sequentially into output_global. For the example above, IntervalGather treats
// scatter_global the same as indices_global.
template<typename GatherIt, typename IndicesIt, typename InputIt,
	typename OutputIt>
MGPU_HOST void IntervalGather(int moveCount, GatherIt gather_global, 
	IndicesIt indices_global, int intervalCount, InputIt input_global,
	OutputIt output_global, CudaContext& context);

// IntervalScatter is a specialization of IntervalMove that loads input data
// sequentially from input_global. For the example above, IntervalScatter treats
// gather_global the same as indices_global.
template<typename ScatterIt, typename IndicesIt, typename InputIt,
	typename OutputIt>
MGPU_HOST void IntervalScatter(int moveCount, ScatterIt scatter_global,
	IndicesIt indices_global, int intervalCount, InputIt input_global,
	OutputIt output_global, CudaContext& context);


////////////////////////////////////////////////////////////////////////////////
// kernels/join.cuh

// RelationalJoin is a sort-merge join that returns indices into one of the four
// relational joins:
//		MgpuJoinKindInner
//		MgpuJoinKindLeft
//		MgpuJoinKindRight
//		MgpuJoinKindOuter.

// A =  100, 101, 103, 103
// B =  100, 100, 102, 103
// Outer join:
//     ai, bi   a[ai], b[bi]
// 0:  (0, 0) -  (100, 100)    // cross-product expansion for key 100
// 1:  (0, 1) -  (100, 100)
// 2:  (1, -) -  (101, ---)    // left-join for key 101
// 3:  (-, 2) -  (---, 102)    // right-join for key 102
// 4:  (3, 3) -  (103, 103)    // cross-product expansion for key 103

// MgpuJoinKindLeft drops the right-join on line 3.
// MgpuJoinKindRight drops the left-join on line 2.
// MgpuJoinKindInner drops both the left- and right-joins.

// The caller passes MGPU_MEM(int) pointers to hold indices. Memory is allocated
// by the join function using the allocator associated with the context. It 
// returns the number of outputs.

// RelationalJoin performs one cudaMemcpyDeviceToHost to retrieve the size of
// the output array. This is a synchronous operation and may prevent queueing
// for callers using streams.
template<MgpuJoinKind Kind, typename InputIt1, typename InputIt2,
	typename Comp>
MGPU_HOST int RelationalJoin(InputIt1 a_global, int aCount, InputIt2 b_global,
	int bCount, MGPU_MEM(int)* ppAJoinIndices, MGPU_MEM(int)* ppBJoinIndices, 
	Comp comp, CudaContext& context);
 
// Specialization of RelationJoil with Comp = mgpu::less<T>.
template<MgpuJoinKind Kind, typename InputIt1, typename InputIt2>
MGPU_HOST int RelationalJoin(InputIt1 a_global, int aCount, InputIt2 b_global,
	int bCount, MGPU_MEM(int)* ppAJoinIndices, MGPU_MEM(int)* ppBJoinIndices, 
	CudaContext& context);


////////////////////////////////////////////////////////////////////////////////
// kernels/sets.cuh

// SetOpKeys implements multiset operations with C++ set_* semantics.
// MgpuSetOp may be:
//		MgpuSetOpIntersection -		like std::set_intersection
//		MgpuSetOpUnion -			like std::set_union
//		MgpuSetOpDiff -				like std::set_difference
//		MgpuSetOpSymDiff -			like std::set_symmetric_difference

// Setting Duplicates to false increases performance for inputs with no 
// duplicate keys in either array.

// The caller passes MGPU_MEM(T) pointers to hold outputs. Memory is allocated
// by the multiset function using the allocator associated with the context. It 
// returns the number of outputs.

// SetOpKeys performs one cudaMemcpyDeviceToHost to retrieve the size of
// the output array. This is a synchronous operation and may prevent queueing
// for callers using streams.

// If compact = true, SetOpKeys pre-allocates an output buffer is large as the
// sum of the input arrays. Partials results are computed into this temporary
// array before being moved into the final array. It consumes more space but
// results in higher performance.
template<MgpuSetOp Op, bool Duplicates, typename It1, typename It2,
	typename T, typename Comp>
MGPU_HOST int SetOpKeys(It1 a_global, int aCount, It2 b_global, int bCount,
	MGPU_MEM(T)* ppKeys_global, Comp comp, CudaContext& context, 
	bool compact = true);

// Specialization of SetOpKeys with Comp = mgpu::less<T>.
template<MgpuSetOp Op, bool Duplicates, typename It1, typename It2, typename T>
MGPU_HOST int SetOpKeys(It1 a_global, int aCount, It2 b_global, int bCount,
	MGPU_MEM(T)* ppKeys_global, CudaContext& context, bool compact = true);

// SetOpPairs runs multiset operations by key and supports value exchange.
template<MgpuSetOp Op, bool Duplicates, typename KeysIt1, typename KeysIt2,
	typename ValsIt1, typename ValsIt2, typename KeyType, typename ValType,
	typename Comp>
MGPU_HOST int SetOpPairs(KeysIt1 aKeys_global, ValsIt1 aVals_global, int aCount,
	KeysIt2 bKeys_global, ValsIt2 bVals_global, int bCount,
	MGPU_MEM(KeyType)* ppKeys_global, MGPU_MEM(ValType)* ppVals_global, 
	Comp comp, CudaContext& context);

// Specialization of SetOpPairs with Comp = mgpu::less<T>.
template<MgpuSetOp Op, bool Duplicates, typename KeysIt1, typename KeysIt2,
	typename ValsIt1, typename ValsIt2, typename KeyType, typename ValType>
MGPU_HOST int SetOpPairs(KeysIt1 aKeys_global, ValsIt1 aVals_global, int aCount,
	KeysIt2 bKeys_global, ValsIt2 bVals_global, int bCount,
	MGPU_MEM(KeyType)* ppKeys_global, MGPU_MEM(ValType)* ppVals_global, 
	CudaContext& context);

////////////////////////////////////////////////////////////////////////////////
// kernels/segreducecsr.cuh

// SegReducePreprocessData is defined in segreduce.cuh. It includes:
// -	limits for CSR->tiles
// -	packed thread codes for each thread in the reduction
// -	(optional) CSR2 array of filtered segment offsets
struct SegReducePreprocessData;

// SegReduceCsr runs a segmented reduction given an input and a sorted list of
// segment start offsets. This implementation requires operators support 
// commutative (a + b = b + a) and associative (a + (b + c) = (a + b) + c)
// evaluation.

// In the segmented reduction, reduce-by-key, and Spmv documentation, "segment"
// and "row" are used interchangably. A 
// 

// InputIt data_global		- Data value input.
// int count				- Size of input array data_global.
// CsrIt csr_global			- List of integers for start of each segment. 
//							  The first entry must be 0 (indicating that the 
//							  first segment starts at offset 0).
//							  Equivalent to exc-scan of segment sizes.
//							  If supportEmpty is false: must be ascending.
//							  If supportEmpty is true: must be non-descending.
// int numSegments			- Size of segment list csr_global. Must be >= 1.
// bool supportEmpty		- Basic seg-reduce code does not support empty 
//							  segments.
//							  Set supportEmpty = true to add pre- and post-
//							  processing to support empty segments.
// OutputIt dest_global		- Output array for segmented reduction. Allocate
//							  numSegments elements. Should be same data type as 
//							  InputIt and identity.
// T identity				- Identity for reduction operation. Eg, use 0 for 
//							  addition or 1 for multiplication.
// Op op					- Reduction operator. Model on std::plus<>. MGPU
//							  provides operators mgpu::plus<>, minus<>, 
//							  multiplies<>, modulus<>, bit_or<> bit_and<>,
//							  bit_xor<>, maximum<>, and minimum<>.
// CudaContext& context		- MGPU context support object. All kernels are 
//							  launched on the associated stream.
template<typename InputIt, typename CsrIt, typename OutputIt, typename T,
	typename Op>
MGPU_HOST void SegReduceCsr(InputIt data_global, int count, CsrIt csr_global, 
	int numSegments, bool supportEmpty, OutputIt dest_global, T identity, Op op, 
	CudaContext& context);

// IndirectReduceCsr is like SegReduceCsr but with one level of source 
// indirection. The start of each segment/row i in data_global starts at 
// sources_global[i].
// SourcesIt sources_global	- List of integers for source data of each segment.
//							  Must be numSegments in size.
template<typename InputIt, typename CsrIt, typename SourcesIt, 
	typename OutputIt, typename T, typename Op>
MGPU_HOST void IndirectReduceCsr(InputIt data_global, int count,
	CsrIt csr_global, SourcesIt sources_global, int numSegments,
	bool supportEmpty, OutputIt dest_global, T identity, Op op, 
	CudaContext& context);

// SegReduceCsrPreprocess accelerates multiple seg-reduce calls on different 
// data with the same segment geometry. Partitioning and CSR->CSR2 transform is
// off-loaded to a preprocessing pass. The actual reduction is evaluated by
// SegReduceApply. 
template<typename T, typename CsrIt>
MGPU_HOST void SegReduceCsrPreprocess(int count, CsrIt csr_global, 
	int numSegments, bool supportEmpty, 
	std::auto_ptr<SegReducePreprocessData>* ppData, CudaContext& context);

template<typename InputIt, typename DestIt, typename T, typename Op>
MGPU_HOST void SegReduceApply(const SegReducePreprocessData& preprocess, 
	InputIt data_global, T identity, Op op, DestIt dest_global,
	CudaContext& context);

////////////////////////////////////////////////////////////////////////////////
// kernels/reducebykey.csr

typedef SegReducePreprocessData ReduceByKeyPreprocessData;

// ReduceByKey runs a segmented reduction given a data input and a matching set
// of keys. This implementation requires operators support commutative 
// (a + b = b + a) and associative (a + (b + c) = (a + b) + c) evaluation.
// It roughly matches the behavior of thrust::reduce_by_key.

// KeysIt keys_global		- Key identifier for the segment.
// InputIt data_global		- Data value input.
// int count				- Size of input arrays keys_global and
//							  data_global.
// ValType identity			- Identity for reduction operation. Eg, use 0 for 
//							  addition or 1 for multiplication.
// Op op					- Reduction operator. Model on std::plus<>. MGPU
//							  provides operators mgpu::plus<>, minus<>, 
//							  multiplies<>, modulus<>, bit_or<> bit_and<>,
//							  bit_xor<>, maximum<>, and minimum<>.
// Comp comp				- Operator for comparing adjacent adjacent keys.
//							  Must return true if two adjacent keys are in the 
//							  same segment. Use mgpu::equal_to<KeyType>() by
//							  default.
// KeyType* keysDest_global	- If this pointer is not null, return the first
//							  key from each segment. Must be sized to at least
//							  the number of segments.
// DestIt dest_global		- Holds the reduced data. Must be sized to at least
//							  the number of segments.
// int* count_host			- The number of segments, returned in host memory.
//							  May be null.
// int* count_global		- The number of segments, returned in device memory.
//							  This avoids a D->H synchronization. May be null.
// CudaContext& context		- MGPU context support object.
template<typename KeysIt, typename InputIt, typename DestIt,
	typename KeyType, typename ValType, typename Op, typename Comp>
MGPU_HOST void ReduceByKey(KeysIt keys_global, InputIt data_global, int count,
	ValType identity, Op op, Comp comp, KeyType* keysDest_global, 
	DestIt dest_global, int* count_host, int* count_global, 
	CudaContext& context);

// ReduceByKeyPreprocess accelerates multiple reduce-by-key calls on different
// data with the same segment geometry. The actual reduction is evaluated by
// ReduceByKeyApply.
// Note that the caller must explicitly specify the ValType argument. Kernel 
// tunings are based on the value type, not the key type.
template<typename ValType, typename KeyType, typename KeysIt, typename Comp>
MGPU_HOST void ReduceByKeyPreprocess(int count, KeysIt keys_global, 
	KeyType* keysDest_global, Comp comp, int* count_host, int* count_global,
	std::auto_ptr<ReduceByKeyPreprocessData>* ppData, CudaContext& context);

template<typename InputIt, typename DestIt, typename T, typename Op>
MGPU_HOST void ReduceByKeyApply(const ReduceByKeyPreprocessData& preprocess, 
	InputIt data_global, T identity, Op op, DestIt dest_global,
	CudaContext& context);

////////////////////////////////////////////////////////////////////////////////
// kernels/spmvcsr.cuh

typedef SegReducePreprocessData SpmvPreprocessData;

// SpmvCsr[Unary|Binary] evaluates the product of a sparse matrix (CSR format)
// with a dense vector. 
// SpmvCsrIndirect[Unary|Binary] uses indirection to lookup the start of each
// matrix_global and cols_global on a per-row basis.

// Unary methods reduce on the right-hand side vector values.
// Binary methods reduce the product of the left-hand side matrix value with the
// right-hand side vector values.

// MatrixIt matrix_global	- Left-hand side data for binary Spmv. There are nz
//							  non-zero matrix elements. These are loaded and
//							  combined with the vector values with mulOp.
// ColsIt cols_global		- Row identifiers for the right-hand side of the
//							  matrix/value products. If element i is the k'th
//							  non-zero in row j, the product is formed as
//							      matrix_global[i] * vec_global[cols_global[i]] 
//							  for direct indexing, or,
//							      m = source_global[j] + k
//							      matrix_global[m] * vec_global[cols_global[m]].
// int nz					- Number of non-zeros in LHS matrix. Size of 
//							  matrix_global and cols_global.
// CsrIt csr_global			- List of integers for start of each row. 
//							  The first entry must be 0 (indicating that the 
//							  first row starts at offset 0).
//							  Equivalent to exc-scan of row sizes.
//							  If supportEmpty is false: must be ascending.
//							  If supportEmpty is true: must be non-descending.
// SourcesIt sources_global	- An indirection array to source each row's data.
//							  The size of each row i is
//								   size_i = csr_global[i + 1] - csr_global[i].
//							  The starting offset for both the data and column
//							  identifiers is
//								   offset_i = sources_global[i].
//							  The direct Spmv methods (i.e. those not taking
//							  a sources_global parameter) can be thought of as
//							  indirect methods with sources_global = csr_global.
// int numRows				- Size of segment list csr_global. Must be >= 1.
// VecIt vec_global			- Input array. Size is the width of the matrix.
//							  For unary Spmv, these values are reduced.
//							  For binary Spmv, the products of the matrix and 
//							  vector values are reduced.
// bool supportEmpty		- Basic seg-reduce code does not support empty rows.
//							  Set supportEmpty = true to add pre- and post-
//							  processing to support empty rows.
// DestIt dest_global		- Output array. Must be numRows in size.
// T identity				- Identity for reduction operation. Eg, use 0 for 
//							  addition or 1 for multiplication.
// MulOp mulOp				- Reduction operator for combining matrix value with
//							  vector value. Only defined for binary Spmv.
//							  Use mgpu::multiplies<T>() for default behavior.
// AddOp addOp				- Reduction operator for reducing vector values 
//						      (unary Spmv) or matrix-vector products (binary
//							  Spmv). Use mgpu::plus<T>() for default behavior.
// CudaContext& context		- MGPU context support object. All kernels are 
//							  launched on the associated stream.
template<typename ColsIt, typename CsrIt, typename VecIt, typename DestIt,
	typename T, typename AddOp>
MGPU_HOST void SpmvCsrUnary(ColsIt cols_global, int nz, CsrIt csr_global, 
	int numRows, VecIt vec_global, bool supportEmpty, DestIt dest_global,
	T identity, AddOp addOp, CudaContext& context);

template<typename MatrixIt, typename ColsIt, typename CsrIt, typename VecIt,
	typename DestIt, typename T, typename MulOp, typename AddOp>
MGPU_HOST void SpmvCsrBinary(MatrixIt matrix_global, ColsIt cols_global, 
	int nz, CsrIt csr_global, int numRows, VecIt vec_global, 
	bool supportEmpty, DestIt dest_global, T identity, MulOp mulOp, AddOp addOp, 
	CudaContext& context);

template<typename ColsIt, typename CsrIt, typename SourcesIt, typename VecIt,
	typename DestIt, typename T, typename AddOp>
MGPU_HOST void SpmvCsrIndirectUnary(ColsIt cols_global, int nz, 
	CsrIt csr_global, SourcesIt sources_global, int numRows, VecIt vec_global, 
	bool supportEmpty, DestIt dest_global, T identity, AddOp addOp, 
	CudaContext& context);

template<typename MatrixIt, typename ColsIt, typename CsrIt, typename SourcesIt, 
	typename VecIt, typename DestIt, typename T, typename MulOp, typename AddOp>
MGPU_HOST void SpmvCsrIndirectBinary(MatrixIt matrix_global, ColsIt cols_global,
	int nz, CsrIt csr_global, SourcesIt sources_global, int numRows,
	VecIt vec_global, bool supportEmpty, DestIt dest_global, T identity,
	MulOp mulOp, AddOp addOp, CudaContext& context);

// SpmvPreprocess[Unary|Binary] accelerates multiple Spmv calls on different 
// matrix/vector pairs with the same matrix geometry. The actual reduction is
// evaluated Spmv[Unary|Binary]Apply.
template<typename T, typename CsrIt>
MGPU_HOST void SpmvPreprocessUnary(int nz, CsrIt csr_global, int numRows,
	bool supportEmpty, std::auto_ptr<SpmvPreprocessData>* ppData, 
	CudaContext& context);

template<typename T, typename CsrIt>
MGPU_HOST void SpmvPreprocessBinary(int nz, CsrIt csr_global, int numRows,
	bool supportEmpty, std::auto_ptr<SpmvPreprocessData>* ppData,
	CudaContext& context);

template<typename ColsIt, typename VecIt, typename DestIt, typename T,
	typename MulOp, typename AddOp>
MGPU_HOST void SpmvUnaryApply(const SpmvPreprocessData& preprocess,
	ColsIt cols_global, VecIt vec_global, DestIt dest_global, T identity, 
	AddOp addOp, CudaContext& context);

template<typename MatrixIt, typename ColsIt, typename VecIt, typename DestIt, 
	typename T, typename MulOp, typename AddOp>
MGPU_HOST void SpmvBinaryApply(const SpmvPreprocessData& preprocess,
	MatrixIt matrix_global, ColsIt cols_global, VecIt vec_global, 
	DestIt dest_global, T identity, MulOp mulOp, AddOp addOp,
	CudaContext& context);

} // namespace mgpu

