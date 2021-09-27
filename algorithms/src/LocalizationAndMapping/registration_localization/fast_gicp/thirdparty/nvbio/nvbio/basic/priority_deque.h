/*
 * nvbio
 * Copyright (c) 2011-2014, NVIDIA CORPORATION. All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *    * Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *    * Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *    * Neither the name of the NVIDIA CORPORATION nor the
 *      names of its contributors may be used to endorse or promote products
 *      derived from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NVIDIA CORPORATION BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

//
// This software is a modification of:
//
//   https://github.com/nmcclatchey/Priority-Deque/blob/master/priority_deque.hpp
//
//

/*----------------------------------------------------------------------------*\
|   Copyright (C) 2012-2013 Nathaniel McClatchey                               |
|   Released under the Boost Software License Version 1.0, which may be found  |
| at http://www.boost.org/LICENSE_1_0.txt                                      |
\*----------------------------------------------------------------------------*/

/*! @file priority_deque.h
//    priority_deque.hpp provides the class priority_deque as a thin wrapper
//  around the functions provided by interval_heap.hpp.
//  @remark Exception-safety: If the means of movement -- move, copy, or swap,
//  depending on exception specifications -- do not throw, the guarantee is as
//  strong as that of the action on the underlying container. (Unless otherwise
//  specified)
//  @note Providing a stronger guarantee is impractical.
*/

#pragma once

#include <nvbio/basic/types.h>
#include <nvbio/basic/interval_heap.h>

//  Default comparison (std::less)
#include <functional>
//  Default container (std::vector)
#include <vector>

namespace nvbio {

/// \page priority_deques_page Priority Deques
///
/// This module implements a priority deque adaptor, allowing to push/pop from both ends of the container:
///
/// - priority_deque
///
/// \section ExampleSection Example
///
///\code
/// // build a simple priority_deque over 4 integers
/// typedef vector_view<uint32*>                vector_type;
/// typedef priority_deque<uint32, vector_type> deque_type;
///
/// uint32 deque_storage[4] = { 5, 3, 8, 1 }
///
/// // construct the deque
/// deque_type deque( vector_type( 4u, deque_storage ) );
///
/// // pop from both ends
/// printf( "%u\n", deque.top() );      // -> 8
/// deque.pop_top();
/// printf( "%u\n", deque.bottom() );   // -> 1
/// deque.pop_bottom();
/// printf( "%u\n", deque.top() );      // -> 5
/// deque.pop_top();
///
/// // perhaps push one more item
/// deque.push( 7 );
///
/// // and keep popping
/// printf( "%u\n", deque.bottom() );   // -> 3
/// deque.pop_bottom();
/// printf( "%u\n", deque.bottom() );   // -> 7
/// deque.pop_bottom();
///\endcode
///

///@addtogroup Basic
///@{

///@defgroup PriorityDeques Priority Deques
/// This module implements a priority deque adaptor, allowing to push/pop from both ends of the container.
///@{

//! @brief Efficient double-ended priority queue.
template <typename Type, typename Sequence =std::vector<Type>,
          typename Compare =::std::less<typename Sequence::value_type> >
class priority_deque;

//! @brief Swaps the elements of two priority deques.
template <typename Type, typename Sequence, typename Compare>
void swap (priority_deque<Type, Sequence, Compare>& deque1,
           priority_deque<Type, Sequence, Compare>& deque2);

//----------------------------Priority Deque Class-----------------------------|
/*! @brief Efficient double-ended priority queue.
 *  @author Nathaniel McClatchey
 *  @copyright Boost Software License Version 1.0
 *  @param Type Type of elements in the priority deque.
 *  @param Sequence Underlying sequence container. Must provide random-access
 *  iterators, %front(), %push_back(Type const &), and %pop_back().
 *  Defaults to std::vector<Type>.
 *  @param Compare Comparison class. %Compare(A, B) should return true if %A
 *  should be placed earlier than %B in a strict weak ordering.
 *  Defaults to std::less<Type>, which encapsulates operator<.
 *  @details Priority deques are adaptors, designed to provide efficient
 *  insertion and access to both ends of a weakly-ordered list of elements.
 *  As a container adaptor, priority_deque is implemented on top of another
 *  container type. By default, this is std::vector, but a different container
 *  may be specified explicitly.
 *  Although the priority deque does permit iteration through its elements,
 *  there is no ordering guaranteed, as different implementations may benefit
 *  from different structures, and all iterators should be discarded after using
 *  any function not labeled const.
 *  @note %priority_deque does not provide a stable ordering. If both A<B and
 *  B<A are false, then the order in which they appear may differ from the order
 *  in which they were added to the priority deque.
 *  @remark %priority_deque replicates the interface of the STL
 *  @a priority_queue class template.
 *  @remark %priority_deque is most useful when removals are interspersed with
 *  insertions. If no further insertions are to be performed after the first
 *  removal, consider using an array and a sorting algorithm instead.
 *  @remark %priority_deque sorts elements as they are added, removed, and
 *  modified by its member functions. If the elements are modified by some means
 *  other than the public member functions, the order must be restoreed before
 *  the priority_deque is used.
 *  @see priority_queue
 */
template <typename Type, typename Sequence, typename Compare>
class priority_deque {
//----------------------------------Public-------------------------------------|
 public:
//---------------------------------Typedefs------------------------------------|
//! @details Underlying container type.
  typedef Sequence                                    container_type;
  typedef typename container_type::value_type         value_type;
  typedef Compare                                     value_compare;
//! @details STL Container specifies that this is an unsigned integral type.
  typedef typename container_type::size_type          size_type;
//! @details May be used to examine, but not modify, an element in the deque.
  typedef typename container_type::const_reference    const_reference;
  typedef typename container_type::reference          reference;
  typedef typename container_type::pointer            pointer;
  typedef pointer                                     const_pointer;
//! @details May be used to examine, but not modify, elements in the deque.
  typedef typename container_type::const_iterator     const_iterator;
  typedef const_iterator                              iterator;
//! @details STL Container specifies that this is a signed integral type.
  typedef typename container_type::difference_type    difference_type;
//-------------------------------Constructors----------------------------------|
  enum Constructed { CONSTRUCTED };

//! @brief Constructs a new priority deque.
  NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
  explicit priority_deque             (const Compare&  =Compare(),
                                       const Sequence& =Sequence());

  template <typename InputIterator>
  NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
  priority_deque                      (InputIterator first, InputIterator last,
                                       const Compare&  =Compare(),
                                       const Sequence& =Sequence());

  /// O(1)  Creates a new priority deque from an already heapified container
  ///
  NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
  priority_deque(const Sequence& seq, const bool constructed = false);

  /// O(1)  Creates a new priority deque from an already heapified container
  ///
  NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
  priority_deque(const Sequence& seq, const Constructed flag) :
      sequence_(seq) {}

//-----------------------------Restricted Access-------------------------------|
//! @brief Copies an element into the priority deque.
  NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
  void                    push        (const value_type&);

//!@{
//! @brief Accesses a maximal element in the deque.
  NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
  const_reference         maximum     (void) const;
//! @brief Accesses a minimal element in the deque.
  NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
  const_reference         minimum     (void) const;
//! @details Identical to std::priority_queue top(). @see @a maximum
  NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
  const_reference  top         (void) const  { return maximum(); };
//! @details Identical to std::priority_queue top(). @see @a maximum
  NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
  const_reference  bottom      (void) const  { return bottom(); };
//!@}
//!@{
//! @brief Removes a maximal element from the deque.
  NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
  void                    pop_top    (void);
//! @brief Removes a minimal element from the deque.
  NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
  void                    pop_bottom (void);
//! @details Identical to std::priority_queue pop(). @see @a pop_maximum
  NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
  void                    pop        (void)        { pop_top(); };
//!@}

//--------------------------------Deque Size-----------------------------------|
//!@{
//! @brief Returns true if the priority deque is empty, false if it is not.
  NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
  bool                    empty       (void) const  {return sequence_.empty();};
//! @brief Returns the number of elements in the priority deque.
  NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
  size_type               size        (void) const  {return sequence_.size(); };
//! @brief Returns the maximum number of elements that can be contained.
  NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
  size_type               max_size    (void) const  {return sequence_.max_size();};
//!@}

//--------------------------Whole-Deque Operations-----------------------------|
//! @brief Removes all elements from the priority deque.
  NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
  void                    clear       (void)        { sequence_.clear();      };
//! @brief Moves the elements from this deque into another, and vice-versa.
  NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
  void                    swap        (priority_deque<Type, Sequence,Compare>&);
//!@{
//! @brief Merges a sequence of elements into the priority deque.
  template <typename InputIterator>
  NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
  void                    merge       (InputIterator first, InputIterator last);
//! @brief Merges a container's elements into the priority deque.
  template <typename SourceContainer>
  NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
  void                    merge       (const SourceContainer& source) { merge(source.begin(), source.end()); }
//!@}

//-------------------------------Random Access---------------------------------|
//!@{
//! @brief Returns a const iterator at the beginning of the sequence.
  NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
  const_iterator          begin       (void) const  {return sequence_.begin();};
//! @brief Returns a const iterator past the end of the sequence.
  NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
  const_iterator          end         (void) const  { return sequence_.end(); };
//! @brief Modifies a specified element of the deque.
  NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
  void                    update      (const_iterator, const value_type&);
//! @brief Removes a specified element from the deque.
  NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
  void                    erase       (const_iterator);
//!@}

//---------------------------Boost.Heap Concepts-------------------------------|
// size() has constant complexity
  static const bool constant_time_size    = true;
// priority deque does not have ordered iterators
  static const bool has_ordered_iterators = false;
// priority deque is efficiently mergable
  static const bool is_mergable           = true;
// priority deque does not have a stable heap order
  static const bool is_stable             = false;
// priority deque does not have a reserve() member
  static const bool has_reserve           = false;

//--------------------------------Protected------------------------------------|
 protected:
  NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
  container_type&        sequence  (void)        { return sequence_; };
  NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
  const container_type&  sequence  (void) const  { return sequence_; };
  NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
  value_compare&         compare   (void)        { return compare_; };
  NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
  const value_compare&   compare   (void) const  { return compare_; };
//---------------------------------Private-------------------------------------|
 private:
  Sequence sequence_;
  Compare  compare_;
};

//-------------------------------Constructors----------------------------------|
//----------------------------Default Constructor------------------------------|
/** @param comp Comparison class.
//  @param seq Container class.
//  @post Deque contains copies of all elements from @a sequence.
//
//  @remark Complexity: O(n)
*/
template <typename T, typename S, typename C>
priority_deque<T, S, C>::priority_deque (const C& comp, const S& seq)
  : sequence_(seq), compare_(comp)
{
  heap::make_interval_heap(sequence_.begin(), sequence_.end(), compare_);
}

template <typename T, typename S, typename C>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
priority_deque<T, S, C>::priority_deque(const S& seq, const bool constructed)
  : sequence_(seq)
{
  if (!constructed)
    heap::make_interval_heap(sequence_.begin(), sequence_.end(), compare_);
}

//---------------------------Create from Iterators-----------------------------|
/** @param first,last Range of elements.
//  @param comp Instance of comparison class.
//  @param seq Instance of container class.
//  @post Deque contains copies of all elements in @a sequence (if specified)
//  and in the range [ @a first, @a last).
//
//  @remark Complexity: O(n)
*/
template <typename T, typename S, typename C>
template <typename InputIterator>
priority_deque<T, S, C>::priority_deque (InputIterator first,InputIterator last,
                                         const C& comp, const S& seq)
: sequence_(seq), compare_(comp)
{
    sequence_.insert(sequence_.end(), first, last);
    heap::make_interval_heap(sequence_.begin(), sequence_.end(), compare_);
}

//-----------------------------Restricted Access-------------------------------|
//------------------------------Insert / Emplace-------------------------------|
/** @param  value Element to add the the priority deque.
//  @post Priority deque contains @a value or a copy of @a value.
//  @post All iterators and references are invalidated.
//
//  @remark Complexity: O(log n)
*/
template <typename T, typename Sequence, typename Compare>
void priority_deque<T, Sequence, Compare>::push (const value_type& value) {
  sequence_.push_back(value);
  heap::push_interval_heap(sequence_.begin(), sequence_.end(), compare_);
}

//---------------------------Observe Maximum/Minimum---------------------------|
/** @return Const reference to a maximal element in the priority deque.
//  @pre  Priority deque contains one or more elements.
//  @see  minimum, pop_maximum
//
//  @remark Complexity: O(1)
*/
template <typename T, typename Sequence, typename Compare>
typename priority_deque<T, Sequence, Compare>::const_reference
  priority_deque<T, Sequence, Compare>::maximum  (void) const
{
  NVBIO_CUDA_DEBUG_ASSERT(!empty(),
    "Empty priority deque has no maximal element. Reference undefined.");
  const_iterator it = sequence_.begin() + 1;
  return (it == sequence_.end()) ? sequence_.front() : *it;
}
/** @return Const reference to a minimal element in the priority deque.
//  @pre  Priority deque contains one or more elements.
//  @see  maximum, pop_minimum
//
//  @remark Complexity: O(1)
*/
template <typename T, typename Sequence, typename Compare>
typename priority_deque<T, Sequence, Compare>::const_reference
  priority_deque<T, Sequence, Compare>::minimum  (void) const
{
  NVBIO_CUDA_DEBUG_ASSERT(!empty(),
    "Empty priority deque has no minimal element. Reference undefined.");
  return sequence_.front();
}
//---------------------------Remove Maximum/Minimum----------------------------|
/** @pre  Priority deque contains one or more elements.
//  @post A maximal element has been removed from the priority deque.
//  @post All iterators and references are invalidated.
//  @see  maximum, pop, pop_minimum
//
//  @remark Complexity: O(log n)
*/
template <typename T, typename Sequence, typename Compare>
void priority_deque<T, Sequence, Compare>::pop_bottom (void) {
  NVBIO_CUDA_DEBUG_ASSERT(!empty(),
    "Empty priority deque has no maximal element. Removal impossible.");
  heap::pop_interval_heap_min(sequence_.begin(), sequence_.end(), compare_);
  sequence_.pop_back();
}
/** @pre  Priority deque contains one or more elements.
//  @post A minimal element has been removed from the priority deque.
//  @post All iterators and references are invalidated.
//  @see  minimum, pop_maximum
//
//  @remark Complexity: O(log n)
*/
template <typename T, typename Sequence, typename Compare>
void priority_deque<T, Sequence, Compare>::pop_top (void) {
  NVBIO_CUDA_DEBUG_ASSERT(!empty(),
    "Empty priority deque has no minimal element. Removal undefined.");
  heap::pop_interval_heap_max(sequence_.begin(), sequence_.end(), compare_);
  sequence_.pop_back();
}

//--------------------------Whole-Deque Operations-----------------------------|
//-----------------------------------Merge-------------------------------------|
/** @param first,last Input iterators bounding the range [ @a first, @a last)
//  @post Priority deque contains its original elements, and copies of those in
//  the range.
//  @post All iterators and references are invalidated.
//
//  @remark Complexity: O(n)
//  @remark Exception safety: Basic.
*/
template <typename T, typename S, typename C>
template <typename InputIterator>
void priority_deque<T, S, C>::merge (InputIterator first, InputIterator last) {
  sequence_.insert(sequence_.end(), first, last);
  heap::make_interval_heap(sequence_.begin(), sequence_.end(), compare_);
}

//----------------------------Swap Specialization------------------------------|
/** @param other Priority deque with which to swap.
//  @post Deque contains the elements from @a source, and @a source contains the
//  elements from this deque.
//  @post All iterators and references are invalidated.
//  @note Sequence containers are required to have swap functions.
//  @remark Complexity: O(1)
*/
template <typename T, typename S, typename C>
void priority_deque<T, S, C>::swap (priority_deque<T, S, C>& other) {
  sequence_.swap(other.sequence_);
}

/** @relates priority_deque
//  @param deque1,deque2 Priority deques.
//  @post @a deque1 contains the elements originally in @a deque2, and @a deque2
//  contains the elements originally in @a deque1
//  @post All iterators and references are invalidated.
//
//  @remark Complexity: O(1)
*/
template <typename T, typename S, typename C>
void swap (priority_deque<T, S, C>& deque1,
                  priority_deque<T, S, C>& deque2)
{
  deque1.swap(deque2);
}

//---------------------------Random-Access Mutators----------------------------|
/** @param  random_it A valid iterator in the range [begin, end).
//  @param  value The new value.
//  @pre  Priority deque contains one or more elements.
//  @post The element at @a random_it is set to @a value.
//  @post All iterators and references are invalidated.
//  @see erase
//
//  Elements within the deque may be unordered.
//  @remark Complexity: O(log n)
//  @remark Exception safety: Basic. Exceptions won't lose elements, but may
//  corrupt the heap.
*/
template <typename T, typename S, typename C>
void priority_deque<T, S, C>::update (const_iterator random_it,
                                      const value_type& value)
{
  const difference_type index = random_it - begin();
  NVBIO_CUDA_DEBUG_ASSERT((0 <= index) &&
    (index < end() - begin()), "Iterator out of bounds; can't set element.");
//  Providing the strong guarantee would require saving a copy.
  *(sequence_.begin() + index) = value;
  heap::update_interval_heap(sequence_.begin(),sequence_.end(),index,compare_);
}

/** @param  random_it  An iterator in the range [begin, end)
//  @pre  Priority deque contains one or more elements.
//  @post The deque no longer contains the element previously at @a random_it.
//  @post All iterators and references are invalidated.
//  @see set
//  @remark Complexity: O(log n)
*/
template <typename T, typename Sequence, typename Compare>
void priority_deque<T, Sequence, Compare>::erase (const_iterator random_it) {
  const difference_type index = random_it - begin();
  NVBIO_CUDA_DEBUG_ASSERT((0 <= index) &&
    (index < end() - begin()), "Iterator out of bounds; can't erase element.");
  heap::pop_interval_heap(sequence_.begin(), sequence_.end(),index,compare_);
  sequence_.pop_back();
}

///@} PriorityDeques
///@} Basic

} // namespace nvbio
