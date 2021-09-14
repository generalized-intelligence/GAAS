#ifndef PRIORITY_DEQUE_VERIFY_HPP_
#define PRIORITY_DEQUE_VERIFY_HPP_

#include "priority_deque.hpp"
#include "interval_heap.hpp"

//    Thouroughly checks a priority deque for heap corruption and returns the
//  first instance of corruption (or the end iterator if there is none).
//    Note: Assumes comparison object is default-constructed.
template <typename T, typename S, typename C>
typename boost::container::priority_deque<T, S, C>::const_iterator
  is_valid_until (const boost::container::priority_deque<T, S, C>& pd)
{
  C compare;
  return boost::heap::is_interval_heap_until(pd.begin(), pd.end(), compare);
}



#endif // PRIORITY_DEQUE_VERIFY_HPP_
