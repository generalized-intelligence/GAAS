/*
 * divsufsortxx.h
 * Copyright (c) 2003-2008 Yuta Mori All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use,
 * copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following
 * conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 */

// r3

#ifndef _DIVSUFSORTXX_H_
#define _DIVSUFSORTXX_H_

#include <algorithm>
#include <stack>


namespace divsufsortxx {

namespace helper {

/* Insertionsort for small size groups. */
template<typename ISAIterator_type, typename SAIterator_type>
void
insertionsort(const ISAIterator_type ISAd,
              SAIterator_type first, SAIterator_type last) {
typedef typename std::iterator_traits<SAIterator_type>::value_type pos_type;
typedef typename std::iterator_traits<ISAIterator_type>::value_type value_type;
  SAIterator_type a, b;
  pos_type t;
  value_type v, x;

  for(a = first + 1; a < last; ++a) {
    for(v = ISAd[t = *a], b = a; v < (x = ISAd[*(b - 1)]);) {
      do { *b = *(b - 1); } while((first < --b) && (*(b - 1) < 0));
      if(b <= first) { break; }
    }
    if(v == x) { *(b - 1) = ~*(b - 1); }
    *b = t;
  }
}


/*---------------------------------------------------------------------------*/

/* Heapsort */

template<typename StringIterator_type, typename SAIterator_type, typename pos_type>
void
fixdown(const StringIterator_type Td, SAIterator_type SA, pos_type i, pos_type size) {
typedef typename std::iterator_traits<StringIterator_type>::value_type value_type;
  pos_type j, k, t;
  value_type c, d, e;

  for(t = SA[i], c = Td[t]; (j = 2 * i + 1) < size; SA[i] = SA[k], i = k) {
    d = Td[SA[k = j++]];
    if(d < (e = Td[SA[j]])) { k = j; d = e; }
    if(d <= c) { break; }
  }
  SA[i] = t;
}

/* Simple top-down heapsort. */
template<typename StringIterator_type, typename SAIterator_type>
void
heapsort(const StringIterator_type Td, SAIterator_type SA,
         typename std::iterator_traits<SAIterator_type>::value_type size) {
typedef typename std::iterator_traits<SAIterator_type>::value_type pos_type;
  pos_type i, m;
  pos_type t;

  m = size;
  if((size % 2) == 0) {
    m--;
    if(Td[SA[m / 2]] < Td[SA[m]]) {
      std::iter_swap(SA + m, SA + m / 2);
    }
  }

  for(i = m / 2 - 1; 0 <= i; --i) {
    fixdown(Td, SA, i, m);
  }

  if((size % 2) == 0) {
    std::iter_swap(SA, SA + m);
    fixdown(Td, SA, pos_type(0), m);
  }

  for(i = m - 1; 0 < i; --i) {
    t = SA[0];
    SA[0] = SA[i];
    fixdown(Td, SA, pos_type(0), i);
    SA[i] = t;
  }
}


/*---------------------------------------------------------------------------*/

static
const int
log2table[256]= {
 -1, 0, 1, 1, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3,
  4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4,
  5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5,
  5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5,
  6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6,
  6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6,
  6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6,
  6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6,
  7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7,
  7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7,
  7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7,
  7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7,
  7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7,
  7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7,
  7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7,
  7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7
};

template<typename numeric_type>
int
lg(numeric_type n) {
  int lgsize = 0;
  while(255 < n) { lgsize += 8, n >>= 8; }
  return lgsize + log2table[n];
}

template<typename Iterator1_type, typename Iterator2_type>
void
vecswap(Iterator1_type first1, Iterator1_type last1, Iterator2_type first2) {
  for(; first1 != last1; ++first1, ++first2) {
    std::iter_swap(first1, first2);
  }
//  std::swap_ranges(first1, last1, first2);
}


/*---------------------------------------------------------------------------*/

/* Returns the median of three elements. */
template<typename StringIterator_type, typename SAIterator_type>
SAIterator_type
median3(const StringIterator_type Td,
        SAIterator_type v1, SAIterator_type v2, SAIterator_type v3) {
  if(Td[*v1] > Td[*v2]) { std::swap(v1, v2); }
  if(Td[*v2] > Td[*v3]) {
    if(Td[*v1] > Td[*v3]) { return v1; }
    else { return v3; }
  }
  return v2;
}

/* Returns the median of five elements. */
template<typename StringIterator_type, typename SAIterator_type>
SAIterator_type
median5(const StringIterator_type Td,
        SAIterator_type v1, SAIterator_type v2, SAIterator_type v3,
        SAIterator_type v4, SAIterator_type v5) {
  if(Td[*v2] > Td[*v3]) { std::swap(v2, v3); }
  if(Td[*v4] > Td[*v5]) { std::swap(v4, v5); }
  if(Td[*v2] > Td[*v4]) { std::swap(v2, v4); std::swap(v3, v5); }
  if(Td[*v1] > Td[*v3]) { std::swap(v1, v3); }
  if(Td[*v1] > Td[*v4]) { std::swap(v1, v4); std::swap(v3, v5); }
  if(Td[*v3] > Td[*v4]) { return v4; }
  return v3;
}

/* Returns the pivot element. */
template<typename StringIterator_type, typename SAIterator_type>
SAIterator_type
pivot(const StringIterator_type Td,
      SAIterator_type first, SAIterator_type last) {
typedef typename std::iterator_traits<SAIterator_type>::difference_type difference_type;
  difference_type t = last - first;
  SAIterator_type middle = first + t / 2;

  if(t <= 512) {
    if(t <= 32) {
      return median3(Td, first, middle, last - 1);
    } else {
      t >>= 2;
      return median5(Td,
                     first, first + t, middle,
                     last - 1 - t, last - 1);
    }
  }
  t >>= 3;
  return median3(Td,
           median3(Td, first, first + t, first + (t << 1)),
           median3(Td, middle - t, middle, middle + t),
           median3(Td, last - 1 - (t << 1), last - 1 - t, last - 1));
}

/* Two-stage double-index controlled ternary partition */
template<typename StringIterator_type, typename SAIterator_type>
bool
partition(const StringIterator_type Td,
          SAIterator_type first1, SAIterator_type first2, SAIterator_type last,
          SAIterator_type &mfirst, SAIterator_type &mlast,
          const typename std::iterator_traits<StringIterator_type>::value_type &v) {
  SAIterator_type a, b, c, d;
  typename std::iterator_traits<StringIterator_type>::value_type x = 0;

  for(b = first2; (b < last) && ((x = Td[*b]) == v); ++b) { }
  if(((a = b) < last) && (x < v)) {
    for(; (++b < last) && ((x = Td[*b]) <= v);) {
      if(x == v) { std::iter_swap(b, a++); }
    }
  }
  for(c = last; (b < --c) && ((x = Td[*c]) == v);) { }
  if((b < (d = c)) && (x > v)) {
    for(; (b < --c) && ((x = Td[*c]) >= v);) {
      if(x == v) { std::iter_swap(c, d--); }
    }
  }
  for(; b < c;) {
    std::iter_swap(b, c);
    for(; (++b < c) && ((x = Td[*b]) <= v);) {
      if(x == v) { std::iter_swap(b, a++); }
    }
    for(; (b < --c) && ((x = Td[*c]) >= v);) {
      if(x == v) { std::iter_swap(c, d--); }
    }
  }

  if(a <= d) {
    vecswap(b - std::min(a - first1, b - a), b, first1);
    vecswap(last - std::min(d + 1 - b, last - d - 1), last, b);
    mfirst = first1 + (b - a), mlast = last - (d + 1 - b);
    return true;
  }
  mfirst = first1, mlast = last;
  return false;
}


/*---------------------------------------------------------------------------*/

template<typename a_type, typename b_type, typename c_type>
struct stackinfo3 {
  a_type m_a;
  b_type m_b;
  c_type m_c;
  stackinfo3(a_type a, b_type b, c_type c) : m_a(a), m_b(b), m_c(c) { }
};

template<typename a_type, typename b_type, typename c_type, typename d_type>
struct stackinfo4 {
  a_type m_a;
  b_type m_b;
  c_type m_c;
  d_type m_d;
  stackinfo4(a_type a, b_type b, c_type c, d_type d) : m_a(a), m_b(b), m_c(c), m_d(d) { }
};

} /* namespace helper */

#define STACK_POP3(_a, _b, _c)\
  {\
    if(stack.empty()) { return; }\
    stackinfo_type tempinfo = stack.top();\
    (_a) = tempinfo.m_a, (_b) = tempinfo.m_b, (_c) = tempinfo.m_c;\
    stack.pop();\
  }
#define STACK_POP4(_a, _b, _c, _d)\
  {\
    if(stack.empty()) { return; }\
    stackinfo_type tempinfo = stack.top();\
    (_a) = tempinfo.m_a, (_b) = tempinfo.m_b, (_c) = tempinfo.m_c, (_d) = tempinfo.m_d;\
    stack.pop();\
  }
#define STACK_PUSH3(_a, _b, _c)\
  stack.push(stackinfo_type((_a), (_b), (_c)))
#define STACK_PUSH4(_a, _b, _c, _d)\
  stack.push(stackinfo_type((_a), (_b), (_c), (_d)))


namespace substring {

template<typename StringIterator_type, typename SAIterator_type>
int
compare(StringIterator_type T,
        const SAIterator_type p1, const SAIterator_type p2,
        typename std::iterator_traits<SAIterator_type>::value_type depth) {
  StringIterator_type U1 = T + depth + *p1,
                      U2 = T + depth + *p2,
                      U1n = T + *(p1 + 1) + 2,
                      U2n = T + *(p2 + 1) + 2;
  for(; (U1 < U1n) && (U2 < U2n) && (*U1 == *U2); ++U1, ++U2) { }

  return U1 < U1n ?
        (U2 < U2n ? (*U2 < *U1) * 2 - 1 : 1) :
//        (U2 < U2n ? *U1 - *U2 : 1) :
        (U2 < U2n ? -1 : 0);
}

template<typename StringIterator_type, typename SAIterator_type>
int
compare_last(StringIterator_type T,
             const SAIterator_type p1, const SAIterator_type p2,
             typename std::iterator_traits<SAIterator_type>::value_type depth,
             typename std::iterator_traits<SAIterator_type>::value_type size) {
  StringIterator_type U1 = T + depth + *p1,
                      U2 = T + depth + *p2,
                      U1n = T + size,
                      U2n = T + *(p2 + 1) + 2;
  for(; (U1 < U1n) && (U2 < U2n) && (*U1 == *U2); ++U1, ++U2) { }

  return U1 < U1n ?
        (U2 < U2n ? (*U2 < *U1) * 2 - 1 : 1) :
//        (U2 < U2n ? *U1 - *U2 : 1) :
        (U2 < U2n ? -1 : 0);
}

template<typename StringIterator_type, typename SAIterator_type>
void
insertionsort(StringIterator_type T, const SAIterator_type PA,
              SAIterator_type first, SAIterator_type last,
              typename std::iterator_traits<SAIterator_type>::value_type depth) {
  SAIterator_type a, b;
  typename std::iterator_traits<SAIterator_type>::value_type t;
  int r;

  for(a = last - 1; first < a; --a) {
    for(t = *(a - 1), b = a; 0 < (r = compare(T, PA + t, PA + *b, depth));) {
      do { *(b - 1) = *b; } while((++b < last) && (*b < 0));
      if(last <= b) { break; }
    }
    if(r == 0) { *b = ~*b; }
    *(b - 1) = t;
  }
}

template<typename StringIterator_type, typename PAIterator_type>
class substring_wrapper {
private:
  StringIterator_type m_Td;
  const PAIterator_type m_PA;
public:
  typedef typename std::iterator_traits<StringIterator_type>::value_type value_type;
  typedef value_type * pointer;
  typedef value_type & reference;
  typedef ptrdiff_t difference_type;
  typedef std::random_access_iterator_tag iterator_category;

  typedef size_t size_type;

  substring_wrapper(StringIterator_type Td, const PAIterator_type PA) : m_Td(Td), m_PA(PA) {
  }

  value_type
  operator[](size_type i) const {
    return m_Td[m_PA[i]];
  }

  substring_wrapper<StringIterator_type, PAIterator_type> &
  operator=(const substring_wrapper<StringIterator_type, PAIterator_type> &wrapper) {
    return *this;
  }
};

template<typename SAIterator_type>
SAIterator_type
partition(const SAIterator_type PA,
          SAIterator_type first, SAIterator_type last,
          typename std::iterator_traits<SAIterator_type>::value_type depth) {
  SAIterator_type a, b;
  for(a = first, b = last - 1;; ++a, --b) {
    for(; (a <= b) && ((PA[*a] + depth) >= (PA[*a + 1] + 1)); ++a) { *a = ~*a; }
    for(; (a < b) && ((PA[*b] + depth) <  (PA[*b + 1] + 1)); --b) { }
    if(b <= a) { break; }
    std::iter_swap(b, a);
    *a = ~*a;
  }
  if(first < a) { *first = ~*first; }
  return a;
}

template<typename stack_type, typename StringIterator_type, typename SAIterator_type>
void
mintrosort(stack_type &stack, const StringIterator_type T, const SAIterator_type PA,
           SAIterator_type first, SAIterator_type last,
           typename std::iterator_traits<SAIterator_type>::value_type depth) {
typedef typename std::iterator_traits<SAIterator_type>::value_type pos_type;
typedef typename std::iterator_traits<StringIterator_type>::value_type value_type;
typedef substring_wrapper<StringIterator_type, SAIterator_type> wrapper_type;
typedef typename stack_type::value_type stackinfo_type;

  SAIterator_type a, b, c;
  value_type v, x;
  int limit;

  for(limit = helper::lg(last - first);;) {

    if((last - first) <= 8) {
      if(1 < (last - first)) { insertionsort(T, PA, first, last, depth); }
      STACK_POP4(first, last, depth, limit);
      continue;
    }

    const StringIterator_type Td = T + depth;
    if(limit-- == 0) { helper::heapsort(wrapper_type(Td, PA), first, value_type( last - first )); }
    if(limit < 0) {
      for(a = first + 1, v = Td[PA[*first]]; a < last; ++a) {
        if((x = Td[PA[*a]]) != v) {
          if(1 < (a - first)) { break; }
          v = x;
          first = a;
        }
      }
      if(Td[PA[*first] - 1] < v) {
        first = partition(PA, first, a, depth);
      }
      if((a - first) <= (last - a)) {
        if(1 < (a - first)) {
          STACK_PUSH4(a, last, depth, -1);
          last = a, depth += 1, limit = helper::lg(a - first);
        } else {
          first = a, limit = -1;
        }
      } else {
        if(1 < (last - a)) {
          STACK_PUSH4(first, a, depth + 1, helper::lg(a - first));
          first = a, limit = -1;
        } else {
          last = a, depth += 1, limit = helper::lg(a - first);
        }
      }
      continue;
    }

    /* partition */
    a = helper::pivot(wrapper_type(Td, PA), first, last);
    std::iter_swap(first, a);
    if(helper::partition(wrapper_type(Td, PA), first, first + 1, last, a, c, Td[PA[*first]]) != false) {
      b = (Td[PA[*a]] <= Td[PA[*a] - 1]) ? a : partition(PA, a, c, depth);

      if((a - first) <= (last - c)) {
        if((last - c) <= (c - b)) {
          STACK_PUSH4(b, c, depth + 1, helper::lg(c - b));
          STACK_PUSH4(c, last, depth, limit);
          last = a;
        } else if((a - first) <= (c - b)) {
          STACK_PUSH4(c, last, depth, limit);
          STACK_PUSH4(b, c, depth + 1, helper::lg(c - b));
          last = a;
        } else {
          STACK_PUSH4(c, last, depth, limit);
          STACK_PUSH4(first, a, depth, limit);
          first = b, last = c, depth += 1, limit = helper::lg(c - b);
        }
      } else {
        if((a - first) <= (c - b)) {
          STACK_PUSH4(b, c, depth + 1, helper::lg(c - b));
          STACK_PUSH4(first, a, depth, limit);
          first = c;
        } else if((last - c) <= (c - b)) {
          STACK_PUSH4(first, a, depth, limit);
          STACK_PUSH4(b, c, depth + 1, helper::lg(c - b));
          first = c;
        } else {
          STACK_PUSH4(first, a, depth, limit);
          STACK_PUSH4(c, last, depth, limit);
          first = b, last = c, depth += 1, limit = helper::lg(c - b);
        }
      }
    } else {
      limit += 1;
      if(Td[PA[*first] - 1] < Td[PA[*first]]) {
        first = partition(PA, first, last, depth);
        limit = helper::lg(last - first);
      }
      depth += 1;
    }
  }
}

/* Merge-forward with internal buffer. */
template<typename StringIterator_type, typename SAIterator_type, typename BufIterator_type>
void
merge_forward(const StringIterator_type T,
              const SAIterator_type PA, BufIterator_type buf,
              SAIterator_type first, SAIterator_type middle, SAIterator_type last,
              typename std::iterator_traits<SAIterator_type>::value_type depth) {
  SAIterator_type i, k;
  BufIterator_type j, bufend;
  typename std::iterator_traits<SAIterator_type>::value_type t;
  int r;

  bufend = buf + (middle - first);
  helper::vecswap(first, middle, buf);

  for(t = *first, i = first, j = buf, k = middle;;) {
    r = compare(T, PA + *j, PA + *k, depth);
    if(r < 0) {
      do {
        *i++ = *j; *j++ = *i;
        if(bufend <= j) { *(bufend - 1) = t; return; }
      } while(*j < 0);
    } else if(r > 0) {
      do {
        *i++ = *k; *k++ = *i;
        if(last <= k) {
          do { *i++ = *j; *j++ = *i; } while(j < bufend);
          *(bufend - 1) = t;
          return;
        }
      } while(*k < 0);
    } else {
      *k = ~*k;
      do {
        *i++ = *j; *j++ = *i;
        if(bufend <= j) { *(bufend - 1) = t; return; }
      } while(*j < 0);

      do {
        *i++ = *k; *k++ = *i;
        if(last <= k) {
          do { *i++ = *j; *j++ = *i; } while(j < bufend);
          *(bufend - 1) = t;
          return;
        }
      } while(*k < 0);
    }
  }
}

/* Merge-backward with internal buffer. */
template<typename StringIterator_type, typename SAIterator_type, typename BufIterator_type>
void
merge_backward(const StringIterator_type T,
               const SAIterator_type PA, BufIterator_type buf,
               SAIterator_type first, SAIterator_type middle, SAIterator_type last,
               typename std::iterator_traits<SAIterator_type>::value_type depth) {
  SAIterator_type p1, p2;
  SAIterator_type i, k;
  BufIterator_type j, bufend;
  typename std::iterator_traits<SAIterator_type>::value_type t;
  int r, x;

  bufend = buf + (last - middle);
  helper::vecswap(middle, last, buf);

  x = 0;
  if(*(bufend - 1) < 0) { x |=  1; p1 = PA + ~*(bufend - 1); }
  else                  {          p1 = PA +  *(bufend - 1); }
  if(*(middle - 1) < 0) { x |=  2; p2 = PA + ~*(middle - 1); }
  else                  {          p2 = PA +  *(middle - 1); }
  for(t = *(last - 1), i = last - 1, j = bufend - 1, k = middle - 1;;) {

    r = compare(T, p1, p2, depth);
    if(r > 0) {
      if(x & 1) { do { *i-- = *j; *j-- = *i; } while(*j < 0); x ^= 1; }
      *i-- = *j; *j = *i;
      if(j <= buf) { *buf = t; return; }

      if(*--j < 0) { x |=  1; p1 = PA + ~*j; }
      else         {          p1 = PA +  *j; }
    } else if(r < 0) {
      if(x & 2) { do { *i-- = *k; *k-- = *i; } while(*k < 0); x ^= 2; }
      *i-- = *k; *k = *i;
      if(k <= first) {
        while(buf < j) { *i-- = *j; *j-- = *i; }
        *i = *j, *buf = t;
        return;
      }

      if(*--k < 0) { x |=  2; p2 = PA + ~*k; }
      else         {          p2 = PA +  *k; }
    } else {
      if(x & 1) { do { *i-- = *j; *j-- = *i; } while(*j < 0); x ^= 1; }
      *i-- = ~*j; *j = *i;
      if(j <= buf) { *buf = t; return; }
      --j;

      if(x & 2) { do { *i-- = *k; *k-- = *i; } while(*k < 0); x ^= 2; }
      *i-- = *k; *k = *i;
      if(k <= first) {
        while(buf < j) { *i-- = *j; *j-- = *i; }
        *i = *j, *buf = t;
        return;
      }
      --k;

      if(*j < 0) { x |=  1; p1 = PA + ~*j; }
      else       {          p1 = PA +  *j; }
      if(*k < 0) { x |=  2; p2 = PA + ~*k; }
      else       {          p2 = PA +  *k; }
    }
  }
}

/* Faster merge (based on divide and conquer technique). */
template<typename stack_type, typename StringIterator_type, typename SAIterator_type, typename BufIterator_type>
void
merge(stack_type &stack, const StringIterator_type T, const SAIterator_type PA,
      SAIterator_type first, SAIterator_type middle, SAIterator_type last,
      BufIterator_type buf, typename std::iterator_traits<SAIterator_type>::value_type bufsize,
      typename std::iterator_traits<SAIterator_type>::value_type depth) {
typedef typename std::iterator_traits<SAIterator_type>::difference_type difference_type;
typedef typename stack_type::value_type stackinfo_type;
#define GETIDX(a) ((0 <= (a)) ? (a) : (~(a)))
#define MERGE_CHECK(a)\
  {\
    if((0 <= *(a)) &&\
       (compare(T, PA + GETIDX(*((a) - 1)), PA + *(a), depth) == 0)) {\
      *(a) = ~*(a);\
    }\
  }

  SAIterator_type i, j;
  difference_type m, len, half;
  int check, next;

  for(check = 0;;) {

    if((last - middle) <= bufsize) {
      if((first < middle) && (middle < last)) {
        merge_backward(T, PA, buf, first, middle, last, depth);
      }
      if(check & 1) { MERGE_CHECK(first); }
      if(check & 2) { MERGE_CHECK(last); }
      STACK_POP4(first, middle, last, check);
      continue;
    }

    if((middle - first) <= bufsize) {
      if(first < middle) {
        merge_forward(T, PA, buf, first, middle, last, depth);
      }
      if(check & 1) { MERGE_CHECK(first); }
      if(check & 2) { MERGE_CHECK(last); }
      STACK_POP4(first, middle, last, check);
      continue;
    }

    for(m = 0, len = std::min(middle - first, last - middle), half = len >> 1;
        0 < len;
        len = half, half >>= 1) {
      if(compare(T, PA + GETIDX(*(middle + m + half)),
                    PA + GETIDX(*(middle - m - half - 1)), depth) < 0) {
        m += half + 1;
        half -= ((len & 1) == 0);
      }
    }

    if(0 < m) {
      helper::vecswap(middle - m, middle, middle);
      i = j = middle, next = 0;
      if((middle + m) < last) {
        if(*(middle + m) < 0) {
          for(; *(i - 1) < 0; --i) { }
          *(middle + m) = ~*(middle + m);
        }
        for(j = middle; *j < 0; ++j) { }
        next = 1;
      }
      if((i - first) <= (last - j)) {
        STACK_PUSH4(j, middle + m, last, (check &  2) | (next & 1));
        middle -= m, last = i, check = (check & 1);
      } else {
        if((i == middle) && (middle == j)) { next <<= 1; }
        STACK_PUSH4(first, middle - m, i, (check & 1) | (next & 2));
        first = j, middle += m, check = (check & 2) | (next & 1);
      }
    } else {
      if(check & 1) { MERGE_CHECK(first); }
      MERGE_CHECK(middle);
      if(check & 2) { MERGE_CHECK(last); }
      STACK_POP4(first, middle, last, check);
    }
  }
#undef GETIDX
#undef MERGE_CHECK
}

template<typename StringIterator_type, typename SAIterator_type, typename BufIterator_type>
void
sort(const StringIterator_type T, const SAIterator_type PA,
     SAIterator_type first, SAIterator_type last,
     BufIterator_type buf, typename std::iterator_traits<SAIterator_type>::value_type bufsize,
     typename std::iterator_traits<SAIterator_type>::value_type depth,
     typename std::iterator_traits<SAIterator_type>::value_type size,
     bool lastsuffix, int blocksize = 1024) {
typedef typename std::iterator_traits<SAIterator_type>::value_type pos_type;
  std::stack<helper::stackinfo4<SAIterator_type, SAIterator_type, pos_type, int> > stack1;
  std::stack<helper::stackinfo4<SAIterator_type, SAIterator_type, SAIterator_type, int> > stack2;

  SAIterator_type a, b;
  SAIterator_type curbuf;
  pos_type i, j, k, curbufsize;

  if(lastsuffix != false) { ++first; }
  for(a = first, i = 0; blocksize < (last - a); a += blocksize, ++i) {
    mintrosort(stack1, T, PA, a, a + blocksize, depth);
    curbuf = a + blocksize;
    curbufsize = pos_type( last - (a + blocksize) );
    if(bufsize <= curbufsize) {
      for(b = a, k = blocksize, j = i; j & 1; b -= k, k <<= 1, j >>= 1) {
        merge(stack2, T, PA, b - k, b, b + k, curbuf, curbufsize, depth);
      }
    } else {
      for(b = a, k = blocksize, j = i; j & 1; b -= k, k <<= 1, j >>= 1) {
        merge(stack2, T, PA, b - k, b, b + k, buf, bufsize, depth);
      }
    }
  }
  mintrosort(stack1, T, PA, a, last, depth);
  for(k = blocksize; i != 0; k <<= 1, i >>= 1) {
    if(i & 1) {
      merge(stack2, T, PA, a - k, a, last, buf, bufsize, depth);
      a -= k;
    }
  }

  if(lastsuffix != false) {
    /* Insert last type B* suffix. */
    for(a = first, i = *(first - 1);
        (a < last) && ((*a < 0) || (0 < compare_last(T, PA + i, PA + *a, depth, size)));
        ++a) {
      *(a - 1) = *a;
    }
    *(a - 1) = i;
  }
}


} /* namespace substring */



namespace doubling {

template<typename ISAIterator_type, typename SAIterator_type>
void
updategroup(ISAIterator_type ISA, const SAIterator_type SA,
            SAIterator_type first, SAIterator_type last) {
typedef typename std::iterator_traits<ISAIterator_type>::value_type value_type;
typedef typename std::iterator_traits<SAIterator_type>::value_type  sa_type;
  SAIterator_type a, b;
  value_type t;

  for(a = first; a < last; ++a) {
    if(0 <= *a) {
      b = a;
      do { ISA[*a] = sa_type( a - SA ); } while((++a < last) && (0 <= *a));
      *b = sa_type( b - a );
      if(last <= a) { break; }
    }
    b = a;
    do { *a = ~*a; } while(*++a < 0);
    t = value_type( a - SA );
    do { ISA[*b] = t; } while(++b <= a);
  }
}

template<typename stack_type, typename ISAIterator_type, typename SAIterator_type>
void
introsort(stack_type &stack, ISAIterator_type ISA, const ISAIterator_type ISAd,
          const SAIterator_type SA, SAIterator_type first, SAIterator_type last) {
typedef typename std::iterator_traits<ISAIterator_type>::value_type value_type;
typedef typename std::iterator_traits<SAIterator_type>::value_type  sa_type;
typedef typename stack_type::value_type stackinfo_type;

  SAIterator_type a, b, c;
  value_type v, x;

  for(int limit = helper::lg(last - first);;) {

    if((last - first) <= 8) {
      if(1 < (last - first)) {
        helper::insertionsort(ISAd, first, last);
        updategroup(ISA, SA, first, last);
      } else if((last - first) == 1) { *first = -1; }
      STACK_POP3(first, last, limit);
      continue;
    }

    if(limit-- == 0) {
      helper::heapsort(ISAd, first, value_type( last - first ));
      for(a = last - 1, v = ISAd[*a]; first < a;) {
        if((x = ISAd[*--a]) == v) { *a = ~*a; }
        else { v = x; }
      }
      updategroup(ISA, SA, first, last);
      STACK_POP3(first, last, limit);
      continue;
    }

    a = helper::pivot(ISAd, first, last);
    std::iter_swap(first, a);
    if(helper::partition(ISAd, first, first + 1, last, a, b, ISAd[*first]) != false) {

      /* update ranks */
      for(c = first, v = value_type( a - SA - 1 ); c < a; ++c) { ISA[*c] = v; }
      if(b < last) { for(c = a, v = value_type( b - SA - 1 ); c < b; ++c) { ISA[*c] = v; } }
      if((b - a) == 1) { *a = - 1; }

      if((a - first) <= (last - b)) {
        if(first < a) {
          STACK_PUSH3(b, last, limit);
          last = a;
        } else {
          first = b;
        }
      } else {
        if(b < last) {
          STACK_PUSH3(first, a, limit);
          first = b;
        } else {
          last = a;
        }
      }
    } else {
      STACK_POP3(first, last, limit);
    }
  }
}

template<typename ISAIterator_type, typename SAIterator_type>
void
sort(ISAIterator_type ISA, SAIterator_type first, SAIterator_type last) {
typedef typename std::iterator_traits<SAIterator_type>::value_type pos_type;
  std::stack<helper::stackinfo3<SAIterator_type, SAIterator_type, int> > stack;
  SAIterator_type a, b;
  pos_type t, skip, depth, size;

  for(size = pos_type( last - first ), depth = 1; -size < *first; depth *= 2) {
    a = first, skip = 0;
    do {
      if((t = *a) < 0) { a -= t; skip += t; }
      else {
        if(skip != 0) { *(a + skip) = skip; skip = 0; }
        b = first + ISA[t] + 1;
        introsort(stack, ISA, ISA + depth, first, a, b);
        a = b;
      }
    } while(a < last);
    if(skip != 0) { *(a + skip) = skip; }
  }
}

} /* namespace doubling */



namespace tandemrepeat {

template<typename stack_type, typename ISAIterator_type, typename SAIterator_type, typename pos_type>
void
introsort(stack_type &stack, ISAIterator_type ISA, ISAIterator_type ISAd,
          const SAIterator_type SA, SAIterator_type first, SAIterator_type last,
          pos_type &budget, int &chance, pos_type size) {
typedef typename std::iterator_traits<ISAIterator_type>::value_type value_type;
typedef typename stack_type::value_type stackinfo_type;
#define UPDATE_BUDGET(n)\
  {\
    budget -= (n);\
    if(budget <= 0) {\
      budget += size;\
      if(--chance == 0) { break; }\
    }\
  }

  SAIterator_type a, b, c, d, e;
  pos_type s, t;
  value_type v, x;
  int limit, next;

  for(limit = helper::lg(last - first);;) {

    if(limit < 0) {
      if(limit == -1) {
        /* tandem repeat partition */
        helper::partition(ISAd - 1, first, first, last, a, b, value_type( last - SA - 1 ));

        /* update ranks */
        if(a < last) {
          for(c = first, v = value_type( a - SA - 1 ); c < a; ++c) { ISA[*c] = v; }
        }
        if(b < last) {
          for(c = a, v = value_type( b - SA - 1 ); c < b; ++c) { ISA[*c] = v; }
        }

        /* push */
        STACK_PUSH4(ISAd, a, b, 0);
        STACK_PUSH4(ISAd - 1, first, last, -2);
        if((a - first) <= (last - b)) {
          if(first < a) {
            STACK_PUSH4(ISAd, b, last, helper::lg(last - b)); last = a;
          } else { first = b; }
        } else {
          if(b < last) {
            STACK_PUSH4(ISAd, first, a, helper::lg(a - first)); first = b;
          } else { last = a; }
        }
        limit = helper::lg(last - first);
      } else if(limit == -2) {
        /* tandem repeat copy */
        stackinfo_type temp = stack.top(); stack.pop();
        a = temp.m_b, b = temp.m_c;
        t = value_type( ISAd - ISA );
        v = value_type( b - SA - 1 );
        for(c = first, d = a; c < d; ++c) {
          if((0 <= (s = *c - t)) && (ISA[s] == v)) {
            ISA[s] = value_type( d - SA );
            *d++ = s;
          }
        }
        for(c = last - 1, e = d, d = b; e < d; --c) {
          if((0 <= (s = *c - t)) && (ISA[s] == v)) {
            *--d = s;
            ISA[s] = value_type( d - SA );
          }
        }
        STACK_POP4(ISAd, first, last, limit);
      } else {
        /* sorted partition */
        if(0 <= *first) {
          a = first;
          do { ISA[*a] = value_type( a - SA ); } while((++a < last) && (0 <= *a));
          first = a;
        }
        if(first < last) {
          a = first; do { *a = ~*a; } while(*++a < 0); ++a;
          if(a < last) {
            for(c = first, v = value_type( a - SA - 1 ); c < a; ++c) { ISA[*c] = v; }
          }

          /* push */
          next = (ISA[*first] == ISAd[*first]) ? -1 : helper::lg(a - first);
          UPDATE_BUDGET(value_type( last - first ));
          if((a - first) <= (last - a)) {
            if(first < a) {
              STACK_PUSH4(ISAd, a, last, -3);
              ISAd += 1, last = a, limit = next;
            } else {
              first = a, limit = -3;
            }
          } else {
            if(a < last) {
              STACK_PUSH4(ISAd + 1, first, a, next);
              first = a, limit = -3;
            } else {
              ISAd += 1, last = a, limit = next;
            }
          }
        } else {
          STACK_POP4(ISAd, first, last, limit);
        }
      }
      continue;
    }

    if((last - first) <= 8) {
      if(1 < (last - first)) {
        helper::insertionsort(ISAd, first, last);
        limit = -3;
      } else {
        STACK_POP4(ISAd, first, last, limit);
      }
      continue;
    }

    if(limit-- == 0) {
      helper::heapsort(ISAd, first, value_type( last - first ));
      for(a = last - 1, v = ISAd[*a]; first < a;) {
        if((x = ISAd[*--a]) == v) { *a = ~*a; }
        else { v = x; }
      }
      limit = -3;
      continue;
    }

    a = helper::pivot(ISAd, first, last);
    std::iter_swap(first, a);
    if(helper::partition(ISAd, first, first + 1, last, a, b, ISAd[*first]) != false) {
      next = (ISA[*a] == ISAd[*a]) ? -1 : helper::lg(b - a);

      /* update ranks */
      for(c = first, v = value_type( a - SA - 1 ); c < a; ++c) { ISA[*c] = v; }
      if(b < last) { for(c = a, v = value_type( b - SA - 1 ); c < b; ++c) { ISA[*c] = v; } }

      /* push */
      UPDATE_BUDGET(value_type( last - first ));
      if((a - first) <= (last - b)) {
        if((last - b) <= (b - a)) {
          STACK_PUSH4(ISAd + 1, a, b, next);
          STACK_PUSH4(ISAd, b, last, limit);
          last = a;
        } else if((a - first) <= (b - a)) {
          STACK_PUSH4(ISAd, b, last, limit);
          STACK_PUSH4(ISAd + 1, a, b, next);
          last = a;
        } else {
          STACK_PUSH4(ISAd, b, last, limit);
          STACK_PUSH4(ISAd, first, a, limit);
          ISAd += 1, first = a, last = b, limit = next;
        }
      } else {
        if((a - first) <= (b - a)) {
          STACK_PUSH4(ISAd + 1, a, b, next);
          STACK_PUSH4(ISAd, first, a, limit);
          first = b;
        } else if((last - b) <= (b - a)) {
          STACK_PUSH4(ISAd, first, a, limit);
          STACK_PUSH4(ISAd + 1, a, b, next);
          first = b;
        } else {
          STACK_PUSH4(ISAd, first, a, limit);
          STACK_PUSH4(ISAd, b, last, limit);
          ISAd += 1, first = a, last = b, limit = next;
        }
      }
    } else {
      limit = (ISA[*first] == ISAd[*first]) ? -1 : (limit + 1), ISAd += 1;
      UPDATE_BUDGET(value_type( last - first ));
    }
  }

  for(; stack.empty() == false; stack.pop()) {
    stackinfo_type temp = stack.top();
    if(temp.m_d == -3) {
      first = temp.m_b, last = temp.m_c;
      for(a = first; a < last; ++a) {
        if(0 <= *a) {
          b = a;
          do { ISA[*a] = value_type( a - SA ); } while((++a < last) && (0 <= *a));
          if(last <= a) { break; }
        }
        b = a;
        do { *a = ~*a; } while(*++a < 0);
        v = value_type( a - SA );
        do { ISA[*b] = v; } while(++b <= a);
      }
    }
  }
#undef UPDATE_BUDGET
}

template<typename ISAIterator_type, typename SAIterator_type>
bool
sort(ISAIterator_type ISA, SAIterator_type first, SAIterator_type last) {
typedef typename std::iterator_traits<SAIterator_type>::value_type pos_type;
  std::stack<helper::stackinfo4<ISAIterator_type, SAIterator_type, SAIterator_type, int> > stack;
  SAIterator_type a, b;
  pos_type t, skip, size, budget;
  int chance;

  size = pos_type( last - first );
  if(-size < *first) {
    a = first, skip = 0, budget = size;
//    chance = helper::lg(size);
    chance = helper::lg(size) * 2 / 3 + 1;
//    chance = helper::lg(size) / 2 + 1;
    do {
      if((t = *a) < 0) { a -= t; skip += t; }
      else {
        skip = 0;
        b = first + ISA[t] + 1;
        introsort(stack, ISA, ISA + 1, first, a, b, budget, chance, size);
        if(chance == 0) {
          /* Switch to Larsson-Sadakane sorting algorithm. */
          if(first < a) { *first = -pos_type(a - first); }
          return false;
        }
        a = b;
      }
    } while(a < last);
  }

  return true;
}


} /* namespace tandemrepeat */


#undef STACK_POP3
#undef STACK_POP4
#undef STACK_PUSH3
#undef STACK_PUSH4


static const int EXTRA_SPACE = 0;

namespace core {
#define BUCKET_A(c0) bucket_A[(c0)]
#define BUCKET_B(c0, c1) (bucket_B[((alphabetsize_type)(c1)) * alphabetsize + (alphabetsize_type)(c0)])
#define BUCKET_BSTAR(c0, c1) (bucket_B[((alphabetsize_type)(c0)) * alphabetsize + (alphabetsize_type)(c1)])

static const int MERGE_BUFSIZE = 256;

/* Sorts suffixes of type B*. */
template<typename StringIterator_type, typename SAIterator_type, typename pos_type, typename alphabetsize_type>
pos_type
sort_typeBstar(const StringIterator_type T, SAIterator_type SA,
               pos_type *bucket_A, pos_type *bucket_B,
               pos_type n, pos_type SAsize, alphabetsize_type alphabetsize) {
typedef typename std::iterator_traits<StringIterator_type>::value_type value_type;
  pos_type i, j, k, t, m, bufsize;
  alphabetsize_type c0, c1;

  /* Initialize bucket arrays. */
  for(i = 0; i < alphabetsize; ++i) { bucket_A[i] = 0; }
  for(i = 0; i < alphabetsize * alphabetsize; ++i) { bucket_B[i] = 0; }

  /* Count the number of occurrences of the first one or two characters of each
     type A, B and B* suffix. Moreover, store the beginning position of all
     type B* suffixes into the array SA. */
  for(i = n - 1, m = SAsize; 0 <= i;) {
    /* type A suffix. */
    do { ++BUCKET_A(T[i]); } while((0 <= --i) && (T[i] >= T[i + 1]));
    if(0 <= i) {
      /* type B* suffix. */
      ++BUCKET_BSTAR(T[i], T[i + 1]);
      SA[--m] = i;
      /* type B suffix. */
      for(--i; (0 <= i) && (T[i] <= T[i + 1]); --i) {
        ++BUCKET_B(T[i], T[i + 1]);
      }
    }
  }
  m = SAsize - m;

  /* Calculate the index of start/end point of each bucket. */
  for(c0 = 0, i = -1, j = 0; c0 < alphabetsize; ++c0) {
    t = i + BUCKET_A(c0);
    BUCKET_A(c0) = i + j; /* start point */
    i = t + BUCKET_B(c0, c0);
    for(c1 = c0 + 1; c1 < alphabetsize; ++c1) {
      j += BUCKET_BSTAR(c0, c1);
      BUCKET_BSTAR(c0, c1) = j; /* end point */
      i += BUCKET_B(c0, c1);
    }
  }

  if(0 < m) {
    /* Sort the type B* suffixes by their first two characters. */
    SAIterator_type PAb = SA + SAsize - m, ISAb = SA + m;
    for(i = m - 2; 0 <= i; --i) {
      t = PAb[i], c0 = T[t], c1 = T[t + 1];
      SA[--BUCKET_BSTAR(c0, c1)] = i;
    }
    t = PAb[m - 1], c0 = T[t], c1 = T[t + 1];
    SA[--BUCKET_BSTAR(c0, c1)] = m - 1;

    /* Sort the type B* substrings using sssort. */
    bufsize = SAsize - 2 * m;
    if(MERGE_BUFSIZE < bufsize) {
      SAIterator_type buf = SA + m;
      for(c0 = alphabetsize - 1, j = m; 0 < j; --c0) {
        for(c1 = alphabetsize - 1; c0 < c1; j = i, --c1) {
          i = BUCKET_BSTAR(c0, c1);
          if(1 < (j - i)) {
            substring::sort(T, PAb, SA + i, SA + j, buf, bufsize, 2, n, *(SA + i) == (m - 1));
          }
        }
      }
    } else {
      pos_type *lbuf = NULL;
      int err = 0;
      try {
        lbuf = new pos_type[MERGE_BUFSIZE]; if(lbuf == NULL) { throw; }
        for(c0 = alphabetsize - 1, j = m; 0 < j; --c0) {
          for(c1 = alphabetsize - 1; c0 < c1; j = i, --c1) {
            i = BUCKET_BSTAR(c0, c1);
            if(1 < (j - i)) {
              substring::sort(T, PAb, SA + i, SA + j, lbuf, MERGE_BUFSIZE, 2, n, *(SA + i) == (m - 1));
            }
          }
        }
      } catch(...) {
        err = -1;
      }
      delete[] lbuf;
      if(err != 0) { throw; }
    }

    /* Compute ranks of type B* substrings. */
    for(i = m - 1; 0 <= i; --i) {
      if(0 <= SA[i]) {
        j = i;
        do { ISAb[SA[i]] = i; } while((0 <= --i) && (0 <= SA[i]));
        SA[i + 1] = i - j;
        if(i <= 0) { break; }
      }
      j = i;
      do { ISAb[SA[i] = ~SA[i]] = j; } while(SA[--i] < 0);
      ISAb[SA[i]] = j;
    }

    /* Construct the inverse suffix array of type B* suffixes using tandemrepeat/doubling sorting algorithms. */
    if(tandemrepeat::sort(ISAb, SA, SA + m) == false) {
      doubling::sort(ISAb, SA, SA + m);
    }

    /* Set the sorted order of tyoe B* suffixes. */
    for(i = n - 1, j = m; 0 <= i;) {
      for(--i; (0 <= i) && (T[i] >= T[i + 1]); --i) { }
      if(0 <= i) {
        SA[ISAb[--j]] = i;
        for(--i; (0 <= i) && (T[i] <= T[i + 1]); --i) { }
      }
    }

    /* Calculate the index of start/end point of each bucket. */
    for(c0 = alphabetsize - 1, i = n - 1, k = m - 1; 0 <= c0; --c0) {
      for(c1 = alphabetsize - 1; c0 < c1; --c1) {
        t = i - BUCKET_B(c0, c1);
        BUCKET_B(c0, c1) = i + 1; /* end point */

        /* Move all type B* suffixes to the correct position. */
        for(i = t, j = BUCKET_BSTAR(c0, c1);
            j <= k;
            --i, --k) { SA[i] = SA[k]; }
      }
      t = i - BUCKET_B(c0, c0);
      BUCKET_B(c0, c0) = i + 1; /* end point */
      if(c0 < (alphabetsize - 1)) {
        BUCKET_BSTAR(c0, c0 + 1) = t + 1; /* start point */
      }
      i = BUCKET_A(c0);
    }
  }

  return m;
}

/* Constructs the suffix array by using the sorted order of type B* suffixes. */
template<typename StringIterator_type, typename SAIterator_type, typename pos_type, typename alphabetsize_type>
void
constructSA_from_typeBstar(const StringIterator_type T, SAIterator_type SA,
                           pos_type *bucket_A, pos_type *bucket_B,
                           pos_type n, pos_type m, alphabetsize_type alphabetsize) {
typedef typename std::iterator_traits<StringIterator_type>::value_type value_type;
  SAIterator_type i, j, t = SA;
  pos_type s;
  alphabetsize_type c0, c1, c2;

  /** An implementation version of MSufSort3's second stage. **/

  if(0 < m) {
    /* Construct the sorted order of type B suffixes by using
       the sorted order of type B* suffixes. */
    for(c1 = alphabetsize - 2; 0 <= c1; --c1) {
      /* Scan the suffix array from right to left. */
      for(i = SA + BUCKET_BSTAR(c1, c1 + 1),
          j = SA + (BUCKET_A(c1 + 1) + 1),
          c2 = -1;
          i < j;) {
        if(0 <= (s = *--j)) {
          if((0 <= --s) && ((c0 = T[s]) <= c1)) {
            *j = ~(s + 1);
            if((0 < s) && (T[s - 1] > c0)) { s = ~s; }
            if(c2 == c0) { *--t = s; }
            else {
              if(0 <= c2) { BUCKET_B(c2, c1) = t - SA; }
              *(t = SA + BUCKET_B(c2 = c0, c1) - 1) = s;
            }
          }
        } else {
          *j = ~s;
        }
      }
    }
  }

  /* Construct the suffix array by using
     the sorted order of type B suffixes. */
  *(t = SA + (BUCKET_A(c2 = T[n - 1]) + 1)) = n - 1;
  /* Scan the suffix array from left to right. */
  for(i = SA, j = SA + n; i < j; ++i) {
    if(0 <= (s = *i)) {
      if((0 <= --s) && ((c0 = T[s]) >= T[s + 1])) {
        if((0 < s) && (T[s - 1] < c0)) { s = ~s; }
        if(c0 == c2) { *++t = s; }
        else {
          BUCKET_A(c2) = t - SA;
          *(t = SA + (BUCKET_A(c2 = c0) + 1)) = s;
        }
      }
    } else {
      *i = ~s;
    }
  }
}


/* Constructs the burrows-wheeler transformed string directly
   by using the sorted order of type B* suffixes. */
template<typename StringIterator_type, typename SAIterator_type, typename pos_type, typename alphabetsize_type>
SAIterator_type
constructBWT_from_typeBstar(const StringIterator_type T, SAIterator_type SA,
                            pos_type *bucket_A, pos_type *bucket_B,
                            pos_type n, pos_type m, alphabetsize_type alphabetsize) {
typedef typename std::iterator_traits<StringIterator_type>::value_type value_type;
  SAIterator_type i, j, t = SA, orig;
  pos_type s;
  alphabetsize_type c0, c1, c2;

  /** An implementation version of MSufSort3's semidirect BWT. **/

  if(0 < m) {
    /* Construct the sorted order of type B suffixes by using
       the sorted order of type B* suffixes. */
    for(c1 = alphabetsize - 2; 0 <= c1; --c1) {
      /* Scan the suffix array from right to left. */
      for(i = SA + BUCKET_BSTAR(c1, c1 + 1),
          j = SA + (BUCKET_A(c1 + 1) + 1),
          c2 = -1;
          i < j;) {
        if(0 <= (s = *--j)) {
          if((0 <= --s) && ((c0 = T[s]) <= c1)) {
            *j = ~((pos_type)c0);
            if((0 < s) && (T[s - 1] > c0)) { s = ~s; }
            if(c0 == c2) { *--t = s; }
            else {
              if(0 <= c2) { BUCKET_B(c2, c1) = pos_type( t - SA ); }
              *(t = SA + BUCKET_B(c2 = c0, c1) - 1) = s;
            }
          }
        } else {
          *j = ~s;
        }
      }
    }
  }

  /* Construct the BWTed string by using
     the sorted order of type B suffixes. */
  c0 = T[s = n - 1];
  if(T[s - 1] < c0) { s = ~((pos_type)T[s - 1]); }
  *(t = SA + (BUCKET_A(c2 = c0) + 1)) = s;
  /* Scan the suffix array from left to right. */
  for(i = SA, j = SA + n, orig = SA; i < j; ++i) {
    if(0 <= (s = *i)) {
      if((0 <= --s) && ((c0 = T[s]) >= T[s + 1])) {
        *i = c0;
        if((0 < s) && (T[s - 1] < c0)) { s = ~((pos_type)T[s - 1]); }
        if(c0 == c2) { *++t = s; }
        else {
          BUCKET_A(c2) = pos_type( t - SA );
          *(t = SA + (BUCKET_A(c2 = c0) + 1)) = s;
        }
      } else if(s < 0) { orig = i; }
    } else {
      *i = ~s;
    }
  }

  return orig;
}

#undef BUCKET_A
#undef BUCKET_B
#undef BUCKET_BSTAR
#undef TYPEB_START
#undef TYPEB_END

} /* namespace core */

/*---------------------------------------------------------------------------*/


/**
 * Constructs the suffix array of a given string.
 *
 * @param T An input string iterator.
 * @param T_last An input string iterator.
 * @param SA An output iterator.
 * @param SA_last An output iterator.
 * @param alphabetsize
 * @return 0 if no error occurred, -1 or -2 otherwise.
 */
template<typename StringIterator_type, typename SAIterator_type, typename alphabetsize_type>
int
constructSA(const StringIterator_type T, const StringIterator_type T_last,
            SAIterator_type SA, SAIterator_type SA_last,
            alphabetsize_type alphabetsize = 256) {
typedef typename std::iterator_traits<SAIterator_type>::value_type pos_type;
  pos_type *bucket_A = NULL, *bucket_B = NULL;
  pos_type n = T_last - T, m = SA_last - SA;
  int err = 0;

  /* Check arguments. */
  if((n < 0) || (m < n)) { return -1; }
  else if(n == 0) { return 0; }
  else if(n == 1) { *SA = 0; return 0; }

  try {
    bucket_A = new pos_type[alphabetsize]; if(bucket_A == NULL) { throw; }
    bucket_B = new pos_type[alphabetsize * alphabetsize]; if(bucket_B == NULL) { throw; }
    m = core::sort_typeBstar(T, SA, bucket_A, bucket_B, n, m, alphabetsize);
    core::constructSA_from_typeBstar(T, SA, bucket_A, bucket_B, n, m, alphabetsize);
  } catch(...) {
    err = -2;
  }

  delete [] bucket_B;
  delete [] bucket_A;

  return err;
}

template<typename StringIterator_type, typename SAIterator_type, typename alphabetsize_type>
int
constructSA(const StringIterator_type T, SAIterator_type SA,
            typename std::iterator_traits<SAIterator_type>::value_type n,
            alphabetsize_type alphabetsize = 256) {
  return constructSA(T, T + n, SA, SA + n, alphabetsize);
}

/**
 * Constructs the burrows-wheeler transformed string of a given string.
 *
 * @param T An input string iterator.
 * @param T_last An input string iterator.
 * @param U An output string iterator.
 * @param U_last An output string iterator.
 * @param SA A temporary iterator.
 * @param SA_last A temporary iterator.
 * @param alphabetsize
 * @return 0 if no error occurred, -1 or -2 otherwise.
 */
template<typename StringIterator1_type, typename StringIterator2_type,
         typename SAIterator_type, typename alphabetsize_type>
typename std::iterator_traits<SAIterator_type>::value_type
constructBWT(const StringIterator1_type T, const StringIterator1_type T_last,
             StringIterator2_type U, StringIterator2_type U_last,
             SAIterator_type SA, SAIterator_type SA_last,
             alphabetsize_type alphabetsize = 256) {
typedef typename std::iterator_traits<StringIterator2_type>::value_type value_type;
typedef typename std::iterator_traits<SAIterator_type>::value_type pos_type;
  SAIterator_type piter, i, k;
  StringIterator2_type j;
  pos_type *bucket_A = NULL, *bucket_B = NULL;
  pos_type n = pos_type( T_last - T ), m = pos_type( SA_last - SA ), u = pos_type( U_last - U ), pidx = -1;

  /* Check arguments. */
  if((n < pos_type(0)) || (m < n)) { return -1; }
  else if(n == 0) { return 0; }
  else if(n == 1) { if(0 < u) { U[0] = T[0]; } return 1; }

  try {
    bucket_A = new pos_type[alphabetsize]; if(bucket_A == NULL) { throw; }
    bucket_B = new pos_type[alphabetsize * alphabetsize]; if(bucket_B == NULL) { throw; }
    m = core::sort_typeBstar(T, SA, bucket_A, bucket_B, n, m, alphabetsize);
    piter = core::constructBWT_from_typeBstar(T, SA, bucket_A, bucket_B, n, m, alphabetsize);
    pidx = pos_type( piter - SA + 1 );
    if(0 < u) {
      U[pos_type(0)] = T[n - 1];
      if(pidx <= u) {
        for(i = SA, j = U + 1; i < piter; ++i, ++j) { *j = value_type(*i); }
        if(n <= u) { for(i += 1; i < SA_last; ++i, ++j) { *j = value_type(*i); } }
        else { for(i += 1, k = SA + u - 1; i < k; ++i, ++j) { *j = value_type(*i); } }
      } else {
        for(i = SA, j = U + 1, k = SA + u - 1; i < k; ++i, ++j) { *j = value_type(*i); }
      }
    }
  } catch(...) {
    pidx = -2;
  }

  delete [] bucket_B;
  delete [] bucket_A;

  return pidx;
}

template<typename pos_type, typename StringIterator1_type, typename StringIterator2_type, typename alphabetsize_type>
pos_type
constructBWT(const StringIterator1_type T, const StringIterator1_type T_last,
             StringIterator2_type U, StringIterator2_type U_last,
             alphabetsize_type alphabetsize = 256) {
  pos_type *SA = NULL;
  pos_type n = T_last - T, pidx = -1;

  /* Check arguments. */
  if(n < 0) { return -1; }
  else if(n == 0) { return 0; }
  else if(n == 1) { if(U < U_last) { U[pos_type(0)] = T[pos_type(0)]; } return 1; }

  try {
    SA = new pos_type[n]; if(SA == NULL) { throw; }
    pidx = constructBWT(T, T_last, U, U_last, SA, SA + n, alphabetsize);
  } catch(...) {
    pidx = -2;
  }
  delete [] SA;

  return pidx;
}

template<typename pos_type, typename StringIterator_type, typename alphabetsize_type>
pos_type
constructBWT(StringIterator_type T, StringIterator_type T_last,
             alphabetsize_type alphabetsize = 256) {
  return constructBWT<pos_type>(T, T_last, T, T_last, alphabetsize);
}

template<typename StringIterator1_type, typename StringIterator2_type,
         typename SAIterator_type, typename alphabetsize_type>
typename std::iterator_traits<SAIterator_type>::value_type
constructBWT(const StringIterator1_type T, StringIterator2_type U, SAIterator_type SA,
             typename std::iterator_traits<SAIterator_type>::value_type n,
             alphabetsize_type alphabetsize = 256) {
  return constructBWT(T, T + n, U, U + n, SA, SA + n, alphabetsize);
}

template<typename pos_type, typename StringIterator1_type, typename StringIterator2_type, typename alphabetsize_type>
pos_type
constructBWT(const StringIterator1_type T, StringIterator2_type U,
             pos_type n, alphabetsize_type alphabetsize = 256) {
  return constructBWT<pos_type>(T, T + n, U, U + n, alphabetsize);
}

template<typename pos_type, typename StringIterator_type, typename alphabetsize_type>
pos_type
constructBWT(StringIterator_type T, pos_type n, alphabetsize_type alphabetsize = 256) {
  return constructBWT<pos_type>(T, T + n, alphabetsize);
}


} /* namespace divsufsortxx */

#endif /* _DIVSUFSORTXX_H_ */
