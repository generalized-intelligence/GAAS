#ifndef FAST_CORNER_UTILITIES_H
#define FAST_CORNER_UTILITIES_H

#if __ARM_NEON__
#include <arm_neon.h>
#elif __SSE2__
#include <emmintrin.h>
#endif

namespace fast
{

/// Check if the pointer is aligned to the specified byte granularity
template<int bytes> bool is_aligned(const void* ptr);
template<> inline bool is_aligned<8>(const void* ptr) { return ((reinterpret_cast<std::size_t>(ptr)) & 0x7) == 0; }
template<> inline bool is_aligned<16>(const void* ptr) { return ((reinterpret_cast<std::size_t>(ptr)) & 0xF) == 0; }


struct Less
{
   template <class T1, class T2> static bool eval(const T1 a, const T2 b)
   {
      return a < b;
   }
   static short prep_t(short pixel_val, short barrier)
   {
      return pixel_val - barrier;
   }
};

struct Greater
{
   template <class T1, class T2> static bool eval(const T1 a, const T2 b)
   {
      return a > b;
   }
   static short prep_t(short pixel_val, short barrier)
   {
      return pixel_val + barrier;
   }
};

#if __SSE2__

#define CHECK_BARRIER(lo, hi, other, flags)       \
  {                 \
  __m128i diff = _mm_subs_epu8(lo, other);      \
  __m128i diff2 = _mm_subs_epu8(other, hi);     \
  __m128i z = _mm_setzero_si128();        \
  diff = _mm_cmpeq_epi8(diff, z);         \
  diff2 = _mm_cmpeq_epi8(diff2, z);       \
  flags = ~(_mm_movemask_epi8(diff) | (_mm_movemask_epi8(diff2) << 16)); \
  }
     
  template <bool Aligned> inline __m128i load_si128(const void* addr) { return _mm_loadu_si128((const __m128i*)addr); }
  template <> inline __m128i load_si128<true>(const void* addr) { return _mm_load_si128((const __m128i*)addr); }

#endif

} // namespace fast

#endif
