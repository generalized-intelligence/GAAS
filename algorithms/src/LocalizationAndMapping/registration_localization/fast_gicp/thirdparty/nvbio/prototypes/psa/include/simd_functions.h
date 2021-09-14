/*
 * Copyright (c) 2013 NVIDIA Corporation. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without 
 * modification, are permitted provided that the following conditions are met:
 *
 *   Redistributions of source code must retain the above copyright notice, 
 *   this list of conditions and the following disclaimer.
 *
 *   Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 *   Neither the name of NVIDIA Corporation nor the names of its contributors
 *   may be used to endorse or promote products derived from this software 
 *   without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE 
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/* Release 1.1
 * 
 * (1) Use of incorrect symbol in multiple-inclusion guard has been corrected.
 * (2) 44 additional functions were added to the initial set of 38 functions.
 * (3) The emulation paths for many existing functions were optimized for sm_2x
 */

#if !defined (SIMD_FUNCTIONS_H__)
#define SIMD_FUNCTIONS_H__

/*
  This header file contains inline functions that implement intra-word SIMD
  operations, that are hardware accelerated on sm_3x (Kepler) GPUs. Efficient
  emulation code paths are provided for earlier architectures (sm_1x, sm_2x)
  to make the code portable across all GPUs supported by CUDA. The following 
  functions are currently implemented:

  vabs2(a)        per-halfword absolute value, with wrap-around: |a|
  vabsdiffs2(a,b) per-halfword absolute difference of signed integer: |a - b|
  vabsdiffu2(a,b) per-halfword absolute difference of unsigned integer: |a - b|
  vabsss2(a)      per-halfword abs. value, with signed saturation: sat.s16(|a|)
  vadd2(a,b)      per-halfword (un)signed addition, with wrap-around: a + b
  vaddss2(a,b)    per-halfword addition with signed saturation: sat.s16 (a + b)
  vaddus2(a,b)    per-halfword addition with unsigned saturation: sat.u16 (a+b)
  vavgs2(a,b)     per-halfword signed rounded average: (a+b+((a+b)>=0)) >> 1
  vavgu2(a,b)     per-halfword unsigned rounded average: (a + b + 1) / 2
  vcmpeq2(a,b)    per-halfword (un)signed comparison: a == b ? 0xffff : 0
  vcmpges2(a,b)   per-halfword signed comparison: a >= b ? 0xffff : 0
  vcmpgeu2(a,b)   per-halfword unsigned comparison: a >= b ? 0xffff : 0
  vcmpgts2(a,b)   per-halfword signed comparison: a > b ? 0xffff : 0
  vcmpgtu2(a,b)   per-halfword unsigned comparison: a > b ? 0xffff : 0
  vcmples2(a,b)   per-halfword signed comparison: a <= b ? 0xffff : 0
  vcmpleu2(a,b)   per-halfword unsigned comparison: a <= b ? 0xffff : 0
  vcmplts2(a,b)   per-halfword signed comparison: a < b ? 0xffff : 0
  vcmpltu2(a,b)   per-halfword unsigned comparison: a < b ? 0xffff : 0
  vcmpne2(a,b)    per-halfword (un)signed comparison: a != b ? 0xffff : 0
  vhaddu2(a,b)    per-halfword unsigned average: (a + b) / 2
  vmaxs2(a,b)     per-halfword signed maximum: max(a, b)
  vmaxu2(a,b)     per-halfword unsigned maximum: max(a, b)
  vmins2(a,b)     per-halfword signed minimum: min(a, b)
  vminu2(a,b)     per-halfword unsigned minimum: min(a, b)
  vneg2(a,b)      per-halfword negation, with wrap-around: -a
  vnegss2(a,b)    per-halfword negation, with signed saturation: sat.s16(-a)
  vsads2(a,b)     per-halfword sum of abs diff of signed: sum{0,1}(|a-b|)
  vsadu2(a,b)     per-halfword sum of abs diff of unsigned: sum{0,1}(|a-b|)
  vseteq2(a,b)    per-halfword (un)signed comparison: a == b ? 1 : 0
  vsetges2(a,b)   per-halfword signed comparison: a >= b ? 1 : 0
  vsetgeu2(a,b)   per-halfword unsigned comparison: a >= b ? 1 : 0
  vsetgts2(a,b)   per-halfword signed comparison: a > b ? 1 : 0
  vsetgtu2(a,b)   per-halfword unsigned comparison: a > b ? 1 : 0
  vsetles2(a,b)   per-halfword signed comparison: a <= b ? 1 : 0 
  vsetleu2(a,b)   per-halfword unsigned comparison: a <= b ? 1 : 0 
  vsetlts2(a,b)   per-halfword signed comparison: a < b ? 1 : 0
  vsetltu2(a,b)   per-halfword unsigned comparison: a < b ? 1 : 0
  vsetne2(a,b)    per-halfword (un)signed comparison: a != b ? 1 : 0
  vsub2(a,b)      per-halfword (un)signed subtraction, with wrap-around: a - b
  vsubss2(a,b)    per-halfword subtraction with signed saturation: sat.s16(a-b)
  vsubus2(a,b)    per-halfword subtraction w/ unsigned saturation: sat.u16(a-b)
  
  vabs4(a)        per-byte absolute value, with wrap-around: |a|
  vabsdiffs4(a,b) per-byte absolute difference of signed integer: |a - b|
  vabsdiffu4(a,b) per-byte absolute difference of unsigned integer: |a - b|
  vabsss4(a)      per-byte absolute value, with signed saturation: sat.s8(|a|)
  vadd4(a,b)      per-byte (un)signed addition, with wrap-around: a + b
  vaddss4(a,b)    per-byte addition with signed saturation: sat.s8 (a + b)
  vaddus4(a,b)    per-byte addition with unsigned saturation: sat.u8 (a + b)
  vavgs4(a,b)     per-byte signed rounded average: (a + b + ((a+b) >= 0)) >> 1
  vavgu4(a,b)     per-byte unsigned rounded average: (a + b + 1) / 2
  vcmpeq4(a,b)    per-byte (un)signed comparison: a == b ? 0xff : 0
  vcmpges4(a,b)   per-byte signed comparison: a >= b ? 0xff : 0
  vcmpgeu4(a,b)   per-byte unsigned comparison: a >= b ? 0xff : 0
  vcmpgts4(a,b)   per-byte signed comparison: a > b ? 0xff : 0
  vcmpgtu4(a,b)   per-byte unsigned comparison: a > b ? 0xff : 0
  vcmples4(a,b)   per-byte signed comparison: a <= b ? 0xff : 0
  vcmpleu4(a,b)   per-byte unsigned comparison: a <= b ? 0xff : 0
  vcmplts4(a,b)   per-byte signed comparison: a < b ? 0xff : 0
  vcmpltu4(a,b)   per-byte unsigned comparison: a < b ? 0xff : 0
  vcmpne4(a,b)    per-byte (un)signed comparison: a != b ? 0xff: 0
  vhaddu4(a,b)    per-byte unsigned average: (a + b) / 2
  vmaxs4(a,b)     per-byte signed maximum: max(a, b)
  vmaxu4(a,b)     per-byte unsigned maximum: max(a, b)
  vmins4(a,b)     per-byte signed minimum: min(a, b)
  vminu4(a,b)     per-byte unsigned minimum: min(a, b)
  vneg4(a,b)      per-byte negation, with wrap-around: -a
  vnegss4(a,b)    per-byte negation, with signed saturation: sat.s8(-a)
  vsads4(a,b)     per-byte sum of abs difference of signed: sum{0,3}(|a-b|)
  vsadu4(a,b)     per-byte sum of abs difference of unsigned: sum{0,3}(|a-b|)
  vseteq4(a,b)    per-byte (un)signed comparison: a == b ? 1 : 0
  vsetges4(a,b)   per-byte signed comparison: a >= b ? 1 : 0
  vsetgeu4(a,b)   per-byte unsigned comparison: a >= b ? 1 : 0
  vsetgts4(a,b)   per-byte signed comparison: a > b ? 1 : 0
  vsetgtu4(a,b)   per-byte unsigned comparison: a > b ? 1 : 0
  vsetles4(a,b)   per-byte signed comparison: a <= b ? 1 : 0
  vsetleu4(a,b)   per-byte unsigned comparison: a <= b ? 1 : 0
  vsetlts4(a,b)   per-byte signed comparison: a < b ? 1 : 0
  vsetltu4(a,b)   per-byte unsigned comparison: a < b ? 1 : 0
  vsetne4(a,b)    per-byte (un)signed comparison: a != b ? 1: 0
  vsub4(a,b)      per-byte (un)signed subtraction, with wrap-around: a - b
  vsubss4(a,b)    per-byte subtraction with signed saturation: sat.s8 (a - b)
  vsubus4(a,b)    per-byte subtraction with unsigned saturation: sat.u8 (a - b)
*/

static __device__ __forceinline__ unsigned int vabs2(unsigned int a)
{
    unsigned int r;
#if __CUDA_ARCH__ >= 300
    unsigned int b = 0, c = 0;
    asm ("vabsdiff2.s32.s32.s32 %0,%1,%2,%3;" : "=r"(r) :"r"(a),"r"(b),"r"(c));
#elif __CUDA_ARCH__ >= 200
    asm ("{                       \n\t"
         ".reg .u32 a,m,r;        \n\t"
         "mov.b32  a,%1;          \n\t"
         "prmt.b32 m,a,0,0xbb99;  \n\t" // msb ? 0xffff : 0000
         "xor.b32  r,a,m;         \n\t" // conditionally invert bits
         "and.b32  m,m,0x00010001;\n\t" // msb ? 0x1 : 0
         "add.u32  r,r,m;         \n\t" // conditionally add 1
         "mov.b32  %0,r;          \n\t"
         "}"
         : "=r"(r) : "r"(a));
#else  /* __CUDA_ARCH__ >= 200 */
    asm ("{                       \n\t"
         ".reg .u32 a,m,r,s;      \n\t"
         "mov.b32  a,%1;          \n\t"
         "and.b32  m,a,0x80008000;\n\t" // extract msb
         "and.b32  r,a,0x7fff7fff;\n\t" // clear msb
         "shr.u32  s,m,15;        \n\t" // build lsb mask
         "sub.u32  m,m,s;         \n\t" //  from msb
         "xor.b32  r,r,m;         \n\t" // conditionally invert lsbs
         "add.u32  r,r,s;         \n\t" // conditionally add 1
         "mov.b32  %0,r;          \n\t"
         "}"
         : "=r"(r) : "r"(a));
#endif /* __CUDA_ARCH__ >= 200 */
    return r;           // halfword-wise absolute value, with wrap-around
}

static __device__ __forceinline__ unsigned int vabsss2(unsigned int a)
{
    unsigned int r;
#if __CUDA_ARCH__ >= 300
    unsigned int b = 0, c = 0;
    asm("vabsdiff2.s32.s32.s32.sat %0,%1,%2,%3;":"=r"(r):"r"(a),"r"(b),"r"(c));
#elif __CUDA_ARCH__ >= 200
    asm ("{                       \n\t"
         ".reg .u32 a,m,r;        \n\t"
         "mov.b32  a,%1;          \n\t"
         "prmt.b32 m,a,0,0xbb99;  \n\t" // msb ? 0xffff : 0000
         "xor.b32  r,a,m;         \n\t" // conditionally invert bits
         "and.b32  m,m,0x00010001;\n\t" // msb ? 0x1 : 0
         "add.u32  r,r,m;         \n\t" // conditionally add 1
         "prmt.b32 m,r,0,0xbb99;  \n\t" // msb ? 0xffff : 0000
         "and.b32  m,m,0x00010001;\n\t" // msb ? 0x1 : 0
         "sub.u32  r,r,m;         \n\t" // subtract 1 if result wrapped around
         "mov.b32  %0,r;          \n\t"
         "}"
         : "=r"(r) : "r"(a));
#else  /* __CUDA_ARCH__ >= 200 */
    asm ("{                       \n\t"
         ".reg .u32 a,m,r,s;      \n\t"
         "mov.b32  a,%1;          \n\t"
         "and.b32  m,a,0x80008000;\n\t" // extract msb
         "and.b32  r,a,0x7fff7fff;\n\t" // clear msb
         "shr.u32  s,m,15;        \n\t" // build lsb mask
         "sub.u32  m,m,s;         \n\t" //  from msb
         "xor.b32  r,r,m;         \n\t" // conditionally invert lsbs
         "add.u32  r,r,s;         \n\t" // conditionally add 1
         "and.b32  m,r,0x80008000;\n\t" // extract msb (1 if wrap-around)
         "shr.u32  s,m,15;        \n\t" // msb ? 1 : 0
         "sub.u32  r,r,s;         \n\t" // subtract 1 if result wrapped around
         "mov.b32  %0,r;          \n\t"
         "}"
         : "=r"(r) : "r"(a));
#endif /* __CUDA_ARCH__ >= 200 */
    return r;           // halfword-wise absolute value with signed saturation
}

static __device__ __forceinline__ unsigned int vadd2(unsigned int a, unsigned int b)
{
    unsigned int s, t;
#if __CUDA_ARCH__ >= 300
    s = 0;
    asm ("vadd2.u32.u32.u32 %0,%1,%2,%3;" : "=r"(t) : "r"(a), "r"(b), "r"(s));
#else  /* __CUDA_ARCH__ >= 300 */
    s = a ^ b;          // sum bits
    t = a + b;          // actual sum
    s = s ^ t;          // determine carry-ins for each bit position
    s = s & 0x00010000; // carry-in to high word (= carry-out from low word)
    t = t - s;          // subtract out carry-out from low word
#endif /* __CUDA_ARCH__ >= 300 */
    return t;           // halfword-wise sum, with wrap around
}

static __device__ __forceinline__ unsigned int vaddss2 (unsigned int a, unsigned int b)
{
    unsigned int r;
#if __CUDA_ARCH__ >= 300
    unsigned int c = 0;
    asm ("vadd2.s32.s32.s32.sat %0,%1,%2,%3;" : "=r"(r):"r"(a),"r"(b),"r"(c));
#else  /* __CUDA_ARCH__ >= 300 */
    int ahi, alo, blo, bhi, rhi, rlo;
    ahi = (int)((a & 0xffff0000U));
    bhi = (int)((b & 0xffff0000U));
#if __CUDA_ARCH__ < 200
    alo = (int)(a << 16);
    blo = (int)(b << 16);
#elif __CUDA_ARCH__ < 350
    // work around (we would want left shifts at least for sm_2x)
    asm ("prmt.b32 %0,%1,0,0x1044;" : "=r"(alo) : "r"(a));
    asm ("prmt.b32 %0,%1,0,0x1044;" : "=r"(blo) : "r"(b));
#else
    asm ("shf.l.clamp.b32 %0,0,%1,16;" : "=r"(alo) : "r"(a));
    asm ("shf.l.clamp.b32 %0,0,%1,16;" : "=r"(blo) : "r"(b));
#endif
    asm ("add.sat.s32 %0,%1,%2;" : "=r"(rlo) : "r"(alo), "r"(blo));
    asm ("add.sat.s32 %0,%1,%2;" : "=r"(rhi) : "r"(ahi), "r"(bhi));
#if __CUDA_ARCH__ < 200
    r = ((unsigned int)rhi & 0xffff0000U) | ((unsigned int)rlo >> 16);
#else
    asm ("prmt.b32 %0,%1,%2,0x7632;" : "=r"(r) : "r"(rlo), "r"(rhi));
#endif /* __CUDA_ARCH__ < 200 */
#endif /* __CUDA_ARCH__ >= 300 */
    return r;           // halfword-wise sum with signed saturation
}

static __device__ __forceinline__ unsigned int vaddus2 (unsigned int a, unsigned int b)
{
    unsigned int r;
#if __CUDA_ARCH__ >= 300
    unsigned int c = 0;
    asm ("vadd2.u32.u32.u32.sat %0,%1,%2,%3;" : "=r"(r):"r"(a),"r"(b),"r"(c));
#else  /* __CUDA_ARCH__ >= 300 */
    int alo, blo, rlo, ahi, bhi, rhi;
    asm ("{                              \n\t"
         "and.b32     %0, %4, 0xffff;    \n\t"
         "and.b32     %1, %5, 0xffff;    \n\t"
#if __CUDA_ARCH__ < 350
         "shr.u32     %2, %4, 16;        \n\t"
         "shr.u32     %3, %5, 16;        \n\t"
#else  /* __CUDA_ARCH__ < 350 */
         "shf.r.clamp.b32  %2, %4, 0, 16;\n\t"
         "shf.r.clamp.b32  %3, %5, 0, 16;\n\t"
#endif /* __CUDA_ARCH__ < 350 */
         "}"
         : "=r"(alo), "=r"(blo), "=r"(ahi), "=r"(bhi) 
         : "r"(a), "r"(b));
    rlo = min (alo + blo, 65535);
    rhi = min (ahi + bhi, 65535);
    r = (rhi << 16) + rlo;
#endif /* __CUDA_ARCH__ >= 300 */
    return r;           // halfword-wise sum with unsigned saturation
}

static __device__ __forceinline__ unsigned int vavgs2(unsigned int a, unsigned int b)
{
    unsigned int r;
#if __CUDA_ARCH__ >= 300
    unsigned int c = 0;
    asm ("vavrg2.s32.s32.s32 %0,%1,%2,%3;" : "=r"(r) : "r"(a), "r"(b), "r"(c));
#else  /* __CUDA_ARCH__ >= 300 */
    // avgs (a + b) = ((a + b) < 0) ? ((a + b) >> 1) : ((a + b + 1) >> 1). The 
    // two expressions can be re-written as follows to avoid needing additional
    // intermediate bits: ((a + b) >> 1) = (a >> 1) + (b >> 1) + ((a & b) & 1),
    // ((a + b + 1) >> 1) = (a >> 1) + (b >> 1) + ((a | b) & 1). The difference
    // between the two is ((a ^ b) & 1). Note that if (a + b) < 0, then also
    // ((a + b) >> 1) < 0, since right shift rounds to negative infinity. This
    // means we can compute ((a + b) >> 1) then conditionally add ((a ^ b) & 1)
    // depending on the sign bit of the shifted sum. By handling the msb sum 
    // bit of the result separately, we avoid carry-out during summation and
    // also can use (potentially faster) logical right shifts.
    asm ("{                      \n\t"
         ".reg .u32 a,b,c,r,s,t,u,v;\n\t"
         "mov.b32 a,%1;          \n\t"
         "mov.b32 b,%2;          \n\t"
         "and.b32 u,a,0xfffefffe;\n\t" // prevent shift crossing chunk boundary
         "and.b32 v,b,0xfffefffe;\n\t" // prevent shift crossing chunk boundary
         "xor.b32 s,a,b;         \n\t" // a ^ b
         "and.b32 t,a,b;         \n\t" // a & b
         "shr.u32 u,u,1;         \n\t" // a >> 1
         "shr.u32 v,v,1;         \n\t" // b >> 1
         "and.b32 c,s,0x00010001;\n\t" // (a ^ b) & 1
         "and.b32 s,s,0x80008000;\n\t" // extract msb (a ^ b)
         "and.b32 t,t,0x00010001;\n\t" // (a & b) & 1
         "add.u32 r,u,v;         \n\t" // (a>>1)+(b>>1) 
         "add.u32 r,r,t;         \n\t" // (a>>1)+(b>>1)+(a&b&1); rec. msb cy-in
         "xor.b32 r,r,s;         \n\t" // compute msb sum bit: a ^ b ^ cy-in
         "shr.u32 t,r,15;        \n\t" // sign ((a + b) >> 1)
         "not.b32 t,t;           \n\t" // ~sign ((a + b) >> 1)
         "and.b32 t,t,c;         \n\t" // ((a ^ b) & 1) & ~sign ((a + b) >> 1)
         "add.u32 r,r,t;         \n\t" // conditionally add ((a ^ b) & 1)
         "mov.b32 %0,r;          \n\t"
         "}"
         : "=r"(r) : "r"(a), "r"(b));
#endif /* __CUDA_ARCH__ >= 300 */
    return r;           // halfword-wise average of signed integers
}

static __device__ __forceinline__ unsigned int vavgu2(unsigned int a, unsigned int b)
{
    unsigned int r, c;
#if __CUDA_ARCH__ >= 300
    c = 0;
    asm ("vavrg2.u32.u32.u32 %0,%1,%2,%3;" : "=r"(r) : "r"(a), "r"(b), "r"(c));
#else  /* __CUDA_ARCH__ >= 300 */
    // HAKMEM #23: a + b = 2 * (a | b) - (a ^ b) ==>
    // (a + b + 1) / 2 = (a | b) - ((a ^ b) >> 1)
    c = a ^ b;           
    r = a | b;
    c = c & 0xfffefffe; // ensure shift doesn't cross half-word boundaries
    c = c >> 1;
    r = r - c;
#endif /* __CUDA_ARCH__ >= 300 */
    return r;           // halfword-wise average of unsigned integers
}

static __device__ __forceinline__ unsigned int vhaddu2(unsigned int a, unsigned int b)
{
    // HAKMEM #23: a + b = 2 * (a & b) + (a ^ b) ==>
    // (a + b) / 2 = (a & b) + ((a ^ b) >> 1)
    unsigned int r, s;
    s = a ^ b;
    r = a & b;
    s = s & 0xfffefffe; // ensure shift doesn't cross halfword boundaries
    s = s >> 1;
    r = r + s;
    return r;           // halfword-wise average of unsigned ints, rounded down
}

static __device__ __forceinline__ unsigned int vcmpeq2(unsigned int a, unsigned int b)
{
    unsigned int r, c;
#if __CUDA_ARCH__ >= 300
    c = 0;
    asm ("vset2.u32.u32.eq %0,%1,%2,%3;" : "=r"(r) : "r"(a), "r"(b), "r"(c));
    c = r << 16;        // convert bool
    r = c - r;          //  into mask
#else  /* __CUDA_ARCH__ >= 300 */
    // inspired by Alan Mycroft's null-byte detection algorithm:
    // null_byte(x) = ((x - 0x01010101) & (~x & 0x80808080))
    r = a ^ b;          // 0x0000 if a == b
    c = r | 0x80008000; // set msbs, to catch carry out
    r = r ^ c;          // extract msbs, msb = 1 if r < 0x8000
    c = c - 0x00010001; // msb = 0, if r was 0x0000 or 0x8000
    c = r & ~c;         // msb = 1, if r was 0x0000
#if __CUDA_ARCH__ >= 200
    asm ("prmt.b32 %0,%1,0,0xbb99;" : "=r"(r) : "r"(c));// convert msbs to mask
#else  /* __CUDA_ARCH__ >= 200 */
    asm ("shr.u32 %0,%1,15;" : "=r"(r) : "r"(c)); // convert
    asm ("sub.u32 %0,%1,%0;" : "+r"(r) : "r"(c)); //  msbs to
    asm ("or.b32  %0,%1,%0;" : "+r"(r) : "r"(c)); //   mask
#endif /* __CUDA_ARCH__ >= 200 */
#endif /* __CUDA_ARCH__ >= 300 */
    return r;           // halfword-wise (un)signed eq comparison, mask result
}

static __device__ __forceinline__ unsigned int vcmpges2(unsigned int a, unsigned int b)
{
    unsigned int r;
#if __CUDA_ARCH__ >= 300
    unsigned int c = 0;
    asm ("vset2.s32.s32.ge %0,%1,%2,%3;" : "=r"(r) : "r"(a), "r"(b), "r"(c));
    c = r << 16;        // convert bool
    r = c - r;          //  to mask
#else  /* __CUDA_ARCH__ >= 300 */
    asm ("{                             \n\t"
         ".reg .u32 a, b, r, s, t, u;   \n\t"
         "mov.b32        a,%1;          \n\t" 
         "mov.b32        b,%2;          \n\t"
         "and.b32        s,a,0xffff0000;\n\t" // high word of a
         "and.b32        t,b,0xffff0000;\n\t" // high word of b
         "set.ge.s32.s32 u,s,t;         \n\t" // compare two high words
         "cvt.s32.s16    s,a;           \n\t" // sign-extend low word of a
         "cvt.s32.s16    t,b;           \n\t" // sign-extend low word of b
         "set.ge.s32.s32 s,s,t;         \n\t" // compare two low words
#if __CUDA_ARCH__ >= 200
         "prmt.b32       r,s,u,0x7632;  \n\t" // combine low and high results
#else  /* __CUDA_ARCH__ >= 200 */
         "and.b32        u,u,0xffff0000;\n\t" // mask comparison result hi word
         "and.b32        s,s,0x0000ffff;\n\t" // mask comparison result lo word
         "or.b32         r,s,u;         \n\t" // combine the two results
#endif /* __CUDA_ARCH__ >= 200 */
         "mov.b32        %0,r;          \n\t"
         "}"
         : "=r"(r) : "r"(a), "r"(b));
#endif /* __CUDA_ARCH__ >= 300 */
    return r;           // halfword-wise signed gt-eq comparison, mask result
}

static __device__ __forceinline__ unsigned int vcmpgeu2(unsigned int a, unsigned int b)
{
    unsigned int r, c;
#if __CUDA_ARCH__ >= 300
    c = 0;
    asm ("vset2.u32.u32.ge %0,%1,%2,%3;" : "=r"(r) : "r"(a), "r"(b), "r"(c));
    c = r << 16;        // convert bool
    r = c - r;          //  into mask
#else  /* __CUDA_ARCH__ >= 300 */
    asm ("not.b32 %0,%0;" : "+r"(b));
    c = vavgu2 (a, b);  // (a + ~b + 1) / 2 = (a - b) / 2
#if __CUDA_ARCH__ >= 200
    asm ("prmt.b32 %0,%1,0,0xbb99;" : "=r"(r) : "r"(c));// build mask from msbs
#else  /* __CUDA_ARCH__ >= 200 */
    asm ("and.b32 %0,%0,0x80008000;" : "+r"(c));  // msb = carry-outs
    asm ("shr.u32 %0,%1,15;" : "=r"(r) : "r"(c)); // build mask
    asm ("sub.u32 %0,%1,%0;" : "+r"(r) : "r"(c)); //  from
    asm ("or.b32  %0,%1,%0;" : "+r"(r) : "r"(c)); //   msbs
#endif /* __CUDA_ARCH__ >= 200 */
#endif /* __CUDA_ARCH__ >= 300 */
    return r;           // halfword-wise unsigned gt-eq comparison, mask result
}

static __device__ __forceinline__ unsigned int vcmpgts2(unsigned int a, unsigned int b)
{
    unsigned int r;
#if __CUDA_ARCH__ >= 300
    unsigned int c = 0;
    asm ("vset2.s32.s32.gt %0,%1,%2,%3;" : "=r"(r) : "r"(a), "r"(b), "r"(c));
    c = r << 16;        // convert bool
    r = c - r;          //  to mask
#else  /* __CUDA_ARCH__ >= 300 */
    asm ("{                             \n\t"
         ".reg .u32 a, b, r, s, t, u;   \n\t"
         "mov.b32        a,%1;          \n\t" 
         "mov.b32        b,%2;          \n\t"
         "and.b32        s,a,0xffff0000;\n\t" // high word of a
         "and.b32        t,b,0xffff0000;\n\t" // high word of b
         "set.gt.s32.s32 u,s,t;         \n\t" // compare two high words
         "cvt.s32.s16    s,a;           \n\t" // sign-extend low word of a
         "cvt.s32.s16    t,b;           \n\t" // sign-extend low word of b
         "set.gt.s32.s32 s,s,t;         \n\t" // compare two low words
#if __CUDA_ARCH__ >= 200
         "prmt.b32       r,s,u,0x7632;  \n\t" // combine low and high results
#else  /* __CUDA_ARCH__ >= 200 */
         "and.b32        u,u,0xffff0000;\n\t" // mask comparison result hi word
         "and.b32        s,s,0x0000ffff;\n\t" // mask comparison result lo word
         "or.b32         r,s,u;         \n\t" // combine the two results
#endif /* __CUDA_ARCH__ >= 200 */
         "mov.b32        %0,r;          \n\t"
         "}"
         : "=r"(r) : "r"(a), "r"(b));
#endif /* __CUDA_ARCH__ >= 300 */
    return r;           // halfword-wise signed gt comparison with mask result
}

static __device__ __forceinline__ unsigned int vcmpgtu2(unsigned int a, unsigned int b)
{
    unsigned int r, c;
#if __CUDA_ARCH__ >= 300
    c = 0;
    asm ("vset2.u32.u32.gt %0,%1,%2,%3;" : "=r"(r) : "r"(a), "r"(b), "r"(c));
    c = r << 16;        // convert bool
    r = c - r;          //  into mask
#else  /* __CUDA_ARCH__ >= 300 */
    asm ("not.b32 %0,%0;" : "+r"(b));
    c = vhaddu2 (a, b); // (a + ~b) / 2 = (a - b) / 2 [rounded down]
#if __CUDA_ARCH__ >= 200
    asm ("prmt.b32 %0,%1,0,0xbb99;" : "=r"(r) : "r"(c));// build mask from msbs
#else  /* __CUDA_ARCH__ >= 200 */
    asm ("and.b32 %0,%0,0x80008000;" : "+r"(c));  // msb = carry-outs
    asm ("shr.u32 %0,%1,15;" : "=r"(r) : "r"(c)); // build mask
    asm ("sub.u32 %0,%1,%0;" : "+r"(r) : "r"(c)); //  from
    asm ("or.b32  %0,%1,%0;" : "+r"(r) : "r"(c)); //   msbs
#endif /* __CUDA_ARCH__ >= 200 */
#endif /* __CUDA_ARCH__ >= 300 */
    return r;           // halfword-wise unsigned gt comparison, mask result
}

static __device__ __forceinline__ unsigned int vcmples2(unsigned int a, unsigned int b)
{
    unsigned int r;
#if __CUDA_ARCH__ >= 300
    unsigned int c = 0;
    asm ("vset2.s32.s32.le %0,%1,%2,%3;" : "=r"(r) : "r"(a), "r"(b), "r"(c));
    c = r << 16;        // convert bool
    r = c - r;          //  to mask
#else  /* __CUDA_ARCH__ >= 300 */
    asm ("{                             \n\t"
         ".reg .u32 a, b, r, s, t, u;   \n\t"
         "mov.b32        a,%1;          \n\t" 
         "mov.b32        b,%2;          \n\t"
         "and.b32        s,a,0xffff0000;\n\t" // high word of a
         "and.b32        t,b,0xffff0000;\n\t" // high word of b
         "set.le.s32.s32 u,s,t;         \n\t" // compare two high words
         "cvt.s32.s16    s,a;           \n\t" // sign-extend low word of a
         "cvt.s32.s16    t,b;           \n\t" // sign-extend low word of b
         "set.le.s32.s32 s,s,t;         \n\t" // compare two low words
#if __CUDA_ARCH__ >= 200
         "prmt.b32       r,s,u,0x7632;  \n\t" // combine low and high results
#else  /* __CUDA_ARCH__ >= 200 */
         "and.b32        u,u,0xffff0000;\n\t" // mask comparison result hi word
         "and.b32        s,s,0x0000ffff;\n\t" // mask comparison result lo word
         "or.b32         r,s,u;         \n\t" // combine the two results
#endif /* __CUDA_ARCH__ >= 200 */
         "mov.b32        %0,r;          \n\t"
         "}"
         : "=r"(r) : "r"(a), "r"(b));
#endif /* __CUDA_ARCH__ >= 300 */
    return r;           // halfword-wise signed lt-eq comparison, mask result
}

static __device__ __forceinline__ unsigned int vcmpleu2(unsigned int a, unsigned int b)
{
    unsigned int r, c;
#if __CUDA_ARCH__ >= 300
    c = 0;
    asm ("vset2.u32.u32.le %0,%1,%2,%3;" : "=r"(r) : "r"(a), "r"(b), "r"(c));
    c = r << 16;        // convert bool
    r = c - r;          //  into mask
#else  /* __CUDA_ARCH__ >= 300 */
    asm ("not.b32 %0,%0;" : "+r"(a));
    c = vavgu2 (a, b);  // (b + ~a + 1) / 2 = (b - a) / 2
#if __CUDA_ARCH__ >= 200
    asm ("prmt.b32 %0,%1,0,0xbb99;" : "=r"(r) : "r"(c));// build mask from msbs
#else  /* __CUDA_ARCH__ >= 200 */
    asm ("and.b32 %0,%0,0x80008000;" : "+r"(c));  // msb = carry-outs
    asm ("shr.u32 %0,%1,15;" : "=r"(r) : "r"(c)); // build mask
    asm ("sub.u32 %0,%1,%0;" : "+r"(r) : "r"(c)); //  from
    asm ("or.b32  %0,%1,%0;" : "+r"(r) : "r"(c)); //   msbs
#endif /* __CUDA_ARCH__ >= 200 */
#endif /* __CUDA_ARCH__ >= 300 */
    return r;           // halfword-wise unsigned lt-eq comparison, mask result
}

static __device__ __forceinline__ unsigned int vcmplts2(unsigned int a, unsigned int b)
{
    unsigned int r;
#if __CUDA_ARCH__ >= 300
    unsigned int c = 0;
    asm ("vset2.s32.s32.lt %0,%1,%2,%3;" : "=r"(r) : "r"(a), "r"(b), "r"(c));
    c = r << 16;        // convert bool
    r = c - r;          //  to mask
#else  /* __CUDA_ARCH__ >= 300 */
    asm ("{                             \n\t"
         ".reg .u32 a, b, r, s, t, u;   \n\t"
         "mov.b32        a,%1;          \n\t" 
         "mov.b32        b,%2;          \n\t"
         "and.b32        s,a,0xffff0000;\n\t" // high word of a
         "and.b32        t,b,0xffff0000;\n\t" // high word of b
         "set.lt.s32.s32 u,s,t;         \n\t" // compare two high words
         "cvt.s32.s16    s,a;           \n\t" // sign-extend low word of a
         "cvt.s32.s16    t,b;           \n\t" // sign-extend low word of b
         "set.lt.s32.s32 s,s,t;         \n\t" // compare two low words
#if __CUDA_ARCH__ >= 200
         "prmt.b32       r,s,u,0x7632;  \n\t" // combine low and high results
#else  /* __CUDA_ARCH__ >= 200 */
         "and.b32        u,u,0xffff0000;\n\t" // mask comparison result hi word
         "and.b32        s,s,0x0000ffff;\n\t" // mask comparison result lo word
         "or.b32         r,s,u;         \n\t" // combine the two results
#endif /* __CUDA_ARCH__ >= 200 */
         "mov.b32        %0,r;          \n\t"
         "}"
         : "=r"(r) : "r"(a), "r"(b));
#endif /* __CUDA_ARCH__ >= 300 */
    return r;           // halfword-wise signed lt comparison with mask result
}

static __device__ __forceinline__ unsigned int vcmpltu2(unsigned int a, unsigned int b)
{
    unsigned int r, c;
#if __CUDA_ARCH__ >= 300
    c = 0;
    asm ("vset2.u32.u32.lt %0,%1,%2,%3;" : "=r"(r) : "r"(a), "r"(b), "r"(c));
    c = r << 16;        // convert bool
    r = c - r;          //  into mask
#else  /* __CUDA_ARCH__ >= 300 */
    asm ("not.b32 %0,%0;" : "+r"(a));
    c = vhaddu2 (a, b); // (b + ~a) / 2 = (b - a) / 2 [rounded down]
#if __CUDA_ARCH__ >= 200
    asm ("prmt.b32 %0,%1,0,0xbb99;" : "=r"(r) : "r"(c));// build mask from msbs
#else  /* __CUDA_ARCH__ >= 200 */
    asm ("and.b32 %0,%0,0x80008000;" : "+r"(c));  // msb = carry-outs
    asm ("shr.u32 %0,%1,15;" : "=r"(r) : "r"(c)); // build mask
    asm ("sub.u32 %0,%1,%0;" : "+r"(r) : "r"(c)); //  from
    asm ("or.b32  %0,%1,%0;" : "+r"(r) : "r"(c)); //   msbs
#endif /* __CUDA_ARCH__ >= 200 */
#endif /* __CUDA_ARCH__ >= 300 */
    return r;           // halfword-wise unsigned lt comparison, mask result
}

static __device__ __forceinline__ unsigned int vcmpne2(unsigned int a, unsigned int b)
{
    unsigned int r, c;
#if __CUDA_ARCH__ >= 300
    c = 0;
    asm ("vset2.u32.u32.ne %0,%1,%2,%3;" : "=r"(r) : "r"(a), "r"(b), "r"(c));
    c = r << 16;        // convert bool
    r = c - r;          //  into mask
#else  /* __CUDA_ARCH__ >= 300 */
    // inspired by Alan Mycroft's null-byte detection algorithm:
    // null_byte(x) = ((x - 0x01010101) & (~x & 0x80808080))
    r = a ^ b;          // 0x0000 if a == b
    c = r | 0x80008000; // set msbs, to catch carry out
    c = c - 0x00010001; // msb = 0, if r was 0x0000 or 0x8000
    c = r | c;          // msb = 1, if r was not 0x0000
#if __CUDA_ARCH__ >= 200
    asm ("prmt.b32 %0,%1,0,0xbb99;" : "=r"(r) : "r"(c));// build mask from msbs
#else  /* __CUDA_ARCH__ >= 200 */
    asm ("and.b32 %0,%0,0x80008000;" : "+r"(c));  // extract msbs
    asm ("shr.u32 %0,%1,15;" : "=r"(r) : "r"(c)); // build mask
    asm ("sub.u32 %0,%1,%0;" : "+r"(r) : "r"(c)); //  from
    asm ("or.b32  %0,%1,%0;" : "+r"(r) : "r"(c)); //   msbs
#endif /* __CUDA_ARCH__ >= 200 */
#endif /* __CUDA_ARCH__ >= 300 */
    return r;           // halfword-wise (un)signed ne comparison, mask result
}

static __device__ __forceinline__ unsigned int vabsdiffu2(unsigned int a, unsigned int b)
{
    unsigned int r, s;
#if __CUDA_ARCH__ >= 300
    s = 0;
    asm ("vabsdiff2.u32.u32.u32 %0,%1,%2,%3;" : "=r"(r) :"r"(a),"r"(b),"r"(s));
#else  /* __CUDA_ARCH__ >= 300 */
    unsigned int t, u, v;
    s = a & 0x0000ffff; // extract low halfword
    r = b & 0x0000ffff; // extract low halfword
    u = max (r, s);     // maximum of low halfwords
    v = min (r, s);     // minimum of low halfwords
    s = a & 0xffff0000; // extract high halfword
    r = b & 0xffff0000; // extract high halfword
    t = max (r, s);     // maximum of high halfwords
    s = min (r, s);     // minimum of high halfwords
    r = u | t;          // maximum of both halfwords
    s = v | s;          // minimum of both halfwords
    r = r - s;          // |a - b| = max(a,b) - min(a,b);
#endif /* __CUDA_ARCH__ >= 300 */
    return r;           // halfword-wise absolute difference of unsigned ints
}

static __device__ __forceinline__ unsigned int vmaxs2(unsigned int a, unsigned int b)
{
    unsigned int r, s;
#if __CUDA_ARCH__ >= 300
    s = 0;
    asm ("vmax2.s32.s32.s32 %0,%1,%2,%3;" : "=r"(r) : "r"(a), "r"(b), "r"(s));
#else  /* __CUDA_ARCH__ >= 300 */
    unsigned int t, u;
    asm ("cvt.s32.s16 %0,%1;" : "=r"(r) : "r"(a)); // extract low halfword
    asm ("cvt.s32.s16 %0,%1;" : "=r"(s) : "r"(b)); // extract low halfword
    t = max((int)r,(int)s); // maximum of low halfwords
    r = a & 0xffff0000;     // extract high halfword
    s = b & 0xffff0000;     // extract high halfword
    u = max((int)r,(int)s); // maximum of high halfwords
    r = u | (t & 0xffff);   // combine halfword maximums
#endif /* __CUDA_ARCH__ >= 300 */
    return r;           // halfword-wise maximum of signed integers
}

static __device__ __forceinline__ unsigned int vmaxu2(unsigned int a, unsigned int b)
{
    unsigned int r, s;
#if __CUDA_ARCH__ >= 300
    s = 0;
    asm ("vmax2.u32.u32.u32 %0,%1,%2,%3;" : "=r"(r) : "r"(a), "r"(b), "r"(s));
#else  /* __CUDA_ARCH__ >= 300 */
    unsigned int t, u;
    r = a & 0x0000ffff; // extract low halfword
    s = b & 0x0000ffff; // extract low halfword
    t = max (r, s);     // maximum of low halfwords
    r = a & 0xffff0000; // extract high halfword
    s = b & 0xffff0000; // extract high halfword
    u = max (r, s);     // maximum of high halfwords
    r = t | u;          // combine halfword maximums
#endif /* __CUDA_ARCH__ >= 300 */
    return r;           // halfword-wise maximum of unsigned integers
}

static __device__ __forceinline__ unsigned int vmins2(unsigned int a, unsigned int b)
{
    unsigned int r, s;
#if __CUDA_ARCH__ >= 300
    s = 0;
    asm ("vmin2.s32.s32.s32 %0,%1,%2,%3;" : "=r"(r) : "r"(a), "r"(b), "r"(s));
#else  /* __CUDA_ARCH__ >= 300 */
    unsigned int t, u;
    asm ("cvt.s32.s16 %0,%1;" : "=r"(r) : "r"(a)); // extract low halfword
    asm ("cvt.s32.s16 %0,%1;" : "=r"(s) : "r"(b)); // extract low halfword
    t = min((int)r,(int)s); // minimum of low halfwords
    r = a & 0xffff0000;     // extract high halfword
    s = b & 0xffff0000;     // extract high halfword
    u = min((int)r,(int)s); // minimum of high halfwords
    r = u | (t & 0xffff);   // combine halfword minimums
#endif /* __CUDA_ARCH__ >= 300 */
    return r;           // halfword-wise minimum of signed integers
}

static __device__ __forceinline__ unsigned int vminu2(unsigned int a, unsigned int b)
{
    unsigned int r, s;
#if __CUDA_ARCH__ >= 300
    s = 0;
    asm ("vmin2.u32.u32.u32 %0,%1,%2,%3;" : "=r"(r) : "r"(a), "r"(b), "r"(s));
#else  /* __CUDA_ARCH__ >= 300 */
    unsigned int t, u;
    r = a & 0x0000ffff; // extract low halfword
    s = b & 0x0000ffff; // extract low halfword
    t = min (r, s);     // minimum of low halfwords
    r = a & 0xffff0000; // extract high halfword
    s = b & 0xffff0000; // extract high halfword
    u = min (r, s);     // minimum of high halfwords
    r = t | u;          // combine halfword minimums
#endif /* __CUDA_ARCH__ >= 300 */
    return r;           // halfword-wise minimum of unsigned integers
}

static __device__ __forceinline__ unsigned int vseteq2(unsigned int a, unsigned int b)
{
    unsigned int r, c;
#if __CUDA_ARCH__ >= 300
    c = 0;
    asm ("vset2.u32.u32.eq %0,%1,%2,%3;" : "=r"(r) : "r"(a), "r"(b), "r"(c));
#else  /* __CUDA_ARCH__ >= 300 */
    // inspired by Alan Mycroft's null-byte detection algorithm:
    // null_byte(x) = ((x - 0x01010101) & (~x & 0x80808080))
    r = a ^ b;          // 0x0000 if a == b
    c = r | 0x80008000; // set msbs, to catch carry out
    r = r ^ c;          // extract msbs, msb = 1 if r < 0x8000
    c = c - 0x00010001; // msb = 0, if r was 0x0000 or 0x8000
    c = r & ~c;         // msb = 1, if r was 0x0000
    r = c >> 15;        // convert to bool
#endif /* __CUDA_ARCH__ >= 300 */
    return r;           // halfword-wise (un)signed eq comparison, bool result
}

static __device__ __forceinline__ unsigned int vsetges2(unsigned int a, unsigned int b)
{
    unsigned int r;
#if __CUDA_ARCH__ >= 300
    unsigned int c = 0;
    asm ("vset2.s32.s32.ge %0,%1,%2,%3;" : "=r"(r) : "r"(a), "r"(b), "r"(c));
#else  /* __CUDA_ARCH__ >= 300 */
    asm ("{                             \n\t"
         ".reg .u32 a, b, r, s, t, u;   \n\t"
         "mov.b32        a,%1;          \n\t" 
         "mov.b32        b,%2;          \n\t"
         "and.b32        s,a,0xffff0000;\n\t" // high word of a
         "and.b32        t,b,0xffff0000;\n\t" // high word of b
         "set.ge.s32.s32 u,s,t;         \n\t" // compare two high words
         "cvt.s32.s16    s,a;           \n\t" // sign-extend low word of a
         "cvt.s32.s16    t,b;           \n\t" // sign-extend low word of b
         "set.ge.s32.s32 s,s,t;         \n\t" // compare two low words
#if __CUDA_ARCH__ >= 200
         "prmt.b32       r,s,u,0x7632;  \n\t" // combine low and high results
         "and.b32        r,r,0x00010001;\n\t" // convert from mask to bool
#else  /* __CUDA_ARCH__ >= 200 */
         "and.b32        u,u,0x00010000;\n\t" // extract bool result of hi word
         "and.b32        s,s,0x00000001;\n\t" // extract bool result of lo word
         "or.b32         r,s,u;         \n\t" // combine the two results
#endif /* __CUDA_ARCH__ >= 200 */
         "mov.b32        %0,r;          \n\t"
         "}"
         : "=r"(r) : "r"(a), "r"(b));
#endif /* __CUDA_ARCH__ >= 300 */
    return r;           // halfword-wise signed gt-eq comparison, bool result
}

static __device__ __forceinline__ unsigned int vsetgeu2(unsigned int a, unsigned int b)
{
    unsigned int r, c;
#if __CUDA_ARCH__ >= 300
    c = 0;
    asm ("vset2.u32.u32.ge %0,%1,%2,%3;" : "=r"(r) : "r"(a), "r"(b), "r"(c));
#else  /* __CUDA_ARCH__ >= 300 */
    asm ("not.b32 %0,%0;" : "+r"(b));
    c = vavgu2 (a, b);  // (a + ~b + 1) / 2 = (a - b) / 2
    c = c & 0x80008000; // msb = carry-outs
    r = c >> 15;        // convert to bool
#endif /* __CUDA_ARCH__ >= 300 */
    return r;           // halfword-wise unsigned gt-eq comparison, bool result
}

static __device__ __forceinline__ unsigned int vsetgts2(unsigned int a, unsigned int b)
{
    unsigned int r;
#if __CUDA_ARCH__ >= 300
    unsigned int c = 0;
    asm ("vset2.s32.s32.gt %0,%1,%2,%3;" : "=r"(r) : "r"(a), "r"(b), "r"(c));
#else  /* __CUDA_ARCH__ >= 300 */
    asm ("{                             \n\t"
         ".reg .u32 a, b, r, s, t, u;   \n\t"
         "mov.b32        a,%1;          \n\t" 
         "mov.b32        b,%2;          \n\t"
         "and.b32        s,a,0xffff0000;\n\t" // high word of a
         "and.b32        t,b,0xffff0000;\n\t" // high word of b
         "set.gt.s32.s32 u,s,t;         \n\t" // compare two high words
         "cvt.s32.s16    s,a;           \n\t" // sign-extend low word of a
         "cvt.s32.s16    t,b;           \n\t" // sign-extend low word of b
         "set.gt.s32.s32 s,s,t;         \n\t" // compare two low words
#if __CUDA_ARCH__ >= 200
         "prmt.b32       r,s,u,0x7632;  \n\t" // combine low and high results
         "and.b32        r,r,0x00010001;\n\t" // convert from mask to bool
#else  /* __CUDA_ARCH__ >= 200 */
         "and.b32        u,u,0x00010000;\n\t" // extract bool result of hi word
         "and.b32        s,s,0x00000001;\n\t" // extract bool result of lo word
         "or.b32         r,s,u;         \n\t" // combine the two results
#endif /* __CUDA_ARCH__ >= 200 */
         "mov.b32        %0,r;          \n\t"
         "}"
         : "=r"(r) : "r"(a), "r"(b));
#endif /* __CUDA_ARCH__ >= 300 */
    return r;           // halfword-wise signed gt comparison with bool result
}

static __device__ __forceinline__ unsigned int vsetgtu2(unsigned int a, unsigned int b)
{
    unsigned int r, c;
#if __CUDA_ARCH__ >= 300
    c = 0;
    asm ("vset2.u32.u32.gt %0,%1,%2,%3;" : "=r"(r) : "r"(a), "r"(b), "r"(c));
#else  /* __CUDA_ARCH__ >= 300 */
    asm ("not.b32 %0,%0;" : "+r"(b));
    c = vhaddu2 (a, b); // (a + ~b) / 2 = (a - b) / 2 [rounded down]
    c = c & 0x80008000; // msbs = carry-outs
    r = c >> 15;        // convert to bool
#endif /* __CUDA_ARCH__ >= 300 */
    return r;           // halfword-wise unsigned gt comparison, bool result
}

static __device__ __forceinline__ unsigned int vsetles2(unsigned int a, unsigned int b)
{
    unsigned int r;
#if __CUDA_ARCH__ >= 300
    unsigned int c = 0;
    asm ("vset2.s32.s32.le %0,%1,%2,%3;" : "=r"(r) : "r"(a), "r"(b), "r"(c));
#else  /* __CUDA_ARCH__ >= 300 */
    asm ("{                             \n\t"
         ".reg .u32 a, b, r, s, t, u;   \n\t"
         "mov.b32        a,%1;          \n\t" 
         "mov.b32        b,%2;          \n\t"
         "and.b32        s,a,0xffff0000;\n\t" // high word of a
         "and.b32        t,b,0xffff0000;\n\t" // high word of b
         "set.le.s32.s32 u,s,t;         \n\t" // compare two high words
         "cvt.s32.s16    s,a;           \n\t" // sign-extend low word of a
         "cvt.s32.s16    t,b;           \n\t" // sign-extend low word of b
         "set.le.s32.s32 s,s,t;         \n\t" // compare two low words
#if __CUDA_ARCH__ >= 200
         "prmt.b32       r,s,u,0x7632;  \n\t" // combine low and high results
         "and.b32        r,r,0x00010001;\n\t" // convert from mask to bool
#else  /* __CUDA_ARCH__ >= 200 */
         "and.b32        u,u,0x00010000;\n\t" // extract bool result of hi word
         "and.b32        s,s,0x00000001;\n\t" // extract bool result of lo word
         "or.b32         r,s,u;         \n\t" // combine the two results
#endif /* __CUDA_ARCH__ >= 200 */
         "mov.b32        %0,r;          \n\t"
         "}"
         : "=r"(r) : "r"(a), "r"(b));
#endif /* __CUDA_ARCH__ >= 300 */
    return r;           // halfword-wise signed lt-eq comparison, bool result
}

static __device__ __forceinline__ unsigned int vsetleu2(unsigned int a, unsigned int b)
{
    unsigned int r, c;
#if __CUDA_ARCH__ >= 300
    c = 0;
    asm ("vset2.u32.u32.le %0,%1,%2,%3;" : "=r"(r) : "r"(a), "r"(b), "r"(c));
#else  /* __CUDA_ARCH__ >= 300 */
    asm ("not.b32 %0,%0;" : "+r"(a));
    c = vavgu2 (a, b);  // (b + ~a + 1) / 2 = (b - a) / 2
    c = c & 0x80008000; // msb = carry-outs
    r = c >> 15;        // convert to bool
#endif /* __CUDA_ARCH__ >= 300 */
    return r;           // halfword-wise unsigned lt-eq comparison, bool result
}

static __device__ __forceinline__ unsigned int vsetlts2(unsigned int a, unsigned int b)
{
    unsigned int r;
#if __CUDA_ARCH__ >= 300
    unsigned int c = 0;
    asm ("vset2.s32.s32.lt %0,%1,%2,%3;" : "=r"(r) : "r"(a), "r"(b), "r"(c));
#else  /* __CUDA_ARCH__ >= 300 */
    asm ("{                             \n\t"
         ".reg .u32 a, b, r, s, t, u;   \n\t"
         "mov.b32        a,%1;          \n\t" 
         "mov.b32        b,%2;          \n\t"
         "and.b32        s,a,0xffff0000;\n\t" // high word of a
         "and.b32        t,b,0xffff0000;\n\t" // high word of b
         "set.lt.s32.s32 u,s,t;         \n\t" // compare two high words
         "cvt.s32.s16    s,a;           \n\t" // sign-extend low word of a
         "cvt.s32.s16    t,b;           \n\t" // sign-extend low word of b
         "set.lt.s32.s32 s,s,t;         \n\t" // compare two low words
#if __CUDA_ARCH__ >= 200
         "prmt.b32       r,s,u,0x7632;  \n\t" // combine low and high results
         "and.b32        r,r,0x00010001;\n\t" // convert from mask to bool
#else  /* __CUDA_ARCH__ >= 200 */
         "and.b32        u,u,0x00010000;\n\t" // extract bool result of hi word
         "and.b32        s,s,0x00000001;\n\t" // extract bool result of lo word
         "or.b32         r,s,u;         \n\t" // combine the two results
#endif /* __CUDA_ARCH__ >= 200 */
         "mov.b32        %0,r;          \n\t"
         "}"
         : "=r"(r) : "r"(a), "r"(b));
#endif /* __CUDA_ARCH__ >= 300 */
    return r;           // halfword-wise signed lt comparison with bool result
}

static __device__ __forceinline__ unsigned int vsetltu2(unsigned int a, unsigned int b)
{
    unsigned int r, c;
#if __CUDA_ARCH__ >= 300
    c = 0;
    asm ("vset2.u32.u32.lt %0,%1,%2,%3;" : "=r"(r) : "r"(a), "r"(b), "r"(c));
#else  /* __CUDA_ARCH__ >= 300 */
    asm ("not.b32 %0,%0;" : "+r"(a));
    c = vhaddu2 (a, b); // (b + ~a) / 2 = (b - a) / 2 [rounded down]
    c = c & 0x80008000; // msb = carry-outs
    r = c >> 15;        // convert to bool
#endif /* __CUDA_ARCH__ >= 300 */
    return r;           // halfword-wise unsigned lt comparison, bool result
}

static __device__ __forceinline__ unsigned int vsetne2(unsigned int a, unsigned int b)
{
    unsigned int r, c;
#if __CUDA_ARCH__ >= 300
    c = 0;
    asm ("vset2.u32.u32.ne %0,%1,%2,%3;" : "=r"(r) : "r"(a), "r"(b), "r"(c));
#else  /* __CUDA_ARCH__ >= 300 */
    // inspired by Alan Mycroft's null-byte detection algorithm:
    // null_byte(x) = ((x - 0x01010101) & (~x & 0x80808080))
    r = a ^ b;          // 0x0000 if a == b
    c = r | 0x80008000; // set msbs, to catch carry out
    c = c - 0x00010001; // msb = 0, if r was 0x0000 or 0x8000
    c = r | c;          // msb = 1, if r was not 0x0000
    c = c & 0x80008000; // extract msbs
    r = c >> 15;        // convert to bool
#endif /* __CUDA_ARCH__ >= 300 */
    return r;           // halfword-wise (un)signed ne comparison, bool result
}

static __device__ __forceinline__ unsigned int vsadu2(unsigned int a, unsigned int b)
{
    unsigned int r, s;
#if __CUDA_ARCH__ >= 300
    s = 0;
    asm("vabsdiff2.u32.u32.u32.add %0,%1,%2,%3;":"=r"(r):"r"(a),"r"(b),"r"(s));
#else  /* __CUDA_ARCH__ >= 300 */
    unsigned int t, u, v;
    s = a & 0x0000ffff; // extract low halfword
    r = b & 0x0000ffff; // extract low halfword
    u = max (r, s);     // maximum of low halfwords
    v = min (r, s);     // minimum of low halfwords
    s = a & 0xffff0000; // extract high halfword
    r = b & 0xffff0000; // extract high halfword
    t = max (r, s);     // maximum of high halfwords
    s = min (r, s);     // minimum of high halfwords
    u = u - v;          // low halfword: |a - b| = max(a,b) - min(a,b); 
    t = t - s;          // high halfword: |a - b| = max(a,b) - min(a,b);
#if __CUDA_ARCH__ < 350
    asm ("shr.u32 %0,%0,16;" : "+r"(t));
#else  /*__CUDA_ARCH__ < 350 */
    asm ("shf.r.clamp.b32  %0,%0,0,16;" : "+r"(t));
#endif /*__CUDA_ARCH__ < 350 */
    r = t + u;          // sum absolute halfword differences
#endif /* __CUDA_ARCH__ >= 300 */
    return r;           // halfword-wise sum of abs differences of unsigned int
}

static __device__ __forceinline__ unsigned int vsub2(unsigned int a, unsigned int b)
{
    unsigned int s, t;
#if __CUDA_ARCH__ >= 300
    s = 0;
    asm ("vsub2.u32.u32.u32 %0,%1,%2,%3;" : "=r"(t) : "r"(a), "r"(b), "r"(s));
#else  /* __CUDA_ARCH__ >= 300 */
    s = a ^ b;          // sum bits
    t = a - b;          // actual sum
    s = s ^ t;          // determine carry-ins for each bit position
    s = s & 0x00010000; // borrow to high word 
    t = t + s;          // compensate for borrow from low word
#endif /* __CUDA_ARCH__ >= 300 */
    return t;           // halfword-wise difference
}

static __device__ __forceinline__ unsigned int vsubss2 (unsigned int a, unsigned int b)
{
    unsigned int r;
#if __CUDA_ARCH__ >= 300
    unsigned int c = 0;
    asm ("vsub2.s32.s32.s32.sat %0,%1,%2,%3;" : "=r"(r):"r"(a),"r"(b),"r"(c));
#else  /* __CUDA_ARCH__ >= 300 */
    int ahi, alo, blo, bhi, rhi, rlo;
    ahi = (int)((a & 0xffff0000U));
    bhi = (int)((b & 0xffff0000U));
#if __CUDA_ARCH__ < 200
    alo = (int)(a << 16);
    blo = (int)(b << 16);
#elif __CUDA_ARCH__ < 350
    // work around (we would want left shifts at least for sm_2x)
    asm ("prmt.b32 %0,%1,0,0x1044;" : "=r"(alo) : "r"(a));
    asm ("prmt.b32 %0,%1,0,0x1044;" : "=r"(blo) : "r"(b));
#else  /* __CUDA_ARCH__ < 350 */
    asm ("shf.l.clamp.b32 %0,0,%1,16;" : "=r"(alo) : "r"(a));
    asm ("shf.l.clamp.b32 %0,0,%1,16;" : "=r"(blo) : "r"(b));
#endif /* __CUDA_ARCH__ < 350 */
    asm ("sub.sat.s32 %0,%1,%2;" : "=r"(rlo) : "r"(alo), "r"(blo));
    asm ("sub.sat.s32 %0,%1,%2;" : "=r"(rhi) : "r"(ahi), "r"(bhi));
#if __CUDA_ARCH__ < 200
    r = ((unsigned int)rhi & 0xffff0000U) | ((unsigned int)rlo >> 16);
#else  /* __CUDA_ARCH__ < 200 */
    asm ("prmt.b32 %0,%1,%2,0x7632;" : "=r"(r) : "r"(rlo), "r"(rhi));
#endif /* __CUDA_ARCH__ < 200 */
#endif /* __CUDA_ARCH__ >= 300 */
    return r;           // halfword-wise difference with signed saturation
}

static __device__ __forceinline__ unsigned int vsubus2 (unsigned int a, unsigned int b)
{
    unsigned int r;
#if __CUDA_ARCH__ >= 300
    unsigned int c = 0;
    asm ("vsub2.u32.u32.u32.sat %0,%1,%2,%3;" : "=r"(r):"r"(a),"r"(b),"r"(c));
#else  /* __CUDA_ARCH__ >= 300 */
    int alo, blo, rlo, ahi, bhi, rhi;
    asm ("{                              \n\t"
         "and.b32     %0, %4, 0xffff;    \n\t"
         "and.b32     %1, %5, 0xffff;    \n\t"
#if __CUDA_ARCH__ < 350
         "shr.u32     %2, %4, 16;        \n\t"
         "shr.u32     %3, %5, 16;        \n\t"
#else  /* __CUDA_ARCH__ < 350 */
         "shf.r.clamp.b32  %2, %4, 0, 16;\n\t"
         "shf.r.clamp.b32  %3, %5, 0, 16;\n\t"
#endif /* __CUDA_ARCH__ < 350 */
         "}"
         : "=r"(alo), "=r"(blo), "=r"(ahi), "=r"(bhi) 
         : "r"(a), "r"(b));
    rlo = max ((int)(alo - blo), 0);
    rhi = max ((int)(ahi - bhi), 0);
    r = rhi * 65536 + rlo;
#endif /* __CUDA_ARCH__ >= 300 */
    return r;           // halfword-wise difference with unsigned saturation
}

static __device__ __forceinline__ unsigned int vneg2(unsigned int a)
{
    return vsub2 (0, a);// halfword-wise negation with wrap-around
}

static __device__ __forceinline__ unsigned int vnegss2(unsigned int a)
{
    return vsubss2(0,a);// halfword-wise negation with signed saturation
}

static __device__ __forceinline__ unsigned int vabsdiffs2(unsigned int a, unsigned int b)
{
    unsigned int r, s;
#if __CUDA_ARCH__ >= 300
    s = 0;
    asm ("vabsdiff2.s32.s32.s32 %0,%1,%2,%3;" : "=r"(r) :"r"(a),"r"(b),"r"(s));
#else  /* __CUDA_ARCH__ >= 300 */
    s = vcmpges2 (a, b);// mask = 0xff if a >= b
    r = a ^ b;          //
    s = (r & s) ^ b;    // select a when a >= b, else select b => max(a,b)
    r = s ^ r;          // select a when b >= a, else select b => min(a,b)
    r = vsub2 (s, r);   // |a - b| = max(a,b) - min(a,b);
#endif /* __CUDA_ARCH__ >= 300 */
    return r;           // halfword-wise absolute difference of signed integers
}

static __device__ __forceinline__ unsigned int vsads2(unsigned int a, unsigned int b)
{
    unsigned int r, s;
#if __CUDA_ARCH__ >= 300
    s = 0;
    asm("vabsdiff2.s32.s32.s32.add %0,%1,%2,%3;":"=r"(r):"r"(a),"r"(b),"r"(s));
#else  /* __CUDA_ARCH__ >= 300 */
    s = vabsdiffs2 (a, b);
    r = (s >> 16) + (s & 0x0000ffff);
#endif /*  __CUDA_ARCH__ >= 300 */
    return r;           // halfword-wise sum of abs. differences of signed ints
}

static __device__ __forceinline__ unsigned int vabs4(unsigned int a)
{
    unsigned int r;
#if __CUDA_ARCH__ >= 300
    unsigned int b = 0, c = 0;
    asm ("vabsdiff4.s32.s32.s32 %0,%1,%2,%3;" : "=r"(r) :"r"(a),"r"(b),"r"(c));
#elif __CUDA_ARCH__ >= 200
    asm ("{                       \n\t"
         ".reg .u32 a,m,r;        \n\t"
         "mov.b32  a,%1;          \n\t"
         "prmt.b32 m,a,0,0xba98;  \n\t" // msb ? 0xff : 00
         "xor.b32  r,a,m;         \n\t" // conditionally invert bits
         "and.b32  m,m,0x01010101;\n\t" // msb ? 0x1 : 0
         "add.u32  r,r,m;         \n\t" // conditionally add 1
         "mov.b32  %0,r;          \n\t"
         "}"
         : "=r"(r) : "r"(a));
#else  /* __CUDA_ARCH__ >= 200 */
    asm ("{                  \n\t"
         ".reg .u32 a,m,r,s; \n\t"
         "mov.b32  a,%1;          \n\t"
         "and.b32  m,a,0x80808080;\n\t" // extract msb
         "and.b32  r,a,0x7f7f7f7f;\n\t" // clear msb
         "shr.u32  s,m,7;         \n\t" // build lsb mask
         "sub.u32  m,m,s;         \n\t" //  from msb
         "xor.b32  r,r,m;         \n\t" // conditionally invert lsbs
         "add.u32  r,r,s;         \n\t" // conditionally add 1
         "mov.b32  %0,r;          \n\t"
         "}"
         : "=r"(r) : "r"(a));
#endif /* __CUDA_ARCH__ >= 200 */
    return r;           // byte-wise absolute value, with wrap-around
}

static __device__ __forceinline__ unsigned int vabsss4(unsigned int a)
{
    unsigned int r;
#if __CUDA_ARCH__ >= 300
    unsigned int b = 0, c = 0;
    asm("vabsdiff4.s32.s32.s32.sat %0,%1,%2,%3;":"=r"(r):"r"(a),"r"(b),"r"(c));
#elif __CUDA_ARCH__ >= 200
    asm ("{                       \n\t"
         ".reg .u32 a,m,r;        \n\t"
         "mov.b32  a,%1;          \n\t"
         "prmt.b32 m,a,0,0xba98;  \n\t" // msb ? 0xff : 00
         "xor.b32  r,a,m;         \n\t" // conditionally invert bits
         "and.b32  m,m,0x01010101;\n\t" // msb ? 0x1 : 0
         "add.u32  r,r,m;         \n\t" // conditionally add 1
         "prmt.b32 m,r,0,0xba98;  \n\t" // msb ? 0xff : 00
         "and.b32  m,m,0x01010101;\n\t" // msb ? 0x1 : 0
         "sub.u32  r,r,m;         \n\t" // subtract 1 if result wrapped around
         "mov.b32  %0,r;          \n\t"
         "}"
         : "=r"(r) : "r"(a));
#else  /* __CUDA_ARCH__ >= 200 */
    asm ("{                       \n\t"
         ".reg .u32 a,m,r,s;      \n\t"
         "mov.b32  a,%1;          \n\t"
         "and.b32  m,a,0x80808080;\n\t" // extract msb
         "and.b32  r,a,0x7f7f7f7f;\n\t" // clear msb
         "shr.u32  s,m,7;         \n\t" // build lsb mask
         "sub.u32  m,m,s;         \n\t" //  from msb
         "xor.b32  r,r,m;         \n\t" // conditionally invert lsbs
         "add.u32  r,r,s;         \n\t" // conditionally add 1
         "and.b32  m,r,0x80808080;\n\t" // extract msb (1 if wrap-around)
         "shr.u32  s,m,7;         \n\t" // msb ? 1 : 0
         "sub.u32  r,r,s;         \n\t" // subtract 1 if result wrapped around
         "mov.b32  %0,r;          \n\t"
         "}"
         : "=r"(r) : "r"(a));
#endif /* __CUDA_ARCH__ >= 200 */
    return r;           // byte-wise absolute value with signed saturation
}

static __device__ __forceinline__ unsigned int vadd4(unsigned int a, unsigned int b)
{
#if __CUDA_ARCH__ >= 300
    unsigned int r, c = 0;
    asm ("vadd4.u32.u32.u32 %0,%1,%2,%3;" : "=r"(r) : "r"(a), "r"(b), "r"(c));
#else  /* __CUDA_ARCH__ >= 300 */
    unsigned int r, s, t;
    s = a ^ b;          // sum bits
    r = a & 0x7f7f7f7f; // clear msbs
    t = b & 0x7f7f7f7f; // clear msbs
    s = s & 0x80808080; // msb sum bits
    r = r + t;          // add without msbs, record carry-out in msbs
    r = r ^ s;          // sum of msb sum and carry-in bits, w/o carry-out
#endif /* __CUDA_ARCH__ >= 300 */
    return r;           // byte-wise sum, with wrap-around
}

static __device__ __forceinline__ unsigned int vaddss4 (unsigned int a, unsigned int b)
{
#if __CUDA_ARCH__ >= 300
    unsigned int r, c = 0;
    asm ("vadd4.sat.s32.s32.s32 %0,%1,%2,%3;" : "=r"(r):"r"(a),"r"(b),"r"(c));
#else  /* __CUDA_ARCH__ >= 300 */
    /*
      For signed saturation, saturation is controlled by the overflow signal: 
      ovfl = (carry-in to msb) XOR (carry-out from msb). Overflow can only 
      occur when the msbs of both inputs are the same. The defined response to
      overflow is to deliver 0x7f when the addends are positive (bit 7 clear),
      and 0x80 when the addends are negative (bit 7 set). The truth table for
      the msb is

      a   b   cy_in  res  cy_out  ovfl
      --------------------------------
      0   0       0    0       0     0
      0   0       1    1       0     1
      0   1       0    1       0     0
      0   1       1    0       1     0
      1   0       0    1       0     0
      1   0       1    0       1     0
      1   1       0    0       1     1
      1   1       1    1       1     0

      The seven low-order bits can be handled by simple wrapping addition with
      the carry out from bit 6 recorded in the msb (thus corresponding to the 
      cy_in in the truth table for the msb above). ovfl can be computed in many
      equivalent ways, here we use ovfl = (a ^ carry_in) & ~(a ^ b) since we 
      already need to compute (a ^ b) for the msb sum bit computation. First we
      compute the normal, wrapped addition result. When overflow is detected,
      we mask off the msb of the result, then compute a mask covering the seven
      low order bits, which are all set to 1. This sets the byte to 0x7f as we
      previously cleared the msb. In the overflow case, the sign of the result
      matches the sign of either of the inputs, so we extract the sign of a and
      add it to the low order bits, which turns 0x7f into 0x80, the correct 
      result for an overflowed negative result.
    */
    unsigned int r;
    asm ("{                         \n\t" 
         ".reg .u32 a,b,r,s,t,u;    \n\t"
         "mov.b32  a, %1;           \n\t" 
         "mov.b32  b, %2;           \n\t"
         "and.b32  r, a, 0x7f7f7f7f;\n\t" // clear msbs
         "and.b32  t, b, 0x7f7f7f7f;\n\t" // clear msbs
         "xor.b32  s, a, b;         \n\t" // sum bits = (a ^ b)
         "add.u32  r, r, t;         \n\t" // capture msb carry-in in bit 7
         "xor.b32  t, a, r;         \n\t" // a ^ carry_in
         "not.b32  u, s;            \n\t" // ~(a ^ b)
         "and.b32  t, t, u;         \n\t" // ovfl = (a ^ carry_in) & ~(a ^ b)
         "and.b32  s, s, 0x80808080;\n\t" // msb sum bits
         "xor.b32  r, r, s;         \n\t" // msb result = (a ^ b ^ carry_in)
#if __CUDA_ARCH__ >= 200
         "prmt.b32 s,a,0,0xba98;    \n\t" // sign(a) ? 0xff : 0
         "xor.b32  s,s,0x7f7f7f7f;  \n\t" // sign(a) ? 0x80 : 0x7f
         "prmt.b32 t,t,0,0xba98;    \n\t" // ovfl ? 0xff : 0
         "and.b32  s,s,t;           \n\t" // ovfl ? (sign(a) ? 0x80:0x7f) : 0
         "not.b32  t,t;             \n\t" // ~ovfl
         "and.b32  r,r,t;           \n\t" // ovfl ? 0 : a + b
         "or.b32   r,r,s;           \n\t" // ovfl ? (sign(a) ? 0x80:0x7f) : a+b
#else  /* __CUDA_ARCH__ >= 200 */
         "and.b32  t, t, 0x80808080;\n\t" // ovfl ? 0x80 : 0
         "shr.u32  s, t, 7;         \n\t" // ovfl ? 1 : 0
         "not.b32  u, t;            \n\t" // ovfl ? 0x7f : 0xff
         "and.b32  r, r, u;         \n\t" // ovfl ? (a + b) & 0x7f : a + b
         "and.b32  u, a, t;         \n\t" // ovfl ? a & 0x80 : 0
         "sub.u32  t, t, s;         \n\t" // ovfl ? 0x7f : 0
         "shr.u32  u, u, 7;         \n\t" // ovfl ? sign(a) : 0
         "or.b32   r, r, t;         \n\t" // ovfl ? 0x7f : a + b
         "add.u32  r, r, u;         \n\t" // ovfl ? 0x7f+sign(a) : a + b
#endif /* __CUDA_ARCH__ >= 200 */
         "mov.b32  %0, r;           \n\t"
         "}"
         : "=r"(r) : "r"(a), "r"(b));
#endif /* __CUDA_ARCH__ >= 300 */
    return r;           // byte-wise sum with signed saturation
}

static __device__ __forceinline__ unsigned int vaddus4 (unsigned int a, unsigned int b)
{
#if __CUDA_ARCH__ >= 300
    unsigned int r, c = 0;
    asm ("vadd4.u32.u32.u32.sat %0,%1,%2,%3;" : "=r"(r):"r"(a),"r"(b),"r"(c));
#else  /* __CUDA_ARCH__ >= 300 */
    // This code uses the same basic approach used for non-saturating addition.
    // The seven low-order bits in each byte are summed by regular addition,
    // with the carry-out from bit 6 (= carry-in for the msb) being recorded 
    // in bit 7, while the msb is handled separately.
    //
    // The fact that this is a saturating addition simplifies the handling of
    // the msb. When carry-out from the msb occurs, the entire byte must be
    // written as 0xff, and the computed msb is overwritten in the process. 
    // The corresponding entries in the truth table for the result msb thus 
    // become "don't cares":
    //
    // a  b  cy-in  res  cy-out
    // ------------------------
    // 0  0    0     0     0
    // 0  0    1     1     0
    // 0  1    0     1     0
    // 0  1    1     X     1
    // 1  0    0     1     0
    // 1  0    1     X     1
    // 1  1    0     X     1
    // 1  1    1     X     1
    //
    // As is easily seen, the simplest implementation of the result msb bit is 
    // simply (a | b | cy-in), with masking needed to isolate the msb. Note 
    // that this computation also makes the msb handling redundant with the 
    // clamping to 0xFF, because the msb is already set to 1 when saturation 
    // occurs. This means we only need to apply saturation to the seven lsb
    // bits in each byte, by overwriting with 0x7F. Saturation is controlled
    // by carry-out from the msb, which can be represented by various Boolean
    // expressions. Since to compute (a | b | cy-in) we need to compute (a | b)
    // anyhow, most efficient of these is cy-out = ((a & b) | cy-in) & (a | b).
    unsigned int r;
    asm ("{                         \n\t" 
         ".reg .u32 a,b,r,s,t,m;    \n\t"
         "mov.b32  a, %1;           \n\t" 
         "mov.b32  b, %2;           \n\t"
         "or.b32   m, a, b;         \n\t" // (a | b)
         "and.b32  r, a, 0x7f7f7f7f;\n\t" // clear msbs
         "and.b32  t, b, 0x7f7f7f7f;\n\t" // clear msbs
         "and.b32  m, m, 0x80808080;\n\t" // (a | b), isolate msbs
         "add.u32  r, r, t;         \n\t" // add w/o msbs, record msb-carry-ins
         "and.b32  t, a, b;         \n\t" // (a & b)
         "or.b32   t, t, r;         \n\t" // (a & b) | cy-in)
         "or.b32   r, r, m;         \n\t" // msb = cy-in | (a | b)
         "and.b32  t, t, m;         \n\t" // cy-out=((a&b)|cy-in)&(a|b),in msbs
#if __CUDA_ARCH__ >= 200
         "prmt.b32 t, t, 0, 0xba98; \n\t" // cy-out ? 0xff : 0
#else  /* __CUDA_ARCH__ >= 200 */
         "shr.u32  s, t, 7;         \n\t" // cy-out ? 1 : 0
         "sub.u32  t, t, s;         \n\t" // lsb-overwrite: cy-out ? 0x7F : 0
#endif /* __CUDA_ARCH__ >= 200 */
         "or.b32   r, r, t;         \n\t" // conditionally overwrite lsbs
         "mov.b32  %0, r;           \n\t"
         "}"
         : "=r"(r) : "r"(a), "r"(b));
#endif /* __CUDA_ARCH__ >= 300 */
    return r;           // byte-wise sum with unsigned saturation
}

static __device__ __forceinline__ unsigned int vavgs4(unsigned int a, unsigned int b)
{
    unsigned int r;
#if __CUDA_ARCH__ >= 300
    unsigned int c = 0;
    asm ("vavrg4.s32.s32.s32 %0,%1,%2,%3;" : "=r"(r) : "r"(a), "r"(b), "r"(c));
#else  /* __CUDA_ARCH__ >= 300 */
    // avgs (a + b) = ((a + b) < 0) ? ((a + b) >> 1) : ((a + b + 1) >> 1). The 
    // two expressions can be re-written as follows to avoid needing additional
    // intermediate bits: ((a + b) >> 1) = (a >> 1) + (b >> 1) + ((a & b) & 1),
    // ((a + b + 1) >> 1) = (a >> 1) + (b >> 1) + ((a | b) & 1). The difference
    // between the two is ((a ^ b) & 1). Note that if (a + b) < 0, then also
    // ((a + b) >> 1) < 0, since right shift rounds to negative infinity. This
    // means we can compute ((a + b) >> 1) then conditionally add ((a ^ b) & 1)
    // depending on the sign bit of the shifted sum. By handling the msb sum 
    // bit of the result separately, we avoid carry-out during summation and
    // also can use (potentially faster) logical right shifts.
    asm ("{                      \n\t"
         ".reg .u32 a,b,c,r,s,t,u,v;\n\t"
         "mov.b32 a,%1;          \n\t" 
         "mov.b32 b,%2;          \n\t" 
         "and.b32 u,a,0xfefefefe;\n\t" // prevent shift crossing chunk boundary
         "and.b32 v,b,0xfefefefe;\n\t" // prevent shift crossing chunk boundary
         "xor.b32 s,a,b;         \n\t" // a ^ b
         "and.b32 t,a,b;         \n\t" // a & b
         "shr.u32 u,u,1;         \n\t" // a >> 1
         "shr.u32 v,v,1;         \n\t" // b >> 1
         "and.b32 c,s,0x01010101;\n\t" // (a ^ b) & 1
         "and.b32 s,s,0x80808080;\n\t" // extract msb (a ^ b)
         "and.b32 t,t,0x01010101;\n\t" // (a & b) & 1
         "add.u32 r,u,v;         \n\t" // (a>>1)+(b>>1) 
         "add.u32 r,r,t;         \n\t" // (a>>1)+(b>>1)+(a&b&1); rec. msb cy-in
         "xor.b32 r,r,s;         \n\t" // compute msb sum bit: a ^ b ^ cy-in
         "shr.u32 t,r,7;         \n\t" // sign ((a + b) >> 1)
         "not.b32 t,t;           \n\t" // ~sign ((a + b) >> 1)
         "and.b32 t,t,c;         \n\t" // ((a ^ b) & 1) & ~sign ((a + b) >> 1)
         "add.u32 r,r,t;         \n\t" // conditionally add ((a ^ b) & 1)
         "mov.b32 %0,r;          \n\t"
         "}"
         : "=r"(r) : "r"(a), "r"(b));
#endif /* __CUDA_ARCH__ >= 300 */
    return r;           // byte-wise average of signed integers
}

static __device__ __forceinline__ unsigned int vavgu4(unsigned int a, unsigned int b)
{
    unsigned int r, c;
#if __CUDA_ARCH__ >= 300
    c = 0;
    asm ("vavrg4.u32.u32.u32 %0,%1,%2,%3;" : "=r"(r) : "r"(a), "r"(b), "r"(c));
#else  /* __CUDA_ARCH__ >= 300 */
    // HAKMEM #23: a + b = 2 * (a | b) - (a ^ b) ==>
    // (a + b + 1) / 2 = (a | b) - ((a ^ b) >> 1)
    c = a ^ b;           
    r = a | b;
    c = c & 0xfefefefe; // ensure following shift doesn't cross byte boundaries
    c = c >> 1;
    r = r - c;
#endif /* __CUDA_ARCH__ >= 300 */
    return r;           // byte-wise average of unsigned integers
}

static __device__ __forceinline__ unsigned int vhaddu4(unsigned int a, unsigned int b)
{
    // HAKMEM #23: a + b = 2 * (a & b) + (a ^ b) ==>
    // (a + b) / 2 = (a & b) + ((a ^ b) >> 1)
    unsigned int r, s;   
    s = a ^ b;           
    r = a & b;
    s = s & 0xfefefefe; // ensure following shift doesn't cross byte boundaries
    s = s >> 1;
    s = r + s;
    return s;           // byte-wise average of unsigned integers, rounded down
}

static __device__ __forceinline__ unsigned int vcmpeq4(unsigned int a, unsigned int b)
{
    unsigned int c, r;
#if __CUDA_ARCH__ >= 300
    r = 0;
    asm ("vset4.u32.u32.eq %0,%1,%2,%3;" : "=r"(c) : "r"(a), "r"(b), "r"(r));
    r = c << 8;         // convert bool
    r = r - c;          //  to mask
#else  /* __CUDA_ARCH__ >= 300 */
    // inspired by Alan Mycroft's null-byte detection algorithm:
    // null_byte(x) = ((x - 0x01010101) & (~x & 0x80808080))
    r = a ^ b;          // 0x00 if a == b
    c = r | 0x80808080; // set msbs, to catch carry out
    r = r ^ c;          // extract msbs, msb = 1 if r < 0x80
    c = c - 0x01010101; // msb = 0, if r was 0x00 or 0x80
    c = r & ~c;         // msb = 1, if r was 0x00
#if __CUDA_ARCH__ >= 200
    asm ("prmt.b32 %0,%1,0,0xba98;" : "=r"(r) : "r"(c));// convert msbs to mask
#else  /* __CUDA_ARCH__ >= 200 */
    asm ("shr.u32 %0,%1,7;" : "=r"(r) : "r"(c));  // convert
    asm ("sub.u32 %0,%1,%0;" : "+r"(r) : "r"(c)); //  msbs to
    asm ("or.b32  %0,%1,%0;" : "+r"(r) : "r"(c)); //   mask
#endif /* __CUDA_ARCH__ >= 200 */
#endif /* __CUDA_ARCH__ >= 300 */
    return r;           // byte-wise (un)signed eq comparison with mask result
}

static __device__ __forceinline__ unsigned int vcmpges4(unsigned int a, unsigned int b)
{
    unsigned int r;
#if __CUDA_ARCH__ >= 300
    unsigned int c = 0;
    asm ("vset4.s32.s32.ge %0,%1,%2,%3;" : "=r"(r) : "r"(a), "r"(b), "r"(c));
    c = r << 8;         // convert bool
    r = c - r;          //  to mask
#else  /* __CUDA_ARCH__ >= 300 */
    asm ("{                          \n\t"
         ".reg .u32 a, b, r, s, t, u;\n\t"
         "mov.b32     a,%1;          \n\t" 
         "mov.b32     b,%2;          \n\t"
         "xor.b32     s,a,b;         \n\t" // a ^ b
         "or.b32      r,a,0x80808080;\n\t" // set msbs
         "and.b32     t,b,0x7f7f7f7f;\n\t" // clear msbs
         "sub.u32     r,r,t;         \n\t" // subtract lsbs, msb: ~borrow-in
         "xor.b32     t,r,a;         \n\t" // msb: ~borrow-in ^ a
         "xor.b32     r,r,s;         \n\t" // msb: ~sign(res) = a^b^~borrow-in
         "and.b32     t,t,s;         \n\t" // msb: ovfl= (~bw-in ^ a) & (a ^ b)
         "xor.b32     t,t,r;         \n\t" // msb: ge = ovfl != ~sign(res)
#if __CUDA_ARCH__ >= 200
         "prmt.b32    r,t,0,0xba98;  \n\t" // build mask from msbs
#else  /* __CUDA_ARCH__ >= 200 */
         "and.b32     t,t,0x80808080;\n\t" // isolate msbs = ovfl
         "shr.u32     r,t,7;         \n\t" // build mask
         "sub.u32     r,t,r;         \n\t" //  from
         "or.b32      r,r,t;         \n\t" //   msbs
#endif /* __CUDA_ARCH__ >= 200 */
         "mov.b32     %0,r;          \n\t"
         "}"
         : "=r"(r) : "r"(a), "r"(b));
#endif /* __CUDA_ARCH__ >= 300 */
    return r;           // byte-wise signed gt-eq comparison with mask result
}

static __device__ __forceinline__ unsigned int vcmpgeu4(unsigned int a, unsigned int b)
{
    unsigned int r, c;
#if __CUDA_ARCH__ >= 300
    c = 0;
    asm ("vset4.u32.u32.ge %0,%1,%2,%3;" : "=r"(r) : "r"(a), "r"(b), "r"(c));
    c = r << 8;         // convert bool
    r = c - r;          //  to mask
#else  /* __CUDA_ARCH__ >= 300 */
    asm ("not.b32 %0,%0;" : "+r"(b));
    c = vavgu4 (a, b);  // (a + ~b + 1) / 2 = (a - b) / 2
#if __CUDA_ARCH__ >= 200
    asm ("prmt.b32 %0,%1,0,0xba98;" : "=r"(r) : "r"(c));// build mask from msbs
#else  /* __CUDA_ARCH__ >= 200 */
    asm ("and.b32 %0,%0,0x80808080;" : "+r"(c));  // msb = carry-outs
    asm ("shr.u32 %0,%1,7;" : "=r"(r) : "r"(c));  // build mask
    asm ("sub.u32 %0,%1,%0;" : "+r"(r) : "r"(c)); //  from
    asm ("or.b32  %0,%1,%0;" : "+r"(r) : "r"(c)); //   msbs
#endif /* __CUDA_ARCH__ >= 200 */
#endif /* __CUDA_ARCH__ >= 300 */
    return r;           // byte-wise unsigned gt-eq comparison with mask result
}

static __device__ __forceinline__ unsigned int vcmpgts4(unsigned int a, unsigned int b)
{
    unsigned int r;
#if __CUDA_ARCH__ >= 300
    unsigned int c = 0;
    asm ("vset4.s32.s32.gt %0,%1,%2,%3;" : "=r"(r) : "r"(a), "r"(b), "r"(c));
    c = r << 8;         // convert bool
    r = c - r;          //  to mask
#else  /* __CUDA_ARCH__ >= 300 */
    /* a <= b <===> a + ~b < 0 */
    asm ("{                       \n\t" 
         ".reg .u32 a,b,r,s,t,u;  \n\t"
         "mov.b32  a,%1;          \n\t" 
         "mov.b32  b,%2;          \n\t"
         "not.b32  b,b;           \n\t"
         "and.b32  r,a,0x7f7f7f7f;\n\t" // clear msbs
         "and.b32  t,b,0x7f7f7f7f;\n\t" // clear msbs
         "xor.b32  s,a,b;         \n\t" // sum bits = (a ^ b)
         "add.u32  r,r,t;         \n\t" // capture msb carry-in in bit 7
         "xor.b32  t,a,r;         \n\t" // a ^ carry_in
         "not.b32  u,s;           \n\t" // ~(a ^ b)
         "and.b32  t,t,u;         \n\t" // msb: ovfl = (a ^ carry_in) & ~(a^b)
         "xor.b32  r,r,u;         \n\t" // msb: ~result = (~(a ^ b) ^ carry_in)
         "xor.b32  t,t,r;         \n\t" // msb: gt = ovfl != sign(~res)
#if __CUDA_ARCH__ >= 200
         "prmt.b32 r,t,0,0xba98;  \n\t" // build mask from msbs
#else  /* __CUDA_ARCH__ >= 200 */
         "and.b32  t,t,0x80808080;\n\t" // isolate msbs
         "shr.u32  r,t,7;         \n\t" // build mask
         "sub.u32  r,t,r;         \n\t" //  from
         "or.b32   r,r,t;         \n\t" //   msbs
#endif /* __CUDA_ARCH__ >= 200 */
         "mov.b32  %0,r;          \n\t"
         "}"
         : "=r"(r) : "r"(a), "r"(b));
#endif /* __CUDA_ARCH__ >= 300 */
    return r;           // byte-wise signed gt comparison with mask result
}

static __device__ __forceinline__ unsigned int vcmpgtu4(unsigned int a, unsigned int b)
{
    unsigned int r, c;
#if __CUDA_ARCH__ >= 300
    c = 0;
    asm ("vset4.u32.u32.gt %0,%1,%2,%3;" : "=r"(r) : "r"(a), "r"(b), "r"(c));
    c = r << 8;         // convert bool
    r = c - r;          //  to mask
#else  /* __CUDA_ARCH__ >= 300 */
    asm ("not.b32 %0,%0;" : "+r"(b));
    c = vhaddu4 (a, b); // (a + ~b) / 2 = (a - b) / 2 [rounded down]
#if __CUDA_ARCH__ >= 200
    asm ("prmt.b32 %0,%1,0,0xba98;" : "=r"(r) : "r"(c));// build mask from msbs
#else  /* __CUDA_ARCH__ >= 200 */
    asm ("and.b32 %0,%0,0x80808080;" : "+r"(c));  // msb = carry-outs
    asm ("shr.u32 %0,%1,7;" : "=r"(r) : "r"(c));  // build mask
    asm ("sub.u32 %0,%1,%0;" : "+r"(r) : "r"(c)); //  from
    asm ("or.b32  %0,%1,%0;" : "+r"(r) : "r"(c)); //   msbs
#endif /* __CUDA_ARCH__ >= 200 */
#endif /* __CUDA_ARCH__ >= 300 */
    return r;           // byte-wise unsigned gt comparison with mask result
}

static __device__ __forceinline__ unsigned int vcmples4(unsigned int a, unsigned int b)
{
    unsigned int r;
#if __CUDA_ARCH__ >= 300
    unsigned int c = 0;
    asm ("vset4.s32.s32.le %0,%1,%2,%3;" : "=r"(r) : "r"(a), "r"(b), "r"(c));
    c = r << 8;         // convert bool
    r = c - r;          //  to mask
#else  /* __CUDA_ARCH__ >= 300 */
    /* a <= b <===> a + ~b < 0 */
    asm ("{                       \n\t" 
         ".reg .u32 a,b,r,s,t,u;  \n\t"
         "mov.b32  a,%1;          \n\t" 
         "mov.b32  b,%2;          \n\t"
         "not.b32  u,b;           \n\t" // ~b
         "and.b32  r,a,0x7f7f7f7f;\n\t" // clear msbs
         "and.b32  t,u,0x7f7f7f7f;\n\t" // clear msbs
         "xor.b32  u,a,b;         \n\t" // sum bits = (a ^ b)
         "add.u32  r,r,t;         \n\t" // capture msb carry-in in bit 7
         "xor.b32  t,a,r;         \n\t" // a ^ carry_in
         "not.b32  s,u;           \n\t" // ~(a ^ b)
         "and.b32  t,t,u;         \n\t" // msb: ovfl = (a ^ carry_in) & (a ^ b)
         "xor.b32  r,r,s;         \n\t" // msb: result = (a ^ ~b ^ carry_in)
         "xor.b32  t,t,r;         \n\t" // msb: le = ovfl != sign(res)
#if __CUDA_ARCH__ >= 200
         "prmt.b32 r,t,0,0xba98;  \n\t" // build mask from msbs
#else  /* __CUDA_ARCH__ >= 200 */
         "and.b32  t,t,0x80808080;\n\t" // isolate msbs
         "shr.u32  r,t,7;         \n\t" // build mask
         "sub.u32  r,t,r;         \n\t" //  from
         "or.b32   r,r,t;         \n\t" //   msbs
#endif /* __CUDA_ARCH__ >= 200 */
         "mov.b32  %0,r;          \n\t"
         "}"
         : "=r"(r) : "r"(a), "r"(b));
#endif /* __CUDA_ARCH__ >= 300 */
    return r;           // byte-wise signed lt-eq comparison with mask result
}

static __device__ __forceinline__ unsigned int vcmpleu4(unsigned int a, unsigned int b)
{
    unsigned int r, c;
#if __CUDA_ARCH__ >= 300
    c = 0;
    asm ("vset4.u32.u32.le %0,%1,%2,%3;" : "=r"(r) : "r"(a), "r"(b), "r"(c));
    c = r << 8;         // convert bool
    r = c - r;          //  to mask
#else  /* __CUDA_ARCH__ >= 300 */
    asm ("not.b32 %0,%0;" : "+r"(a));
    c = vavgu4 (a, b);  // (b + ~a + 1) / 2 = (b - a) / 2
#if __CUDA_ARCH__ >= 200
    asm ("prmt.b32 %0,%1,0,0xba98;" : "=r"(r) : "r"(c));// build mask from msbs
#else  /* __CUDA_ARCH__ >= 200 */
    asm ("and.b32 %0,%0,0x80808080;" : "+r"(c));  // msb = carry-outs
    asm ("shr.u32 %0,%1,7;" : "=r"(r) : "r"(c));  // build mask
    asm ("sub.u32 %0,%1,%0;" : "+r"(r) : "r"(c)); //  from
    asm ("or.b32  %0,%1,%0;" : "+r"(r) : "r"(c)); //   msbs
#endif /* __CUDA_ARCH__ >= 200 */
#endif /* __CUDA_ARCH__ >= 300 */
    return r;           // byte-wise unsigned lt-eq comparison with mask result
}

static __device__ __forceinline__ unsigned int vcmplts4(unsigned int a, unsigned int b)
{
    unsigned int r;
#if __CUDA_ARCH__ >= 300
    unsigned int c = 0;
    asm ("vset4.s32.s32.lt %0,%1,%2,%3;" : "=r"(r) : "r"(a), "r"(b), "r"(c));
    c = r << 8;         // convert bool
    r = c - r;          //  to mask
#else  /* __CUDA_ARCH__ >= 300 */
    asm ("{                          \n\t"
         ".reg .u32 a, b, r, s, t, u;\n\t"
         "mov.b32     a,%1;          \n\t" 
         "mov.b32     b,%2;          \n\t"
         "not.b32     u,b;           \n\t" // ~b
         "xor.b32     s,u,a;         \n\t" // a ^ ~b
         "or.b32      r,a,0x80808080;\n\t" // set msbs
         "and.b32     t,b,0x7f7f7f7f;\n\t" // clear msbs
         "sub.u32     r,r,t;         \n\t" // subtract lsbs, msb: ~borrow-in
         "xor.b32     t,r,a;         \n\t" // msb: ~borrow-in ^ a
         "not.b32     u,s;           \n\t" // msb: ~(a^~b)
         "xor.b32     r,r,s;         \n\t" // msb: res = a ^ ~b ^ ~borrow-in
         "and.b32     t,t,u;         \n\t" // msb: ovfl= (~bw-in ^ a) & ~(a^~b)
         "xor.b32     t,t,r;         \n\t" // msb: lt = ovfl != sign(res)
#if __CUDA_ARCH__ >= 200
         "prmt.b32    r,t,0,0xba98;  \n\t" // build mask from msbs
#else  /* __CUDA_ARCH__ >= 200 */
         "and.b32     t,t,0x80808080;\n\t" // isolate msbs
         "shr.u32     r,t,7;         \n\t" // build mask
         "sub.u32     r,t,r;         \n\t" //  from
         "or.b32      r,r,t;         \n\t" //   msbs
#endif /* __CUDA_ARCH__ >= 200 */
         "mov.b32     %0,r;          \n\t"
         "}"
         : "=r"(r) : "r"(a), "r"(b));
#endif /* __CUDA_ARCH__ >= 300 */
    return r;           // byte-wise signed lt comparison with mask result
}

static __device__ __forceinline__ unsigned int vcmpltu4(unsigned int a, unsigned int b)
{
    unsigned int r, c;
#if __CUDA_ARCH__ >= 300
    c = 0;
    asm ("vset4.u32.u32.lt %0,%1,%2,%3;" : "=r"(r) : "r"(a), "r"(b), "r"(c));
    c = r << 8;         // convert bool
    r = c - r;          //  to mask
#else  /* __CUDA_ARCH__ >= 300 */
    asm ("not.b32 %0,%0;" : "+r"(a));
    c = vhaddu4 (a, b); // (b + ~a) / 2 = (b - a) / 2 [rounded down]
#if __CUDA_ARCH__ >= 200
    asm ("prmt.b32 %0,%1,0,0xba98;" : "=r"(r) : "r"(c));// build mask from msbs
#else  /* __CUDA_ARCH__ >= 200 */
    asm ("and.b32 %0,%0,0x80808080;" : "+r"(c));  // msb = carry-outs
    asm ("shr.u32 %0,%1,7;" : "=r"(r) : "r"(c));  // build mask
    asm ("sub.u32 %0,%1,%0;" : "+r"(r) : "r"(c)); //  from
    asm ("or.b32  %0,%1,%0;" : "+r"(r) : "r"(c)); //   msbs
#endif /* __CUDA_ARCH__ >= 200 */
#endif /* __CUDA_ARCH__ >= 300 */
    return r;           // byte-wise unsigned lt comparison with mask result
}

static __device__ __forceinline__ unsigned int vcmpne4(unsigned int a, unsigned int b)
{
    unsigned int r, c;
#if __CUDA_ARCH__ >= 300
    c = 0;
    asm ("vset4.u32.u32.ne %0,%1,%2,%3;" : "=r"(r) : "r"(a), "r"(b), "r"(c));
    c = r << 8;         // convert bool
    r = c - r;          //  to mask
#else  /* __CUDA_ARCH__ >= 300 */
    // inspired by Alan Mycroft's null-byte detection algorithm:
    // null_byte(x) = ((x - 0x01010101) & (~x & 0x80808080))
    r = a ^ b;          // 0x00 if a == b
    c = r | 0x80808080; // set msbs, to catch carry out
    c = c - 0x01010101; // msb = 0, if r was 0x00 or 0x80
    c = r | c;          // msb = 1, if r was not 0x00
#if __CUDA_ARCH__ >= 200
    asm ("prmt.b32 %0,%1,0,0xba98;" : "=r"(r) : "r"(c));// build mask from msbs
#else  /* __CUDA_ARCH__ >= 200 */
    asm ("and.b32 %0,%0,0x80808080;" : "+r"(c));  // extract msbs
    asm ("shr.u32 %0,%1,7;" : "=r"(r) : "r"(c));  // build mask
    asm ("sub.u32 %0,%1,%0;" : "+r"(r) : "r"(c)); //  from
    asm ("or.b32  %0,%1,%0;" : "+r"(r) : "r"(c)); //   msbs
#endif /* __CUDA_ARCH__ >= 200 */
#endif /* __CUDA_ARCH__ >= 300 */
    return r;           // byte-wise (un)signed ne comparison with mask result
}

static __device__ __forceinline__ unsigned int vabsdiffu4(unsigned int a, unsigned int b)
{
    unsigned int r, s;
#if __CUDA_ARCH__ >= 300
    s = 0;
    asm ("vabsdiff4.u32.u32.u32 %0,%1,%2,%3;" : "=r"(r) :"r"(a),"r"(b),"r"(s));
#else  /* __CUDA_ARCH__ >= 300 */
    s = vcmpgeu4 (a, b);// mask = 0xff if a >= b
    r = a ^ b;          //
    s = (r & s) ^ b;    // select a when a >= b, else select b => max(a,b)
    r = s ^ r;          // select a when b >= a, else select b => min(a,b)
    r = s - r;          // |a - b| = max(a,b) - min(a,b);
#endif /* __CUDA_ARCH__ >= 300 */
    return r;           // byte-wise absolute difference of unsigned integers
}

static __device__ __forceinline__ unsigned int vmaxs4(unsigned int a, unsigned int b)
{
    unsigned int r, s;
#if __CUDA_ARCH__ >= 300
    s = 0;
    asm ("vmax4.s32.s32.s32 %0,%1,%2,%3;" : "=r"(r) : "r"(a), "r"(b), "r"(s));
#else  /* __CUDA_ARCH__ >= 300 */
    s = vcmpges4 (a, b);// mask = 0xff if a >= b
    r = a & s;          // select a when b >= a
    s = b & ~s;         // select b when b < a
    r = r | s;          // combine byte selections
#endif /* __CUDA_ARCH__ >= 300 */
    return r;           // byte-wise maximum of signed integers
}

static __device__ __forceinline__ unsigned int vmaxu4(unsigned int a, unsigned int b)
{
    unsigned int r, s;
#if __CUDA_ARCH__ >= 300
    s = 0;
    asm ("vmax4.u32.u32.u32 %0,%1,%2,%3;" : "=r"(r) : "r"(a), "r"(b), "r"(s));
#else  /* __CUDA_ARCH__ >= 300 */
    s = vcmpgeu4 (a, b);// mask = 0xff if a >= b
    r = a & s;          // select a when b >= a
    s = b & ~s;         // select b when b < a
    r = r | s;          // combine byte selections
#endif /* __CUDA_ARCH__ >= 300 */
    return r;           // byte-wise maximum of unsigned integers
}

static __device__ __forceinline__ unsigned int vmins4(unsigned int a, unsigned int b)
{
    unsigned int r, s;
#if __CUDA_ARCH__ >= 300
    s = 0;
    asm ("vmin4.s32.s32.s32 %0,%1,%2,%3;" : "=r"(r) : "r"(a), "r"(b), "r"(s));
#else  /* __CUDA_ARCH__ >= 300 */
    s = vcmpges4 (b, a);// mask = 0xff if a >= b
    r = a & s;          // select a when b >= a
    s = b & ~s;         // select b when b < a
    r = r | s;          // combine byte selections
#endif /* __CUDA_ARCH__ >= 300 */
    return r;           // byte-wise minimum of signed integers
}

static __device__ __forceinline__ unsigned int vminu4(unsigned int a, unsigned int b)
{
    unsigned int r, s;
#if __CUDA_ARCH__ >= 300
    s = 0;
    asm ("vmin4.u32.u32.u32 %0,%1,%2,%3;" : "=r"(r) : "r"(a), "r"(b), "r"(s));
#else  /* __CUDA_ARCH__ >= 300 */
    s = vcmpgeu4 (b, a);// mask = 0xff if a >= b
    r = a & s;          // select a when b >= a
    s = b & ~s;         // select b when b < a
    r = r | s;          // combine byte selections
#endif /* __CUDA_ARCH__ >= 300 */
    return r;           // byte-wise minimum of unsigned integers
}
static __device__ __forceinline__ unsigned int vseteq4(unsigned int a, unsigned int b)
{
    unsigned int r, c;
#if __CUDA_ARCH__ >= 300
    c = 0;
    asm ("vset4.u32.u32.eq %0,%1,%2,%3;" : "=r"(r) : "r"(a), "r"(b), "r"(c));
#else  /* __CUDA_ARCH__ >= 300 */
    // inspired by Alan Mycroft's null-byte detection algorithm:
    // null_byte(x) = ((x - 0x01010101) & (~x & 0x80808080))
    r = a ^ b;          // 0x00 if a == b
    c = r | 0x80808080; // set msbs, to catch carry out
    r = r ^ c;          // extract msbs, msb = 1 if r < 0x80
    c = c - 0x01010101; // msb = 0, if r was 0x00 or 0x80
    c = r & ~c;         // msb = 1, if r was 0x00
    r = c >> 7;         // convert to bool
#endif /* __CUDA_ARCH__ >= 300 */
    return r;           // byte-wise (un)signed eq comparison with bool result
}

static __device__ __forceinline__ unsigned int vsetles4(unsigned int a, unsigned int b)
{
    unsigned int r;
#if __CUDA_ARCH__ >= 300
    unsigned int c = 0;
    asm ("vset4.s32.s32.le %0,%1,%2,%3;" : "=r"(r) : "r"(a), "r"(b), "r"(c));
#else  /* __CUDA_ARCH__ >= 300 */
    /* a <= b <===> a + ~b < 0 */
    asm ("{                       \n\t" 
         ".reg .u32 a,b,r,s,t,u;  \n\t"
         "mov.b32  a,%1;          \n\t" 
         "mov.b32  b,%2;          \n\t"
         "not.b32  u,b;           \n\t" // ~b
         "and.b32  r,a,0x7f7f7f7f;\n\t" // clear msbs
         "and.b32  t,u,0x7f7f7f7f;\n\t" // clear msbs
         "xor.b32  u,a,b;         \n\t" // sum bits = (a ^ b)
         "add.u32  r,r,t;         \n\t" // capture msb carry-in in bit 7
         "xor.b32  t,a,r;         \n\t" // a ^ carry_in
         "not.b32  s,u;           \n\t" // ~(a ^ b)
         "and.b32  t,t,u;         \n\t" // msb: ovfl = (a ^ carry_in) & (a ^ b)
         "xor.b32  r,r,s;         \n\t" // msb: result = (a ^ ~b ^ carry_in)
         "xor.b32  t,t,r;         \n\t" // msb: le = ovfl != sign(res)
         "and.b32  t,t,0x80808080;\n\t" // isolate msbs
         "shr.u32  r,t,7;         \n\t" // convert to bool
         "mov.b32  %0,r;          \n\t"
         "}"
         : "=r"(r) : "r"(a), "r"(b));
#endif /* __CUDA_ARCH__ >= 300 */
    return r;           // byte-wise signed lt-eq comparison with bool result
}

static __device__ __forceinline__ unsigned int vsetleu4(unsigned int a, unsigned int b)
{
    unsigned int r, c;
#if __CUDA_ARCH__ >= 300
    c = 0;
    asm ("vset4.u32.u32.le %0,%1,%2,%3;" : "=r"(r) : "r"(a), "r"(b), "r"(c));
#else  /* __CUDA_ARCH__ >= 300 */
    asm ("not.b32 %0,%0;" : "+r"(a));
    c = vavgu4 (a, b);  // (b + ~a + 1) / 2 = (b - a) / 2
    c = c & 0x80808080; // msb = carry-outs
    r = c >> 7;         // convert to bool
#endif /* __CUDA_ARCH__ >= 300 */
    return r;           // byte-wise unsigned lt-eq comparison with bool result
}

static __device__ __forceinline__ unsigned int vsetlts4(unsigned int a, unsigned int b)
{
    unsigned int r;
#if __CUDA_ARCH__ >= 300
    unsigned int c = 0;
    asm ("vset4.s32.s32.lt %0,%1,%2,%3;" : "=r"(r) : "r"(a), "r"(b), "r"(c));
#else  /* __CUDA_ARCH__ >= 300 */
    asm ("{                          \n\t"
         ".reg .u32 a, b, r, s, t, u;\n\t"
         "mov.b32     a,%1;          \n\t" 
         "mov.b32     b,%2;          \n\t"
         "not.b32     u,b;           \n\t" // ~b
         "or.b32      r,a,0x80808080;\n\t" // set msbs
         "and.b32     t,b,0x7f7f7f7f;\n\t" // clear msbs
         "xor.b32     s,u,a;         \n\t" // a ^ ~b
         "sub.u32     r,r,t;         \n\t" // subtract lsbs, msb: ~borrow-in
         "xor.b32     t,r,a;         \n\t" // msb: ~borrow-in ^ a
         "not.b32     u,s;           \n\t" // msb: ~(a^~b)
         "xor.b32     r,r,s;         \n\t" // msb: res = a ^ ~b ^ ~borrow-in
         "and.b32     t,t,u;         \n\t" // msb: ovfl= (~bw-in ^ a) & ~(a^~b)
         "xor.b32     t,t,r;         \n\t" // msb: lt = ovfl != sign(res)
         "and.b32     t,t,0x80808080;\n\t" // isolate msbs
         "shr.u32     r,t,7;         \n\t" // convert to bool
         "mov.b32     %0,r;          \n\t"
         "}"
         : "=r"(r) : "r"(a), "r"(b));
#endif /* __CUDA_ARCH__ >= 300 */
    return r;           // byte-wise signed lt comparison with bool result
}

static __device__ __forceinline__ unsigned int vsetltu4(unsigned int a, unsigned int b)
{
    unsigned int r, c;
#if __CUDA_ARCH__ >= 300
    c = 0;
    asm ("vset4.u32.u32.lt %0,%1,%2,%3;" : "=r"(r) : "r"(a), "r"(b), "r"(c));
#else  /* __CUDA_ARCH__ >= 300 */
    asm ("not.b32 %0,%0;" : "+r"(a));
    c = vhaddu4 (a, b); // (b + ~a) / 2 = (b - a) / 2 [rounded down]
    c = c & 0x80808080; // msb = carry-outs
    r = c >> 7;         // convert to bool
#endif /* __CUDA_ARCH__ >= 300 */
    return r;           // byte-wise unsigned lt comparison with bool result
}

static __device__ __forceinline__ unsigned int vsetges4(unsigned int a, unsigned int b)
{
    unsigned int r;
#if __CUDA_ARCH__ >= 300
    unsigned int c = 0;
    asm ("vset4.s32.s32.ge %0,%1,%2,%3;" : "=r"(r) : "r"(a), "r"(b), "r"(c));
#else  /* __CUDA_ARCH__ >= 300 */
    asm ("{                          \n\t"
         ".reg .u32 a, b, r, s, t, u;\n\t"
         "mov.b32     a,%1;          \n\t" 
         "mov.b32     b,%2;          \n\t"
         "xor.b32     s,a,b;         \n\t" // a ^ b
         "or.b32      r,a,0x80808080;\n\t" // set msbs
         "and.b32     t,b,0x7f7f7f7f;\n\t" // clear msbs
         "sub.u32     r,r,t;         \n\t" // subtract lsbs, msb: ~borrow-in
         "xor.b32     t,r,a;         \n\t" // msb: ~borrow-in ^ a
         "xor.b32     r,r,s;         \n\t" // msb: ~sign(res) = a^b^~borrow-in
         "and.b32     t,t,s;         \n\t" // msb: ovfl= (~bw-in ^ a) & (a ^ b)
         "xor.b32     t,t,r;         \n\t" // msb: ge = ovfl != ~sign(res)
         "and.b32     t,t,0x80808080;\n\t" // isolate msbs
         "shr.u32     r,t,7;         \n\t" // convert to bool
         "mov.b32     %0,r;          \n\t"
         "}"
         : "=r"(r) : "r"(a), "r"(b));
#endif /* __CUDA_ARCH__ >= 300 */
    return r;           // byte-wise signed gt-eq comparison with bool result
}

static __device__ __forceinline__ unsigned int vsetgeu4(unsigned int a, unsigned int b)
{
    unsigned int r, c;
#if __CUDA_ARCH__ >= 300
    c = 0;
    asm ("vset4.u32.u32.ge %0,%1,%2,%3;" : "=r"(r) : "r"(a), "r"(b), "r"(c));
#else  /* __CUDA_ARCH__ >= 300 */
    asm ("not.b32 %0,%0;" : "+r"(b));
    c = vavgu4 (a, b);  // (a + ~b + 1) / 2 = (a - b) / 2
    c = c & 0x80808080; // msb = carry-outs
    r = c >> 7;         // convert to bool
#endif /* __CUDA_ARCH__ >= 300 */
    return r;           // byte-wise unsigned gt-eq comparison with bool result
}

static __device__ __forceinline__ unsigned int vsetgts4(unsigned int a, unsigned int b)
{
    unsigned int r;
#if __CUDA_ARCH__ >= 300
    unsigned int c = 0;
    asm ("vset4.s32.s32.gt %0,%1,%2,%3;" : "=r"(r) : "r"(a), "r"(b), "r"(c));
#else  /* __CUDA_ARCH__ >= 300 */
    /* a <= b <===> a + ~b < 0 */
    asm ("{                       \n\t" 
         ".reg .u32 a,b,r,s,t,u;  \n\t"
         "mov.b32  a,%1;          \n\t" 
         "mov.b32  b,%2;          \n\t"
         "not.b32  b,b;           \n\t"
         "and.b32  r,a,0x7f7f7f7f;\n\t" // clear msbs
         "and.b32  t,b,0x7f7f7f7f;\n\t" // clear msbs
         "xor.b32  s,a,b;         \n\t" // sum bits = (a ^ b)
         "add.u32  r,r,t;         \n\t" // capture msb carry-in in bit 7
         "xor.b32  t,a,r;         \n\t" // a ^ carry_in
         "not.b32  u,s;           \n\t" // ~(a ^ b)
         "and.b32  t,t,u;         \n\t" // msb: ovfl = (a ^ carry_in) & ~(a^b)
         "xor.b32  r,r,u;         \n\t" // msb: ~result = (~(a ^ b) ^ carry_in)
         "xor.b32  t,t,r;         \n\t" // msb: gt = ovfl != sign(~res)
         "and.b32  t,t,0x80808080;\n\t" // isolate msbs
         "shr.u32  r,t,7;         \n\t" // convert to bool
         "mov.b32  %0,r;          \n\t"
         "}"
         : "=r"(r) : "r"(a), "r"(b));
#endif /* __CUDA_ARCH__ >= 300 */
    return r;           // byte-wise signed gt comparison with mask result
}

static __device__ __forceinline__ unsigned int vsetgtu4(unsigned int a, unsigned int b)
{
    unsigned int r, c;
#if __CUDA_ARCH__ >= 300
    c = 0;
    asm ("vset4.u32.u32.gt %0,%1,%2,%3;" : "=r"(r) : "r"(a), "r"(b), "r"(c));
#else  /* __CUDA_ARCH__ >= 300 */
    asm ("not.b32 %0,%0;" : "+r"(b));
    c = vhaddu4 (a, b); // (a + ~b) / 2 = (a - b) / 2 [rounded down]
    c = c & 0x80808080; // msb = carry-outs
    r = c >> 7;         // convert to bool
#endif /* __CUDA_ARCH__ >= 300 */
    return r;           // byte-wise unsigned gt comparison with bool result
}

static __device__ __forceinline__ unsigned int vsetne4(unsigned int a, unsigned int b)
{
    unsigned int r, c;
#if __CUDA_ARCH__ >= 300
    c = 0;
    asm ("vset4.u32.u32.ne %0,%1,%2,%3;" : "=r"(r) : "r"(a), "r"(b), "r"(c));
#else  /* __CUDA_ARCH__ >= 300 */
    // inspired by Alan Mycroft's null-byte detection algorithm:
    // null_byte(x) = ((x - 0x01010101) & (~x & 0x80808080))
    r = a ^ b;          // 0x00 if a == b
    c = r | 0x80808080; // set msbs, to catch carry out
    c = c - 0x01010101; // msb = 0, if r was 0x00 or 0x80
    c = r | c;          // msb = 1, if r was not 0x00
    c = c & 0x80808080; // extract msbs
    r = c >> 7;         // convert to bool
#endif /* __CUDA_ARCH__ >= 300 */
    return r;           // byte-wise (un)signed ne comparison with bool result
}

static __device__ __forceinline__ unsigned int vsadu4(unsigned int a, unsigned int b)
{
    unsigned int r, s;
#if __CUDA_ARCH__ >= 300
    s = 0;
    asm("vabsdiff4.u32.u32.u32.add %0,%1,%2,%3;":"=r"(r):"r"(a),"r"(b),"r"(s));
#else  /* __CUDA_ARCH__ >= 300 */
    r = vabsdiffu4 (a, b);
    s = r >> 8;
    r = (r & 0x00ff00ff) + (s & 0x00ff00ff);
    r = ((r << 16) + r) >> 16;
#endif /*  __CUDA_ARCH__ >= 300 */
    return r;           // byte-wise sum of absol. differences of unsigned ints
}

static __device__ __forceinline__ unsigned int vsub4(unsigned int a, unsigned int b)
{
#if __CUDA_ARCH__ >= 300
    unsigned int r, c = 0;
    asm ("vsub4.u32.u32.u32 %0,%1,%2,%3;" : "=r"(r) : "r"(a), "r"(b), "r"(c));
#else  /* __CUDA_ARCH__ >= 300 */
    unsigned int r, s, t;
    s = a ^ ~b;         // inverted sum bits
    r = a | 0x80808080; // set msbs
    t = b & 0x7f7f7f7f; // clear msbs
    s = s & 0x80808080; // inverted msb sum bits
    r = r - t;          // subtract w/o msbs, record inverted borrows in msb
    r = r ^ s;          // combine inverted msb sum bits and borrows
#endif /* __CUDA_ARCH__ >= 300 */
    return r;           // byte-wise difference
}

static __device__ __forceinline__ unsigned int vsubss4(unsigned int a, unsigned int b)
{
    unsigned int r;
#if __CUDA_ARCH__ >= 300
    unsigned int c = 0;
    asm ("vsub4.s32.s32.s32.sat %0,%1,%2,%3;" : "=r"(r) :"r"(a),"r"(b),"r"(c));
#else  /* __CUDA_ARCH__ >= 300 */
    /*
      For signed saturation, saturation is controlled by the overflow signal: 
      ovfl = (borrow-in to msb) XOR (borrow-out from msb). Overflow can only 
      occur when the msbs of both inputs are differemt. The defined response to
      overflow is to deliver 0x7f when the addends are positive (bit 7 clear),
      and 0x80 when the addends are negative (bit 7 set). The truth table for
      the msb is

      a   b  bw_in  res  bw_out  ovfl  a^~bw_in  ~(a^~b) (a^~bw_in)&~(a^~b)
      ---------------------------------------------------------------------
      0   0      0    0       0     0         1        0                  0
      0   0      1    1       1     0         0        0                  0
      0   1      0    1       1     1         1        1                  1
      0   1      1    0       1     0         0        1                  0
      1   0      0    1       0     0         0        1                  0
      1   0      1    0       0     1         1        1                  1
      1   1      0    0       0     0         0        0                  0
      1   1      1    1       1     0         1        0                  0

      The seven low-order bits can be handled by wrapping subtraction with the
      borrow-out from bit 6 recorded in the msb (thus corresponding to the 
      bw_in in the truth table for the msb above). ovfl can be computed in many
      equivalent ways, here we use ovfl = (a ^ ~borrow_in) & ~(a ^~b) since we 
      already need to compute (a ^~b) and ~borrow-in for the msb result bit 
      computation. First we compute the normal, wrapped subtraction result. 
      When overflow is detected, we mask off the result's msb, then compute a
      mask covering the seven low order bits, which are all set to 1. This sets
      the byte to 0x7f as we previously cleared the msb. In the overflow case, 
      the sign of the result matches the sign of input a, so we extract the 
      sign of a and add it to the low order bits, which turns 0x7f into 0x80, 
      the correct result for an overflowed negative result.
    */
    asm ("{                          \n\t"
         ".reg .u32 a,b,r,s,t,u,v,w; \n\t"
         "mov.b32     a,%1;          \n\t" 
         "mov.b32     b,%2;          \n\t"
         "not.b32     u,b;           \n\t" // ~b
         "xor.b32     s,u,a;         \n\t" // a ^ ~b
         "or.b32      r,a,0x80808080;\n\t" // set msbs
         "and.b32     t,b,0x7f7f7f7f;\n\t" // clear msbs
         "sub.u32     r,r,t;         \n\t" // subtract lsbs, msb: ~borrow-in
         "xor.b32     t,r,a;         \n\t" // msb: ~borrow-in ^ a
         "not.b32     u,s;           \n\t" // msb: ~(a^~b)
         "and.b32     s,s,0x80808080;\n\t" // msb: a ^ ~b
         "xor.b32     r,r,s;         \n\t" // msb: res = a ^ ~b ^ ~borrow-in
         "and.b32     t,t,u;         \n\t" // msb: ovfl= (~bw-in ^ a) & ~(a^~b)
#if __CUDA_ARCH__ >= 200
         "prmt.b32    s,a,0,0xba98;  \n\t" // sign(a) ? 0xff : 0
         "xor.b32     s,s,0x7f7f7f7f;\n\t" // sign(a) ? 0x80 : 0x7f
         "prmt.b32    t,t,0,0xba98;  \n\t" // ovfl ? 0xff : 0
         "and.b32     s,s,t;         \n\t" // ovfl ? (sign(a) ? 0x80:0x7f) : 0
         "not.b32     t,t;           \n\t" // ~ovfl
         "and.b32     r,r,t;         \n\t" // ovfl ? 0 : a + b
         "or.b32      r,r,s;         \n\t" // ovfl ? (sign(a) ? 0x80:0x7f) :a+b
#else  /* __CUDA_ARCH__ >= 200 */
         "and.b32     t,t,0x80808080;\n\t" // ovfl ? 0x80 : 0
         "shr.u32     s,t,7;         \n\t" // ovfl ? 1 : 0
         "not.b32     u,t;           \n\t" // ovfl ? 0x7f : 0xff
         "and.b32     r,r,u;         \n\t" // ovfl ? (a - b) & 0x7f : a - b
         "and.b32     u,a,t;         \n\t" // ovfl ? a & 0x80 : 0
         "sub.u32     t,t,s;         \n\t" // ovfl ? 0x7f : 0
         "shr.u32     u,u,7;         \n\t" // ovfl ? sign(a) : 0
         "or.b32      r,r,t;         \n\t" // ovfl ? 0x7f : a - b
         "add.u32     r,r,u;         \n\t" // ovfl ? 0x7f+sign(a) : a - b
#endif /* __CUDA_ARCH__ >= 200 */
         "mov.b32     %0,r;          \n\t"
         "}"
         : "=r"(r) : "r"(a), "r"(b));
#endif /* __CUDA_ARCH__ >= 300 */
    return r;           // byte-wise difference with signed saturation
}

static __device__ __forceinline__ unsigned int vsubus4(unsigned int a, unsigned int b)
{
    unsigned int r;
#if __CUDA_ARCH__ >= 300
    unsigned int c = 0;
    asm ("vsub4.u32.u32.u32.sat %0,%1,%2,%3;" : "=r"(r) :"r"(a),"r"(b),"r"(c));
#else  /* __CUDA_ARCH__ >= 300 */
    // This code uses the same basic approach used for the non-saturating 
    // subtraction. The seven low-order bits in each byte are subtracted by 
    // regular subtraction with the inverse of the borrow-out from bit 6 (= 
    // inverse of borrow-in for the msb) being recorded in bit 7, while the 
    // msb is handled separately.
    //
    // Clamping to 0 needs happens when there is a borrow-out from the msb.
    // This is simply accomplished by ANDing the normal addition result with
    // a mask based on the inverted msb borrow-out: ~borrow-out ? 0xff : 0x00.
    // The borrow-out information is generated from the msb. Since we already 
    // have the msb's ~borrow-in and (a^~b) available from the computation of
    // the msb result bit, the most efficient way to compute msb ~borrow-out 
    // is: ((a ^ ~b) & ~borrow-in) | (~b & a). The truth table for the msb is
    //
    // a b bw-in res ~bw-out a^~b (a^~b)&~bw-in (a&~b) ((a^~b)&~bw-in)|(a&~b)
    //                                                        
    // 0 0  0     0     1      1        1          0          1
    // 0 0  1     1     0      1        0          0          0
    // 0 1  0     1     0      0        0          0          0
    // 0 1  1     0     0      0        0          0          0
    // 1 0  0     1     1      0        0          1          1
    // 1 0  1     0     1      0        0          1          1
    // 1 1  0     0     1      1        1          0          1
    // 1 1  1     1     0      1        0          0          0
    //
    asm ("{                       \n\t"
         ".reg .u32 a,b,r,s,t,u;  \n\t"
         "mov.b32  a,%1;          \n\t"
         "mov.b32  b,%2;          \n\t"
         "not.b32  u,b;           \n\t" // ~b
         "xor.b32  s,u,a;         \n\t" // a ^ ~b
         "and.b32  u,u,a;         \n\t" // a & ~b
         "or.b32   r,a,0x80808080;\n\t" // set msbs
         "and.b32  t,b,0x7f7f7f7f;\n\t" // clear msbs
         "sub.u32  r,r,t;         \n\t" // subtract lsbs, msb: ~borrow-in
         "and.b32  t,r,s;         \n\t" // msb: (a ^ ~b) & ~borrow-in
         "and.b32  s,s,0x80808080;\n\t" // msb: a ^ ~b
         "xor.b32  r,r,s;         \n\t" // msb: res = a ^ ~b ^ ~borrow-in
         "or.b32   t,t,u;         \n\t" // msb: bw-out = ((a^~b)&~bw-in)|(a&~b)
#if __CUDA_ARCH__ >= 200
         "prmt.b32 t,t,0,0xba98;  \n\t" // ~borrow-out ? 0xff : 0
#else  /* __CUDA_ARCH__ >= 200 */
         "and.b32  t,t,0x80808080;\n\t" // isolate msb: ~borrow-out
         "shr.u32  s,t,7;         \n\t" // build mask
         "sub.u32  s,t,s;         \n\t" //  from
         "or.b32   t,t,s;         \n\t" //   msb
#endif /* __CUDA_ARCH__ >= 200 */
         "and.b32  r,r,t;         \n\t" // cond. clear result if msb borrow-out
         "mov.b32  %0,r;          \n\t"
         "}"
         : "=r"(r) : "r"(a) , "r"(b));
#endif /* __CUDA_ARCH__ >= 300 */
    return r;           // byte-wise difference with unsigned saturation
}

static __device__ __forceinline__ unsigned int vneg4(unsigned int a)
{
    return vsub4 (0, a);// byte-wise negation with wrap-around
}

static __device__ __forceinline__ unsigned int vnegss4(unsigned int a)
{
    unsigned int r;
#if __CUDA_ARCH__ >= 300
    unsigned int s = 0;
    asm ("vsub4.s32.s32.s32.sat %0,%1,%2,%3;" : "=r"(r) :"r"(s),"r"(a),"r"(s));
#else  /* __CUDA_ARCH__ >= 300 */
    r = vsub4 (0, a);   //
    asm ("{                       \n\t"
         ".reg .u32 a, r, s;      \n\t"
         "mov.b32  r,%0;          \n\t"
         "mov.b32  a,%1;          \n\t"
         "and.b32  a,a,0x80808080;\n\t" // extract msb
         "and.b32  s,a,r;         \n\t" // wrap-around if msb set in a and -a
         "shr.u32  s,s,7;         \n\t" // msb ? 1 : 0
         "sub.u32  r,r,s;         \n\t" // subtract 1 if result wrapped around
         "mov.b32  %0,r;          \n\t"
         "}"
         : "+r"(r) : "r"(a));
#endif /* __CUDA_ARCH__ >= 300 */
    return r;           // byte-wise negation with signed saturation
}

static __device__ __forceinline__ unsigned int vabsdiffs4(unsigned int a, unsigned int b)
{
    unsigned int r, s;
#if __CUDA_ARCH__ >= 300
    s = 0;
    asm ("vabsdiff4.s32.s32.s32 %0,%1,%2,%3;" : "=r"(r) :"r"(a),"r"(b),"r"(s));
#else  /* __CUDA_ARCH__ >= 300 */
    s = vcmpges4 (a, b);// mask = 0xff if a >= b
    r = a ^ b;          //
    s = (r & s) ^ b;    // select a when a >= b, else select b => max(a,b)
    r = s ^ r;          // select a when b >= a, else select b => min(a,b)
    r = vsub4 (s, r);   // |a - b| = max(a,b) - min(a,b);
#endif /* __CUDA_ARCH__ >= 300 */
    return r;           // byte-wise absolute difference of signed integers
}

static __device__ __forceinline__ unsigned int vsads4(unsigned int a, unsigned int b)
{
    unsigned int r, s;
#if __CUDA_ARCH__ >= 300
    s = 0;
    asm("vabsdiff4.s32.s32.s32.add %0,%1,%2,%3;":"=r"(r):"r"(a),"r"(b),"r"(s));
#else  /* __CUDA_ARCH__ >= 300 */
    r = vabsdiffs4 (a, b);
    s = r >> 8;
    r = (r & 0x00ff00ff) + (s & 0x00ff00ff);
    r = ((r << 16) + r) >> 16;
#endif /*  __CUDA_ARCH__ >= 300 */
    return r;           // byte-wise sum of absolute differences of signed ints
}

#endif /* SIMD_FUNCTIONS_H__ */
