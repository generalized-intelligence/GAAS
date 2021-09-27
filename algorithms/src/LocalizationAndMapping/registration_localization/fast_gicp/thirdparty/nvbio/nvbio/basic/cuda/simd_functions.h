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

#if !defined (SIMD_FUNCTIONS_H__)
#define SIMD_FUNCTIONS_H__

/*
  This header file contains inline functions that implement intra-word SIMD
  operations, that are hardware accelerated on sm_3x (Kepler) GPUs. Efficient
  emulation code paths are provided for earlier architectures (sm_1x, sm_2x)
  to make the code portable across all GPUs supported by CUDA. The following 
  functions are currently implemented:

  vabsdiffu2(a,b) per-halfword unsigned absolute difference: |a - b|
  vadd2(a,b)      per-halfword (un)signed addition, with wrap-around: a + b
  vaddss2(a,b)    per-halfword addition with signed saturation: sat.s16 (a + b)
  vaddus2(a,b)    per-halfword addition with unsigned saturation: sat.u16 (a+b)
  vavgs2(a,b)     per-halfword signed rounded average: (a+b+((a+b)>=0)) >> 1
  vavgu2(a,b)     per-halfword unsigned rounded average: (a + b + 1) / 2
  vcmpeq2(a,b)    per-halfword (un)signed comparison: a == b ? 0xffff : 0
  vcmpgeu2(a,b)   per-halfword unsigned comparison: a >= b ? 0xffff : 0
  vcmpgtu2(a,b)   per-halfword unsigned comparison: a > b ? 0xffff : 0
  vcmpleu2(a,b)   per-halfword unsigned comparison: a <= b ? 0xffff : 0
  vcmpltu2(a,b)   per-halfword unsigned comparison: a < b ? 0xffff : 0
  vcmpne2(a,b)    per-halfword (un)signed comparison: a != b ? 0xffff : 0
  vhaddu2(a,b)    per-halfword unsigned average: (a + b) / 2
  vmaxu2(a,b)     per-halfword unsigned maximum: max(a, b)
  vminu2(a,b)     per-halfword unsigned minimum: min(a, b)
  vseteq2(a,b)    per-halfword (un)signed comparison: a == b ? 1 : 0
  vsetgeu2(a,b)   per-halfword unsigned comparison: a >= b ? 1 : 0
  vsetgtu2(a,b)   per-halfword unsigned comparison: a > b ? 1 : 0
  vsetleu2(a,b)   per-halfword unsigned comparison: a <= b ? 1 : 0 
  vsetltu2(a,b)   per-halfword unsigned comparison: a < b ? 1 : 0
  vsetne2(a,b)    per-halfword (un)signed comparison: a != b ? 1 : 0
  vsub2(a,b)      per-halfword (un)signed subtraction, with wrap-around: a - b
  
  vabsdiffu4(a,b) per-byte unsigned absolute difference: |a - b|
  vadd4(a,b)      per-byte (un)signed addition, with wrap-around: a + b
  vaddss4(a,b)    per-byte addition with signed saturation: sat.s8 (a + b)
  vaddus4(a,b)    per-byte addition with unsigned saturation: sat.u8 (a + b)
  vavgs4(a,b)     per-byte signed rounded average: (a + b + ((a+b) >= 0)) >> 1
  vavgu4(a,b)     per-byte unsigned rounded average: (a + b + 1) / 2
  vcmpeq4(a,b)    per-byte (un)signed comparison: a == b ? 0xff : 0
  vcmpgeu4(a,b)   per-byte unsigned comparison: a >= b ? 0xff : 0
  vcmpgtu4(a,b)   per-byte unsigned comparison: a > b ? 0xff : 0
  vcmpleu4(a,b)   per-byte unsigned comparison: a <= b ? 0xff : 0
  vcmpltu4(a,b)   per-byte unsigned comparison: a < b ? 0xff : 0
  vcmpne4(a,b)    per-byte (un)signed comparison: a != b ? 0xff: 0
  vhaddu4(a,b)    per-byte unsigned average: (a + b) / 2
  vmaxu4(a,b)     per-byte unsigned maximum: max(a, b)
  vminu4(a,b)     per-byte unsigned minimum: min(a, b)
  vseteq4(a,b)    per-byte (un)signed comparison: a == b ? 1 : 0
  vsetgeu4(a,b)   per-byte unsigned comparison: a >= b ? 1 : 0
  vsetgtu4(a,b)   per-byte unsigned comparison: a > b ? 1 : 0
  vsetleu4(a,b)   per-byte unsigned comparison: a <= b ? 1 : 0
  vsetltu4(a,b)   per-byte unsigned comparison: a < b ? 1 : 0
  vsetne4(a,b)    per-byte (un)signed comparison: a != b ? 1: 0
  vsub4(a,b)      per-byte (un)signed subtraction, with wrap-around: a - b
*/

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
    return t;           // halfword-wise sum
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
    return r;
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
    rlo = ::min (alo + blo, 65535);
    rhi = ::min (ahi + bhi, 65535);
    r = (rhi << 16) + rlo;
#endif /* __CUDA_ARCH__ >= 300 */
    return r;
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
         "xor.b32 r,r,s;         \n\t" // compute masb sum bit: a ^ b ^ cy-in
         "shr.u32 t,r,15;        \n\t" // sign ((a + b) >> 1)
         "not.b32 t,t;           \n\t" // ~sign ((a + b) >> 1)
         "and.b32 t,t,c;         \n\t" // ((a ^ b) & 1) & ~sign ((a + b) >> 1)
         "add.u32 r,r,t;         \n\t" // conditionally add ((a ^ b) & 1)
         "mov.b32 %0,r;          \n\t"
         "}"
         : "=r"(r) : "r"(a), "r"(b));
#endif /* __CUDA_ARCH__ >= 300 */
    return r;           // halfword-wise signed average
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
    return r;           // halfword-wise unsigned average
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
    return r;           // halfword-wise unsigned average [rounded down]
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
    r = c >> 15;        // convert
    r = c - r;          //  msbs to
    r = c | r;          //   mask
#endif /* __CUDA_ARCH__ >= 300 */
    return r;           // halfword-wise unsigned eq comparison, mask result
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
    asm ("and.b32 %0,%0,0x80008000;" : "+r"(c));  // msb = carry-outs
    asm ("shr.u32 %0,%1,15;" : "=r"(r) : "r"(c)); // build mask
    asm ("sub.u32 %0,%1,%0;" : "+r"(r) : "r"(c)); //  from
    asm ("or.b32  %0,%1,%0;" : "+r"(r) : "r"(c)); //   msbs
#endif /* __CUDA_ARCH__ >= 300 */
    return r;           // halfword-wise unsigned gt-eq comparison, mask result
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
    asm ("and.b32 %0,%0,0x80008000;" : "+r"(c));  // msb = carry-outs
    asm ("shr.u32 %0,%1,15;" : "=r"(r) : "r"(c)); // build mask
    asm ("sub.u32 %0,%1,%0;" : "+r"(r) : "r"(c)); //  from
    asm ("or.b32  %0,%1,%0;" : "+r"(r) : "r"(c)); //   msbs
#endif /* __CUDA_ARCH__ >= 300 */
    return r;           // halfword-wise unsigned gt comparison, mask result
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
    asm ("and.b32 %0,%0,0x80008000;" : "+r"(c));  // msb = carry-outs
    asm ("shr.u32 %0,%1,15;" : "=r"(r) : "r"(c)); // build mask
    asm ("sub.u32 %0,%1,%0;" : "+r"(r) : "r"(c)); //  from
    asm ("or.b32  %0,%1,%0;" : "+r"(r) : "r"(c)); //   msbs
#endif /* __CUDA_ARCH__ >= 300 */
    return r;           // halfword-wise unsigned lt-eq comparison, mask result
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
    asm ("and.b32 %0,%0,0x80008000;" : "+r"(c));  // msb = carry-outs
    asm ("shr.u32 %0,%1,15;" : "=r"(r) : "r"(c)); // build mask
    asm ("sub.u32 %0,%1,%0;" : "+r"(r) : "r"(c)); //  from
    asm ("or.b32  %0,%1,%0;" : "+r"(r) : "r"(c)); //   msbs
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
    asm ("and.b32 %0,%0,0x80008000;" : "+r"(c));  // extract msbs
    asm ("shr.u32 %0,%1,15;" : "=r"(r) : "r"(c)); // build mask
    asm ("sub.u32 %0,%1,%0;" : "+r"(r) : "r"(c)); //  from
    asm ("or.b32  %0,%1,%0;" : "+r"(r) : "r"(c)); //   msbs
#endif /* __CUDA_ARCH__ >= 300 */
    return r;           // halfword-wise unsigned ne comparison, mask result
}

static __device__ __forceinline__ unsigned int vabsdiffu2(unsigned int a, unsigned int b)
{
    unsigned int r, s;
#if __CUDA_ARCH__ >= 300
    s = 0;
    asm ("vabsdiff2.u32.u32.u32 %0,%1,%2,%3;" : "=r"(r) : "r"(a),"r"(b),"r"(s));
#else  /* __CUDA_ARCH__ >= 300 */
    unsigned int t, u, v;
    s = a & 0x0000ffff; // extract low halfword
    r = b & 0x0000ffff; // extract low halfword
    u = ::max (r, s);     // maximum of low halfwords
    v = ::min (r, s);     // minimum of low halfwords
    s = a & 0xffff0000; // extract high halfword
    r = b & 0xffff0000; // extract high halfword
    t = ::max (r, s);     // maximum of high halfwords
    s = ::min (r, s);     // minimum of high halfwords
    r = u | t;          // maximum of both halfwords
    s = v | s;          // minimum of both halfwords
    r = r - s;          // |a - b| = max(a,b) - min(a,b);
#endif /* __CUDA_ARCH__ >= 300 */
    return r;           // halfword-wide unsigned absolute difference
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
    t = ::max (r, s);     // maximum of low halfwords
    r = a & 0xffff0000; // extract high halfword
    s = b & 0xffff0000; // extract high halfword
    u = ::max (r, s);     // maximum of high halfwords
    r = t | u;          // combine halfword maximums
#endif /* __CUDA_ARCH__ >= 300 */
    return r;           // halfword-wise unsigned maximum
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
    t = ::min (r, s);     // minimum of low halfwords
    r = a & 0xffff0000; // extract high halfword
    s = b & 0xffff0000; // extract high halfword
    u = ::min (r, s);     // minimum of high halfwords
    r = t | u;          // combine halfword minimums
#endif /* __CUDA_ARCH__ >= 300 */
    return r;           // halfword-wise unsigned minimum
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
    return r;           // halfword-wise unsigned eq comparison, bool result
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
    return r;           // halfword-wise unsigned ne comparison, bool result
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
    return r;           // byte-wise sum
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
    // The fact that this is a saturating addition simplfies the handling of
    // the msb. When carry-out from the msb occurs, the entire byte must be
    // written as 0xff, and the computed msb is overwritten in the process. 
    // The corresponding entries in the truth table for the msb sum bit thus
    // become "don't cares":
    //
    // a  b  cy-in  sum  cy-out
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
    // As is easily seen, the simplest implementation of the sum bit is simply
    // (a | b) & 0x80808080, with masking needed to isolate the msb. Note that
    // this computation also makes the msb handling redundant with the clamping
    // to 0xFF, because the msb is already set to 1 whenever saturation kicks
    // in. This means we only need to apply saturation to the seven low-order
    // bits in each byte, by overwriting with 0x7F. Saturation is controlled
    // by carry-out from the msb, which can be represented by various Boolean
    // expressions. As we need to compute (a | b) & 0x80808080 anyhow, the most
    // efficient of these is cy-out = ((a & b) | cy-in) & (a | b) & 0x80808080.
    unsigned int r;
    asm ("{                         \n\t" 
         ".reg .u32 a,b,r,s,t,m;    \n\t"
         "mov.b32  a, %1;           \n\t" 
         "mov.b32  b, %2;           \n\t"
         "or.b32   m, a, b;         \n\t"
         "and.b32  r, a, 0x7f7f7f7f;\n\t" // clear msbs
         "and.b32  t, b, 0x7f7f7f7f;\n\t" // clear msbs
         "and.b32  m, m, 0x80808080;\n\t" // result msbs
         "add.u32  r, r, t;         \n\t" // add w/o msbs, record msb-carry-ins
         "and.b32  t, a, b;         \n\t" // (a & b)
         "or.b32   t, t, r;         \n\t" // (a & b) | cy-in)
         "and.b32  t, t, m;         \n\t" // ((a&b)|cy-in) & ((a|b)&0x80808080)
         "shr.u32  s, t, 7;         \n\t" //
         "sub.u32  t, t, s;         \n\t" // lsb-overwrt: msb cy-out ? 0 : 0x7F
         "or.b32   t, t, m;         \n\t" // merge msb and lsb overwrite
         "or.b32   r, r, t;         \n\t" // overwrite bits with 1 as needed
         "mov.b32  %0, r;           \n\t"
         "}"
         : "=r"(r) : "r"(a), "r"(b));
#endif /* __CUDA_ARCH__ >= 300 */
    return r;
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

      a   b   cy_in  sum  cy_out  ovfl
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
         "xor.b32  s, a, b;         \n\t" // sum bits
         "and.b32  r, a, 0x7f7f7f7f;\n\t" // clear msbs
         "and.b32  t, b, 0x7f7f7f7f;\n\t" // clear msbs
         "and.b32  s, s, 0x80808080;\n\t" // msb sum bits
         "add.u32  r, r, t;         \n\t" // capture msb carry-in in bit 7
         "xor.b32  t, a, r;         \n\t" // a ^ carry_in
         "xor.b32  r, r, s;         \n\t" // msb sum bit = (a ^ b ^ carry_in)
         "not.b32  s, s;            \n\t" // ~(a ^ b)
         "and.b32  t, t, s;         \n\t" // ovfl = (a ^ carry_in) & ~(a ^ b)
         "and.b32  t, t, 0x80808080;\n\t" // ovfl ? 0x80 : 0
         "shr.u32  s, t, 7;         \n\t" // ovfl ? 1 : 0
         "not.b32  u, t;            \n\t" // ovfl ? 0x7f : 0xff
         "and.b32  r, r, u;         \n\t" // ovfl ? (a + b) & 0x7f : a + b
         "and.b32  u, a, t;         \n\t" // ovfl ? a & 0x80 : 0
         "sub.u32  t, t, s;         \n\t" // ovfl ? 0x7f : 0
         "shr.u32  u, u, 7;         \n\t" // ovfl ? sign(a) : 0
         "or.b32   r, r, t;         \n\t" // ovfl ? 0x7f : a + b
         "add.u32  r, r, u;         \n\t" // ovfl ? 0x7f+sign(a) : a + b
         "mov.b32  %0, r;           \n\t"
         "}"
         : "=r"(r) : "r"(a), "r"(b));
#endif /* __CUDA_ARCH__ >= 300 */
    return r;
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
         "xor.b32 r,r,s;         \n\t" // compute masb sum bit: a ^ b ^ cy-in
         "shr.u32 t,r,7;         \n\t" // sign ((a + b) >> 1)
         "not.b32 t,t;           \n\t" // ~sign ((a + b) >> 1)
         "and.b32 t,t,c;         \n\t" // ((a ^ b) & 1) & ~sign ((a + b) >> 1)
         "add.u32 r,r,t;         \n\t" // conditionally add ((a ^ b) & 1)
         "mov.b32 %0,r;          \n\t"
         "}"
         : "=r"(r) : "r"(a), "r"(b));
#endif /* __CUDA_ARCH__ >= 300 */
    return r;           // halfword-wise signed average
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
    return r;           // byte-wise unsigned average
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
    return s;           // byte-wise unsigned average [rounded down]
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
    r = c >> 7;         // build mask
    r = c - r;          //  from 
    r = r | c;          //   msbs
#endif /* __CUDA_ARCH__ >= 300 */
    return r;           // byte-wise unsigned eq comparison with mask result
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
    asm ("and.b32 %0,%0,0x80808080;" : "+r"(c));  // msb = carry-outs
    asm ("shr.u32 %0,%1,7;" : "=r"(r) : "r"(c));  // build mask
    asm ("sub.u32 %0,%1,%0;" : "+r"(r) : "r"(c)); //  from
    asm ("or.b32  %0,%1,%0;" : "+r"(r) : "r"(c)); //   msbs
#endif /* __CUDA_ARCH__ >= 300 */
    return r;           // byte-wise unsigned gt-eq comparison with mask result
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
    asm ("and.b32 %0,%0,0x80808080;" : "+r"(c));  // msb = carry-outs
    asm ("shr.u32 %0,%1,7;" : "=r"(r) : "r"(c));  // build mask
    asm ("sub.u32 %0,%1,%0;" : "+r"(r) : "r"(c)); //  from
    asm ("or.b32  %0,%1,%0;" : "+r"(r) : "r"(c)); //   msbs
#endif /* __CUDA_ARCH__ >= 300 */
    return r;           // byte-wise unsigned gt comparison with mask result
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
    asm ("and.b32 %0,%0,0x80808080;" : "+r"(c));  // msb = carry-outs
    asm ("shr.u32 %0,%1,7;" : "=r"(r) : "r"(c));  // build mask
    asm ("sub.u32 %0,%1,%0;" : "+r"(r) : "r"(c)); //  from
    asm ("or.b32  %0,%1,%0;" : "+r"(r) : "r"(c)); //   msbs
#endif /* __CUDA_ARCH__ >= 300 */
    return r;           // byte-wise unsigned lt-eq comparison with bool result
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
    asm ("and.b32 %0,%0,0x80808080;" : "+r"(c));  // msb = carry-outs
    asm ("shr.u32 %0,%1,7;" : "=r"(r) : "r"(c));  // build mask
    asm ("sub.u32 %0,%1,%0;" : "+r"(r) : "r"(c)); //  from
    asm ("or.b32  %0,%1,%0;" : "+r"(r) : "r"(c)); //   msbs
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
    asm ("and.b32 %0,%0,0x80808080;" : "+r"(c));  // extract msbs
    asm ("shr.u32 %0,%1,7;" : "=r"(r) : "r"(c));  // build mask
    asm ("sub.u32 %0,%1,%0;" : "+r"(r) : "r"(c)); //  from
    asm ("or.b32  %0,%1,%0;" : "+r"(r) : "r"(c)); //   msbs
#endif /* __CUDA_ARCH__ >= 300 */
    return r;           // byte-wise unsigned ne comparison with mask result
}

static __device__ __forceinline__ unsigned int vabsdiffu4(unsigned int a, unsigned int b)
{
    unsigned int r, s;
#if __CUDA_ARCH__ >= 300
    s = 0;
    asm ("vabsdiff4.u32.u32.u32 %0,%1,%2,%3;" : "=r"(r) : "r"(a),"r"(b),"r"(s));
#else  /* __CUDA_ARCH__ >= 300 */
    s = vcmpgeu4 (a, b);// mask = 0xff if a >= b
    r = a ^ b;          //
    s = (r & s) ^ b;    // select a when a >= b, else select b => max(a,b)
    r = s ^ r;          // select a when b >= a, else select b => min(a,b)
    r = s - r;          // |a - b| = max(a,b) - min(a,b);
#endif /* __CUDA_ARCH__ >= 300 */
    return r;           // byte-wise unsigned minimum
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
    return r;           // byte-wise unsigned maximum
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
    return r;           // byte-wise unsigned minimum
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
    return r;           // byte-wise unsigned eq comparison with bool result
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
    return r;           // byte-wise unsigned ne comparison with bool result
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

#endif /* SIMD_FUNCTIONS_H__ */
