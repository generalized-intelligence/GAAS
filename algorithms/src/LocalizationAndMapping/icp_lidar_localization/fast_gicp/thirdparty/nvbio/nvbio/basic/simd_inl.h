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

namespace nvbio {

NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
simd4u8::simd4u8(const uint4 v)
{
    m  =  v.x;
    m |= (v.y << 8);
    m |= (v.z << 16);
    m |= (v.w << 24);
}

NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
simd4u8::simd4u8(const uint8 v)
{
    m  =  v;
    m |= (v << 8);
    m |= (v << 16);
    m |= (v << 24);
}

NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
simd4u8::simd4u8(const uint8 v1, const uint8 v2, const uint8 v3, const uint8 v4)
{
    m  =  v1;
    m |= (v2 << 8);
    m |= (v3 << 16);
    m |= (v4 << 24);
}

NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
simd4u8& simd4u8::operator= (const uint4 v)
{
    m  = v.x;
    m |= (v.y << 8);
    m |= (v.z << 16);
    m |= (v.w << 24);
    return *this;
}
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
simd4u8& simd4u8::operator= (const uchar4 v)
{
    m  = v.x;
    m |= (v.y << 8);
    m |= (v.z << 16);
    m |= (v.w << 24);
    return *this;
}

NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
simd4u8 operator== (const simd4u8 op1, const simd4u8 op2)
{
#if defined(NVBIO_DEVICE_COMPILATION)
    return simd4u8( vcmpeq4( op1.m ,op2.m ), simd4u8::base_rep_tag() );
#else
    return simd4u8(
        get<0>(op1) == get<0>(op2) ? 0xFFu : 0u,
        get<1>(op1) == get<1>(op2) ? 0xFFu : 0u,
        get<2>(op1) == get<2>(op2) ? 0xFFu : 0u,
        get<3>(op1) == get<3>(op2) ? 0xFFu : 0u );
#endif
}
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
simd4u8 operator!= (const simd4u8 op1, const simd4u8 op2)
{
#if defined(NVBIO_DEVICE_COMPILATION)
    return simd4u8( vcmpne4( op1.m ,op2.m ), simd4u8::base_rep_tag() );
#else
    return simd4u8(
        get<0>(op1) != get<0>(op2) ? 0xFFu : 0u,
        get<1>(op1) != get<1>(op2) ? 0xFFu : 0u,
        get<2>(op1) != get<2>(op2) ? 0xFFu : 0u,
        get<3>(op1) != get<3>(op2) ? 0xFFu : 0u );
#endif
}
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
simd4u8 operator>= (const simd4u8 op1, const simd4u8 op2)
{
#if defined(NVBIO_DEVICE_COMPILATION)
    return simd4u8( vcmpgeu4( op1.m ,op2.m ), simd4u8::base_rep_tag() );
#else
    return simd4u8(
        get<0>(op1) <= get<0>(op2) ? 0xFFu : 0u,
        get<1>(op1) <= get<1>(op2) ? 0xFFu : 0u,
        get<2>(op1) <= get<2>(op2) ? 0xFFu : 0u,
        get<3>(op1) <= get<3>(op2) ? 0xFFu : 0u );
#endif
}
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
simd4u8 operator> (const simd4u8 op1, const simd4u8 op2)
{
#if defined(NVBIO_DEVICE_COMPILATION)
    return simd4u8( vcmpgtu4( op1.m ,op2.m ), simd4u8::base_rep_tag() );
#else
    return simd4u8(
        get<0>(op1) > get<0>(op2) ? 0xFFu : 0u,
        get<1>(op1) > get<1>(op2) ? 0xFFu : 0u,
        get<2>(op1) > get<2>(op2) ? 0xFFu : 0u,
        get<3>(op1) > get<3>(op2) ? 0xFFu : 0u );
#endif
}
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
simd4u8 operator<= (const simd4u8 op1, const simd4u8 op2)
{
#if defined(NVBIO_DEVICE_COMPILATION)
    return simd4u8( vcmpleu4( op1.m ,op2.m ), simd4u8::base_rep_tag() );
#else
    return simd4u8(
        get<0>(op1) <= get<0>(op2) ? 0xFFu : 0u,
        get<1>(op1) <= get<1>(op2) ? 0xFFu : 0u,
        get<2>(op1) <= get<2>(op2) ? 0xFFu : 0u,
        get<3>(op1) <= get<3>(op2) ? 0xFFu : 0u );
#endif
}
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
simd4u8 operator< (const simd4u8 op1, const simd4u8 op2)
{
#if defined(NVBIO_DEVICE_COMPILATION)
    return simd4u8( vcmpltu4( op1.m ,op2.m ), simd4u8::base_rep_tag() );
#else
    return simd4u8(
        get<0>(op1) < get<0>(op2) ? 0xFFu : 0u,
        get<1>(op1) < get<1>(op2) ? 0xFFu : 0u,
        get<2>(op1) < get<2>(op2) ? 0xFFu : 0u,
        get<3>(op1) < get<3>(op2) ? 0xFFu : 0u );
#endif
}
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
simd4u8 operator+ (const simd4u8 op1, const simd4u8 op2)
{
#if defined(NVBIO_DEVICE_COMPILATION)
    return simd4u8( vadd4( op1.m, op2.m ), simd4u8::base_rep_tag() ); // per-byte (un)signed addition, with wrap-around: a + b
#else
    return simd4u8(
        get<0>(op1) + get<0>(op2),
        get<1>(op1) + get<1>(op2),
        get<2>(op1) + get<2>(op2),
        get<3>(op1) + get<3>(op2) );
#endif
}
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
simd4u8& operator+= (simd4u8& op1, const simd4u8 op2)
{
#if defined(__CUDA_ARCH__) && __CUDA_ARCH__ > 0
    op1.m = vadd4( op1.m, op2.m ); // per-byte (un)signed addition, with wrap-around: a + b
#else
    op1 = simd4u8(
        get<0>(op1) + get<0>(op2),
        get<1>(op1) + get<1>(op2),
        get<2>(op1) + get<2>(op2),
        get<3>(op1) + get<3>(op2) );
#endif
    return op1;
}
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
simd4u8 operator- (const simd4u8 op1, const simd4u8 op2)
{
#if defined(NVBIO_DEVICE_COMPILATION)
    return simd4u8( vsub4( op1.m, op2.m ), simd4u8::base_rep_tag() ); // per-byte (un)signed subtraction, with wrap-around: a - b
#else
    return simd4u8(
        get<0>(op1) - get<0>(op2),
        get<1>(op1) - get<1>(op2),
        get<2>(op1) - get<2>(op2),
        get<3>(op1) - get<3>(op2) );
#endif
}
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
simd4u8& operator-= (simd4u8& op1, const simd4u8 op2)
{
#if defined(__CUDA_ARCH__) && __CUDA_ARCH__ > 0
    op1.m = vsub4( op1.m, op2.m ); // per-byte (un)signed subtraction, with wrap-around: a - b
#else
    op1 = simd4u8(
        get<0>(op1) - get<0>(op2),
        get<1>(op1) - get<1>(op2),
        get<2>(op1) - get<2>(op2),
        get<3>(op1) - get<3>(op2) );
#endif
    return op1;
}

NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
simd4u8 max(const simd4u8 op1, const simd4u8 op2)
{
#if defined(NVBIO_DEVICE_COMPILATION)
    return simd4u8( vmaxu4( op1.m, op2.m ), simd4u8::base_rep_tag() );
#else
    return simd4u8(
        nvbio::max( get<0>(op1), get<0>(op2) ),
        nvbio::max( get<1>(op1), get<1>(op2) ),
        nvbio::max( get<2>(op1), get<2>(op2) ),
        nvbio::max( get<3>(op1), get<3>(op2) ) );
#endif
}
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
simd4u8 min(const simd4u8 op1, const simd4u8 op2)
{
#if defined(NVBIO_DEVICE_COMPILATION)
    return simd4u8( vminu4( op1.m, op2.m ), simd4u8::base_rep_tag() );
#else
    return simd4u8(
        nvbio::min( get<0>(op1), get<0>(op2) ),
        nvbio::min( get<1>(op1), get<1>(op2) ),
        nvbio::min( get<2>(op1), get<2>(op2) ),
        nvbio::min( get<3>(op1), get<3>(op2) ) );
#endif
}

NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
simd4u8 and_op(const simd4u8 op1, const simd4u8 op2)
{
    return simd4u8( op1.m & op2.m, simd4u8::base_rep_tag() );
}
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
simd4u8 or_op(const simd4u8 op1, const simd4u8 op2)
{
    return simd4u8( op1.m | op2.m, simd4u8::base_rep_tag() );
}

template <uint32 I>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
uint8 get(const simd4u8 op)
{
    return (op.m >> (I*8)) & 255u;
}
template <uint32 I>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
void set(simd4u8 op, const uint8 v)
{
    op.m &= ~(255u << (I*8));
    op.m |= v << (I*8);
}

NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
simd4u8 ternary_op(const simd4u8 mask, const simd4u8 op1, const simd4u8 op2)
{
    return or_op( and_op( mask, op1 ), and_op( ~mask, op2 ) );
}

} // namespace nvbio
