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

/*! \file vector.h
 *   \brief Define host / device vectors
 */

#pragma once

#include <nvbio/basic/types.h>
#include <nvbio/basic/numbers.h>

namespace nvbio {

template <typename T> struct maximum_vector_width         { static const uint32 VALUE = 1; };
template <>           struct maximum_vector_width<char>   { static const uint32 VALUE = 16; };
template <>           struct maximum_vector_width< int8>  { static const uint32 VALUE = 16; };
template <>           struct maximum_vector_width<uint8>  { static const uint32 VALUE = 16; };
template <>           struct maximum_vector_width< int16> { static const uint32 VALUE = 4; };
template <>           struct maximum_vector_width<uint16> { static const uint32 VALUE = 4; };
template <>           struct maximum_vector_width< int32> { static const uint32 VALUE = 4; };
template <>           struct maximum_vector_width<uint32> { static const uint32 VALUE = 4; };
template <>           struct maximum_vector_width< int64> { static const uint32 VALUE = 2; };
template <>           struct maximum_vector_width<uint64> { static const uint32 VALUE = 2; };

template <uint32 VECTOR_WIDTH, typename T>
struct vector_loader {};

template <typename T>
struct vector_loader<1,T> { NVBIO_HOST_DEVICE static void load(const T* ptr, T* vec) { *vec = *ptr; } };

template <> struct vector_loader< 1,char> { NVBIO_HOST_DEVICE static void load(const char* ptr, char* vec) { *vec = *ptr; } };
template <> struct vector_loader< 2,char> { NVBIO_HOST_DEVICE static void load(const char* ptr, char* vec) { const char2 v = *(const char2*)ptr; vec[0] = v.x; vec[1] = v.y; } };
template <> struct vector_loader< 3,char> { NVBIO_HOST_DEVICE static void load(const char* ptr, char* vec) { const char3 v = *(const char3*)ptr; vec[0] = v.x; vec[1] = v.y; vec[2] = v.z; } };
template <> struct vector_loader< 4,char> { NVBIO_HOST_DEVICE static void load(const char* ptr, char* vec) { const char4 v = *(const char4*)ptr; vec[0] = v.x; vec[1] = v.y; vec[2] = v.z;  vec[3] = v.w; } };
template <> struct vector_loader< 8,char>
{
    NVBIO_HOST_DEVICE static void load(const char* ptr, char* vec)
    {
        const uint2 v = *(const uint2*)ptr;
        vec[0] = (char)((v.x >>  0) & 0xFF); vec[1] = (char)((v.x >> 8)  & 0xFF);
        vec[2] = (char)((v.x >> 16) & 0xFF); vec[3] = (char)((v.x >> 24) & 0xFF);
        vec[4] = (char)((v.y >>  0) & 0xFF); vec[5] = (char)((v.y >> 8)  & 0xFF);
        vec[6] = (char)((v.y >> 16) & 0xFF); vec[7] = (char)((v.y >> 24) & 0xFF);
    }
};
template <> struct vector_loader<16,char>
{
    NVBIO_HOST_DEVICE static void load(const char* ptr, char* vec)
    {
        const uint4 v = *(const uint4*)ptr;

        vec[0] = (char)((v.x >>  0) & 0xFF); vec[1] = (char)((v.x >> 8)  & 0xFF);
        vec[2] = (char)((v.x >> 16) & 0xFF); vec[3] = (char)((v.x >> 24) & 0xFF);
        vec[4] = (char)((v.y >>  0) & 0xFF); vec[5] = (char)((v.y >> 8)  & 0xFF);
        vec[6] = (char)((v.y >> 16) & 0xFF); vec[7] = (char)((v.y >> 24) & 0xFF);

        vec[8]  = (char)((v.z >>  0) & 0xFF); vec[9]  = (char)((v.z >> 8)  & 0xFF);
        vec[10] = (char)((v.z >> 16) & 0xFF); vec[11] = (char)((v.z >> 24) & 0xFF);
        vec[12] = (char)((v.w >>  0) & 0xFF); vec[13] = (char)((v.w >> 8)  & 0xFF);
        vec[14] = (char)((v.w >> 16) & 0xFF); vec[15] = (char)((v.w >> 24) & 0xFF);
    }
};

template <> struct vector_loader< 1,int8> { NVBIO_HOST_DEVICE static void load(const int8* ptr, int8* vec) { *vec = *ptr; } };
template <> struct vector_loader< 2,int8> { NVBIO_HOST_DEVICE static void load(const int8* ptr, int8* vec) { const uchar2 v = *(const uchar2*)ptr; vec[0] = v.x; vec[1] = v.y; } };
template <> struct vector_loader< 3,int8> { NVBIO_HOST_DEVICE static void load(const int8* ptr, int8* vec) { const uchar3 v = *(const uchar3*)ptr; vec[0] = v.x; vec[1] = v.y; vec[2] = v.z; } };
template <> struct vector_loader< 4,int8> { NVBIO_HOST_DEVICE static void load(const int8* ptr, int8* vec) { const uchar4 v = *(const uchar4*)ptr; vec[0] = v.x; vec[1] = v.y; vec[2] = v.z;  vec[3] = v.w; } };
template <> struct vector_loader< 8,int8>
{
    NVBIO_HOST_DEVICE static void load(const int8* ptr, int8* vec)
    {
        const uint2 v = *(const uint2*)ptr;
        vec[0] = (int8)((v.x >>  0) & 0xFF); vec[1] = (int8)((v.x >> 8)  & 0xFF);
        vec[2] = (int8)((v.x >> 16) & 0xFF); vec[3] = (int8)((v.x >> 24) & 0xFF);
        vec[4] = (int8)((v.y >>  0) & 0xFF); vec[5] = (int8)((v.y >> 8)  & 0xFF);
        vec[6] = (int8)((v.y >> 16) & 0xFF); vec[7] = (int8)((v.y >> 24) & 0xFF);
    }
};
template <> struct vector_loader<16,int8>
{
    NVBIO_HOST_DEVICE static void load(const int8* ptr, int8* vec)
    {
        const uint4 v = *(const uint4*)ptr;

        vec[0] = (int8)((v.x >>  0) & 0xFF); vec[1] = (int8)((v.x >> 8)  & 0xFF);
        vec[2] = (int8)((v.x >> 16) & 0xFF); vec[3] = (int8)((v.x >> 24) & 0xFF);
        vec[4] = (int8)((v.y >>  0) & 0xFF); vec[5] = (int8)((v.y >> 8)  & 0xFF);
        vec[6] = (int8)((v.y >> 16) & 0xFF); vec[7] = (int8)((v.y >> 24) & 0xFF);

        vec[8]  = (int8)((v.z >>  0) & 0xFF); vec[9]  = (int8)((v.z >> 8)  & 0xFF);
        vec[10] = (int8)((v.z >> 16) & 0xFF); vec[11] = (int8)((v.z >> 24) & 0xFF);
        vec[12] = (int8)((v.w >>  0) & 0xFF); vec[13] = (int8)((v.w >> 8)  & 0xFF);
        vec[14] = (int8)((v.w >> 16) & 0xFF); vec[15] = (int8)((v.w >> 24) & 0xFF);
    }
};

template <> struct vector_loader< 1,uint8> { NVBIO_HOST_DEVICE static void load(const uint8* ptr, uint8* vec) { *vec = *ptr; } };
template <> struct vector_loader< 2,uint8> { NVBIO_HOST_DEVICE static void load(const uint8* ptr, uint8* vec) { const uchar2 v = *(const uchar2*)ptr; vec[0] = v.x; vec[1] = v.y; } };
template <> struct vector_loader< 3,uint8> { NVBIO_HOST_DEVICE static void load(const uint8* ptr, uint8* vec) { const uchar3 v = *(const uchar3*)ptr; vec[0] = v.x; vec[1] = v.y; vec[2] = v.z; } };
template <> struct vector_loader< 4,uint8> { NVBIO_HOST_DEVICE static void load(const uint8* ptr, uint8* vec) { const uchar4 v = *(const uchar4*)ptr; vec[0] = v.x; vec[1] = v.y; vec[2] = v.z;  vec[3] = v.w; } };
template <> struct vector_loader< 8,uint8>
{
    NVBIO_HOST_DEVICE static void load(const uint8* ptr, uint8* vec)
    {
        const uint2 v = *(const uint2*)ptr;
        vec[0] = (uint8)((v.x >>  0) & 0xFF); vec[1] = (uint8)((v.x >> 8)  & 0xFF);
        vec[2] = (uint8)((v.x >> 16) & 0xFF); vec[3] = (uint8)((v.x >> 24) & 0xFF);
        vec[4] = (uint8)((v.y >>  0) & 0xFF); vec[5] = (uint8)((v.y >> 8)  & 0xFF);
        vec[6] = (uint8)((v.y >> 16) & 0xFF); vec[7] = (uint8)((v.y >> 24) & 0xFF);
    }
};
template <> struct vector_loader<16,uint8>
{
    NVBIO_HOST_DEVICE static void load(const uint8* ptr, uint8* vec)
    {
        const uint4 v = *(const uint4*)ptr;

        vec[0] = (uint8)((v.x >>  0) & 0xFF); vec[1] = (uint8)((v.x >> 8)  & 0xFF);
        vec[2] = (uint8)((v.x >> 16) & 0xFF); vec[3] = (uint8)((v.x >> 24) & 0xFF);
        vec[4] = (uint8)((v.y >>  0) & 0xFF); vec[5] = (uint8)((v.y >> 8)  & 0xFF);
        vec[6] = (uint8)((v.y >> 16) & 0xFF); vec[7] = (uint8)((v.y >> 24) & 0xFF);

        vec[8]  = (uint8)((v.z >>  0) & 0xFF); vec[9]  = (uint8)((v.z >> 8)  & 0xFF);
        vec[10] = (uint8)((v.z >> 16) & 0xFF); vec[11] = (uint8)((v.z >> 24) & 0xFF);
        vec[12] = (uint8)((v.w >>  0) & 0xFF); vec[13] = (uint8)((v.w >> 8)  & 0xFF);
        vec[14] = (uint8)((v.w >> 16) & 0xFF); vec[15] = (uint8)((v.w >> 24) & 0xFF);
    }
};

template <> struct vector_loader< 1,int16> { NVBIO_HOST_DEVICE static void load(const int16* ptr, int16* vec) { *vec = *ptr; } };
template <> struct vector_loader< 2,int16> { NVBIO_HOST_DEVICE static void load(const int16* ptr, int16* vec) { const int16_2 v = *(const int16_2*)ptr; vec[0] = v.x; vec[1] = v.y; } };
template <> struct vector_loader< 3,int16> { NVBIO_HOST_DEVICE static void load(const int16* ptr, int16* vec)  { const int16_3 v = *(const int16_3*)ptr; vec[0] = v.x; vec[1] = v.y; vec[2] = v.z; } };
template <> struct vector_loader< 4,int16> { NVBIO_HOST_DEVICE static void load(const int16* ptr, int16* vec)  { const int16_4 v = *(const int16_4*)ptr; vec[0] = v.x; vec[1] = v.y; vec[2] = v.z;  vec[3] = v.w; } };

template <> struct vector_loader< 1,uint16> { NVBIO_HOST_DEVICE static void load(const uint16* ptr, uint16* vec) { *vec = *ptr; } };
template <> struct vector_loader< 2,uint16> { NVBIO_HOST_DEVICE static void load(const uint16* ptr, uint16* vec) { const uint16_2 v = *(const uint16_2*)ptr; vec[0] = v.x; vec[1] = v.y; } };
template <> struct vector_loader< 3,uint16> { NVBIO_HOST_DEVICE static void load(const uint16* ptr, uint16* vec) { const uint16_3 v = *(const uint16_3*)ptr; vec[0] = v.x; vec[1] = v.y; vec[2] = v.z; } };
template <> struct vector_loader< 4,uint16> { NVBIO_HOST_DEVICE static void load(const uint16* ptr, uint16* vec) { const uint16_4 v = *(const uint16_4*)ptr; vec[0] = v.x; vec[1] = v.y; vec[2] = v.z;  vec[3] = v.w; } };

template <> struct vector_loader< 1,int32> { NVBIO_HOST_DEVICE static void load(const int32* ptr, int32* vec) { *vec = *ptr; } };
template <> struct vector_loader< 2,int32> { NVBIO_HOST_DEVICE static void load(const int32* ptr, int32* vec) { const int32_2 v = *(const int32_2*)ptr; vec[0] = v.x; vec[1] = v.y; } };
template <> struct vector_loader< 3,int32> { NVBIO_HOST_DEVICE static void load(const int32* ptr, int32* vec)  { const int32_3 v = *(const int32_3*)ptr; vec[0] = v.x; vec[1] = v.y; vec[2] = v.z; } };
template <> struct vector_loader< 4,int32> { NVBIO_HOST_DEVICE static void load(const int32* ptr, int32* vec)  { const int32_4 v = *(const int32_4*)ptr; vec[0] = v.x; vec[1] = v.y; vec[2] = v.z;  vec[3] = v.w; } };

template <> struct vector_loader< 1,uint32> { NVBIO_HOST_DEVICE static void load(const uint32* ptr, uint32* vec) { *vec = *ptr; } };
template <> struct vector_loader< 2,uint32> { NVBIO_HOST_DEVICE static void load(const uint32* ptr, uint32* vec) { const uint32_2 v = *(const uint32_2*)ptr; vec[0] = v.x; vec[1] = v.y; } };
template <> struct vector_loader< 3,uint32> { NVBIO_HOST_DEVICE static void load(const uint32* ptr, uint32* vec) { const uint32_3 v = *(const uint32_3*)ptr; vec[0] = v.x; vec[1] = v.y; vec[2] = v.z; } };
template <> struct vector_loader< 4,uint32> { NVBIO_HOST_DEVICE static void load(const uint32* ptr, uint32* vec) { const uint32_4 v = *(const uint32_4*)ptr; vec[0] = v.x; vec[1] = v.y; vec[2] = v.z;  vec[3] = v.w; } };

template <> struct vector_loader< 1,int64> { NVBIO_HOST_DEVICE static void load(const int64* ptr, int64* vec) { *vec = *ptr; } };
template <> struct vector_loader< 2,int64> { NVBIO_HOST_DEVICE static void load(const int64* ptr, int64* vec) { const int64_2 v = *(const int64_2*)ptr; vec[0] = v.x; vec[1] = v.y; } };

template <> struct vector_loader< 1,uint64> { NVBIO_HOST_DEVICE static void load(const uint64* ptr, uint64* vec) { *vec = *ptr; } };
template <> struct vector_loader< 2,uint64> { NVBIO_HOST_DEVICE static void load(const uint64* ptr, uint64* vec) { const uint64_2 v = *(const uint64_2*)ptr; vec[0] = v.x; vec[1] = v.y; } };

/// load a vector using vectorized-loads
///
template <uint32 VECTOR_WIDTH, typename T>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE 
void vector_load(const T* ptr, T* vec)
{
    vector_loader<VECTOR_WIDTH,T>::load( ptr, vec );
}

} // namespace nvbio
