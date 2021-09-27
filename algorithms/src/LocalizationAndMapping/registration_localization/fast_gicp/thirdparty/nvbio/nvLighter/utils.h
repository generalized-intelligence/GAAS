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

// utils.h
//

#pragma once

#include <nvbio/basic/numbers.h>
#include <nvbio/basic/threads.h>
#include <nvbio/basic/cuda/arch.h>

enum {
    ERROR_FREE    = 0,
    CORRECTIONS   = 1,
    UNFIXABLE     = 2,
    TRIMMED_READS = 3,
    TRIMMED_BASES = 4
};

typedef nvbio::uint8    uint8;
typedef nvbio::int16     int16;
typedef nvbio::uint16   uint16;
typedef nvbio::int32     int32;
typedef nvbio::uint32   uint32;
typedef nvbio::int64     int64;
typedef nvbio::uint64   uint64;
typedef nvbio::uint64_2 uint64_2;
typedef nvbio::uint64_4 uint64_4;

//static const uint32 SAMPLED_KMERS_FILTER_K = 9;    // optimal number of hashes for a Bloom filter with false probability rate of 0.01
//static const uint32 TRUSTED_KMERS_FILTER_K = 11;   // optimal number of hashes for a Bloom filter with false probability rate of 0.0005
//static const uint32 SAMPLED_KMERS_FILTER_K = 7;    // optimal number of hashes for a Bloom filter with false probability rate of 0.01
//static const uint32 TRUSTED_KMERS_FILTER_K = 11;   // optimal number of hashes for a Bloom filter with false probability rate of 0.0005
static const uint32 SAMPLED_KMERS_FILTER_K = 5;    // this parameter should theoretically achieve a worse false probability rate than 0.01, yet in practice it does better
static const uint32 TRUSTED_KMERS_FILTER_K = 8;    // this parameter should theoretically achieve a worse false probability rate than 0.0005, yet in practice it does better

struct hash_functor1
{
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    uint64 operator() (const uint64 kmer) const { return nvbio::hash( kmer ); }
};
struct hash_functor2
{
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    uint64 operator() (const uint64 kmer) const { return nvbio::hash2( kmer ); }
};

enum { MAX_READ_LENGTH = 2048 };

struct KmerCode
{
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    KmerCode() : mask(0), code(0), len(0), invalid(-1) {}

    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    KmerCode(const int l) : mask(0), code(0), len(l), invalid(-1)
    {
        for (int i = 0 ; i < len; ++i)
        {
            mask = mask << 2u;
            mask = mask | 3u;
        }
    }
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    KmerCode(const KmerCode& k) : mask(k.mask), code(k.code), len(k.len), invalid(k.invalid)
    {}

    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    void restart() { code = 0ull ; invalid = -1 ; } 

    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    void push_back(const uint8 c)
    {
        if (invalid != -1)
            invalid++;

        code = ((code << 2ull) & mask) | uint64(c & 3);
        if (c >= 4)
            invalid = 0;

        if (invalid >= len)
            invalid = -1;
    }
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    void push_front(const uint8 c)
    {
        shift_right( 1 );

        if (c >= 4)
            invalid = len - 1;

        code = (code | ((uint64(c & 3)) << (2ull * (len - 1)))) & mask;
    }

    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    void shift_right(int k)
    {
        if (invalid != -1)
            invalid -= k;

        code = (code >> (2ull * k)) & (mask >> (2ull * k));
        if (invalid < 0)
            invalid = -1;
    }

    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    bool is_valid() const { return invalid == -1; }

    uint64 mask;
    uint64 code;
    int    len;
    int    invalid;
};

struct SequenceStats
{
    SequenceStats() : m_reads(0), m_bps(0), m_time(0) {}

    nvbio::Mutex    m_mutex;
    uint64          m_reads;
    uint64          m_bps;
    float           m_time;
};
