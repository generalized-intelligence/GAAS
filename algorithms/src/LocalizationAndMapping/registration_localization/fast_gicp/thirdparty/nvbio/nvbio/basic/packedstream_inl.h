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

#pragma once

#include <nvbio/basic/cached_iterator.h>
#if defined(_OPENMP)
#include <omp.h>
#endif

namespace nvbio {

template <bool BIG_ENDIAN_T, uint32 SYMBOL_SIZE, typename Symbol, typename InputStream, typename IndexType, typename ValueType>
struct packer {
};

template <bool BIG_ENDIAN_T, uint32 SYMBOL_SIZE, typename Symbol, typename InputStream, typename IndexType>
struct packer<BIG_ENDIAN_T,SYMBOL_SIZE,Symbol,InputStream,IndexType,uint32>
{
    static NVBIO_FORCEINLINE NVBIO_HOST_DEVICE Symbol get_symbol(InputStream stream, const IndexType sym_idx)
    {
        const uint32 SYMBOL_COUNT = 1u << SYMBOL_SIZE;
        const uint32 SYMBOL_MASK  = SYMBOL_COUNT - 1u;

        typedef typename unsigned_type<IndexType>::type index_type;

        const uint64     bit_idx  = uint64(sym_idx) * SYMBOL_SIZE;
        const index_type word_idx = index_type( bit_idx >> 5u );

        if (is_pow2<SYMBOL_SIZE>())
        {
            const uint32 word = stream[ word_idx ];
            const uint32 symbol_offset = BIG_ENDIAN_T ? (32u - SYMBOL_SIZE  - uint32(bit_idx & 31u)) : uint32(bit_idx & 31u);
            const uint32 symbol = (word >> symbol_offset) & SYMBOL_MASK;

            return Symbol( symbol );
        }
        else
        {
            const uint32 word1 = stream[ word_idx ];
            const uint32 symbol_offset = uint32(bit_idx & 31u);
            const uint32 symbol1 = (word1 >> symbol_offset) & SYMBOL_MASK;

            // check if we need to read a second word
            const uint32 read_bits = nvbio::min( 32u - symbol_offset, SYMBOL_SIZE );
            const uint32 rem_bits  = SYMBOL_SIZE - read_bits;
            if (rem_bits)
            {
                const uint32 rem_mask = (1u << rem_bits) - 1u;

                const uint32 word2 = stream[ word_idx+1 ];
                const uint32 symbol2 = word2 & rem_mask;

                return Symbol( symbol1 | (symbol2 << read_bits) );
            }
            else
                return Symbol( symbol1 );
        }
    }

    static NVBIO_FORCEINLINE NVBIO_HOST_DEVICE void set_symbol(InputStream stream, const IndexType sym_idx, Symbol sym)
    {
        const uint32 SYMBOL_COUNT = 1u << SYMBOL_SIZE;
        const uint32 SYMBOL_MASK  = SYMBOL_COUNT - 1u;

        typedef typename unsigned_type<IndexType>::type index_type;

        const uint64     bit_idx  = uint64(sym_idx) * SYMBOL_SIZE;
        const index_type word_idx = index_type( bit_idx >> 5u );

        if (is_pow2<SYMBOL_SIZE>())
        {
                  uint32 word          = stream[ word_idx ];
            const uint32 symbol_offset = BIG_ENDIAN_T ? (32u - SYMBOL_SIZE - uint32(bit_idx & 31u)) : uint32(bit_idx & 31u);
            const uint32 symbol        = uint32(sym & SYMBOL_MASK) << symbol_offset;

            // clear all bits
            word &= ~(SYMBOL_MASK << symbol_offset);

            // set bits
            stream[ word_idx ] = word | symbol;
        }
        else
        {
                  uint32 word1         = stream[ word_idx ];
            const uint32 symbol_offset = uint32(bit_idx & 31u);
            const uint32 symbol1       = uint32(sym & SYMBOL_MASK) << symbol_offset;

            // clear all bits
            word1 &= ~(SYMBOL_MASK << symbol_offset);

            // set bits
            stream[ word_idx ] = word1 | symbol1;

            // check if we need to write a second word
            const uint32 read_bits = nvbio::min( 32u - symbol_offset, SYMBOL_SIZE );
            const uint32 rem_bits  = SYMBOL_SIZE - read_bits;
            if (rem_bits)
            {
                const uint32 rem_mask = (1u << rem_bits) - 1u;

                      uint32 word2   = stream[ word_idx+1 ];
                const uint32 symbol2 = uint32(sym & SYMBOL_MASK) >> read_bits;

                // clear all bits
                word2 &= ~rem_mask;

                // set bits
                stream[ word_idx+1 ] = word2 | symbol2;
            }
        }
    }
};

template <bool BIG_ENDIAN_T, uint32 SYMBOL_SIZE, typename Symbol, typename InputStream, typename IndexType>
struct packer<BIG_ENDIAN_T,SYMBOL_SIZE,Symbol,InputStream,IndexType,uint64>
{
    static NVBIO_FORCEINLINE NVBIO_HOST_DEVICE Symbol get_symbol(InputStream stream, const IndexType sym_idx)
    {
        const uint32 SYMBOL_COUNT = 1u << SYMBOL_SIZE;
        const uint32 SYMBOL_MASK  = SYMBOL_COUNT - 1u;

        typedef typename unsigned_type<IndexType>::type index_type;

        const uint64     bit_idx  = uint64(sym_idx) * SYMBOL_SIZE;
        const index_type word_idx = index_type( bit_idx >> 6u );

        if (is_pow2<SYMBOL_SIZE>())
        {
            const uint64 word          = stream[ word_idx ];
            const uint32 symbol_offset = BIG_ENDIAN_T ? (64u - SYMBOL_SIZE  - uint32(bit_idx & 63u)) : uint32(bit_idx & 63u);
            const uint32 symbol        = uint32((word >> symbol_offset) & SYMBOL_MASK);

            return Symbol( symbol );
        }
        else
        {
            const uint64 word1         = stream[ word_idx ];
            const uint32 symbol_offset = uint32(bit_idx & 63u);
            const uint32 symbol1       = uint32((word1 >> symbol_offset) & SYMBOL_MASK);

            // check if we need to read a second word
            const uint32 read_bits = nvbio::min( 64u - symbol_offset, SYMBOL_SIZE );
            const uint32 rem_bits  = SYMBOL_SIZE - read_bits;
            if (rem_bits)
            {
                const uint64 rem_mask = (uint64(1u) << rem_bits) - 1u;

                const uint64 word2    = stream[ word_idx+1 ];
                const uint32 symbol2  = uint32(word2 & rem_mask);

                return Symbol( symbol1 | (symbol2 << read_bits) );
            }
            else
                return Symbol( symbol1 );
        }
    }

    static NVBIO_FORCEINLINE NVBIO_HOST_DEVICE void set_symbol(InputStream stream, const IndexType sym_idx, Symbol sym)
    {
        const uint32 SYMBOL_COUNT = 1u << SYMBOL_SIZE;
        const uint32 SYMBOL_MASK  = SYMBOL_COUNT - 1u;

        typedef typename unsigned_type<IndexType>::type index_type;

        const uint64     bit_idx  = uint64(sym_idx) * SYMBOL_SIZE;
        const index_type word_idx = index_type( bit_idx >> 6u );

        if (is_pow2<SYMBOL_SIZE>())
        {
                  uint64 word = stream[ word_idx ];
            const uint32 symbol_offset = BIG_ENDIAN_T ? (64u - SYMBOL_SIZE - uint32(bit_idx & 63u)) : uint32(bit_idx & 63u);
            const uint64 symbol = uint64(sym & SYMBOL_MASK) << symbol_offset;

            // clear all bits
            word &= ~(uint64(SYMBOL_MASK) << symbol_offset);

            // set bits
            stream[ word_idx ] = word | symbol;
        }
        else
        {
                  uint64 word1 = stream[ word_idx ];
            const uint32 symbol_offset = uint32(bit_idx & 63);
            const uint64 symbol1 = uint64(sym & SYMBOL_MASK) << symbol_offset;

            // clear all bits
            word1 &= ~(uint64(SYMBOL_MASK) << symbol_offset);

            // set bits
            stream[ word_idx ] = word1 | symbol1;

            // check if we need to write a second word
            const uint32 read_bits = nvbio::min( 64u - symbol_offset, SYMBOL_SIZE );
            const uint32 rem_bits  = SYMBOL_SIZE - read_bits;
            if (rem_bits)
            {
                const uint64 rem_mask = (uint64(1u) << rem_bits) - 1u;

                      uint64 word2   = stream[ word_idx+1 ];
                const uint64 symbol2 = uint64(sym & SYMBOL_MASK) >> read_bits;

                // clear all bits
                word2 &= ~rem_mask;

                // set bits
                stream[ word_idx+1 ] = word2 | symbol2;
            }
        }
    }
};

template <bool BIG_ENDIAN_T, uint32 SYMBOL_SIZE, typename Symbol, typename InputStream, typename IndexType>
struct packer<BIG_ENDIAN_T,SYMBOL_SIZE,Symbol,InputStream,IndexType,uint8>
{
    static NVBIO_FORCEINLINE NVBIO_HOST_DEVICE Symbol get_symbol(InputStream stream, const IndexType sym_idx)
    {
        const uint8 SYMBOL_COUNT = uint8(1u) << SYMBOL_SIZE;
        const uint8 SYMBOL_MASK  = SYMBOL_COUNT - uint8(1u);

        typedef typename unsigned_type<IndexType>::type index_type;

        const uint64     bit_idx  = uint64(sym_idx) * SYMBOL_SIZE;
        const index_type word_idx = index_type( bit_idx >> 3u );

        if (is_pow2<SYMBOL_SIZE>())
        {
            const uint8 word = stream[ word_idx ];
            const uint8 symbol_offset = BIG_ENDIAN_T ? (8u - SYMBOL_SIZE - uint8(bit_idx & 7u)) : uint8(bit_idx & 7u);
            const uint8 symbol = (word >> symbol_offset) & SYMBOL_MASK;

            return Symbol( symbol );
        }
        else
        {
            const uint8 word1 = stream[ word_idx ];
            const uint8 symbol_offset = uint8(bit_idx & 7u);
            const uint8 symbol1 = (word1 >> symbol_offset) & SYMBOL_MASK;

            // check if we need to read a second word
            const uint32 read_bits = nvbio::min( 8u - symbol_offset, SYMBOL_SIZE );
            const uint32 rem_bits  = SYMBOL_SIZE - read_bits;
            if (rem_bits)
            {
                const uint8 rem_mask = uint8((1u << rem_bits) - 1u);

                const uint8 word2 = stream[ word_idx+1 ];
                const uint8 symbol2 = word2 & rem_mask;

                return Symbol( symbol1 | (symbol2 << read_bits) );
            }
            else
                return Symbol( symbol1 );
        }
    }

    static NVBIO_FORCEINLINE NVBIO_HOST_DEVICE void set_symbol(InputStream stream, const IndexType sym_idx, Symbol sym)
    {
        const uint8 SYMBOL_COUNT = uint8(1u) << SYMBOL_SIZE;
        const uint8 SYMBOL_MASK  = SYMBOL_COUNT - uint8(1u);

        typedef typename unsigned_type<IndexType>::type index_type;

        const uint64     bit_idx  = uint64(sym_idx) * SYMBOL_SIZE;
        const index_type word_idx = index_type( bit_idx >> 3u );

        if (is_pow2<SYMBOL_SIZE>())
        {
                  uint8 word = stream[ word_idx ];
            const uint8 symbol_offset = BIG_ENDIAN_T ? (8u - SYMBOL_SIZE - uint8(bit_idx & 7u)) : uint8(bit_idx & 7u);
            const uint8 symbol = uint32(sym & SYMBOL_MASK) << symbol_offset;

            // clear all bits
            word &= ~(SYMBOL_MASK << symbol_offset);

            // set bits
            stream[ word_idx ] = word | symbol;
        }
        else
        {
                  uint8 word1 = stream[ word_idx ];
            const uint8 symbol_offset = uint8(bit_idx & 7u);
            const uint8 symbol1 = uint8(sym & SYMBOL_MASK) << symbol_offset;

            // clear all bits
            word1 &= ~(SYMBOL_MASK << symbol_offset);

            // set bits
            stream[ word_idx ] = word1 | symbol1;

            // check if we need to write a second word
            const uint32 read_bits = nvbio::min( 8u - symbol_offset, SYMBOL_SIZE );
            const uint32 rem_bits  = SYMBOL_SIZE - read_bits;
            if (rem_bits)
            {
                      uint8 word2   = stream[ word_idx+1 ];
                const uint8 symbol2 = uint32(sym & SYMBOL_MASK) >> read_bits;

                const uint8 rem_mask = uint8((1u << rem_bits) - 1u);

                // clear all bits
                word2 &= ~rem_mask;

                // set bits
                stream[ word_idx+1 ] = word2 | symbol2;
            }
        }
    }
};


template <bool BIG_ENDIAN_T, typename Symbol, typename InputStream, typename IndexType>
struct packer<BIG_ENDIAN_T,2u,Symbol,InputStream,IndexType,uint32>
{
    static NVBIO_FORCEINLINE NVBIO_HOST_DEVICE Symbol get_symbol(InputStream stream, const IndexType sym_idx)
    {
        const uint32 SYMBOL_MASK = 3u;

        typedef typename unsigned_type<IndexType>::type index_type;

        const index_type word_idx = sym_idx >> 4u;

        const uint32 word = stream[ word_idx ];
        const uint32 symbol_offset = BIG_ENDIAN_T ? (30u - (uint32(sym_idx & 15u) << 1)) : uint32((sym_idx & 15u) << 1);
        const uint32 symbol = (word >> symbol_offset) & SYMBOL_MASK;

        return Symbol( symbol );
    }

    static NVBIO_FORCEINLINE NVBIO_HOST_DEVICE void set_symbol(InputStream stream, const IndexType sym_idx, Symbol sym)
    {
        const uint32 SYMBOL_MASK = 3u;

        typedef typename unsigned_type<IndexType>::type index_type;

        const index_type word_idx = sym_idx >> 4u;

              uint32 word = stream[ word_idx ];
        const uint32 symbol_offset = BIG_ENDIAN_T ? (30u - (uint32(sym_idx & 15u) << 1)) : uint32((sym_idx & 15u) << 1);
        const uint32 symbol = uint32(sym & SYMBOL_MASK) << symbol_offset;

        // clear all bits
        word &= ~(SYMBOL_MASK << symbol_offset);

        // set bits
        stream[ word_idx ] = word | symbol;
    }
};
template <bool BIG_ENDIAN_T, typename Symbol, typename InputStream, typename IndexType>
struct packer<BIG_ENDIAN_T,4u,Symbol,InputStream,IndexType,uint32>
{
    static NVBIO_FORCEINLINE NVBIO_HOST_DEVICE Symbol get_symbol(InputStream stream, const IndexType sym_idx)
    {
        const uint32 SYMBOL_MASK = 15u;

        typedef typename unsigned_type<IndexType>::type index_type;

        const index_type word_idx = sym_idx >> 3u;

        const uint32 word = stream[ word_idx ];
        const uint32 symbol_offset = BIG_ENDIAN_T ? (28u - (uint32(sym_idx & 7u) << 2)) : uint32((sym_idx & 7u) << 2);
        const uint32 symbol = (word >> symbol_offset) & SYMBOL_MASK;

        return Symbol( symbol );
    }

    static NVBIO_FORCEINLINE NVBIO_HOST_DEVICE void set_symbol(InputStream stream, const IndexType sym_idx, Symbol sym)
    {
        const uint32 SYMBOL_MASK = 15u;

        typedef typename unsigned_type<IndexType>::type index_type;

        const index_type word_idx = sym_idx >> 3u;

              uint32 word = stream[ word_idx ];
        const uint32 symbol_offset = BIG_ENDIAN_T ? (28u - (uint32(sym_idx & 7u) << 2)) : uint32((sym_idx & 7u) << 2);
        const uint32 symbol = uint32(sym & SYMBOL_MASK) << symbol_offset;

        // clear all bits
        word &= ~(SYMBOL_MASK << symbol_offset);

        // set bits
        stream[ word_idx ] = word | symbol;
    }
};

template <bool BIG_ENDIAN_T, typename Symbol, typename InputStream, typename IndexType>
struct packer<BIG_ENDIAN_T,2u,Symbol,InputStream,IndexType,uint4>
{
    static NVBIO_FORCEINLINE NVBIO_HOST_DEVICE Symbol get_symbol(InputStream stream, const IndexType sym_idx)
    {
        const uint32 SYMBOL_MASK = 3u;

        typedef typename unsigned_type<IndexType>::type index_type;

        const index_type word_idx = sym_idx >> 6u;

        const uint4  word = stream[ word_idx ];
        const uint32 symbol_comp   = (sym_idx & 63u) >> 4u;
        const uint32 symbol_offset = BIG_ENDIAN_T ? (30u - (uint32(sym_idx & 15u) << 1)) : uint32((sym_idx & 15u) << 1);
        const uint32 symbol = (comp( word, symbol_comp ) >> symbol_offset) & SYMBOL_MASK;

        return Symbol( symbol );
    }

    static NVBIO_FORCEINLINE NVBIO_HOST_DEVICE void set_symbol(InputStream stream, const IndexType sym_idx, Symbol sym)
    {
        const uint32 SYMBOL_MASK = 3u;

        typedef typename unsigned_type<IndexType>::type index_type;

        const index_type word_idx = sym_idx >> 6u;

              uint4  word = stream[ word_idx ];
        const uint32 symbol_comp   = (sym_idx & 63u) >> 4u;
        const uint32 symbol_offset = BIG_ENDIAN_T ? (30u - (uint32(sym_idx & 15u) << 1)) : uint32((sym_idx & 15u) << 1);
        const uint32 symbol = uint32(sym & SYMBOL_MASK) << symbol_offset;

        // clear all bits
        select( word, symbol_comp ) &= ~(SYMBOL_MASK << symbol_offset);
        select( word, symbol_comp ) |= symbol;

        // set bits
        stream[ word_idx ] = word;
    }
};
template <bool BIG_ENDIAN_T, typename Symbol, typename InputStream, typename IndexType>
struct packer<BIG_ENDIAN_T,4u,Symbol,InputStream,IndexType,uint4>
{
    static NVBIO_FORCEINLINE NVBIO_HOST_DEVICE Symbol get_symbol(InputStream stream, const IndexType sym_idx)
    {
        const uint32 SYMBOL_MASK = 15u;

        typedef typename unsigned_type<IndexType>::type index_type;

        const index_type word_idx = sym_idx >> 5u;

        const uint4 word = stream[ word_idx ];
        const uint32 symbol_comp   = (sym_idx & 31u) >> 3u;
        const uint32 symbol_offset = BIG_ENDIAN_T ? (28u - (uint32(sym_idx & 7u) << 2)) : uint32((sym_idx & 7u) << 2);
        const uint32 symbol = (comp( word, symbol_comp ) >> symbol_offset) & SYMBOL_MASK;

        return Symbol( symbol );
    }

    static NVBIO_FORCEINLINE NVBIO_HOST_DEVICE void set_symbol(InputStream stream, const IndexType sym_idx, Symbol sym)
    {
        const uint32 SYMBOL_MASK = 15u;

        typedef typename unsigned_type<IndexType>::type index_type;

        const index_type word_idx = sym_idx >> 5u;

              uint4  word = stream[ word_idx ];
        const uint32 symbol_comp   = (sym_idx & 31u) >> 3u;
        const uint32 symbol_offset = BIG_ENDIAN_T ? (28u - (uint32(sym_idx & 7u) << 2)) : uint32((sym_idx & 7u) << 2);
        const uint32 symbol = uint32(sym & SYMBOL_MASK) << symbol_offset;

        // clear all bits
        select( word, symbol_comp ) &= ~(SYMBOL_MASK << symbol_offset);
        select( word, symbol_comp ) |= symbol;

        // set bits
        stream[ word_idx ] = word;
    }
};
template <bool BIG_ENDIAN_T, typename Symbol, typename InputStream, typename IndexType>
struct packer<BIG_ENDIAN_T,8u,Symbol,InputStream,IndexType,uint4>
{
    static NVBIO_FORCEINLINE NVBIO_HOST_DEVICE Symbol get_symbol(InputStream stream, const IndexType sym_idx)
    {
        const uint32 SYMBOL_MASK = 255u;

        typedef typename unsigned_type<IndexType>::type index_type;

        const index_type word_idx = sym_idx >> 4u;

        const uint4 word = stream[ word_idx ];
        const uint32 symbol_comp   = (sym_idx & 15u) >> 2u;
        const uint32 symbol_offset = BIG_ENDIAN_T ? (24u - (uint32(sym_idx & 3u) << 3)) : uint32((sym_idx & 3u) << 3);
        const uint32 symbol = (comp( word, symbol_comp ) >> symbol_offset) & SYMBOL_MASK;

        return Symbol( symbol );
    }

    static NVBIO_FORCEINLINE NVBIO_HOST_DEVICE void set_symbol(InputStream stream, const IndexType sym_idx, Symbol sym)
    {
        const uint32 SYMBOL_MASK = 255u;

        typedef typename unsigned_type<IndexType>::type index_type;

        const index_type word_idx = sym_idx >> 4u;

              uint4  word = stream[ word_idx ];
        const uint32 symbol_comp   = (sym_idx & 15u) >> 2u;
        const uint32 symbol_offset = BIG_ENDIAN_T ? (24u - (uint32(sym_idx & 3u) << 3)) : uint32((sym_idx & 3u) << 3);
        const uint32 symbol = uint32(sym & SYMBOL_MASK) << symbol_offset;

        // clear all bits
        select( word, symbol_comp ) &= ~(SYMBOL_MASK << symbol_offset);
        select( word, symbol_comp ) |= symbol;

        // set bits
        stream[ word_idx ] = word;
    }
};

template <bool BIG_ENDIAN_T, typename Symbol, typename InputStream, typename IndexType>
struct packer<BIG_ENDIAN_T,2u,Symbol,InputStream,IndexType,uint64>
{
    static NVBIO_FORCEINLINE NVBIO_HOST_DEVICE Symbol get_symbol(InputStream stream, const IndexType sym_idx)
    {
        const uint32 SYMBOL_MASK = 3u;

        typedef typename unsigned_type<IndexType>::type index_type;

        const index_type word_idx = sym_idx >> 5u;

        const uint64 word = stream[ word_idx ];
        const uint32 symbol_offset = BIG_ENDIAN_T ? (62u - (uint32(sym_idx & 31u) << 1)) : uint32((sym_idx & 31u) << 1);
        const uint64 symbol = (word >> symbol_offset) & SYMBOL_MASK;

        return Symbol( symbol );
    }

    static NVBIO_FORCEINLINE NVBIO_HOST_DEVICE void set_symbol(InputStream stream, const IndexType sym_idx, Symbol sym)
    {
        const uint32 SYMBOL_MASK = 3u;

        typedef typename unsigned_type<IndexType>::type index_type;

        const index_type word_idx = sym_idx >> 5u;

              uint64 word = stream[ word_idx ];
        const uint32 symbol_offset = BIG_ENDIAN_T ? (62u - (uint32(sym_idx & 31u) << 1)) : uint32((sym_idx & 31u) << 1);
        const uint64 symbol = uint64(sym & SYMBOL_MASK) << symbol_offset;

        // clear all bits
        word &= ~(uint64(SYMBOL_MASK) << symbol_offset);

        // set bits
        stream[ word_idx ] = word | symbol;
    }
};
template <bool BIG_ENDIAN_T, typename Symbol, typename InputStream, typename IndexType>
struct packer<BIG_ENDIAN_T,4u,Symbol,InputStream,IndexType,uint64>
{
    static NVBIO_FORCEINLINE NVBIO_HOST_DEVICE Symbol get_symbol(InputStream stream, const IndexType sym_idx)
    {
        const uint32 SYMBOL_MASK = 15u;

        typedef typename unsigned_type<IndexType>::type index_type;

        const index_type word_idx = sym_idx >> 5u;

        const uint64 word = stream[ word_idx ];
        const uint32 symbol_offset = BIG_ENDIAN_T ? (60u - (uint32(sym_idx & 15u) << 2)) : uint32((sym_idx & 15u) << 2);
        const uint64 symbol = (word >> symbol_offset) & SYMBOL_MASK;

        return Symbol( symbol );
    }

    static NVBIO_FORCEINLINE NVBIO_HOST_DEVICE void set_symbol(InputStream stream, const IndexType sym_idx, Symbol sym)
    {
        const uint32 SYMBOL_MASK = 15u;

        typedef typename unsigned_type<IndexType>::type index_type;

        const index_type word_idx = sym_idx >> 5u;

              uint64 word = stream[ word_idx ];
        const uint32 symbol_offset = BIG_ENDIAN_T ? (60u - (uint32(sym_idx & 15u) << 2)) : uint32((sym_idx & 15u) << 2);
        const uint64 symbol = uint32(sym & SYMBOL_MASK) << symbol_offset;

        // clear all bits
        word &= ~(SYMBOL_MASK << symbol_offset);

        // set bits
        stream[ word_idx ] = word | symbol;
    }
};


template <typename InputStream, typename Symbol, uint32 SYMBOL_SIZE_T, bool BIG_ENDIAN_T, typename IndexType>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE Symbol PackedStream<InputStream,Symbol, SYMBOL_SIZE_T, BIG_ENDIAN_T, IndexType>::get(const index_type sym_idx) const
{
    return packer<BIG_ENDIAN_T, SYMBOL_SIZE,Symbol,InputStream,IndexType,storage_type>::get_symbol( m_stream, sym_idx + m_index );
}
template <typename InputStream, typename Symbol, uint32 SYMBOL_SIZE_T, bool BIG_ENDIAN_T, typename IndexType>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE void PackedStream<InputStream,Symbol, SYMBOL_SIZE_T, BIG_ENDIAN_T, IndexType>::set(const index_type sym_idx, const Symbol sym)
{
    return packer<BIG_ENDIAN_T, SYMBOL_SIZE,Symbol,InputStream,IndexType,storage_type>::set_symbol( m_stream, sym_idx + m_index, sym );
}

// pre-increment operator
//
template <typename InputStream, typename Symbol, uint32 SYMBOL_SIZE_T, bool BIG_ENDIAN_T, typename IndexType>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
PackedStream<InputStream,Symbol, SYMBOL_SIZE_T, BIG_ENDIAN_T, IndexType>&
PackedStream<InputStream,Symbol, SYMBOL_SIZE_T, BIG_ENDIAN_T, IndexType>::operator++ ()
{
    ++m_index;
    return *this;
}

// post-increment operator
//
template <typename InputStream, typename Symbol, uint32 SYMBOL_SIZE_T, bool BIG_ENDIAN_T, typename IndexType>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
PackedStream<InputStream,Symbol, SYMBOL_SIZE_T, BIG_ENDIAN_T, IndexType>
PackedStream<InputStream,Symbol, SYMBOL_SIZE_T, BIG_ENDIAN_T, IndexType>::operator++ (int dummy)
{
    This r( m_stream, m_index );
    ++m_index;
    return r;
}

// pre-decrement operator
//
template <typename InputStream, typename Symbol, uint32 SYMBOL_SIZE_T, bool BIG_ENDIAN_T, typename IndexType>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
PackedStream<InputStream,Symbol, SYMBOL_SIZE_T, BIG_ENDIAN_T, IndexType>&
PackedStream<InputStream,Symbol, SYMBOL_SIZE_T, BIG_ENDIAN_T, IndexType>::operator-- ()
{
    --m_index;
    return *this;
}

// post-decrement operator
//
template <typename InputStream, typename Symbol, uint32 SYMBOL_SIZE_T, bool BIG_ENDIAN_T, typename IndexType>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE 
PackedStream<InputStream,Symbol, SYMBOL_SIZE_T, BIG_ENDIAN_T, IndexType>
PackedStream<InputStream,Symbol, SYMBOL_SIZE_T, BIG_ENDIAN_T, IndexType>::operator-- (int dummy)
{
    This r( m_stream, m_index );
    --m_index;
    return r;
}

// add offset
//
template <typename InputStream, typename Symbol, uint32 SYMBOL_SIZE_T, bool BIG_ENDIAN_T, typename IndexType>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
PackedStream<InputStream,Symbol, SYMBOL_SIZE_T, BIG_ENDIAN_T, IndexType>&
PackedStream<InputStream,Symbol, SYMBOL_SIZE_T, BIG_ENDIAN_T, IndexType>::operator+= (const sindex_type distance)
{
    m_index += distance;
    return *this;
}

// subtract offset
//
template <typename InputStream, typename Symbol, uint32 SYMBOL_SIZE_T, bool BIG_ENDIAN_T, typename IndexType>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
PackedStream<InputStream,Symbol, SYMBOL_SIZE_T, BIG_ENDIAN_T, IndexType>&
PackedStream<InputStream,Symbol, SYMBOL_SIZE_T, BIG_ENDIAN_T, IndexType>::operator-= (const sindex_type distance)
{
    m_index -= distance;
    return *this;
}

// add offset
//
template <typename InputStream, typename Symbol, uint32 SYMBOL_SIZE_T, bool BIG_ENDIAN_T, typename IndexType>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
PackedStream<InputStream,Symbol, SYMBOL_SIZE_T, BIG_ENDIAN_T, IndexType>
PackedStream<InputStream,Symbol, SYMBOL_SIZE_T, BIG_ENDIAN_T, IndexType>::operator+ (const sindex_type distance) const
{
    return This( m_stream, m_index + distance );
}

// subtract offset
//
template <typename InputStream, typename Symbol, uint32 SYMBOL_SIZE_T, bool BIG_ENDIAN_T, typename IndexType>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
PackedStream<InputStream,Symbol, SYMBOL_SIZE_T, BIG_ENDIAN_T, IndexType>
PackedStream<InputStream,Symbol, SYMBOL_SIZE_T, BIG_ENDIAN_T, IndexType>::operator- (const sindex_type distance) const
{
    return This( m_stream, m_index - distance );
}

// difference
//
template <typename InputStream, typename Symbol, uint32 SYMBOL_SIZE_T, bool BIG_ENDIAN_T, typename IndexType>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
typename PackedStream<InputStream,Symbol, SYMBOL_SIZE_T, BIG_ENDIAN_T, IndexType>::sindex_type
PackedStream<InputStream,Symbol, SYMBOL_SIZE_T, BIG_ENDIAN_T, IndexType>::operator- (const PackedStream it) const
{
    return sindex_type( m_index - it.m_index );
}

// assignment operator
//
template <typename Stream>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE PackedStreamRef<Stream>& PackedStreamRef<Stream>::operator= (const PackedStreamRef& ref)
{
    return (*this = Symbol( ref ));
}

// assignment operator
//
template <typename Stream>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE PackedStreamRef<Stream>& PackedStreamRef<Stream>::operator= (const Symbol s)
{
    m_stream.set( s );
    return *this;
}

// conversion operator
//
template <typename Stream>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE PackedStreamRef<Stream>::operator Symbol() const
{
    return m_stream.get();
}

/// less than
///
template <typename InputStream, typename Symbol, uint32 SYMBOL_SIZE_T, bool BIG_ENDIAN_T, typename IndexType>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE bool operator< (
    const PackedStream<InputStream,Symbol,SYMBOL_SIZE_T,BIG_ENDIAN_T,IndexType>& it1,
    const PackedStream<InputStream,Symbol,SYMBOL_SIZE_T,BIG_ENDIAN_T,IndexType>& it2)
{
    return it1.index() < it2.index();
}

/// greater than
///
template <typename InputStream, typename Symbol, uint32 SYMBOL_SIZE_T, bool BIG_ENDIAN_T, typename IndexType>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE bool operator> (
    const PackedStream<InputStream,Symbol,SYMBOL_SIZE_T,BIG_ENDIAN_T,IndexType>& it1,
    const PackedStream<InputStream,Symbol,SYMBOL_SIZE_T,BIG_ENDIAN_T,IndexType>& it2)
{
    return it1.index() > it2.index();
}

/// equality test
///
template <typename InputStream, typename Symbol, uint32 SYMBOL_SIZE_T, bool BIG_ENDIAN_T, typename IndexType>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE bool operator== (
    const PackedStream<InputStream,Symbol,SYMBOL_SIZE_T,BIG_ENDIAN_T,IndexType>& it1,
    const PackedStream<InputStream,Symbol,SYMBOL_SIZE_T,BIG_ENDIAN_T,IndexType>& it2)
{
    return it1.stream() == it2.stream() && it1.index() == it2.index();
}

/// inequality test
///
template <typename InputStream, typename Symbol, uint32 SYMBOL_SIZE_T, bool BIG_ENDIAN_T, typename IndexType>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE bool operator!= (
    const PackedStream<InputStream,Symbol,SYMBOL_SIZE_T,BIG_ENDIAN_T,IndexType>& it1,
    const PackedStream<InputStream,Symbol,SYMBOL_SIZE_T,BIG_ENDIAN_T,IndexType>& it2)
{
    return it1.stream() != it2.stream() || it1.index() != it2.index();
}

template <bool BIG_ENDIAN, uint32 SYMBOL_SIZE, typename Symbol, typename InputStream, typename IndexType, typename ValueType>
struct forward_packer
{
    typedef ForwardPackedStream<InputStream,Symbol,SYMBOL_SIZE,BIG_ENDIAN,IndexType> forward_stream_type;

    static const uint32 SYMBOL_COUNT     = 1u << SYMBOL_SIZE;
    static const uint32 SYMBOL_MASK      = SYMBOL_COUNT - 1u;
    static const uint32 WORD_SIZE        = 8u * uint32( sizeof(ValueType) );
    static const uint32 SYMBOLS_PER_WORD = WORD_SIZE / SYMBOL_SIZE;

    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    static void rebase(forward_stream_type& it)
    {
        const uint32 symbol_idx = it.m_index & (SYMBOLS_PER_WORD-1);

        it.m_word_index  = it.m_index / SYMBOLS_PER_WORD;
        it.m_word_offset = BIG_ENDIAN ? (WORD_SIZE - SYMBOL_SIZE - symbol_idx * SYMBOL_SIZE) : symbol_idx * SYMBOL_SIZE;

        it.m_word = it.m_stream[ it.m_word_index ];
    }

    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    static void next(forward_stream_type& it)
    {
        it.m_index++;

        if (BIG_ENDIAN)
        {
            if (it.m_word_offset > 0)
                it.m_word_offset -= SYMBOL_SIZE;
            else
            {
                // need a new word
                ++it.m_word_index;

                it.m_word        = it.m_stream[ it.m_word_index ];
                it.m_word_offset = WORD_SIZE - SYMBOL_SIZE;
            }
        }
        else
        {
            if (it.m_word_offset < WORD_SIZE - SYMBOL_SIZE)
                it.m_word_offset += SYMBOL_SIZE;
            else
            {
                // need a new word
                ++it.m_word_index;

                it.m_word        = it.m_stream[ it.m_word_index ];
                it.m_word_offset = 0;
            }
        }
    }
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    static void prev(forward_stream_type& it)
    {
        it.m_index--;

        if (BIG_ENDIAN)
        {
            if (it.m_word_offset < WORD_SIZE - SYMBOL_SIZE)
                it.m_word_offset += SYMBOL_SIZE;
            else
            {
                // need a new word
                --it.m_word_index;

                it.m_word        = it.m_stream[ it.m_word_index ];
                it.m_word_offset = 0u;
            }
        }
        else
        {
            if (it.m_word_offset > 0)
                it.m_word_offset -= SYMBOL_SIZE;
            else
            {
                // need a new word
                --it.m_word_index;

                it.m_word        = it.m_stream[ it.m_word_index ];
                it.m_word_offset = WORD_SIZE - SYMBOL_SIZE;
            }
        }
    }
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    static Symbol fetch(const forward_stream_type& it)
    {
        return Symbol( (it.m_word >> it.m_word_offset) & SYMBOL_MASK );
    }
};

template <bool BIG_ENDIAN, uint32 SYMBOL_SIZE, typename Symbol, typename InputStream, typename IndexType>
struct forward_packer<BIG_ENDIAN, SYMBOL_SIZE, Symbol, InputStream, IndexType, uint4>
{
    typedef ForwardPackedStream<InputStream,Symbol,SYMBOL_SIZE,BIG_ENDIAN,IndexType> forward_stream_type;

    static const uint32 SYMBOL_COUNT        = 1u << SYMBOL_SIZE;
    static const uint32 SYMBOL_MASK         = SYMBOL_COUNT - 1u;
    static const uint32 WORD_SIZE           = 128;
    static const uint32 SUBWORD_SIZE        = 32;
    static const uint32 SYMBOLS_PER_WORD    = WORD_SIZE / SYMBOL_SIZE;
    static const uint32 SYMBOLS_PER_SUBWORD = SUBWORD_SIZE / SYMBOL_SIZE;

    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    static void rebase(forward_stream_type& it)
    {
        const uint32 symbol_idx = it.m_index & (SYMBOLS_PER_WORD-1);

        it.m_word_index  = it.m_index / SYMBOLS_PER_WORD;
        it.m_word_offset = it.m_index & (SYMBOLS_PER_WORD-1);

        it.m_word = it.m_stream[ it.m_word_index ];
    }

    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    static void next(forward_stream_type& it)
    {
        it.m_index++;

        if (it.m_word_offset < SYMBOLS_PER_WORD-1)
            it.m_word_offset++;
        else
        {
            // need a new word
            ++it.m_word_index;

            it.m_word        = it.m_stream[ it.m_word_index ];
            it.m_word_offset = 0;
        }
    }
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    static void prev(forward_stream_type& it)
    {
        it.m_index--;

        if (it.m_word_offset > 0)
            it.m_word_offset--;
        else
        {
            // need a new word
            --it.m_word_index;

            it.m_word        = it.m_stream[ it.m_word_index ];
            it.m_word_offset = SYMBOLS_PER_WORD - 1u;
        }
    }
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    static Symbol fetch(const forward_stream_type& it)
    {
        const uint32 word_comp = comp( it.m_word, it.m_word_offset / SYMBOLS_PER_SUBWORD );
        const uint32 word_mod  =                  it.m_word_offset & (SYMBOLS_PER_SUBWORD-1);

        const uint32 word_offset = BIG_ENDIAN ? (SUBWORD_SIZE - SYMBOL_SIZE - word_mod * SYMBOL_SIZE) :
                                                                             (word_mod * SYMBOL_SIZE);

        return Symbol( (word_comp >> word_offset) & SYMBOL_MASK );
    }
};

template <typename InputStream, typename Symbol, uint32 SYMBOL_SIZE_T, bool BIG_ENDIAN_T, typename IndexType>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
Symbol ForwardPackedStream<InputStream,Symbol, SYMBOL_SIZE_T, BIG_ENDIAN_T, IndexType>::get() const
{
    return forward_packer<BIG_ENDIAN_T,SYMBOL_SIZE,Symbol,InputStream,IndexType,storage_type>::fetch( *this );
}

// rebase the iterator
//
template <typename InputStream, typename Symbol, uint32 SYMBOL_SIZE_T, bool BIG_ENDIAN_T, typename IndexType>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE 
void ForwardPackedStream<InputStream,Symbol, SYMBOL_SIZE_T, BIG_ENDIAN_T, IndexType>::rebase(void)
{
    forward_packer<BIG_ENDIAN_T,SYMBOL_SIZE_T,Symbol,InputStream,IndexType,storage_type>::rebase(*this);
}

// pre-increment operator
//
template <typename InputStream, typename Symbol, uint32 SYMBOL_SIZE_T, bool BIG_ENDIAN_T, typename IndexType>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
ForwardPackedStream<InputStream,Symbol, SYMBOL_SIZE_T, BIG_ENDIAN_T, IndexType>&
ForwardPackedStream<InputStream,Symbol, SYMBOL_SIZE_T, BIG_ENDIAN_T, IndexType>::operator++ ()
{
    forward_packer<BIG_ENDIAN_T,SYMBOL_SIZE_T,Symbol,InputStream,IndexType,storage_type>::next(*this);
    return *this;
}

// post-increment operator
//
template <typename InputStream, typename Symbol, uint32 SYMBOL_SIZE_T, bool BIG_ENDIAN_T, typename IndexType>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
ForwardPackedStream<InputStream,Symbol, SYMBOL_SIZE_T, BIG_ENDIAN_T, IndexType>
ForwardPackedStream<InputStream,Symbol, SYMBOL_SIZE_T, BIG_ENDIAN_T, IndexType>::operator++ (int dummy)
{
    This r( m_stream, m_index );
    forward_packer<BIG_ENDIAN_T,SYMBOL_SIZE_T,Symbol,InputStream,IndexType,storage_type>::next(*this);
    return r;
}

// pre-decrement operator
//
template <typename InputStream, typename Symbol, uint32 SYMBOL_SIZE_T, bool BIG_ENDIAN_T, typename IndexType>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
ForwardPackedStream<InputStream,Symbol, SYMBOL_SIZE_T, BIG_ENDIAN_T, IndexType>&
ForwardPackedStream<InputStream,Symbol, SYMBOL_SIZE_T, BIG_ENDIAN_T, IndexType>::operator-- ()
{
    forward_packer<BIG_ENDIAN_T,SYMBOL_SIZE_T,Symbol,InputStream,IndexType,storage_type>::prev(*this);
    return *this;
}

// post-decrement operator
//
template <typename InputStream, typename Symbol, uint32 SYMBOL_SIZE_T, bool BIG_ENDIAN_T, typename IndexType>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE 
ForwardPackedStream<InputStream,Symbol, SYMBOL_SIZE_T, BIG_ENDIAN_T, IndexType>
ForwardPackedStream<InputStream,Symbol, SYMBOL_SIZE_T, BIG_ENDIAN_T, IndexType>::operator-- (int dummy)
{
    This r( m_stream, m_index );
    forward_packer<BIG_ENDIAN_T,SYMBOL_SIZE_T,Symbol,InputStream,IndexType,storage_type>::prev(*this);
    return r;
}


// add offset
//
template <typename InputStream, typename Symbol, uint32 SYMBOL_SIZE_T, bool BIG_ENDIAN_T, typename IndexType>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
ForwardPackedStream<InputStream,Symbol, SYMBOL_SIZE_T, BIG_ENDIAN_T, IndexType>&
ForwardPackedStream<InputStream,Symbol, SYMBOL_SIZE_T, BIG_ENDIAN_T, IndexType>::operator+= (const sindex_type distance)
{
    m_index += distance;
    rebase();
    return *this;
}

// subtract offset
//
template <typename InputStream, typename Symbol, uint32 SYMBOL_SIZE_T, bool BIG_ENDIAN_T, typename IndexType>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
ForwardPackedStream<InputStream,Symbol, SYMBOL_SIZE_T, BIG_ENDIAN_T, IndexType>&
ForwardPackedStream<InputStream,Symbol, SYMBOL_SIZE_T, BIG_ENDIAN_T, IndexType>::operator-= (const sindex_type distance)
{
    m_index -= distance;
    rebase();
    return *this;
}

// add offset
//
template <typename InputStream, typename Symbol, uint32 SYMBOL_SIZE_T, bool BIG_ENDIAN_T, typename IndexType>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
ForwardPackedStream<InputStream,Symbol, SYMBOL_SIZE_T, BIG_ENDIAN_T, IndexType>
ForwardPackedStream<InputStream,Symbol, SYMBOL_SIZE_T, BIG_ENDIAN_T, IndexType>::operator+ (const sindex_type distance) const
{
    return This( m_stream, m_index + distance );
}

// subtract offset
//
template <typename InputStream, typename Symbol, uint32 SYMBOL_SIZE_T, bool BIG_ENDIAN_T, typename IndexType>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
ForwardPackedStream<InputStream,Symbol, SYMBOL_SIZE_T, BIG_ENDIAN_T, IndexType>
ForwardPackedStream<InputStream,Symbol, SYMBOL_SIZE_T, BIG_ENDIAN_T, IndexType>::operator- (const sindex_type distance) const
{
    return This( m_stream, m_index - distance );
}

// difference
//
template <typename InputStream, typename Symbol, uint32 SYMBOL_SIZE_T, bool BIG_ENDIAN_T, typename IndexType>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
typename ForwardPackedStream<InputStream,Symbol, SYMBOL_SIZE_T, BIG_ENDIAN_T, IndexType>::sindex_type
ForwardPackedStream<InputStream,Symbol, SYMBOL_SIZE_T, BIG_ENDIAN_T, IndexType>::operator- (const ForwardPackedStream it) const
{
    return sindex_type( m_index - it.m_index );
}

/// less than
///
template <typename InputStream, typename Symbol, uint32 SYMBOL_SIZE_T, bool BIG_ENDIAN_T, typename IndexType>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE bool operator< (
    const ForwardPackedStream<InputStream,Symbol,SYMBOL_SIZE_T,BIG_ENDIAN_T,IndexType>& it1,
    const ForwardPackedStream<InputStream,Symbol,SYMBOL_SIZE_T,BIG_ENDIAN_T,IndexType>& it2)
{
    return it1.index() < it2.index();
}

/// greater than
///
template <typename InputStream, typename Symbol, uint32 SYMBOL_SIZE_T, bool BIG_ENDIAN_T, typename IndexType>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE bool operator> (
    const ForwardPackedStream<InputStream,Symbol,SYMBOL_SIZE_T,BIG_ENDIAN_T,IndexType>& it1,
    const ForwardPackedStream<InputStream,Symbol,SYMBOL_SIZE_T,BIG_ENDIAN_T,IndexType>& it2)
{
    return it1.index() > it2.index();
}

/// equality test
///
template <typename InputStream, typename Symbol, uint32 SYMBOL_SIZE_T, bool BIG_ENDIAN_T, typename IndexType>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE bool operator== (
    const ForwardPackedStream<InputStream,Symbol,SYMBOL_SIZE_T,BIG_ENDIAN_T,IndexType>& it1,
    const ForwardPackedStream<InputStream,Symbol,SYMBOL_SIZE_T,BIG_ENDIAN_T,IndexType>& it2)
{
    return it1.stream() == it2.stream() && it1.index() == it2.index();
}

/// inequality test
///
template <typename InputStream, typename Symbol, uint32 SYMBOL_SIZE_T, bool BIG_ENDIAN_T, typename IndexType>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE bool operator!= (
    const ForwardPackedStream<InputStream,Symbol,SYMBOL_SIZE_T,BIG_ENDIAN_T,IndexType>& it1,
    const ForwardPackedStream<InputStream,Symbol,SYMBOL_SIZE_T,BIG_ENDIAN_T,IndexType>& it2)
{
    return it1.stream() != it2.stream() || it1.index() != it2.index();
}

namespace priv {

// assign a sequence to a packed stream
//
template <typename InputIterator, typename InputStream, typename Symbol, uint32 SYMBOL_SIZE_T, bool BIG_ENDIAN_T, typename IndexType>
NVBIO_HOST_DEVICE
void serial_assign(
    const IndexType                                                                                 input_len,
    InputIterator                                                                                   input_string,
    PackedStream<InputStream,Symbol,SYMBOL_SIZE_T,BIG_ENDIAN_T,IndexType>                           packed_string)
{
    typedef PackedStream<InputStream,Symbol,SYMBOL_SIZE_T,BIG_ENDIAN_T,IndexType> packed_stream_type;
    typedef typename packed_stream_type::storage_type word_type;

    const uint32 WORD_SIZE = uint32( 8u * sizeof(word_type) );

    const bool   BIG_ENDIAN       = BIG_ENDIAN_T;
    const uint32 SYMBOL_SIZE      = SYMBOL_SIZE_T;
    const uint32 SYMBOLS_PER_WORD = WORD_SIZE / SYMBOL_SIZE;
    const uint32 SYMBOL_COUNT     = 1u << SYMBOL_SIZE;
    const uint32 SYMBOL_MASK      = SYMBOL_COUNT - 1u;

    InputStream words = packed_string.stream();

    const IndexType stream_offset = packed_string.index();
    const uint32    word_offset   = stream_offset & (SYMBOLS_PER_WORD-1);
          uint32    word_rem      = 0;

    if (word_offset)
    {
        // compute how many symbols we still need to encode to fill the current word
        word_rem = SYMBOLS_PER_WORD - word_offset;

        // fetch the word in question
        word_type word = words[ stream_offset / SYMBOLS_PER_WORD ];

        // loop through the word's bp's
        for (uint32 i = 0; i < word_rem; ++i)
        {
            // fetch the bp
            const uint8 bp = input_string[i] & SYMBOL_MASK;

            const uint32       bit_idx = (word_offset + i) * SYMBOL_SIZE;
            const uint32 symbol_offset = BIG_ENDIAN ? (WORD_SIZE - SYMBOL_SIZE - bit_idx) : bit_idx;
            const word_type     symbol = word_type(bp) << symbol_offset;

            // clear all bits
            word &= ~(word_type(SYMBOL_MASK) << symbol_offset);

            // set bits
            word |= symbol;
        }

        // write out the word
        words[ stream_offset / SYMBOLS_PER_WORD ] = word;
    }

  #if defined(_OPENMP) && !defined(NVBIO_DEVICE_COMPILATION)
    // we use this solution because the 'if' clause in the 'pragma omp for' results in 30% slowdown
    // when the if is not taken and the loop is executed serially
    if (input_len > 1000000)
    {
        #pragma omp parallel for
        for (int64 i = word_rem; i < int64( input_len ); i += SYMBOLS_PER_WORD)
        {
            // encode a word's worth of characters
            word_type word = 0u;

            const uint32 n_symbols = nvbio::min( SYMBOLS_PER_WORD, uint32( input_len - IndexType(i) ) );

            // loop through the word's bp's
            for (uint32 j = 0; j < SYMBOLS_PER_WORD; ++j)
            {
                if (j < n_symbols)
                {
                    // fetch the bp
                    const uint8 bp = input_string[IndexType(i) + j] & SYMBOL_MASK;

                    const uint32       bit_idx = j * SYMBOL_SIZE;
                    const uint32 symbol_offset = BIG_ENDIAN ? (WORD_SIZE - SYMBOL_SIZE - bit_idx) : bit_idx;
                    const word_type     symbol = word_type(bp) << symbol_offset;

                    // set bits
                    word |= symbol;
                }
            }

            // write out the word
            const uint32 word_idx = uint32( (stream_offset + IndexType(i)) / SYMBOLS_PER_WORD );

            words[ word_idx ] = word;
        }
    }
    else
  #endif
    {
        for (IndexType i = word_rem; i < input_len; i += SYMBOLS_PER_WORD)
        {
            // encode a word's worth of characters
            word_type word = 0u;

            const uint32 n_symbols = nvbio::min( SYMBOLS_PER_WORD, uint32( input_len - IndexType(i) ) );

            // get the offset to the first symbol
            uint32 symbol_offset = BIG_ENDIAN ? WORD_SIZE - SYMBOL_SIZE : 0u;

            // loop through the word's bp's
            for (uint32 j = 0; j < SYMBOLS_PER_WORD; ++j)
            {
                if (j < n_symbols)
                {
                    // fetch the bp
                    const uint8 bp = input_string[IndexType(i) + j] & SYMBOL_MASK;

                    //const uint32       bit_idx = j * SYMBOL_SIZE;
                    //const uint32 symbol_offset = BIG_ENDIAN ? (WORD_SIZE - SYMBOL_SIZE - bit_idx) : bit_idx;
                    const word_type symbol = word_type(bp) << symbol_offset;

                    // set bits
                    word |= symbol;

                    // move the offset
                    if (BIG_ENDIAN) symbol_offset -= SYMBOL_SIZE;
                    else            symbol_offset += SYMBOL_SIZE;
                }
            }

            // write out the word
            const uint32 word_idx = uint32( (stream_offset + IndexType(i)) / SYMBOLS_PER_WORD );

            words[ word_idx ] = word;
        }
    }
}

} // namespace priv

#if defined(__CUDACC__)

namespace priv {

template <typename InputIterator, typename InputStream, typename Symbol, uint32 SYMBOL_SIZE_T, bool BIG_ENDIAN_T, typename IndexType>
__global__
void assign_kernel(
    const IndexType                                                                                 input_len,
    const InputIterator                                                                             input_string,
    PackedStream<InputStream,Symbol,SYMBOL_SIZE_T,BIG_ENDIAN_T,IndexType>                           packed_string)
{
    typedef PackedStream<InputStream,Symbol,SYMBOL_SIZE_T,BIG_ENDIAN_T,IndexType> packed_stream_type;
    typedef typename packed_stream_type::storage_type                             word_type;

    const uint32 WORD_SIZE = uint32( 8u * sizeof(word_type) );

    const bool   BIG_ENDIAN       = BIG_ENDIAN_T;
    const uint32 SYMBOL_SIZE      = SYMBOL_SIZE_T;
    const uint32 SYMBOLS_PER_WORD = WORD_SIZE / SYMBOL_SIZE;
    const uint32 SYMBOL_COUNT     = 1u << SYMBOL_SIZE;
    const uint32 SYMBOL_MASK      = SYMBOL_COUNT - 1u;

    const IndexType stream_offset = packed_string.index();                  // stream offset, in symbols
    const uint32    word_offset   = stream_offset & (SYMBOLS_PER_WORD-1);   // offset within the first word
    const uint32    word_rem      = SYMBOLS_PER_WORD - word_offset;         // # of remaining symbols to fill the first word

    InputStream words = packed_string.stream();

    const uint32 thread_id = threadIdx.x + blockIdx.x * blockDim.x;

    if (thread_id == 0)
    {
        // fetch the word in question
        word_type word = words[ stream_offset / SYMBOLS_PER_WORD ];

        // loop through the word's bp's
        for (uint32 i = 0; i < word_rem; ++i)
        {
            // fetch the bp
            const uint8 bp = input_string[i] & SYMBOL_MASK;

            const uint32       bit_idx = (word_offset + i) * SYMBOL_SIZE;
            const uint32 symbol_offset = BIG_ENDIAN ? (WORD_SIZE - SYMBOL_SIZE - bit_idx) : bit_idx;
            const word_type     symbol = word_type(bp) << symbol_offset;

            // clear all bits
            word &= ~(uint64(SYMBOL_MASK) << symbol_offset);

            // set bits
            word |= symbol;
        }

        // write out the word
        words[ stream_offset / SYMBOLS_PER_WORD ] = word;
    }
    else
    {
        // check whether this thread should do something
        if (word_rem + (thread_id - 1u) * SYMBOLS_PER_WORD >= input_len)
            return;

        const uint32 i = word_rem + (thread_id - 1u) * SYMBOLS_PER_WORD;

        // encode a word's worth of characters
        word_type word = 0u;

        const uint32 n_symbols = nvbio::min( SYMBOLS_PER_WORD, uint32( input_len - IndexType(i) ) );

        // loop through the word's bp's
        for (uint32 j = 0; j < SYMBOLS_PER_WORD; ++j)
        {
            if (j < n_symbols)
            {
                // fetch the bp
                const uint8 bp = input_string[IndexType(i) + j] & SYMBOL_MASK;

                const uint32       bit_idx = j * SYMBOL_SIZE;
                const uint32 symbol_offset = BIG_ENDIAN ? (WORD_SIZE - SYMBOL_SIZE - bit_idx) : bit_idx;
                const word_type     symbol = word_type(bp) << symbol_offset;

                // set bits
                word |= symbol;
            }
        }

        // write out the word
        const uint32 word_idx = uint32( (stream_offset + IndexType(i)) / SYMBOLS_PER_WORD );

        words[ word_idx ] = word;
    }
}

// assign a sequence to a packed stream
// NOTE: this is a host ONLY function - marking it as host/device would cause compiler misbehaviours
//
template <typename InputIterator, typename InputStream, typename Symbol, uint32 SYMBOL_SIZE_T, bool BIG_ENDIAN_T, typename IndexType>
void device_assign(
    const IndexType                                                                                 input_len,
    const InputIterator                                                                             input_string,
    PackedStream<InputStream,Symbol,SYMBOL_SIZE_T,BIG_ENDIAN_T,IndexType>                           packed_string)
{
    if (input_len == 0)
        return;

    typedef PackedStream<InputStream,Symbol,SYMBOL_SIZE_T,BIG_ENDIAN_T,IndexType> packed_stream_type;
    typedef typename packed_stream_type::storage_type                             word_type;

    const uint32 WORD_SIZE = uint32( 8u * sizeof(word_type) );

    const uint32 SYMBOL_SIZE      = SYMBOL_SIZE_T;
    const uint32 SYMBOLS_PER_WORD = WORD_SIZE / SYMBOL_SIZE;

    const IndexType stream_offset = packed_string.index();                  // stream offset, in symbols

    const uint32 word_begin = util::divide_rz( stream_offset,             SYMBOLS_PER_WORD );
    const uint32 word_end   = util::divide_ri( stream_offset + input_len, SYMBOLS_PER_WORD );

    const uint32 n_words = word_end - word_begin;

    const uint32 blockdim = 128u;
    const uint32 n_blocks = util::divide_ri( n_words, blockdim );

    priv::assign_kernel<<<n_blocks,blockdim>>>( input_len, input_string, packed_string );
    cuda::check_error("assign_kernel()");
}

} // namespace priv

// assign a sequence to a packed stream
//
template <typename InputIterator, typename InputStream, typename Symbol, uint32 SYMBOL_SIZE_T, bool BIG_ENDIAN_T, typename IndexType>
NVBIO_HOST_DEVICE
void assign(
    const device_tag                                                                                tag,
    const IndexType                                                                                 input_len,
    const InputIterator                                                                             input_string,
    PackedStream<InputStream,Symbol,SYMBOL_SIZE_T,BIG_ENDIAN_T,IndexType>                           packed_string)
{
  #if !defined(NVBIO_DEVICE_COMPILATION)
    //
    // this function is being called on the host: spawn a kernel
    //

    priv::device_assign( input_len, input_string, packed_string );
  #else
    //
    // this function is being called on the device: call the serial implementation
    //

    priv::serial_assign( input_len, input_string, packed_string );
  #endif
}

#endif // defined(__CUDACC__)

// assign a sequence to a packed stream
//
template <typename InputIterator, typename InputStream, typename Symbol, uint32 SYMBOL_SIZE_T, bool BIG_ENDIAN_T, typename IndexType>
NVBIO_HOST_DEVICE
void assign(
    const host_tag                                                                                  tag,
    const IndexType                                                                                 input_len,
    const InputIterator                                                                             input_string,
    PackedStream<InputStream,Symbol,SYMBOL_SIZE_T,BIG_ENDIAN_T,IndexType>                           packed_string)
{
    priv::serial_assign( input_len, input_string, packed_string );
}

// assign a sequence to a packed stream
//
template <typename InputIterator, typename InputStream, typename Symbol, uint32 SYMBOL_SIZE_T, bool BIG_ENDIAN_T, typename IndexType>
NVBIO_HOST_DEVICE
void assign(
    const IndexType                                                                                 input_len,
    const InputIterator                                                                             input_string,
    PackedStream<InputStream,Symbol,SYMBOL_SIZE_T,BIG_ENDIAN_T,IndexType>                           packed_string)
{
    // find the system tag of the output packed stream
    typedef typename iterator_system<InputStream>::type    system_tag;

    // and chose which function to call based on it
    assign( system_tag(), input_len, input_string, packed_string );
}

//
// A utility function to transpose a set of packed input streams:
//   the symbols of the i-th input stream is supposed to be stored contiguously in the range [offset(i), offset + N(i)]
//   the *words* of i-th output stream will be stored in strided fashion at out_stream[tid, tid + (N(i)+symbols_per_word-1/symbols_per_word) * stride]
//
// \param stride       output stride
// \param N            length of this thread's string in the input stream
// \param in_offset    offset of this thread's string in the input stream
// \param in_stream    input stream
// \param out_stream   output stream
//
template <uint32 BLOCKDIM, uint32 SYMBOL_SIZE, bool BIG_ENDIAN, typename InStreamIterator, typename OutStreamIterator>
NVBIO_HOST_DEVICE
void transpose_packed_streams(const uint32 stride, const uint32 N, const uint32 in_offset, const InStreamIterator in_stream, OutStreamIterator out_stream)
{
    typedef typename std::iterator_traits<InStreamIterator>::value_type word_type;

    const uint32 SYMBOLS_PER_WORD = (sizeof(word_type)*8) / SYMBOL_SIZE;
          uint32 word_offset      = in_offset & (SYMBOLS_PER_WORD-1);
          uint32 begin_word       = in_offset / SYMBOLS_PER_WORD;
          uint32 end_word         = (in_offset + N + SYMBOLS_PER_WORD-1) / SYMBOLS_PER_WORD;

    // write out the output symbols
    const uint32 N_words = (N + SYMBOLS_PER_WORD-1) / SYMBOLS_PER_WORD;
    word_type cur_word = in_stream[begin_word+0];
    for (uint32 w = 0; w < N_words; ++w)
    {
        if (BIG_ENDIAN == false)
        {
            // fill the first part of the output word
            word_type out_word = cur_word >> (word_offset*SYMBOL_SIZE);

            // fetch the next word
            cur_word = begin_word+w+1 < end_word ? in_stream[begin_word+w+1] : 0u;

            // fill the second part of the output word
            if (word_offset)
                out_word |= cur_word << ((SYMBOLS_PER_WORD - word_offset)*SYMBOL_SIZE);

            out_stream[ stride*w ] = out_word;
        }
        else
        {
            // fill the first part of the output word
            word_type out_word = cur_word << (word_offset*SYMBOL_SIZE);

            // fetch the next word
            cur_word = begin_word+w+1 < end_word ? in_stream[begin_word+w+1] : 0u;

            // fill the second part of the output word
            if (word_offset)
                out_word |= cur_word >> ((SYMBOLS_PER_WORD - word_offset)*SYMBOL_SIZE);

            out_stream[ stride*w ] = out_word;
        }
    }
}

} // namespace nvbio
