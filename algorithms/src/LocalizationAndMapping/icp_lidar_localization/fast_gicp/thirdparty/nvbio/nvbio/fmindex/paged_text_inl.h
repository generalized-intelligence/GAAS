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

namespace nvbio {

uint32 popc_2bit(const uint64* base, const uint32 n, const uint32 mod, const uint32 c);

/// A left shift functor
///
template <typename word_type>
struct divide_ri_functor
{
    typedef word_type argument_type;
    typedef word_type result_type;

    /// constructor
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    divide_ri_functor(const word_type _k) : k(_k) {}

    /// functor operator
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    result_type operator() (const argument_type i) const
    {
        // divide i by k
        return util::divide_ri( i, k );
    }

    const word_type k;
};

template <typename input_storage, typename output_storage, uint32 SYMBOL_SIZE, typename index_type>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
void copy(
    const uint32                                                         len,
    const PackedStream<input_storage,uint8,SYMBOL_SIZE,true,index_type>  in,
          PackedStream<output_storage,uint8,SYMBOL_SIZE,true,index_type> out)
{
    typedef typename std::iterator_traits<input_storage>::value_type word_type;

    const uint32 WORD_SIZE        = 8u * sizeof(word_type);
    const uint32 SYMBOLS_PER_WORD = WORD_SIZE / SYMBOL_SIZE;

    input_storage  in_words  = in.stream();
    output_storage out_words = out.stream();

    const index_type k_start = in.index();
    const index_type n_start = out.index();

    index_type k = k_start;
    index_type n = n_start;

    const index_type k_end = k + len;

    const index_type out_word_begin = util::divide_ri( k,     SYMBOLS_PER_WORD );
    const index_type out_word_end   = util::divide_rz( k_end, SYMBOLS_PER_WORD );

    // check whether the whole segment is contained in one word
    if (out_word_end <= out_word_begin)
    {
        while (k < k_end)
            out[k++ - k_start] = in[n++ - n_start];

        return;
    }

    // align k to a word boundary
    while (k < out_word_begin*SYMBOLS_PER_WORD)
        out[k++ - k_start] = in[n++ - n_start];

    for (index_type out_word = out_word_begin; out_word < out_word_end; ++out_word)
    {
        // fetch a word's worth of input, starting from n
        const uint32 n_word = n / SYMBOLS_PER_WORD;
        const uint32 n_mod  = n & (SYMBOLS_PER_WORD-1);
        const uint32 n_syms = SYMBOLS_PER_WORD - n_mod;

        // fetch the last 'n_syms' symbols of the first word
        word_type in_word = in_words[n_word] << (n_mod * SYMBOL_SIZE);

        if (n_syms < SYMBOLS_PER_WORD)
        {
            // fetch the remaining symbols from the next word (deleting the first 'n_syms)
            in_word |= (in_words[n_word+1] >> (n_syms * SYMBOL_SIZE));
        }

        // ...and write it out
        out_words[ out_word ] = in_word;

        // go forward
        k += SYMBOLS_PER_WORD;
        n += SYMBOLS_PER_WORD;
    }

    // finish copying leftovers
    while (k < k_end)
        out[k++ - k_start] = in[n++ - n_start];
}

// save occurrence counters if needed
template <uint32 SYMBOL_COUNT>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
void save_occurrences(const uint32 k, const uint32 occ_intv_log, const uint32 occ_intv, const uint32* partials, uint32* occ)
{
    // check whether we need to save the occurrence counters
    if ((k & (occ_intv-1)) == 0)
    {
        const uint32 block_idx = k >> occ_intv_log;
        for (uint32 q = 0; q < SYMBOL_COUNT; ++q)
            occ[ SYMBOL_COUNT * block_idx + q ] = partials[q];
    }
}

template <typename input_storage, typename output_storage, uint32 SYMBOL_SIZE, typename index_type>
NVBIO_FORCEINLINE NVBIO_HOST/*_DEVICE*/
void copy(
    const uint32                                                         len,
    const PackedStream<input_storage,uint8,SYMBOL_SIZE,true,index_type>  in,
          PackedStream<output_storage,uint8,SYMBOL_SIZE,true,index_type> out,
    const uint32                                                         occ_intv_log,
    const uint32                                                         occ_intv,
          uint32*                                                        partials,
          uint32*                                                        occ,
    const uint32*                                                        count_table)
{
    typedef typename std::iterator_traits<input_storage>::value_type word_type;

    const uint32 WORD_SIZE        = 8u * sizeof(word_type);
    const uint32 SYMBOLS_PER_WORD = WORD_SIZE / SYMBOL_SIZE;
    const uint32 SYMBOL_COUNT     = 1u << SYMBOL_SIZE;

    input_storage  in_words  = in.stream();
    output_storage out_words = out.stream();

    const index_type k_start = out.index();
    const index_type n_start = in.index();

    index_type k = k_start;
    index_type n = n_start;

    const index_type k_end = k + len;

    const index_type out_word_begin = util::divide_ri( k,     SYMBOLS_PER_WORD );
    const index_type out_word_end   = util::divide_rz( k_end, SYMBOLS_PER_WORD );

    // check whether the whole segment is contained in one word
    if (out_word_end <= out_word_begin)
    {
        while (k < k_end)
        {
            // check whether we need to save the occurrence counters
            save_occurrences<SYMBOL_COUNT>( k, occ_intv_log, occ_intv, partials, occ );

            const uint8 c = in[n++ - n_start];

            out[k++ - k_start] = c;

            ++partials[c];
        }
        return;
    }

    PackedStream<output_storage,uint8,SYMBOL_SIZE,true,index_type> out_base( out.stream() );

    // align k to a word boundary
    while (k < out_word_begin*SYMBOLS_PER_WORD)
    {
        // check whether we need to save the occurrence counters
        save_occurrences<SYMBOL_COUNT>( k, occ_intv_log, occ_intv, partials, occ );

        const uint8 c = in[n++ - n_start];

        out[k++ - k_start] = c;

        ++partials[c];
    }

    for (index_type out_word = out_word_begin; out_word < out_word_end; ++out_word)
    {
        // fetch a word's worth of input, starting from n
        const uint32 n_word = n / SYMBOLS_PER_WORD;
        const uint32 n_mod  = n & (SYMBOLS_PER_WORD-1);
        const uint32 n_syms = SYMBOLS_PER_WORD - n_mod;

        // fetch the last 'n_syms' symbols of the first word
        word_type in_word = in_words[n_word] << (n_mod * SYMBOL_SIZE);

        if (n_syms < SYMBOLS_PER_WORD)
        {
            // fetch the remaining symbols from the next word (deleting the first 'n_syms)
            in_word |= (in_words[n_word+1] >> (n_syms * SYMBOL_SIZE));
        }

        // check whether we need to save the occurrence counters
        save_occurrences<SYMBOL_COUNT>( out_word * SYMBOLS_PER_WORD, occ_intv_log, occ_intv, partials, occ );

        if (SYMBOL_SIZE == 2)
        {
            const uint32 cnts = popc_2bit_all( in_word, count_table );

            partials[0] += (cnts >>  0) & 0xFF;
            partials[1] += (cnts >>  8) & 0xFF;
            partials[2] += (cnts >> 16) & 0xFF;
            partials[3] += (cnts >> 24) & 0xFF;
        }
        else
        {
            // loop through the symbols one by one
            for (uint32 i = 0; i < SYMBOLS_PER_WORD; ++i)
            {
                const uint8 c = (in_word >> (WORD_SIZE - SYMBOL_SIZE - i * SYMBOL_SIZE)) & (SYMBOL_COUNT-1);
                ++partials[c];
            }
            // TODO: use a generalized count-table
        }

        // ...and write it out
        out_words[ out_word ] = in_word;

        // go forward
        k += SYMBOLS_PER_WORD;
        n += SYMBOLS_PER_WORD;
    }

    // finish copying leftovers
    while (k < k_end)
    {
        // check whether we need to save the occurrence counters
        save_occurrences<SYMBOL_COUNT>( k, occ_intv_log, occ_intv, partials, occ );

        const uint8 c = in[n++ - n_start];

        out[k++ - k_start] = c;

        ++partials[c];
    }
}

// constructor
//
template <uint32 SYMBOL_SIZE_T, bool BIG_ENDIAN_T>
PagedText<SYMBOL_SIZE_T,BIG_ENDIAN_T>::PagedText(
    const uint32 page_size,
    const uint32 segment_size,
    const uint32 occ_intv) :
    m_page_size( page_size / sizeof(word_type) ),
    m_segment_size( segment_size / sizeof(word_type) ),
    m_occ_intv( occ_intv ),
    m_occ_intv_w( occ_intv / SYMBOLS_PER_WORD ),
    m_occ_intv_log( nvbio::log2( occ_intv ) ),
    m_page_count( 0 ),
    m_pool_size( 0 )
{
    omp_init_lock( &m_lock );

    gen_2bit_count_table( m_count_table );
}

// destructor
//
template <uint32 SYMBOL_SIZE_T, bool BIG_ENDIAN_T>
PagedText<SYMBOL_SIZE_T,BIG_ENDIAN_T>::~PagedText()
{
    for (uint32 i = 0; i < m_segments.size(); ++i)
        free( m_segments[i] );
}

// alloc new pages
//
template <uint32 SYMBOL_SIZE_T, bool BIG_ENDIAN_T>
void PagedText<SYMBOL_SIZE_T,BIG_ENDIAN_T>::grow()
{
    const uint32 occ_freq = (sizeof(word_type) / sizeof(uint32)) *
                            (m_occ_intv / SYMBOLS_PER_WORD) / SYMBOL_COUNT;

    const uint32 n_pages          = m_segment_size / m_page_size;
    const uint32 ext_page_size    = m_page_size + m_page_size / occ_freq;
    const uint32 ext_segment_size = n_pages * ext_page_size;

    word_type* segment = (word_type*)malloc( ext_segment_size * sizeof(word_type) );

    if (segment == NULL)
    {
        log_error(stderr, "PagedText: failed allocating segment\n");
        //throw bad_alloc( "PagedText: failed allocating segment\n" );
        exit(1);
    }
    else
    {
        // the free page pool has to be able to accomodate all the allocated pages
        if (m_pool.size() < m_page_count + n_pages)
            m_pool.resize( m_page_count + n_pages );

        m_segments.push_back( segment );

        for (uint32 i = 0; i < n_pages; ++i)
            m_pool[ m_pool_size++ ] = segment + ext_page_size * (n_pages - i - 1u);

        m_page_count += n_pages;
    }
}

// alloc a new page
//
template <uint32 SYMBOL_SIZE_T, bool BIG_ENDIAN_T>
typename PagedText<SYMBOL_SIZE_T,BIG_ENDIAN_T>::word_type*
PagedText<SYMBOL_SIZE_T,BIG_ENDIAN_T>::alloc_page()
{
    omp_set_lock( &m_lock );

    if (m_pool_size == 0)
    {
        omp_unset_lock( &m_lock );

        log_error(stderr, "PagedText: exhausted page pool\n");
        //throw bad_alloc( "PagedText: exhausted page pool\n" );
        exit(1);
    }

    word_type* page = m_pool[ --m_pool_size ];
    assert( page != NULL );

    omp_unset_lock( &m_lock );
    return page;
}

// release a page
//
template <uint32 SYMBOL_SIZE_T, bool BIG_ENDIAN_T>
void PagedText<SYMBOL_SIZE_T,BIG_ENDIAN_T>::release_page(word_type* page)
{
    assert( page != NULL );
    omp_set_lock( &m_lock );

    if (m_pool_size >= m_page_count)
    {
        log_error(stderr, "exceeded pool size %u - released more pages than have been allocated\n", m_page_count);
        exit(1);
    }

    m_pool[ m_pool_size++ ] = page;

    omp_unset_lock( &m_lock );
}

// indexing operator - return the i-th symbol
//
template <uint32 SYMBOL_SIZE_T, bool BIG_ENDIAN_T>
uint8 PagedText<SYMBOL_SIZE_T,BIG_ENDIAN_T>::operator[] (const uint64 i) const
{
    // fetch the page containing index 'i'
    const uint32     page_idx = find_page( i );
    const word_type* page = get_page( page_idx );

    const uint32 local_i = uint32( i - m_offsets[ page_idx ] );
    assert( local_i < m_page_size * SYMBOLS_PER_WORD );

    const const_packed_page_type packed_page( page );
    return packed_page[ local_i ];
}

// compute the rank of c in [0,i]
//
template <uint32 SYMBOL_SIZE_T, bool BIG_ENDIAN_T>
uint64 PagedText<SYMBOL_SIZE_T,BIG_ENDIAN_T>::rank(const uint64 i, const uint8 c) const
{
    if (i == uint64(-1))
        return 0u;

    if (i >= size())
        return symbol_frequency(c);

    // fetch the page containing index 'i'
    const uint32 page_idx = find_page( i );

    return rank( page_idx, i, c );
}

// compute the rank of c in [0,i]
//
template <uint32 SYMBOL_SIZE_T, bool BIG_ENDIAN_T>
uint64 PagedText<SYMBOL_SIZE_T,BIG_ENDIAN_T>::rank(const uint32 page_idx, const uint64 i, const uint8 c) const
{
    if (i == uint64(-1))
        return 0u;

    if (i >= size())
        return symbol_frequency(c);

    // fetch the page containing index 'i'
    const word_type* page = get_page( page_idx );
    const uint32*    occ  = (uint32*)( page + m_page_size );

    const uint32 local_i = uint32( i - m_offsets[ page_idx ] );
    assert( local_i < m_page_size * SYMBOLS_PER_WORD );

    // compute the index of the occurrence block containing 'i', and the offset within it
    const uint32 block_idx    = local_i >> m_occ_intv_log;
    const uint32 block_offset = local_i & (m_occ_intv-1);

    // fetch the base occurrence counters for the page and block
    uint64 out =
        m_counters[ SYMBOL_COUNT * page_idx + c ] +
        occ[ SYMBOL_COUNT * block_idx + c ];

    // compute the index of the word containing 'i', and the corresponding modulo
    const uint32 word_idx = block_offset / SYMBOLS_PER_WORD;
    const uint32 word_mod = ~word_type(local_i) & (SYMBOLS_PER_WORD-1);

    const uint32 word_begin = block_idx*m_occ_intv_w;

    return out + popc_2bit( page + word_begin, word_idx, word_mod, c );
}

// reserve
//
template <uint32 SYMBOL_SIZE_T, bool BIG_ENDIAN_T>
void PagedText<SYMBOL_SIZE_T,BIG_ENDIAN_T>::reserve_pages(const uint32 n_pages)
{
    // alloc the pages we need
    m_pool.reserve( n_pages );
    while (m_page_count < n_pages)
        grow();

    m_pages.reserve( n_pages );
    m_new_pages.reserve( n_pages );
    m_counters.reserve( (n_pages+1) * SYMBOL_COUNT );
    m_new_counters.reserve( (n_pages+1) * SYMBOL_COUNT );
    m_offsets.reserve( n_pages + 1 );
    m_new_offsets.reserve( n_pages + 1 );
}

// reserve free pages
//
template <uint32 SYMBOL_SIZE_T, bool BIG_ENDIAN_T>
void PagedText<SYMBOL_SIZE_T,BIG_ENDIAN_T>::reserve_free_pages(const uint32 n_pages)
{
    // alloc the pages we need
    m_pool.reserve( n_pages );
    while (m_pool_size < n_pages)
        grow();
}

// reserve
//
template <uint32 SYMBOL_SIZE_T, bool BIG_ENDIAN_T>
void PagedText<SYMBOL_SIZE_T,BIG_ENDIAN_T>::reserve(const uint64 n)
{
    reserve_pages( (uint32)util::divide_ri( n, (m_page_size * SYMBOLS_PER_WORD * 2)/3 ) );
}

// needed host memory
//
template <uint32 SYMBOL_SIZE_T, bool BIG_ENDIAN_T>
uint64 PagedText<SYMBOL_SIZE_T,BIG_ENDIAN_T>::needed_host_memory(const uint64 n) const
{
    const uint32 occ_freq = (sizeof(word_type) / sizeof(uint32)) *
                            (m_occ_intv / SYMBOLS_PER_WORD) / SYMBOL_COUNT;

    const uint32 ext_page_size = m_page_size + m_page_size / occ_freq;

    const uint64 n_pages = util::divide_ri( n, (m_page_size * SYMBOLS_PER_WORD * 2)/3 );
    return n_pages * ext_page_size * sizeof(word_type);
}

// resize and copy a given vector
//
template <uint32 SYMBOL_SIZE_T, bool BIG_ENDIAN_T>
void PagedText<SYMBOL_SIZE_T,BIG_ENDIAN_T>::resize(const uint64 n, const uint8* c)
{
    const uint32 PAGE_SYMBOLS = m_page_size * SYMBOLS_PER_WORD;

    // alloc the given number of pages
    const uint32 n_pages = util::divide_ri( n, PAGE_SYMBOLS );

    reserve_pages( n_pages );

    // alloc the pages we need
    m_pages.resize( n_pages );
    for (uint32 i = 0; i < n_pages; ++i)
        m_pages[i] = alloc_page();

    // setup the page offsets
    m_offsets.resize( n_pages + 1 );

    #pragma omp parallel for
    for (int32 i = 0; i < int32(n_pages); ++i)
        m_offsets[i] = uint64(i) * PAGE_SYMBOLS;

    m_offsets[ n_pages ] = n;

    // setup the symbol counters
    m_counters.resize( (n_pages+1) * SYMBOL_COUNT, uint64(0) );

    if (c != NULL)
    {
        #pragma omp parallel for
        for (int32 i = 0; i < int32(n_pages); ++i)
        {
            const uint64 begin = uint64(i) * PAGE_SYMBOLS;
            const uint64 end   = nvbio::min( n, begin + PAGE_SYMBOLS );

            // get a new page
            word_type* page_storage = m_pages[i];

            packed_page_type page( page_storage );

            // fill the page contents
            nvbio::assign( uint32( end - begin ), c + begin, page );

            // update the occurrence counters
            uint64* cnts = &m_counters[ i * SYMBOL_COUNT ];
            uint32* occ  = (uint32*)( page_storage + m_page_size );

            for (uint32 j = 0; j < uint32( end - begin ); ++j)
            {
                // check whether we need to the save the occurrence counters
                if ((j & (m_occ_intv-1)) == 0)
                {
                    for (uint32 q = 0; q < SYMBOL_COUNT; ++q)
                        occ[q] = cnts[q];

                    occ += SYMBOL_COUNT;
                }

                const uint8 cc = c[ begin + j ] & (SYMBOL_COUNT-1);
                ++cnts[ cc ];
            }
        }
    }

    // do an exclusive prefix-sum on the occurrence counters
    nvbio::vector<host_tag,uint8> temp_storage;
    for (uint32 j = 0; j < SYMBOL_COUNT; ++j)
    {
        exclusive_scan(
            n_pages+1,
            make_strided_iterator( &m_counters[j], SYMBOL_COUNT ),
            make_strided_iterator( &m_counters[j], SYMBOL_COUNT ),
            thrust::plus<uint64>(),
            uint64(0),
            temp_storage );
    }

    // do an error check on the occurrence counters
    const uint64* cnts = symbol_frequencies();
    uint64 n_occ = 0;
    for (uint32 j = 0; j < SYMBOL_COUNT; ++j)
        n_occ += cnts[j];

    if (n_occ != n)
    {
        log_error(stderr, "mismatching occurrence counters: expected %llu symbols, got %llu\n", n, n_occ );
        //throw runtime_error( "mismatching occurrence counters: expected %llu symbols, got %llu\n", n, n_occ );
        exit(1);
    }

    build_buckets( m_offsets.back(), (uint32)m_offsets.size(), &m_offsets[0], BUCKET_SIZE, m_buckets );
}


template <uint32 SYMBOL_SIZE, bool BIG_ENDIAN>
struct copy_insert_pages
{
    static const uint32 SYMBOL_COUNT = 1u << SYMBOL_SIZE;

    typedef PagedText<SYMBOL_SIZE,BIG_ENDIAN>                 paged_text_type;
    typedef typename paged_text_type::word_type               word_type;
    typedef typename paged_text_type::const_packed_page_type  const_packed_page_type;
    typedef typename paged_text_type::packed_page_type              packed_page_type;

    copy_insert_pages(
        const uint32        _N,
        const uint32        _in_leaves,
        const uint32*       _leaf_ids,
        const uint64*       _g,
        const uint8*        _c,
        paged_text_type*    _text) :
        N           ( _N ),
        in_leaves   ( _in_leaves ),
        leaf_ids    ( _leaf_ids ),
        g           ( _g ),
        c           ( _c ),
        text        ( _text )
    {}

    NVBIO_FORCEINLINE
    void alloc_page(
        const uint32        k_out,
        uint32*             partials,
        uint32*             out_leaf,
        word_type**         out_page,
        packed_page_type*   out_stream,
        uint32**            occ) const
    {
        // write out the old partials
        for (uint32 q = 0; q < SYMBOL_COUNT; ++q)
            text->m_new_counters[ SYMBOL_COUNT * *out_leaf + q ] = partials[q];

        // do a tiny error check
        {
            const uint32 cnt = std::accumulate( partials, partials + SYMBOL_COUNT, 0u );

            if (cnt != k_out)
            {
                log_error(stderr, "alloc_page(%u) : expected %u occurrences, got %u\n", *out_leaf, k_out, cnt);
                //throw runtime_error( "alloc_page(%u) : expected %u occurrences, got %u\n", *out_leaf, k_out, cnt );
                exit(1);
            }
        }

        // and reset them
        for (uint32 q = 0; q < SYMBOL_COUNT; ++q)
            partials[q] = 0;

        // alloc a new page
        (*out_leaf)++;
        *out_page   = text->alloc_page();
        *out_stream = packed_page_type( *out_page );
        *occ        = (uint32*)( *out_page + text->m_page_size );

        // write out the new leaf offset
        text->m_new_offsets[ *out_leaf ] = text->m_new_offsets[ *out_leaf-1 ] + k_out;

        // save the new page
        text->m_new_pages[ *out_leaf ] = *out_page;
    }

    void operator() (const uint32 in_leaf) const
    {
        NVBIO_VAR_UNUSED const uint32 LEAF_SYMBOLS = text->m_page_size * paged_text_type::SYMBOLS_PER_WORD;

              uint32 out_leaf       = leaf_ids[ in_leaf ];
        const uint32 out_leaf_begin = leaf_ids[ in_leaf ];
        const uint32 out_leaf_end   = leaf_ids[ in_leaf+1u ];
        const uint64 in_leaf_begin  = text->m_offsets[ in_leaf ];
        const uint64 in_leaf_end    = text->m_offsets[ in_leaf + 1u ];
        const uint32 in_leaf_size   = in_leaf_end - in_leaf_begin;

        const uint32 g_begin = lower_bound_index( in_leaf_begin, g, N );
        const uint32 g_end   = in_leaf < in_leaves-1 ?
                               lower_bound_index( in_leaf_end,   g, N ) : N;

        if (g_begin == g_end)
        {
            //
            // special case: this leaf got no insertions and doesn't need to be copied
            //

            word_type* in_page = text->m_pages[ in_leaf ];

            // write out the new leaf offset
            text->m_new_offsets[ out_leaf ] = in_leaf_begin + g_begin;

            // save the new page
            text->m_new_pages[ out_leaf ] = in_page;

            // write out the new counters
            for (uint32 q = 0; q < SYMBOL_COUNT; ++q)
            {
                text->m_new_counters[ SYMBOL_COUNT * out_leaf + q ] =
                    text->m_counters[ SYMBOL_COUNT * (in_leaf+1u) + q ] -
                    text->m_counters[ SYMBOL_COUNT * (in_leaf+0u) + q ];
            }
            return;
        }

        word_type* in_page  = text->m_pages[ in_leaf ];
        word_type* out_page = text->alloc_page();
        uint32*    occ      = (uint32*)( out_page + text->m_page_size );

        const_packed_page_type  in_stream( in_page );
              packed_page_type out_stream( out_page );

        // compute the maximum number of elements we'll place in each page
        const uint32 elements_per_page = util::divide_ri( in_leaf_size + g_end - g_begin, out_leaf_end - out_leaf_begin );

        // write out the new leaf offset
        text->m_new_offsets[ out_leaf ] = in_leaf_begin + g_begin;

        // save the new page
        text->m_new_pages[ out_leaf ] = out_page;

        uint32 k_in  = 0u;
        uint32 k_out = 0u;

        uint32 partials[SYMBOL_COUNT];
        for (uint32 q = 0; q < SYMBOL_COUNT; ++q)
            partials[q] = 0;

        for (uint32 j = g_begin; j < g_end; ++j)
        {
            // fetch the next insertion
            const uint32 g_pos = uint32( g[j] - in_leaf_begin );
            const uint8  cc    = c[j] & (SYMBOL_COUNT-1);
            //assert( g[j] >= in_leaf_begin && g[j] < in_leaf_end );

            if (g_pos - k_in)
            {
                const uint32 m = nvbio::min( g_pos, in_leaf_size ) - k_in;
                assert( m <= LEAF_SYMBOLS );

                // the current page can hold only the first 'r' symbols
                const uint32 r = nvbio::min( elements_per_page - k_out, m );

                copy( r, in_stream + k_in, out_stream + k_out, text->m_occ_intv_log, text->m_occ_intv, partials, occ, text->m_count_table );

                k_out += r;

                // check if we need a new page
                if (r < m)
                {
                    alloc_page(
                        k_out,
                        partials,
                        &out_leaf,
                        &out_page,
                        &out_stream,
                        &occ );

                    // copy the remaining symbols
                    copy( m - r, in_stream + k_in + r, out_stream, text->m_occ_intv_log, text->m_occ_intv, partials, occ, text->m_count_table );

                    k_out = m - r;
                }

                k_in = g_pos;
            }

            if (k_out < elements_per_page)
            {
                // save current occurrence counters
                save_occurrences<SYMBOL_COUNT>( k_out, text->m_occ_intv_log, text->m_occ_intv, partials, occ );

                out_stream[ k_out++ ] = cc;
            }
            else
            {
                alloc_page(
                    k_out,
                    partials,
                    &out_leaf,
                    &out_page,
                    &out_stream,
                    &occ );

                k_out = 0u;

                // save current occurrence counters
                save_occurrences<SYMBOL_COUNT>( k_out, text->m_occ_intv_log, text->m_occ_intv, partials, occ );

                out_stream[ k_out++ ] = cc;
            }
            // update partial occurrence counters
            ++partials[ cc ];
        }

        if (in_leaf_size > k_in)
        {
            const uint32 m = in_leaf_size - k_in;
            assert( m <= LEAF_SYMBOLS );

            // the current page can hold only the first 'r' symbols
            const uint32 r = nvbio::min( elements_per_page - k_out, m );

            copy( r, in_stream + k_in, out_stream + k_out, text->m_occ_intv_log, text->m_occ_intv, partials, occ, text->m_count_table );

            k_out += r;

            // check if we need a new page
            if (r < m)
            {
                alloc_page(
                    k_out,
                    partials,
                    &out_leaf,
                    &out_page,
                    &out_stream,
                    &occ );

                // copy the remaining symbols
                copy( m - r, in_stream + k_in + r, out_stream, text->m_occ_intv_log, text->m_occ_intv, partials, occ, text->m_count_table );
            }
        }

        // write out the final partials
        for (uint32 q = 0; q < SYMBOL_COUNT; ++q)
            text->m_new_counters[ SYMBOL_COUNT * out_leaf + q ] = partials[q];

        // release the input page
        text->release_page( in_page );

        if (out_leaf+1 != out_leaf_end)
        {
            log_error(stderr, "mismatching number of output leaves: leaf[%u/%u] : expected %u, got %u\n",
                in_leaf, in_leaves,
                out_leaf_end - out_leaf_begin,
                out_leaf     - out_leaf_begin);
            log_error(stderr, "  in-size    : %u\n", in_leaf_size);
            log_error(stderr, "  insertions : %u\n", uint32( g_end - g_begin ));

            //throw runtime_error( "mismatching number of output leaves: leaf[%u/%u] : expected %u, got %u\n",
            //    in_leaf, in_leaves,
            //    out_leaf_end - out_leaf_begin,
            //    out_leaf     - out_leaf_begin );
            exit(1);
        }
    }

    const uint32        N;
    const uint32        in_leaves;
    const uint32*       leaf_ids;
    const uint64*       g;
    const uint8*        c;
    paged_text_type*    text;
};

template <uint32 SYMBOL_SIZE, bool BIG_ENDIAN>
struct copy_merge_pages
{
    static const uint32 SYMBOL_COUNT = 1u << SYMBOL_SIZE;

    typedef PagedText<SYMBOL_SIZE,BIG_ENDIAN>                 paged_text_type;
    typedef typename paged_text_type::word_type               word_type;
    typedef typename paged_text_type::const_packed_page_type  const_packed_page_type;
    typedef typename paged_text_type::packed_page_type              packed_page_type;

    copy_merge_pages(
        const uint64        _N,
        const uint32        _out_leaves,
        const uint32        _in_leaves,
        paged_text_type*    _text) :
        N           ( _N ),
        out_leaves  ( _out_leaves ),
        in_leaves   ( _in_leaves ),
        text        ( _text )
    {}

    void operator() (const uint32 out_leaf) const
    {
        NVBIO_VAR_UNUSED const uint32 LEAF_SYMBOLS = text->m_page_size * paged_text_type::SYMBOLS_PER_WORD;

        const uint64 out_leaf_begin = uint64( out_leaf ) * LEAF_SYMBOLS;
        const uint64 out_leaf_end   = nvbio::min( uint64( out_leaf + 1u ) * LEAF_SYMBOLS, N );
        const uint32 out_leaf_size  = uint32( out_leaf_end - out_leaf_begin );

        // alloc a new page
        word_type*       out_page = text->alloc_page();
        packed_page_type out_stream( out_page );
        uint32*          occ      = (uint32*)( out_page + text->m_page_size );

        // write out the new leaf offset
        text->m_new_offsets[ out_leaf ] = out_leaf_begin;

        // save the new page
        text->m_new_pages[ out_leaf ] = out_page;

        uint32 partials[SYMBOL_COUNT];
        for (uint32 q = 0; q < SYMBOL_COUNT; ++q)
            partials[q] = 0;

        uint32 k_out = 0;

        uint32 in_leaf = upper_bound_index( out_leaf_begin, raw_pointer( text->m_offsets ), in_leaves+1 ) - 1u;

        for (; k_out < out_leaf_size && in_leaf < in_leaves; ++in_leaf)
        {
            const uint64 in_leaf_begin = text->m_offsets[ in_leaf ];
            const uint64 in_leaf_end   = text->m_offsets[ in_leaf+1 ];
            const uint32 in_leaf_size  = uint32( in_leaf_end - in_leaf_begin );

            const word_type* in_page = text->m_pages[ in_leaf ];
            const const_packed_page_type in_stream( in_page );
            assert( in_page != NULL );

            const uint32 k_in = in_leaf_begin >= out_leaf_begin ? 0u : uint32( out_leaf_begin - in_leaf_begin );
            const uint32 r    = nvbio::min( out_leaf_size - k_out, in_leaf_size - k_in );

            copy( r, in_stream + k_in, out_stream + k_out, text->m_occ_intv_log, text->m_occ_intv, partials, occ, text->m_count_table );

            k_out += r;
        }

        // write out the final partials
        for (uint32 q = 0; q < SYMBOL_COUNT; ++q)
            text->m_new_counters[ SYMBOL_COUNT * out_leaf + q ] = partials[q];

        const uint32 cnt = std::accumulate( partials, partials + SYMBOL_COUNT, 0u );
        if (cnt != k_out)
        {
            log_error(stderr, "merge_pages(%u) : expected %u occurrences, got %u\n", out_leaf, k_out, cnt);
            //throw runtime_error( "merge_pages(%u) : expected %u occurrences, got %u\n", out_leaf, k_out, cnt );
            exit(1);
        }
    }

    const uint64        N;
    const uint32        out_leaves;
    const uint32        in_leaves;
    paged_text_type*    text;
};

// perform a batch of parallel insertions
//
template <uint32 SYMBOL_SIZE_T, bool BIG_ENDIAN_T>
inline void PagedText<SYMBOL_SIZE_T,BIG_ENDIAN_T>::insert(const uint32 n, const uint64* g, const uint8* c)
{
    const uint32 LEAF_SYMBOLS = m_page_size * SYMBOLS_PER_WORD;

    const uint32 n_leaves = m_offsets.size() - 1u;

    Timer timer;
    timer.start();

    nvbio::vector<host_tag,uint32> leaf_sizes( n_leaves + 1u );
    nvbio::vector<host_tag,uint32> new_leaf_ids( n_leaves + 1u );
    nvbio::vector<host_tag,uint8>  temp_storage;

    timer.stop();
    NVBIO_VAR_UNUSED const float t0 = timer.seconds();

    timer.start();

    // compute the original leaf sizes
    thrust::adjacent_difference(
        m_offsets.begin(),
        m_offsets.begin() + n_leaves + 1u,
        leaf_sizes.begin() );

    // extend the last leaf to infinity
    const uint64 old_size = m_offsets.back();
    m_offsets.back() = uint64(-1);

    if (n < n_leaves)
    {
        nvbio::vector<host_tag,uint32> g_leaves( n );
        nvbio::vector<host_tag,uint32> ins_leaves( n );
        nvbio::vector<host_tag,uint32> ins_counts( n );
        
        // find how many elements of g fall in each leaf
        nvbio::upper_bound<host_tag>(
            n,
            g,
            n_leaves + 1u,
            m_offsets.begin(),
            g_leaves.begin() );

        // compute which leaves need splitting, and how much
        const uint32 n_touched = nvbio::runlength_encode(
            n,
            g_leaves.begin(),
            ins_leaves.begin(),
            ins_counts.begin(),
            temp_storage );

        log_debug(stderr, "    touched leaves %u, (%.2f%% - %.1fMB)\n", n_touched, 100.0f * float(n_touched) / float(n_leaves), float(n_touched)*(m_page_size*sizeof(word_type)) / float(1024*1024));

        // for each leaf l = h_ins_leaves[i], do h_leaf_sizes[l] += h_ins_counts[i]
        nvbio::transform<host_tag>(
            n_touched,
            thrust::make_permutation_iterator( leaf_sizes.begin(), ins_leaves.begin() ),
            ins_counts.begin(),
            thrust::make_permutation_iterator( leaf_sizes.begin(), ins_leaves.begin() ),
            thrust::plus<uint32>() );
    }
    else
    {
        nvbio::vector<host_tag,uint32> g_leaves( n_leaves + 1u );
        nvbio::vector<host_tag,uint32> ins_counts( n_leaves + 1u );
        
        // for each leaf, find how many elements of g fall inside it
        nvbio::lower_bound<host_tag>(
            n_leaves + 1u,
            m_offsets.begin(),
            n,
            g,
            g_leaves.begin() );

        // make sure that the last leaf includes all elements in g greater than the current size
        g_leaves[ n_leaves ] = n;

        // compute the number of insertions in each leaf
        thrust::adjacent_difference(
            g_leaves.begin(),
            g_leaves.begin() + n_leaves + 1u,
            ins_counts.begin() );

        // for each leaf do h_leaf_sizes[i] += h_ins_counts[i]
        nvbio::transform<host_tag>(
            n_leaves + 1u,
            leaf_sizes.begin(),
            ins_counts.begin(),
            leaf_sizes.begin(),
            thrust::plus<uint32>() );

        /*
        const uint32 n_split = nvbio::reduce(
            n_leaves,
            make_cast_iterator<uint32>(
                thrust::make_transform_iterator(
                    thrust::make_transform_iterator( leaf_sizes.begin() + 1u, divide_ri_functor<uint32>( LEAF_SYMBOLS ) ),
                    not_equal_to_functor<uint32>(1u) ) ),
            thrust::plus<uint32>(),
            temp_storage );
        log_debug(stderr, "    split leaves %u (%.2f%%)\n", n_split, 100.0f * float(n_split) / float(n_leaves));

        const uint32 n_touched = nvbio::reduce(
            n_leaves,
            make_cast_iterator<uint32>(
                thrust::make_transform_iterator( ins_counts.begin() + 1u, not_equal_to_functor<uint32>(0u) ) ),
            thrust::plus<uint32>(),
            temp_storage );
        log_debug(stderr, "    touched leaves %u (%.2f%% - %.1fMB)\n", n_touched, 100.0f * float(n_touched) / float(n_leaves), float(n_touched)*(m_page_size*sizeof(word_type)) / float(1024*1024));
        */
    }

    // reset the end of the last leaf
    m_offsets.back() = old_size;

    //
    // at this point, each old leaf will be split in util::divide_ri( h_leaf_sizes[i], LEAF_SYMBOLS )
    // new leaves
    //

    // do a prefix sum to compute the new leaf numbering
    nvbio::inclusive_scan(
        n_leaves + 1u,
        thrust::make_transform_iterator( leaf_sizes.begin(), divide_ri_functor<uint32>( LEAF_SYMBOLS ) ),
        new_leaf_ids.begin(),
        thrust::plus<uint32>(),
        temp_storage );

    const uint32 out_leaves = new_leaf_ids[ n_leaves ];

    // alloc a new set of page pointers and offsets
    m_new_pages.resize( out_leaves );
    m_new_offsets.resize( out_leaves+1 );
    m_new_counters.resize( (out_leaves+1) * SYMBOL_COUNT, uint64(0) );

    timer.stop();
    NVBIO_VAR_UNUSED const float t1 = timer.seconds();

    timer.start();

    const uint32 BATCH_SIZE = 4*1024;

    reserve_pages( out_leaves + nvbio::min( n_leaves, BATCH_SIZE ) );

    timer.stop();
    NVBIO_VAR_UNUSED const float t2 = timer.seconds();

    const float utilization = (float( size() + n ) / float(LEAF_SYMBOLS)) / float( out_leaves );

    log_debug(stderr, "    copy pages %u -> %u (utilization : %.1f%%)\n",
        n_leaves, out_leaves,
        100.0f * utilization );

    timer.start();

    const copy_insert_pages<SYMBOL_SIZE,BIG_ENDIAN> copy_functor(
        n,
        n_leaves,
        nvbio::raw_pointer( new_leaf_ids ),
        g,
        c,
        this );

    for (uint32 batch_begin = 0; batch_begin < n_leaves; batch_begin += BATCH_SIZE)
    {
        const uint32 batch_end = nvbio::min( n_leaves, batch_begin + BATCH_SIZE );

        //log_verbose(stderr, "  block[%u:%u] (pool: %u)\n", batch_begin, batch_end, m_pool_size);

        // fill the new leaves (one thread per input leaf)
        nvbio::for_each<host_tag>(
            batch_end - batch_begin,
            thrust::make_counting_iterator<uint32>( batch_begin ),
            copy_functor );
    }

    timer.stop();
    NVBIO_VAR_UNUSED const float t3 = timer.seconds();
    //log_verbose(stderr, "  %.2f insertions/s (%.1f%%, %.1f%%, %.1f%%, %.1f%%)\n",
    //    float(n) / (t0 + t1 + t2 + t3),
    //    100.0f * t0 / (t0 + t1 + t2 + t3),
    //    100.0f * t1 / (t0 + t1 + t2 + t3),
    //    100.0f * t2 / (t0 + t1 + t2 + t3),
    //    100.0f * t3 / (t0 + t1 + t2 + t3));

    // write the sentinel offset
    m_new_offsets[ out_leaves ] = m_offsets[ n_leaves ] + n;

    // swap-in the new pages
    m_pages.swap( m_new_pages );
    m_offsets.swap( m_new_offsets );
    m_counters.swap( m_new_counters );

    // do an exclusive prefix-sum on the occurrence counters
    for (uint32 j = 0; j < SYMBOL_COUNT; ++j)
    {
        exclusive_scan(
            out_leaves+1,
            make_strided_iterator( &m_counters[j], SYMBOL_COUNT ),
            make_strided_iterator( &m_counters[j], SYMBOL_COUNT ),
            thrust::plus<uint64>(),
            uint64(0),
            temp_storage );
    }

    // do an error check on the occurrence counters
    const uint64* cnts = symbol_frequencies();
    uint64 n_occ = 0;
    for (uint32 j = 0; j < SYMBOL_COUNT; ++j)
        n_occ += cnts[j];

    if (n_occ != m_offsets[ out_leaves ])
    {
        log_error(stderr, "mismatching occurrence counters: expected %llu symbols, got %llu\n", m_offsets[ out_leaves ], n_occ );
        //throw runtime_error( "mismatching occurrence counters: expected %llu symbols, got %llu\n", m_offsets[ out_leaves ], n_occ );
        exit(1);
    }

    //if (utilization < 0.75f)
    //    defrag();

    build_buckets( m_offsets.back(), (uint32)m_offsets.size(), &m_offsets[0], BUCKET_SIZE, m_buckets );
}

// global symbol frequencies
//
template <uint32 SYMBOL_SIZE_T, bool BIG_ENDIAN_T>
inline void PagedText<SYMBOL_SIZE_T,BIG_ENDIAN_T>::defrag()
{
    const uint32 LEAF_SYMBOLS = m_page_size * SYMBOLS_PER_WORD;

    const uint32 in_leaves  = page_count();
    const uint64 n_symbols  = size();
    const uint32 out_leaves = util::divide_ri( n_symbols, LEAF_SYMBOLS );

    log_debug(stderr, "    defrag %u -> %u\n", in_leaves, out_leaves );

    nvbio::vector<host_tag,uint8> temp_storage;

    // alloc a new set of page pointers and offsets
    m_new_pages.resize( out_leaves );
    m_new_offsets.resize( out_leaves+1 );
    m_new_counters.resize( (out_leaves+1) * SYMBOL_COUNT, uint64(0) );

    const uint32 BATCH_SIZE = 4*1024;

    uint32 in_leaf_begin = 0;

    const copy_merge_pages<SYMBOL_SIZE,BIG_ENDIAN> merge_functor(
        n_symbols,
        out_leaves,
        in_leaves,
        this );

    // loop across output pages
    for (uint32 batch_begin = 0; batch_begin < out_leaves; batch_begin += BATCH_SIZE)
    {
        const uint32 batch_end = nvbio::min( out_leaves, batch_begin + BATCH_SIZE );

        // make sure we have enough free pages
        reserve_free_pages( batch_end - batch_begin );

        // fill the new leaves (one thread per input leaf)
        nvbio::for_each<host_tag>(
            batch_end - batch_begin,
            thrust::make_counting_iterator<uint32>( batch_begin ),
            merge_functor );

        // release input pages that will no longer be needed
        const uint32 in_leaf_end = upper_bound_index( uint64( batch_end ) * LEAF_SYMBOLS, raw_pointer( m_offsets ), in_leaves+1 ) - 1u;

        for (uint32 i = in_leaf_begin; i < in_leaf_end; ++i)
        {
            release_page( m_pages[i] );
            m_pages[i] = NULL;
        }

        in_leaf_begin = in_leaf_end;
    }

    // release any not yet released input pages
    for (uint32 i = in_leaf_begin; i < in_leaves; ++i)
    {
        release_page( m_pages[i] );
        m_pages[i] = NULL;
    }

    // write the sentinel offset
    m_new_offsets[ out_leaves ] = n_symbols;

    // swap-in the new pages
    m_pages.swap( m_new_pages );
    m_offsets.swap( m_new_offsets );
    m_counters.swap( m_new_counters );

    for (uint32 i = 0; i < page_count()-1; ++i)
    {
        const uint32 cnt = std::accumulate(
            &m_counters[i*SYMBOL_COUNT],
            &m_counters[i*SYMBOL_COUNT] + SYMBOL_COUNT, 0u );

        if (cnt != LEAF_SYMBOLS)
            log_error(stderr, "mismatching occurrence counters: at page[%u], expected %llu symbols, got %llu\n", i, LEAF_SYMBOLS, cnt );
    }

    // do an exclusive prefix-sum on the occurrence counters
    for (uint32 j = 0; j < SYMBOL_COUNT; ++j)
    {
        exclusive_scan(
            out_leaves+1,
            make_strided_iterator( &m_counters[j], SYMBOL_COUNT ),
            make_strided_iterator( &m_counters[j], SYMBOL_COUNT ),
            thrust::plus<uint64>(),
            uint64(0),
            temp_storage );
    }

    // do an error check on the occurrence counters
    const uint64* cnts = symbol_frequencies();
    uint64 n_occ = 0;
    for (uint32 j = 0; j < SYMBOL_COUNT; ++j)
        n_occ += cnts[j];

    if (n_occ != n_symbols)
    {
        log_error(stderr, "mismatching occurrence counters: expected %llu symbols, got %llu\n", n_symbols, n_occ );
        //throw runtime_error( "mismatching occurrence counters: expected %llu symbols, got %llu\n", n_symbols, n_occ );
        exit(1);
    }
}

// global symbol frequencies
//
template <uint32 SYMBOL_SIZE_T, bool BIG_ENDIAN_T>
inline const uint64* PagedText<SYMBOL_SIZE_T,BIG_ENDIAN_T>::symbol_frequencies() const
{
    return &m_counters[ page_count() * SYMBOL_COUNT ];
}

// find the page containing the i-th entry
//
template <uint32 SYMBOL_SIZE_T, bool BIG_ENDIAN_T>
inline uint32 PagedText<SYMBOL_SIZE_T,BIG_ENDIAN_T>::find_page(const uint64 i) const
{
    const uint32 b = i >> LOG_BUCKET_SIZE;
    const uint32 lo = m_buckets[b];
    const uint32 hi = m_buckets[b+1];
    return upper_bound_index( i, &m_offsets[lo], hi - lo ) + lo - 1u;
    //return upper_bound_index( i, &m_offsets[0], (uint32)m_offsets.size() ) - 1u;
}

} // namespace nvbio
