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

#include <cub/cub.cuh>
#include <mgpuhost.cuh>
#include <moderngpu.cuh>
#include <nvbio/basic/vector.h>
#include <nvbio/strings/string_set.h>
#include <nvbio/sufsort/sufsort.h>
#include <nvbio/sufsort/sufsort_priv.h>
#include <nvbio/sufsort/compression_sort.h>
#include <nvbio/fmindex/paged_text.h>

namespace nvbio {

//#define QUICK_CHECK
//#define QUICK_CHECK_REPORT
//#define CHECK_COPY
//#define CHECK_INSERTION
//#define CHECK_SORTING
#if defined(QUICK_CHECK_REPORT) || defined(CHECK_SORTING)
  #define HOST_STRING_IDS
#endif

///@addtogroup Sufsort
///@{

///
///@defgroup SetBWTEModule Set-BWTE
/// This module contains functions and classes implementing the <a href="http://arxiv.org/abs/1410.0562">set-bwte</a> algorithm described in: \n
/// "A massively parallel algorithm for constructing the BWT of large string sets" \n
/// http://arxiv.org/abs/1410.0562
///

///@addtogroup SetBWTEModule
///@{

///
/// Helper structure to hold a sorted block during BWTE
///
struct BWTEBlock
{
    uint32                              max_block_suffixes; // reserved space
    uint32                              max_block_strings;  // reserved space
    uint32                              n_strings;          // number of strings
    uint32                              n_suffixes;         // number of suffixes
    nvbio::vector<host_tag,uint32>      h_SA;               // host block SA
    nvbio::vector<host_tag,uint32>      h_cum_lengths;      // host block string lengths
    nvbio::vector<host_tag,uint32>      h_string_ids;       // host block string ids
    nvbio::vector<host_tag,uint32>      h_dollar_off;       // host block dollar offsets
    nvbio::vector<host_tag,uint32>      h_dollar_id;        // host block dollar ids
    nvbio::vector<host_tag,uint64>      h_dollar_pos;       // host block dollar insertion positions
    nvbio::vector<host_tag,uint8>       h_BWT;              // host BWT block storage

    /// reserve space for a maximum block size
    ///
    void reserve(const uint32 _max_block_strings, const uint32 _max_block_suffixes);
};

///
/// A context for the incremental parallel <a href="http://arxiv.org/abs/1410.0562">set-bwte</a> algorithm for computing the BWT of a string-set.
///
/// \tparam SYMBOL_SIZE         the size of the symbols, in bits
/// \tparam BIG_ENDIAN          whether the input/output packed streams are big endian
/// \tparam storage_type        the iterator to the input packed stream storage
/// \tparam offsets_iterator    the iterator to the offsets in the concatenated string-set
///
template <
    uint32   SYMBOL_SIZE,
    bool     BIG_ENDIAN,
    typename storage_type     = const uint32*,
    typename offsets_iterator = const uint64*>
struct BWTEContext
{
    typedef typename std::iterator_traits<offsets_iterator>::value_type         index_type;
    typedef ConcatenatedStringSet<
            PackedStream<storage_type,uint8,SYMBOL_SIZE,BIG_ENDIAN,index_type>,
            offsets_iterator>                                                   string_set_type;

    /// constructor
    ///
    BWTEContext(const int device);

    /// needed device memory
    ///
    uint64 needed_device_memory(const uint32 _max_block_strings, const uint32 _max_block_suffixes) const;

    /// reserve space for a maximum block size
    ///
    void reserve(const uint32 _max_block_strings, const uint32 _max_block_suffixes);

    /// append a new block of strings
    ///
    ///\param block_begin       the beginning of the block of strings to encode in the input string-set
    ///\param block_end         the end of the block of strings to encode in the input string-set
    ///\param string_set        the input string-set
    ///\param BWT_ext           the output BWT
    ///\param BWT_ext_dollars   the output BWT dollars
    ///\param forward           true if appending the result, false if prepending it
    ///
    void append_block(
        const uint32                            block_begin,
        const uint32                            block_end,
        const string_set_type                   string_set,
            PagedText<SYMBOL_SIZE,BIG_ENDIAN>&  BWT_ext,
            SparseSymbolSet&                    BWT_ext_dollars,
        const bool                              forward);

    // sort the given block
    //
    void sort_block(
        const uint32                        block_begin,
        const uint32                        block_end,
        const string_set_type               string_set,
        BWTEBlock&                          block);

    // merge the given sorted block
    //
    void merge_block(
        const uint32                        block_begin,
        const uint32                        block_end,
        const string_set_type               string_set,
        BWTEBlock&                          block,
        PagedText<SYMBOL_SIZE,BIG_ENDIAN>&  BWT_ext,
        SparseSymbolSet&                    BWT_ext_dollars,
        const bool                          forward);

private:
    typedef typename std::iterator_traits<storage_type>::value_type                         radix_type;

    static const uint32 SYMBOL_COUNT    = 1u << SYMBOL_SIZE;
    static const uint32 RADIX_BITS      = uint32( 8u * sizeof(radix_type) );
    static const uint32 DOLLAR_BITS     = RADIX_BITS <= 32 ? 4 : 5;

    typedef priv::ChunkLoader<SYMBOL_SIZE,BIG_ENDIAN,storage_type,offsets_iterator,host_tag,device_tag> chunk_loader_type;
    typedef typename chunk_loader_type::chunk_set_type                                                  chunk_set_type;

    typedef priv::DeviceStringSetRadices<chunk_set_type,SYMBOL_SIZE,DOLLAR_BITS,RADIX_BITS> string_set_handler_type;
    typedef priv::SetSuffixFlattener<SYMBOL_SIZE>                                           suffix_flattener_type;

    static const uint32 SORTING_SLICE_SIZE = 2u; // determines how frequently sorted suffixes are pruned

    // rank the block suffixes wrt BWT_ext
    //
    void rank_block(
        const uint32                        block_begin,
        const uint32                        block_end,
        const string_set_type               string_set,
        const BWTEBlock&                    block,
        PagedText<SYMBOL_SIZE,BIG_ENDIAN>&  BWT_ext,
        SparseSymbolSet&                    BWT_ext_dollars,
        const bool                          forward);

    // insert the block
    //
    void insert_block(
        BWTEBlock&                        block,
        PagedText<SYMBOL_SIZE,BIG_ENDIAN>&  BWT_ext,
        SparseSymbolSet&                    BWT_ext_dollars);

    uint32                          max_block_suffixes;
    uint32                          max_block_strings;

    uint64                          n_strings_ext;
    uint64                          n_suffixes_ext;

    uint64                          n_processed_strings;
    uint64                          n_processed_suffixes;

    mgpu::ContextPtr                mgpu_ctxt;
    string_set_handler_type         string_set_handler;
    cuda::CompressionSort           string_sorter;
    suffix_flattener_type           suffixes;
    chunk_loader_type               chunk_loader;

    BWTEBlock                           block;              // sorted block
    nvbio::vector<host_tag,uint64>      g;                  // host insertion positions
    nvbio::vector<host_tag,uint64>      g_sorted;           // host sorted insertions

    nvbio::vector<device_tag,uint8>     d_BWT_block;        // device block bwt
    nvbio::vector<device_tag,uint32>    d_dollar_off;       // device block dollar offsets
    nvbio::vector<device_tag,uint32>    d_dollar_id;        // device block dollar ids

    nvbio::vector<device_tag,uint2>     d_suffixes;         // device localized suffixes

    nvbio::vector<device_tag,uint8>     d_temp_storage;     // device temporary storage
    nvbio::vector<host_tag,  uint8>     h_temp_storage;     // host temporary storage

    float load_time;
    float sort_time;
    float copy_time;
    float rank_time;
    float insert_time;
    float insert_dollars_time;
};

///
/// Parallel <a href="http://arxiv.org/abs/1410.0562">set-bwte</a> algorithm for computing the BWT of a string-set.
///
/// \param string_set           the input set of strings
/// \param BWT_ext              the output <i>external</i> BWT, stored into a PagedText
/// \param BWT_ext_dollars      the output <i>external</i> BWT dollar symbols, stored as a SparseSymbolSet
/// \param params               the BWT construction parameters
///
template <uint32 SYMBOL_SIZE, bool BIG_ENDIAN, typename storage_type, typename offsets_iterator>
void bwte(
    const ConcatenatedStringSet<
        PackedStream<storage_type,uint8,SYMBOL_SIZE,BIG_ENDIAN,typename std::iterator_traits<offsets_iterator>::value_type>,
        offsets_iterator>                   string_set,
        PagedText<SYMBOL_SIZE,BIG_ENDIAN>&  BWT_ext,
        SparseSymbolSet&                    BWT_ext_dollars,
        BWTParams*                          params = NULL);

///@} SetBWTEModule
///@} Sufsort

} // namespace nvbio

#include <nvbio/sufsort/bwte_inl.h>
