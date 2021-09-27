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

// sample_kmers.h
//

#include "error_correct.h"
#include "utils.h"
#include <nvbio/basic/pipeline_context.h>
#include <nvbio/basic/numbers.h>
#include <nvbio/basic/atomics.h>
#include <nvbio/basic/bloom_filter.h>
#include <nvbio/basic/primitives.h>
#include <nvbio/basic/console.h>
#include <nvbio/basic/timer.h>
#include <nvbio/basic/threads.h>
#include <nvbio/basic/system.h>
#include <nvbio/basic/exceptions.h>
#include <nvbio/basic/cuda/arch.h>
#include <nvbio/basic/cuda/ldg.h>
#include <nvbio/io/sequence/sequence.h>
#include <stdio.h>
#include <stdlib.h>

using namespace nvbio;

///
/// A functor to sample kmers and insert them in a Bloom filter
///
template <typename string_set_type, typename qual_set_type, typename trusted_filter_type>
struct ErrorCorrectFunctor
{
    /// constructor
    ///
    ///\param _k                kmer length
    ///\param _alpha            the sampling frequency
    ///\param _string_set       the input string set to sample
    ///\param _filter           the kmer Bloom filter
    ///
    NVBIO_HOST_DEVICE
    ErrorCorrectFunctor(
        const uint32                _k,
              string_set_type       _string_set,
              qual_set_type         _qual_set,
        const trusted_filter_type   _trusted_filter,
              uint64*               _stats,
        const float                 _max_correction,
        const uint8                 _bad_quality,
        const uint8                 _new_quality) :
        K(int(_k)), kmask( (uint64(1u) << (K*2))-1u ),
        string_set( _string_set ),
        qual_set( _qual_set ),
        trusted_kmers(_trusted_filter),
        stats(_stats),
        max_correction(_max_correction),
        bad_quality(_bad_quality),
        new_quality(_new_quality) {}

    /// is trusted
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    bool is_trusted(const KmerCode& kmer) const
    {
        return kmer.is_valid() && trusted_kmers[ kmer.code ];
    }

    /// build the stored kmers list
    ///
    template <typename string_type>
    NVBIO_HOST_DEVICE
    void mark_solid_kmers(const int read_len, const string_type& read, bool* solid) const
    {
        KmerCode kmer( K );
        for (int i = 0; i < K-1; ++i)
            kmer.push_back( read[i] );

        for (int i = K-1; i < read_len; ++i)
        {
            kmer.push_back( read[i] );

            solid[ i - K + 1 ] = is_trusted( kmer );
        }
    }

    /// find the longest stored kmer
    ///
    NVBIO_HOST_DEVICE
    int2 find_longest_solid_kmer(const int kmer_count, const bool* solid) const
    {
        int longest_count = 0, stored_count = 0;
        int begin = -1;
        for (int i = 0; i < kmer_count; ++i)
        {
            if (solid[i])
                ++stored_count;
            else
            {
                if (longest_count < stored_count)
                {
                    longest_count = stored_count;
                    begin         = i - stored_count;
                }
                stored_count = 0;
            }
        }
        if (longest_count < stored_count)
        {
            longest_count = stored_count;
            begin         = kmer_count - stored_count;
        }

        if (longest_count == 0)
            return make_int2( 0, 0 ); // unreliable read!

        if (longest_count >= kmer_count)
            return make_int2( begin, kmer_count );

        return make_int2( begin, begin + longest_count );
    }

    /// find the next non-solid position, left to right (i.e. in [from,to), from <= to)
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    int find_right_nonsolid(const int from, const int to, const bool* solid) const
    {
        for (int k = from; k < to; ++k)
            if (!solid[k])
                return k;

        return to;
    }

    /// find the next non-solid position, right to left (i.e. in [to,from], from >= to)
    ///
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    int find_left_nonsolid(const int from, const int to, const bool* solid) const
    {
        for (int k = from; k >= to; --k)
            if (!solid[k])
                return k;

        return to - 1;
    }

    /// find the best right change
    ///
    template <typename string_type>
    NVBIO_HOST_DEVICE
    KmerCode best_right_change(const int read_len, const string_type& read, const int from, const int to, const KmerCode& fixed_kmer, int& best_to, int& best_change, int& best_count) const
    {
        KmerCode best_kmer;

        best_to     = -1;
        best_change = -1;
        best_count  =  0;

        for (int j = 0; j < 4; ++j)
        {
            KmerCode kmer = fixed_kmer;
            kmer.push_back( j );
            if (!is_trusted( kmer ))
                continue;

            if (best_to == -1)
                best_to = from-1;

            // check how many kmers this change can fix
            int k;

            for (k = from; k <= to; ++k)
            {
                kmer.push_back( read[k] );
                if (!is_trusted( kmer ))
                    break;
            }
            
            // Try to extend 1 position
            if (k > to && to == read_len - 1)
            {
                for (int m = 0; m < K - 1 - (to - from + 1); ++m)
                {
                    for (int l = 0; l < 4; ++l)
                    {
                        KmerCode tmp_kmer( kmer );
                        tmp_kmer.push_back( l );

                        if (is_trusted( tmp_kmer ))
                        {
                            kmer.push_back( l );
                            ++k;
                            break;
                        }
                    }
                }
            }

            if (k > best_to)
            {
                best_count   = 1;
                best_to      = k;
                best_change  = j;
                best_kmer    = kmer;
            }
            else if (k == best_to)
            {
                ++best_count;
                if (k == from && j == 0)
                {
                    best_count  = 1;
                    best_change = j;
                    best_kmer   = kmer;
                }
                else if (k == from && best_change == 0)  // [jp]: not sure this is correct - it seems it's setting best_count to 1 
                    best_count = 1;                      //       whenever the previous change was an 'A', but not if it was any other letter
            }
        }
        return best_kmer;
    }

    template <typename string_type>
    NVBIO_HOST_DEVICE
    bool check_right(KmerCode tmp_kmer, const string_type& read, const int pos, const int len) const
    {
        for (int t = 0; t < len; ++t)
        {
            tmp_kmer.push_back( read[pos + t] );
            if (!is_trusted( tmp_kmer ))
                return false;
        }
        return true;
    }

    template <typename string_type>
    NVBIO_HOST_DEVICE
    bool adjust_right(const int read_len, const string_type& read, KmerCode kmer, const int pos) const
    {
        // check whether it is possible to extend by K/2 + 1 bases
        if (pos + K/2 + 1 >= read_len)
            return false;

        for (int c = 0; c < 4; ++c)
        {
            if (c == read[pos - 1])
                continue;

            KmerCode tmp_kmer = kmer;
            tmp_kmer.shift_right( 1 );
            tmp_kmer.push_back( c );

            if (is_trusted( tmp_kmer ))
            {
                // test whether this branch makes sense
                if (check_right( tmp_kmer, read, pos, K/2 + 1 ))
                    return true;
            }
        }
        return false;
    }

    template <typename string_type>
    NVBIO_HOST_DEVICE
    void fix_right(
        const int read_len, const string_type& read, const int2 longest_range,
        int*        fix,
        const bool* solid,
        int&        trimStart,
        int&        badSuffix,
        bool&       ambiguous) const
    {
        const int longest_count = longest_range.y - longest_range.x;

        const int kmer_count = read_len - K + 1;

        // from now on, i is the "current" index in the read we are fixing
        int i = longest_range.y;

        // scan right
        KmerCode kmer( K );
        if (longest_range.y >= kmer_count)
        {
            // the kmers are all correct, force skip the correction.
            i = read_len + 1;
        }
        else
        {
            // build the first kmer to fix
            if (longest_count < K)
            {
                for (i = longest_range.y; i < longest_range.y - 1 + K; ++i)
                    kmer.push_back( read[i] );
            }
            else
            {
                // adjust the anchor if necessary
                for (int j = K / 2 - 1; j >= 0; --j)
                {
                    for (i = longest_range.y - j - 1; i < longest_range.y - j + K - 1; ++i)
                        kmer.push_back( read[i] );

                    if (adjust_right( read_len, read, kmer, i ))
                    {
                        // adjust the anchor
                        --i;
                        kmer.shift_right( 1 );
                        break;
                    }
                }
            }
        }

        for (; i < read_len;)
        {
            const int from = i + 1;
            const int to   = (i + K - 1 < read_len) ? i + K - 1 : read_len - 1; 

            int best_to;
            int best_change;
            int best_count;

            // find the best right change
            const KmerCode tmp_kmer = best_right_change( read_len, read, from, to, kmer, best_to, best_change, best_count );

            if (best_to == -1 || (best_count > 1 && (best_to <= to || to - i + 1 < K)))
            {
                trimStart = i;
                break;
            }

            if (best_count <= 1)
            {
                // unambiguous fix
                fix[i] = best_change;
            }
            else
            {
                // ambiguous fix
                fix[i] = -2;
                ambiguous = true;
            }

            if (best_to >= read_len)
                break;
            
            if (best_to <= to)
            {
                // there are multiple errors in the region
                kmer = tmp_kmer;
                kmer.shift_right( 1 );
                i = best_to;
            }
            else
            {
                // search for next error.
                const int k = find_right_nonsolid( to - K + 2, kmer_count, solid );
                    // [jp] shouldn't it be k = from? [to - K + 2 = (i + K - 1) - K + 2 = from + K - 2 - K + 2 = from]

                if (k >= kmer_count)
                    break;

                kmer.restart();
                for (i = k; i < k + K - 1; ++i)
                {
                    if (fix[i] < 0) kmer.push_back( read[i] );
                    else            kmer.push_back( fix[i] );
                }
            }
        }
    }

    /// find the best left change
    ///
    template <typename string_type>
    NVBIO_HOST_DEVICE
    KmerCode best_left_change(const int read_len, const string_type& read, const int from, const int to, const KmerCode& fixed_kmer, int& best_to, int& best_change, int& best_count) const
    {
        KmerCode best_kmer;

        best_to     = read_len + 1;
        best_change = -1;
        best_count  = 0;

        for (int j = 0; j < 4; ++j)
        {
            KmerCode kmer = fixed_kmer;
            kmer.push_front( j );
            
            if (!is_trusted( kmer ))
                continue;

            if (best_to == read_len + 1)
                best_to = from + 1;

            int k;

            // check how many kmers this change can fix
            for (k = from; k >= to; --k)
            {
                kmer.push_front( read[k] );    
                if (!is_trusted( kmer ))
                    break;
            }

            // try extension
            if (k < to && to == 0)
            {
                for (int m = 0; m < K - 1 - (from - to + 1); ++m)
                {
                    for (int l = 0; l < 4; ++l)
                    {
                        KmerCode tmp_kmer( kmer );
                        tmp_kmer.push_front( l );
                        if (is_trusted( tmp_kmer ))
                        {
                            kmer.push_front( l );
                            --k;
                            break;
                        }
                    }
                }
            }

            if (k < best_to)
            {
                best_count  = 1;
                best_to     = k;
                best_change = j;
                best_kmer   = kmer;
            }
            else if (k == best_to)
            {
                ++best_count;
                if (k == from && j == read[from+1])
                {
                    best_count  = 1;
                    best_change = j;
                    best_kmer   = kmer;
                }
                else if (k == from && best_change == read[from+1])
                    best_count = 1;
            }
        }
        return best_kmer;
    }

    template <typename string_type>
    NVBIO_HOST_DEVICE
    bool check_left(KmerCode tmp_kmer, const string_type& read, const int pos, const int len) const
    {
        for (int t = 0; t < len; ++t)
        {
            tmp_kmer.push_front( read[pos - t] );
            if (!is_trusted( tmp_kmer ))
                return false;
        }
        return true;
    }

    template <typename string_type>
    NVBIO_HOST_DEVICE
    bool adjust_left(const int read_len, const string_type& read, KmerCode kmer, const int pos) const
    {
        // check whether it is possible to extend by K/2 + 1 bases
        if (pos - 1 - K/2 < 0)
            return false;

        for (int c = 0; c < 4; ++c)
        {
            if (c == read[pos])
                continue;

            KmerCode tmp_kmer = kmer;
            tmp_kmer.push_back( 0 );   // append an 'A'
            tmp_kmer.push_front( c );
            if (is_trusted( tmp_kmer )) 
            {    
                // test whether this branch makes sense
                if (check_left( tmp_kmer, read, pos - 1, K/2 + 1 ))
                    return true;
            }
        }
        return false;
    }

    template <typename string_type>
    NVBIO_HOST_DEVICE
    void fix_left(
        const int read_len, const string_type& read, const int2 longest_range,
        int*        fix,
        const bool* solid,
        int&        trimStart,
        int&        badPrefix,
        bool&       ambiguous) const
    {
        const int longest_count = longest_range.y - longest_range.x;

        KmerCode kmer( K );

        // from now on, i is the "current" index in the read we are fixing
        int i;

        if (longest_range.x)
        {
            // force skip
            i = -1;
        }
        else
        {
            // set the starting point
            i = longest_range.x - 1;

            if (longest_count < K)
            {
                kmer.restart();
                for (i = longest_range.x; i < longest_range.x + K - 1; ++i)
                    kmer.push_back( read[i] );    

                kmer.push_back( 0 );
            }
            else
            {
                // adjust the left side of the anchor
                for (int j = K / 2 - 1; j >= 0; --j)
                {
                    const int pos = longest_range.x + j;

                    kmer.restart();
                    for (i = pos; i < pos + K; ++i)
                        kmer.push_back( read[i] );

                    if (adjust_left( read_len, read, kmer, pos ))
                    {
                        // adjust the anchor
                        i = pos;
                        kmer.push_back( 0 ); // append an 'A'
                        break;
                    }
                }
            }
        }

        for (; i >= 0;)
        {
            KmerCode fixed_kmer( kmer );

            const int from = i - 1;
            const int to   = nvbio::max( i - K + 1, 0 ); 

            int best_to;
            int best_change;
            int best_count;

            // find the best left change
            const KmerCode tmp_kmer = best_left_change( read_len, read, from, to, kmer, best_to, best_change, best_count );

            if (best_to == read_len + 1 || (best_count > 1 && (best_to >= to || i - to + 1 < K)))
            {
                badPrefix = i + 1;
                break;
            }

            if (best_count <= 1)
                fix[i] = best_change;
            else
            {
                fix[i] = -2;
                ambiguous = true;
            }

            if (best_to < 0)
                break;

            if (best_to >= to)
            {
                kmer = tmp_kmer;
                kmer.push_front( 0 );  // prepend an 'A'
                i = best_to;
            }
            else
            {
                // search for next error
                const int k = find_left_nonsolid( to - 1, 0, solid );

                if (k < 0)
                    break;

                kmer.restart();
                for (i = k + 1; i < k + K; ++i)
                {
                    if (fix[i] < 0) kmer.push_back( read[i] );
                    else            kmer.push_back( fix[i] );
                }
                i = k;
                kmer.push_back( 0 ); // append an 'A'
            }
        }
    }

    template <typename read_type, typename qual_type>
    NVBIO_HOST_DEVICE
    int correct(read_type read, qual_type qual, int& badPrefix, int& badSuffix) const
    {
        badPrefix = 0;
        badSuffix = 0;

        const int read_len = (int)length( read );
        if (read_len < K)
            return 0;

        int  fix[MAX_READ_LENGTH];
        bool solid[MAX_READ_LENGTH];

        int  trimStart = -1;
        bool ambiguous = false;

        //
        // build the solid array
        //

        mark_solid_kmers( read_len, read, solid );

        const int kmer_count = read_len - K + 1;

        //
        // mark trusted kmers
        //

        trimStart = read_len;
        for (int i = 0; i < read_len; ++i)
            fix[i] = -1;

        // find the longest trusted kmer
        const int2  longest_range = find_longest_solid_kmer( kmer_count, solid );
        const int   longest_count = longest_range.y - longest_range.x;

        if (longest_count == 0)
            return -1; // unreliable read!

        // check whether all kmers are reliable
        if (longest_count >= kmer_count)
            return 0;

        // fix the right end of the read
        fix_right( read_len, read, longest_range, fix, solid, trimStart, badSuffix, ambiguous );

        // fix the left end of the read
        fix_left( read_len, read, longest_range, fix, solid, trimStart, badPrefix, ambiguous );

        float correct_count = 0;

        const uint8 N = 4;

        for (int i = 0; i < read_len; ++i)
        {
            if (i >= K && (fix[i - K] >= 0 && read[i - K] < N))
            {
                correct_count -= (qual[i - K] <= bad_quality) ? 0.5f : 1.0f;
            }
            if (fix[i] >= 0 && read[i] < N)
            {
                correct_count += (qual[i] <= bad_quality) ? 0.5f : 1.0f;
            }
            if (correct_count > max_correction)
                return -1; // unreliable correction
        }

        int corrections = 0;

        for (int i = badPrefix; i < trimStart; ++i)
        {
            if (fix[i] < 0)
                continue;

            if (read[i] != fix[i])
            {
                // fix the base
                read[i] = fix[i];

                // fix the quality score
                if (new_quality != uint8('\0'))
                    qual[i] = new_quality;

                ++corrections;
            }
        }

        badSuffix = read_len - trimStart;

        if (corrections == 0 && badPrefix == 0 && badSuffix == 0 && ambiguous)
    		return -1;

        return corrections;
    }

    /// functor operator
    ///
    ///\param s     input string index
    ///
    NVBIO_HOST_DEVICE
    void operator() (const uint32 s) const
    {
        typedef typename string_set_type::string_type                   read_type;
        typedef typename qual_set_type::string_type                     qual_type;

        // fetch the i-th string
        read_type   read = string_set[s];
        qual_type   qual = qual_set[s];

        int badPrefix, badSuffix;

        int corrections = correct( read, qual, badPrefix, badSuffix );

        if (corrections == 0)
            atomic_add( stats + ERROR_FREE, 1u );
        else if (corrections > 0)
            atomic_add( stats + CORRECTIONS, corrections );
        else
            atomic_add( stats + UNFIXABLE, 1u );

        if (badSuffix > 0)
        {
            atomic_add( stats + TRIMMED_READS, 1u );
            atomic_add( stats + TRIMMED_BASES, badSuffix );
        }
    }

    const int                   K;
    const uint64                kmask;
    mutable string_set_type     string_set;
    mutable qual_set_type       qual_set;
    const trusted_filter_type   trusted_kmers;
          uint64*               stats;
    const float                 max_correction;
    const uint8                 bad_quality;
    const uint8                 new_quality;
};

// process the next batch
//
bool ErrorCorrectStage::process(PipelineContext& context)
{
    typedef nvbio::io::SequenceDataEdit<DNA_N,io::SequenceDataView>::sequence_string_set_type string_set_type;
    typedef nvbio::io::SequenceDataEdit<DNA_N,io::SequenceDataView>::qual_string_set_type     qual_set_type;

    // declare the Bloom filter types
    typedef nvbio::blocked_bloom_filter<hash_functor1, hash_functor2, nvbio::cuda::ldg_pointer<uint4> > trusted_filter_type;

    // declare the error corrector functor
    typedef ErrorCorrectFunctor<string_set_type,qual_set_type,trusted_filter_type> functor_type;

    log_debug(stderr, "  error correction... started\n" );

    // fetch the input
    io::SequenceDataHost* h_read_data = context.input<io::SequenceDataHost>( 0 );

    float time = 0.0f;

    // introduce a timing scope
    try
    {
        const nvbio::ScopedTimer<float> timer( &time );

        if (device >= 0)
        {
            // set the device
            cudaSetDevice( device );

            // copy it to the device
            nvbio::io::SequenceDataDevice d_read_data( *h_read_data );

            nvbio::io::SequenceDataView d_read_view( d_read_data );

            // build an editable view
            nvbio::io::SequenceDataEdit<DNA_N,nvbio::io::SequenceDataView> d_read_edit( d_read_view );

            // build the Bloom filter
            trusted_filter_type trusted_filter( TRUSTED_KMERS_FILTER_K, trusted_filter_size, (const uint4*)trusted_filter_storage );

            // build the kmer sampling functor
            const functor_type error_corrector(
                k,
                d_read_edit.sequence_string_set(),
                d_read_edit.qual_string_set(),
                trusted_filter,
                stats_vec,
                max_correction,
                uint8(bad_quality),
                uint8(new_quality) );

            // and apply the functor to all reads in the batch
            device_for_each(
                d_read_view.size(),
                error_corrector );

            cudaDeviceSynchronize();
            nvbio::cuda::check_error("error-correct");

            // fetch the output
            nvbio::io::SequenceDataHost* output = context.output<nvbio::io::SequenceDataHost>();

            // copy the modified device data to the output
            *output = d_read_data;
        }
        else
        {
            omp_set_num_threads( -device );

            // fetch the output
            nvbio::io::SequenceDataHost* output = context.output<nvbio::io::SequenceDataHost>();

            // copy from the input
            *output = *h_read_data;

            nvbio::io::SequenceDataView h_read_view( *output );

            // build an editable view
            nvbio::io::SequenceDataEdit<DNA_N,nvbio::io::SequenceDataView> h_read_edit( h_read_view );

            // build the Bloom filter
            trusted_filter_type trusted_filter( TRUSTED_KMERS_FILTER_K, trusted_filter_size, (const uint4*)trusted_filter_storage );

            // build the kmer sampling functor
            const functor_type error_corrector(
                k,
                h_read_edit.sequence_string_set(),
                h_read_edit.qual_string_set(),
                trusted_filter,
                stats_vec,
                max_correction,
                uint8(bad_quality),
                uint8(new_quality) );

            // and apply the functor to all reads in the batch
            host_for_each(
                h_read_view.size(),
                error_corrector );
        }
    }
    catch (nvbio::cuda_error e)
    {
        log_error(stderr, "[ErrorCorrectStage] caught a nvbio::cuda_error exception:\n");
        log_error(stderr, "  %s\n", e.what());
        exit(1);
    }
    catch (nvbio::bad_alloc e)
    {
        log_error(stderr, "[ErrorCorrectStage] caught a nvbio::bad_alloc exception:\n");
        log_error(stderr, "  %s\n", e.what());
        exit(1);
    }
    catch (nvbio::logic_error e)
    {
        log_error(stderr, "[ErrorCorrectStage] caught a nvbio::logic_error exception:\n");
        log_error(stderr, "  %s\n", e.what());
        exit(1);
    }
    catch (nvbio::runtime_error e)
    {
        log_error(stderr, "[ErrorCorrectStage] caught a nvbio::runtime_error exception:\n");
        log_error(stderr, "  %s\n", e.what());
        exit(1);
    }
    catch (thrust::system::system_error e)
    {
        log_error(stderr, "[ErrorCorrectStage] caught a thrust::system_error exception:\n");
        log_error(stderr, "  %s\n", e.what());
        exit(1);
    }
    catch (std::bad_alloc e)
    {
        log_error(stderr, "[ErrorCorrectStage] caught a std::bad_alloc exception:\n");
        log_error(stderr, "  %s\n", e.what());
        exit(1);
    }
    catch (std::logic_error e)
    {
        log_error(stderr, "[ErrorCorrectStage] caught a std::logic_error exception:\n");
        log_error(stderr, "  %s\n", e.what());
        exit(1);
    }
    catch (std::runtime_error e)
    {
        log_error(stderr, "[ErrorCorrectStage] caught a std::runtime_error exception:\n");
        log_error(stderr, "  %s\n", e.what());
        exit(1);
    }
    catch (...)
    {
        log_error(stderr, "[ErrorCorrectStage] caught an unknown exception!\n");
        exit(1);
    }

    // update the time stats
    stats->m_mutex.lock();
    stats->m_time += time;

    log_info(stderr, "\r  processed reads [%llu, %llu] (%.1fM / %.2fG bps, %.1fK reads/s, %.1fM bps/s - %s<%d>)        ",
        stats->m_reads,
        stats->m_reads + h_read_data->size(),
        1.0e-6f * (h_read_data->bps()),
        1.0e-9f * (stats->m_bps + h_read_data->bps()),
        stats->m_time ? (1.0e-3f * (stats->m_reads + h_read_data->size())) / stats->m_time : 0.0f,
        stats->m_time ? (1.0e-6f * (stats->m_bps   + h_read_data->bps() )) / stats->m_time : 0.0f,
        device >= 0 ? "gpu" : "cpu",
        device >= 0 ? device : -device );

    log_debug_cont(stderr, "\n");
    log_debug(stderr, "  error correction... done\n" );
    log_debug(stderr, "    peak memory : %.1f GB\n", float( peak_resident_memory() ) / float(1024*1024*1024));

    stats->m_reads += h_read_data->size();
    stats->m_bps   += h_read_data->bps();
    stats->m_mutex.unlock();

    return true;
}
