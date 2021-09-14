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

#include "sample_kmers.h"
#include "utils.h"
#include <nvbio/basic/pipeline_context.h>
#include <nvbio/basic/numbers.h>
#include <nvbio/basic/bloom_filter.h>
#include <nvbio/basic/primitives.h>
#include <nvbio/basic/console.h>
#include <nvbio/basic/timer.h>
#include <nvbio/basic/threads.h>
#include <nvbio/basic/system.h>
#include <nvbio/basic/exceptions.h>
#include <nvbio/basic/cuda/ldg.h>
#include <nvbio/basic/cuda/arch.h>
#include <nvbio/io/sequence/sequence.h>
#include <nvbio/strings/prefetcher.h>
#include <stdio.h>
#include <stdlib.h>

using namespace nvbio;

///
/// A functor to sample kmers and insert them in a Bloom filter
///
template <typename string_set_type, typename filter_type>
struct SampleKmersFunctor
{
    /// constructor
    ///
    ///\param _k                kmer length
    ///\param _alpha            the sampling frequency
    ///\param _string_set       the input string set to sample
    ///\param _filter           the kmer Bloom filter
    ///
    NVBIO_HOST_DEVICE
    SampleKmersFunctor(
        const uint32          _k,
        const float           _alpha,
        const string_set_type _string_set,
              filter_type     _filter) :
    k(_k), kmask( (uint64(1u) << (k*2))-1u ), alpha( _alpha ), string_set( _string_set ), filter(_filter) {}

    /// functor operator
    ///
    ///\param i     input string index
    ///
    NVBIO_HOST_DEVICE
    void operator() (const uint32 i) const
    {
        typedef typename string_set_type::string_type                   string_type;
        typedef typename string_traits<string_type>::forward_iterator   forward_iterator;

        // fetch the i-th string
        const string_type string = string_set[i];

        const uint32 len = length( string );
        if (len < k)
            return;

        // build a forward string iterator
        forward_iterator it( string.begin() );

        // start with an empty kmer
        uint64 kmer     = 0u;
        uint32 kmer_len = 0u;

         // initialie a random number generator
        LCG_random random( hash(i) );

        for (uint32 j = 0; j < len; ++j)
        {
            // fetch the next character
            const uint8 c = *it; ++it;

            if (c < 4) // make sure this is not an N
            {
                kmer |= c; // insert the new character at the end of the kmer (in a big-endian encoding)
                if (kmer_len < k)
                    kmer_len++;

                if (kmer_len >= k) // check whether we have an actual 'k'-mer
                {
                    if (float( random.next() ) / float(LCG_random::MAX) < alpha)
                    {
                        // insert the kmer
                        filter.insert( kmer );
                    }
                }

                // shift the kmer to the right, dropping the last symbol
                kmer <<= 2;
                kmer &= kmask;
            }
            else
            {
                // an N, skip all k-mers containing it
                it += k-1;
                j  += k-1;

                // and reset the kmer
                kmer     = 0u;
                kmer_len = 0u;
            }
        }
    }

    const uint32            k;
    const uint64            kmask;
    const float             alpha;
    string_set_type         string_set;
    mutable filter_type     filter;
};


// process the next batch
//
bool SampleKmersStage::process(PipelineContext& context)
{
    typedef nvbio::io::SequenceDataAccess<DNA_N>::sequence_string_set_type string_set_type;

    // declare the Bloom filter type
    typedef nvbio::blocked_bloom_filter<hash_functor1, hash_functor2, uint64_2*> filter_type;

    typedef SampleKmersFunctor<string_set_type,filter_type> functor_type;

    // fetch the input
    nvbio::io::SequenceDataHost* h_read_data = context.input<nvbio::io::SequenceDataHost>( 0 );

    float time = 0.0f;

    // introduce a timing scope
    try
    {
        const nvbio::ScopedTimer<float> timer( &time );

        if (device >= 0)
        {
            //
            // Device (GPU) path
            //

            // set the device
            cudaSetDevice( device );

            // copy it to the device
            nvbio::io::SequenceDataDevice d_read_data( *h_read_data );

            // build a view
            const nvbio::io::SequenceDataAccess<DNA_N> d_read_view( d_read_data );

            // build the Bloom filter
            filter_type filter( SAMPLED_KMERS_FILTER_K, filter_size, (uint64_2*)filter_storage );
            //filter_type filter( filter_size, filter_storage );

            // build the kmer sampling functor
            const functor_type kmer_filter(
                k,
                alpha,
                d_read_view.sequence_string_set(),
                filter );

            device_for_each( d_read_view.size(), kmer_filter );

            cudaDeviceSynchronize();
            cuda::check_error("sample-kmers");
        }
        else
        {
            //
            // Host (CPU) path
            //

            omp_set_num_threads( -device );

            // build a view
            const io::SequenceDataAccess<DNA_N> h_read_view( *h_read_data );

            // build the Bloom filter
            filter_type filter( SAMPLED_KMERS_FILTER_K, filter_size, (uint64_2*)filter_storage );

            // build the kmer sampling functor
            const functor_type kmer_filter(
                k,
                alpha,
                h_read_view.sequence_string_set(),
                filter );

            host_for_each(
                h_read_view.size(),
                kmer_filter );
        }
    }
    catch (nvbio::cuda_error e)
    {
        log_error(stderr, "[SampleKmersStage] caught a nvbio::cuda_error exception:\n");
        log_error(stderr, "  %s\n", e.what());
        exit(1);
    }
    catch (nvbio::bad_alloc e)
    {
        log_error(stderr, "[SampleKmersStage] caught a nvbio::bad_alloc exception:\n");
        log_error(stderr, "  %s\n", e.what());
        exit(1);
    }
    catch (nvbio::logic_error e)
    {
        log_error(stderr, "[SampleKmersStage] caught a nvbio::logic_error exception:\n");
        log_error(stderr, "  %s\n", e.what());
        exit(1);
    }
    catch (nvbio::runtime_error e)
    {
        log_error(stderr, "[SampleKmersStage] caught a nvbio::runtime_error exception:\n");
        log_error(stderr, "  %s\n", e.what());
        exit(1);
    }
    catch (thrust::system::system_error e)
    {
        log_error(stderr, "[SampleKmersStage] caught a thrust::system_error exception:\n");
        log_error(stderr, "  %s\n", e.what());
        exit(1);
    }
    catch (std::bad_alloc e)
    {
        log_error(stderr, "[SampleKmersStage] caught a std::bad_alloc exception:\n");
        log_error(stderr, "  %s\n", e.what());
        exit(1);
    }
    catch (std::logic_error e)
    {
        log_error(stderr, "[SampleKmersStage] caught a std::logic_error exception:\n");
        log_error(stderr, "  %s\n", e.what());
        exit(1);
    }
    catch (std::runtime_error e)
    {
        log_error(stderr, "[SampleKmersStage] caught a std::runtime_error exception:\n");
        log_error(stderr, "  %s\n", e.what());
        exit(1);
    }
    catch (...)
    {
        log_error(stderr, "[SampleKmersStage] caught an unknown exception!\n");
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
    log_debug(stderr,"  peak memory : %.1f GB\n", float( peak_resident_memory() ) / float(1024*1024*1024));

    stats->m_reads += h_read_data->size();
    stats->m_bps   += h_read_data->bps();
    stats->m_mutex.unlock();
    return true;
}


///
/// A functor to sample kmers and insert them in a Bloom filter
///
template <typename string_set_type, typename sampled_filter_type, typename trusted_filter_type, typename threshold_type>
struct TrustedKmersFunctor
{
    /// constructor
    ///
    ///\param _k                kmer length
    ///\param _alpha            the sampling frequency
    ///\param _string_set       the input string set to sample
    ///\param _filter           the kmer Bloom filter
    ///
    NVBIO_HOST_DEVICE
    TrustedKmersFunctor(
        const uint32                _k,
        const string_set_type       _string_set,
        const sampled_filter_type   _sampled_filter,
              trusted_filter_type   _trusted_filter,
        const threshold_type        _threshold) :
        k(_k), kmask( (uint64(1u) << (k*2))-1u ),
        string_set( _string_set ),
        sampled_filter(_sampled_filter),
        trusted_filter(_trusted_filter),
        threshold(_threshold) {}

    /// functor operator
    ///
    ///\param i     input string index
    ///
    NVBIO_HOST_DEVICE
    void operator() (const uint32 i) const
    {
        typedef typename string_set_type::string_type                                           string_type;
        typedef nvbio::StringPrefetcher< string_type, nvbio::lmem_cache_tag<MAX_READ_LENGTH> >  string_prefetcher_type;
        typedef typename string_prefetcher_type::string_type                                    local_string_type;
        typedef typename nvbio::string_traits<local_string_type>::forward_iterator              forward_iterator;

        //bool occur[MAX_READ_LENGTH];
        uint32 occur_storage[MAX_READ_LENGTH/32];

        nvbio::PackedStream<uint32*,uint8,1u,false> occur( occur_storage );

        // instantiate a prefetcher
        string_prefetcher_type string_prefetcher;

        // fetch the i-th string
        //const string_type string = string_set[i];
        const local_string_type string = string_prefetcher.load( string_set[i] );

        const uint32 len = length( string );
        if (len < k)
            return;

        // build a forward string iterator
        forward_iterator it( string.begin() );

        // start with an empty kmer
        uint64 kmer     = 0u;
        uint32 kmer_len = 0u;

        const int32 occur_cnt = len - k + 1;

        // initialize all to false
        for (uint32 j = 0; j < (occur_cnt+31)/32; ++j)
            occur_storage[j] = 0u;

        // mark occurring kmers
        for (uint32 j = 0; j < len; ++j)
        {
            // fetch the next character
            const uint8 c = *it; ++it;

            if (c < 4) // make sure this is not an N
            {
                kmer |= c; // insert the new character at the end of the kmer (in a big-endian encoding)
                if (kmer_len < k)
                    kmer_len++;

                if (kmer_len >= k) // check whether we have an actual 'k'-mer
                {
                    if (sampled_filter[ kmer ])
                        occur[j - k + 1] = true;
                }

                // shift the kmer to the right, dropping the last symbol
                kmer <<= 2;
                kmer &= kmask;
            }
            else
            {
                // an N, skip all kmers containing it
                it += k-1;
                j  += k-1;

                // and reset the kmer
                kmer     = 0u;
                kmer_len = 0u;
            }
        }

        // mark trusted kmers
        int32 zero_cnt = 0;
        int32 one_cnt  = 0;

        // reset the forward iterator
        it = forward_iterator( string.begin() );

        // start with an empty kmer
        kmer     = 0u;
        kmer_len = 0u;

        // keep a k-bits mask of trusted positions
        const uint64 trusted_mask = (uint64(1u) << k) - 1u;
              uint64 trusted      = 0u;

        for (uint32 j = 0; j < len; ++j)
        {
            if (j >= k)
            {
                if (occur[j - k]) --one_cnt;
                else              --zero_cnt;
            }

            if (j < occur_cnt)
            {
                if (occur[j]) ++one_cnt;
                else          ++zero_cnt;
            }

            const int32 sum = one_cnt + zero_cnt;

            //if (qual[j] <= bad_quality)
            //{
            //    trusted[j] = false;
            //    continue ;
            //}

            trusted |= (one_cnt > threshold[sum]) ? 1u : 0u;

            // fetch the next character
            const uint8 c = *it; ++it;

            if (c < 4) // if an N, skip it (the kmers containing it will be marked as untrusted and skipped as well)
            {
                kmer |= c; // insert the new character at the end of the kmer (in a big-endian encoding)

                if (popc( trusted ) == k) // check whether we have an actual 'k'-mer - i.e. k trusted positions in a row
                    trusted_filter.insert( kmer );
            }

            // shift the kmer to the right, dropping the last symbol
            kmer <<= 2;
            kmer &= kmask;

            // shift the trusted bits by one to the right, dropping the last symbol
            trusted <<= 1;
            trusted &= trusted_mask;
        }
    }

    const uint32                k;
    const uint64                kmask;
    string_set_type             string_set;
    const sampled_filter_type   sampled_filter;
    mutable trusted_filter_type trusted_filter;
    const threshold_type        threshold;
};

// process the next batch
//
bool TrustedKmersStage::process(PipelineContext& context)
{
    typedef nvbio::io::SequenceDataAccess<DNA_N>::sequence_string_set_type string_set_type;

    // fetch the input
    nvbio::io::SequenceDataHost* h_read_data = context.input<nvbio::io::SequenceDataHost>( 0 );

    float time = 0.0f;

    // introduce a timing scope
    try
    {
        const nvbio::ScopedTimer<float> timer( &time );

        if (device >= 0)
        {
            //
            // Device (GPU) path
            //

            // declare the Bloom filter types
            typedef nvbio::blocked_bloom_filter<hash_functor1, hash_functor2, nvbio::cuda::ldg_pointer<uint4> > sampled_filter_type;
            typedef nvbio::blocked_bloom_filter<hash_functor1, hash_functor2, uint64_2*>                        trusted_filter_type;

            typedef TrustedKmersFunctor<string_set_type,sampled_filter_type,trusted_filter_type, cuda::ldg_pointer<uint32> > functor_type;

            // set the device
            cudaSetDevice( device );

            // copy it to the device
            io::SequenceDataDevice d_read_data( *h_read_data );

            // build a view
            const io::SequenceDataAccess<DNA_N> d_read_view( d_read_data );

            // build the Bloom filter
            sampled_filter_type sampled_filter( SAMPLED_KMERS_FILTER_K, sampled_filter_size, (const uint4*)sampled_filter_storage );
            trusted_filter_type trusted_filter( TRUSTED_KMERS_FILTER_K, trusted_filter_size,    (uint64_2*)trusted_filter_storage );

            // build the kmer sampling functor
            const functor_type kmer_filter(
                k,
                d_read_view.sequence_string_set(),
                sampled_filter,
                trusted_filter,
                cuda::make_ldg_pointer(threshold) );

            // and apply the functor to all reads in the batch
            device_for_each(
                d_read_view.size(),
                kmer_filter );

            cudaDeviceSynchronize();
            cuda::check_error("mark-trusted-kmers");
        }
        else
        {
            //
            // Host (CPU) path
            //

            omp_set_num_threads( -device );

            // declare the Bloom filter types
            typedef nvbio::blocked_bloom_filter<hash_functor1, hash_functor2, const uint64_2*>         sampled_filter_type;
            typedef nvbio::blocked_bloom_filter<hash_functor1, hash_functor2,       uint64_2*>         trusted_filter_type;

            typedef TrustedKmersFunctor<string_set_type,sampled_filter_type,trusted_filter_type,const uint32*> functor_type;

            // build a view
            const nvbio::io::SequenceDataAccess<DNA_N> h_read_view( *h_read_data );

            // build the Bloom filter
            sampled_filter_type sampled_filter( SAMPLED_KMERS_FILTER_K, sampled_filter_size, (const uint64_2*)sampled_filter_storage );
            trusted_filter_type trusted_filter( TRUSTED_KMERS_FILTER_K, trusted_filter_size,       (uint64_2*)trusted_filter_storage );

            // build the kmer sampling functor
            const TrustedKmersFunctor<string_set_type,sampled_filter_type,trusted_filter_type,const uint32*> kmer_filter(
                k,
                h_read_view.sequence_string_set(),
                sampled_filter,
                trusted_filter,
                threshold );

            // and apply the functor to all reads in the batch
            host_for_each(
                h_read_view.size(),
                kmer_filter );
        }
    }
    catch (nvbio::cuda_error e)
    {
        log_error(stderr, "[TrustedKmersStage] caught a nvbio::cuda_error exception:\n");
        log_error(stderr, "  %s\n", e.what());
        exit(1);
    }
    catch (nvbio::bad_alloc e)
    {
        log_error(stderr, "[TrustedKmersStage] caught a nvbio::bad_alloc exception:\n");
        log_error(stderr, "  %s\n", e.what());
        exit(1);
    }
    catch (nvbio::logic_error e)
    {
        log_error(stderr, "[TrustedKmersStage] caught a nvbio::logic_error exception:\n");
        log_error(stderr, "  %s\n", e.what());
        exit(1);
    }
    catch (nvbio::runtime_error e)
    {
        log_error(stderr, "[TrustedKmersStage] caught a nvbio::runtime_error exception:\n");
        log_error(stderr, "  %s\n", e.what());
        exit(1);
    }
    catch (thrust::system::system_error e)
    {
        log_error(stderr, "[TrustedKmersStage] caught a thrust::system_error exception:\n");
        log_error(stderr, "  %s\n", e.what());
        exit(1);
    }
    catch (std::bad_alloc e)
    {
        log_error(stderr, "[TrustedKmersStage] caught a std::bad_alloc exception:\n");
        log_error(stderr, "  %s\n", e.what());
        exit(1);
    }
    catch (std::logic_error e)
    {
        log_error(stderr, "[TrustedKmersStage] caught a std::logic_error exception:\n");
        log_error(stderr, "  %s\n", e.what());
        exit(1);
    }
    catch (std::runtime_error e)
    {
        log_error(stderr, "[TrustedKmersStage] caught a std::runtime_error exception:\n");
        log_error(stderr, "  %s\n", e.what());
        exit(1);
    }
    catch (...)
    {
        log_error(stderr, "[TrustedKmersStage] caught an unknown exception!\n");
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
    log_debug(stderr,"  peak memory : %.1f GB\n", float( peak_resident_memory() ) / float(1024*1024*1024));

    stats->m_reads += h_read_data->size();
    stats->m_bps   += h_read_data->bps();
    stats->m_mutex.unlock();
    return true;
}
