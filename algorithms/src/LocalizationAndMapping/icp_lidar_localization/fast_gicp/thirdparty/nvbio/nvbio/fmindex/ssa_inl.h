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

// constructor
//
inline SSA_value_multiple::SSA_value_multiple(
    const uint32  n,
    const int32*  sa,
    const uint32  K)
{
    m_n = n;

    const uint32 n_words  = (n+1+31) >> 5;
    const uint32 n_blocks = (n+1+63) >> 6;

    m_bitmask.resize( n_words );
    for (uint32 i = 0; i < n_words; ++i)
        m_bitmask[i] = 0u;

    m_blocks.resize( n_blocks );

    // count how many items we need to store
    m_stored = 0;
    for (uint32 i = 0; i <= n; ++i)
    {
        // store a block counter every 64 elements
        if ((i & 63) == 0)
            m_blocks[ i >> 6 ] = m_stored;

        // check whether this SA value is a multiple of K
        if ((sa[i] & (K-1)) == 0)
        {
            m_stored++;
            m_bitmask[ i >> 5 ] |= (1u << (i&31u));
        }
    }

    m_ssa.resize( m_stored );

    // store all the needed values
    m_stored = 0;
    for (uint32 i = 0; i <= n; ++i)
    {
        if ((sa[i] & (K-1)) == 0)
            m_ssa[ m_stored++ ] = sa[i];
    }
}

// constructor
//
// \param fmi      FM index
// \param K        compression factor
template <typename FMIndexType>
SSA_value_multiple::SSA_value_multiple(
    const FMIndexType& fmi,
    const uint32       K)
{
    const uint32 n = fmi.length();

    m_n = n;

    const uint32 n_words  = (n+1+31) >> 5;
    const uint32 n_blocks = (n+1+63) >> 6;

    m_bitmask.resize( n_words );
    for (uint32 i = 0; i < n_words; ++i)
        m_bitmask[i] = 0u;

    m_blocks.resize( n_blocks );

    // count how many items we need to store
    m_stored = 0;
    {
        // calculate SA value
        uint32 isa = 0, sa = n;

        for (uint32 i = 0; i < n; ++i)
        {
            if ((sa & (K-1)) == 0)
            {
                m_stored++;
                m_bitmask[ isa >> 5 ] |= (1u << (isa & 31u));
            }

            --sa;

            isa = basic_inv_psi( fmi, isa );
        }
        if ((sa & (K-1)) == 0)
        {
            m_stored++;
            m_bitmask[ isa >> 5 ] |= (1u << (isa & 31u));
        }
    }

    // compute the block counters, 1 every 64 elements
    m_blocks[0] = 0;
    for (uint32 i = 1; i < n_blocks; ++i)
    {
        m_blocks[i] = m_blocks[i-1] +
            popc( m_bitmask[i*2-1] ) +
            popc( m_bitmask[i*2-2] );
    }

    m_ssa.resize( m_stored );

    // store all the needed values
    {
        // calculate SA value
        uint32 isa = 0, sa = n;

        for (uint32 i = 0; i < n; ++i)
        {
            if ((sa & (K-1)) == 0)
                m_ssa[ index( isa ) ] = sa;

            --sa;

            isa = basic_inv_psi( fmi, isa );
        }
        if ((sa & (K-1)) == 0)
            m_ssa[ index( isa ) ] = sa;
    }

    // NOTE: do we need to handle the fact we don't have sa[0] = -1?
    if (m_ssa[0] == n)
        m_ssa[0] = uint32(-1);
}

inline uint32 SSA_value_multiple::index(const uint32 i) const
{
    // check whether the value is present
    const uint32 word = i >> 5;
    const uint32 word_mask = m_bitmask[ word ];

    //
    // compute the location of the value in the sampled suffix array
    //

    // get the base block value
    const uint32 block = i >> 6;
    uint32 pos = m_blocks[ block ];

    // add the number of bits set in the bitmask containing 'i'
    const uint32 shifted_mask = word_mask & ((1u << (i&31u)) - 1u);
    pos += popc( shifted_mask );

    // add the number of bits set in all the preceding bitmasks
    for (uint32 j = block*2; j < word; ++j)
        pos += popc( m_bitmask[j] );

    return pos;
}

// constructor
//
inline SSA_value_multiple_device::SSA_value_multiple_device(const SSA_value_multiple& ssa) :
    m_n( ssa.m_n ),
    m_stored( ssa.m_stored )
{
    if (m_n != 0)
    {
        const uint32 n_words  = (m_n+31) >> 5;
        const uint32 n_blocks = (m_n+63) >> 6;

        cudaMalloc( &m_ssa,     sizeof(uint32)*ssa.m_stored );
        cudaMalloc( &m_bitmask, sizeof(uint32)*n_words );
        cudaMalloc( &m_blocks,  sizeof(uint32)*n_blocks );

        cudaMemcpy( m_ssa,     &ssa.m_ssa[0],     sizeof(uint32)*m_stored, cudaMemcpyHostToDevice );
        cudaMemcpy( m_bitmask, &ssa.m_bitmask[0], sizeof(uint32)*n_words,  cudaMemcpyHostToDevice );
        cudaMemcpy( m_blocks,  &ssa.m_blocks[0],  sizeof(uint32)*n_blocks, cudaMemcpyHostToDevice );
    }
    else
    {
        m_ssa     = NULL;
        m_bitmask = NULL;
        m_blocks  = NULL;
    }
}

// destructor
//
inline SSA_value_multiple_device::~SSA_value_multiple_device()
{
    cudaFree( m_ssa );
    cudaFree( m_bitmask );
    cudaFree( m_blocks );
}

// fetch the i-th value, if stored, return false otherwise.
//
template <typename SSAIterator, typename BitmaskIterator, typename BlockIterator>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE bool SSA_value_multiple_context<SSAIterator,BitmaskIterator,BlockIterator>::fetch(const uint32 i, uint32& r) const
{
    // check whether the value is present
    const uint32 word = i >> 5;
    const uint32 word_mask = m_bitmask[ word ];
    const uint32 is_there  = word_mask & (1u << (i&31u));
    if (is_there == 0)
        return false;

    //
    // compute the location of the value in the sampled suffix array
    //

    // get the base block value
    const uint32 block = i >> 6;
    uint32 pos = m_blocks[ block ];

    // add the number of bits set in the bitmask containing 'i'
    const uint32 shifted_mask = word_mask & ((1u << (i&31u)) - 1u);
    pos += popc( shifted_mask );

    // add the number of bits set in all the preceding bitmasks
    for (uint32 j = block*2; j < word; ++j)
        pos += popc( m_bitmask[j] );

    r = m_ssa[ pos ];
    return true;
}

// fetch the i-th value, if stored, return false otherwise.
//
template <typename SSAIterator, typename BitmaskIterator, typename BlockIterator>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE bool SSA_value_multiple_context<SSAIterator,BitmaskIterator,BlockIterator>::has(const uint32 i) const
{
    // check whether the value is present
    const uint32 word = i >> 5;
    const uint32 word_mask = m_bitmask[ word ];
    const uint32 is_there  = word_mask & (1u << (i&31u));
    return is_there;
}

// constructor
//
template <uint32 K, typename index_type>
inline SSA_index_multiple<K,index_type>::SSA_index_multiple(
    const index_type  n,
    const index_type* sa)
{
    const uint32 n_items = (n+1+K-1) / K;

    m_n = n;
    m_ssa.resize( n_items );

    // store all the needed values
    for (uint32 i = 0; i < n_items; ++i)
        m_ssa[i] = sa[i*K];
}

// constructor
//
// \param fmi      FM index
// \param K        compression factor
template <uint32 K, typename index_type>
template <typename FMIndexType>
SSA_index_multiple<K,index_type>::SSA_index_multiple(
    const FMIndexType& fmi)
{
    const uint32 n = fmi.length();
    const uint32 n_items = (n+1+K-1) / K;

    m_n = n;
    m_ssa.resize( n_items );

    // calculate SA value
    index_type isa = 0, sa = n;

    for (index_type i = 0; i < n; ++i)
    {
        if ((isa & (K-1)) == 0)
            m_ssa[ isa/K ] = sa;

        --sa;

        isa = basic_inv_psi( fmi, isa );
    }
    if ((isa & (K-1)) == 0)
        m_ssa[ isa/K ] = sa;

    m_ssa[0] = index_type(-1); // before this line, ssa[0] = n
}

// constructor
//
// \param ssa      device-side ssa
template <uint32 K, typename index_type>
SSA_index_multiple<K,index_type>::SSA_index_multiple(
    const SSA_index_multiple_device<K,index_type>& ssa)
{
    m_n = ssa.m_n;
    m_ssa.resize( ssa.m_ssa.size() );

    const uint32 n_items = (m_n+1+K-1) / K;

    cudaMemcpy( &m_ssa[0], thrust::raw_pointer_cast(&ssa.m_ssa[0]), sizeof(index_type)*n_items, cudaMemcpyDeviceToHost );
}

// copy operator
//
// \param ssa      device-side ssa
template <uint32 K, typename index_type>
SSA_index_multiple<K,index_type>& SSA_index_multiple<K,index_type>::operator=(
    const SSA_index_multiple_device<K,index_type>& ssa)
{
    m_n = ssa.m_n;
    m_ssa.resize( ssa.m_ssa.size() );

    const uint32 n_items = (m_n+1+K-1) / K;

    cudaMemcpy( &m_ssa[0], thrust::raw_pointer_cast(&ssa.m_ssa[0]), sizeof(index_type)*n_items, cudaMemcpyDeviceToHost );
    return *this;
}

// constructor
//
template <uint32 K, typename index_type>
SSA_index_multiple_device<K,index_type>::SSA_index_multiple_device(const SSA_index_multiple<K,index_type>& ssa) :
    m_n( ssa.m_n )
{
    const uint32 n_items = (m_n+1+K-1) / K;

    m_ssa.resize( n_items );

    cudaMemcpy( thrust::raw_pointer_cast(&m_ssa[0]), &ssa.m_ssa[0], sizeof(index_type)*n_items, cudaMemcpyHostToDevice );
}

#ifdef __CUDACC__

template <uint32 K, typename index_type, typename FMIndexType>
__global__ void SSA_index_multiple_setup_kernel(
    const uint32          range_begin,
    const uint32          range_end,
    const FMIndexType     fmi,
    index_type*           ssa,
    index_type*           link)
{
    const uint32 grid_size = blockDim.x * gridDim.x;
    const uint32 thread_id = threadIdx.x + blockIdx.x * blockDim.x;

    for (uint32 idx = range_begin + thread_id; idx < range_end; idx += grid_size)
    {
        //
        // Calculate the number of steps needed to go from the starting
        // isa index to the next one, and store the link structure.
        //
        index_type isa = idx * K;
        uint32     steps = 0;

        do
        {
            ++steps;

            isa = basic_inv_psi( fmi, isa );
            NVBIO_CUDA_ASSERT( isa <= fmi.length() );
        }
        while ((isa & (K-1)) != 0);

        ssa[ isa/K ] = steps;
        link[ idx ]  = isa/K;
    }
}

/// constructor
///
/// \param fmi      FM-index
template <uint32 K, typename index_type>
template <typename FMIndexType>
SSA_index_multiple_device<K,index_type>::SSA_index_multiple_device(
    const FMIndexType& fmi)
{
    init( fmi );
}

/// constructor
///
/// \param fmi      FM-index
template <uint32 K, typename index_type>
template <typename FMIndexType>
void SSA_index_multiple_device<K,index_type>::init(
    const FMIndexType& fmi)
{
    m_n = fmi.length();
    const uint32 n_items = (m_n+1+K-1) / K;

    m_ssa.resize( n_items );

    index_type* d_link;
    cudaMalloc( &d_link, sizeof(index_type)*n_items );

    //
    // Compute m_ssa and link: this kernel computes the number of steps
    // taken to go from one isa to the next, and the link structure between
    // them.
    //

    const uint32 BLOCK_SIZE = 128;
    const uint32 max_blocks = (uint32)nvbio::cuda::max_active_blocks(
        SSA_index_multiple_setup_kernel<K,index_type,FMIndexType>, BLOCK_SIZE, 0);

    const uint32 BATCH_SIZE = 128*1024;
    for (uint32 range_begin = 0; range_begin < n_items; range_begin += BATCH_SIZE)
    {
        const uint32 range_end = std::min( range_begin + BATCH_SIZE, uint32( m_n / K + 1 ) );
        const uint32 n_blocks  = std::min( max_blocks, (range_end - range_begin + BLOCK_SIZE-1) / BLOCK_SIZE );

        SSA_index_multiple_setup_kernel<K> <<<n_blocks,BLOCK_SIZE>>>(
            range_begin,
            range_end,
            fmi,
            thrust::raw_pointer_cast(&m_ssa[0]),
            d_link );

        cudaThreadSynchronize();
    }
    //cudaThreadSynchronize();

    // copy ssa & link to the host
    std::vector<index_type> h_link( n_items );
    std::vector<index_type> h_ssa( n_items );

    cudaMemcpy( &h_ssa[0],  thrust::raw_pointer_cast(&m_ssa[0]),  sizeof(index_type)*n_items, cudaMemcpyDeviceToHost );
    cudaMemcpy( &h_link[0], d_link, sizeof(index_type)*n_items, cudaMemcpyDeviceToHost );

    //
    // Walk the link structure between the isa's, and do what is essentially
    // a prefix-sum of the associated number of steps on the way to compute
    // the corresponding sa indices.
    // As the order with which the counters are summed is specified by the
    // link structure, it would be hard to parallelize this (might be possible
    // with parallel list-ranking, but it's likely overkill).
    //
    typedef typename signed_type<index_type>::type sindex_type;

     index_type isa_div_k = 0;
    sindex_type sa        = m_n;
    while (sa > 0)
    {
        if (isa_div_k >= n_items)
            throw std::runtime_error("SSA_index_multiple_device: index out of bounds\n");

        isa_div_k = h_link[ isa_div_k ];
        sa -= h_ssa[ isa_div_k ];

        h_ssa[ isa_div_k ] = index_type( sa );
    }
    h_ssa[0] = index_type(-1);

    // copy ssa to the device
    cudaMemcpy( thrust::raw_pointer_cast(&m_ssa[0]), &h_ssa[0], sizeof(index_type)*n_items, cudaMemcpyHostToDevice );

    cudaFree( d_link );
}

#endif

// fetch the i-th value, if stored, return false otherwise.
//
template <uint32 K, typename Iterator>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE bool SSA_index_multiple_context<K,Iterator>::fetch(const index_type i, index_type& r) const
{
    if ((i & (K-1)) == 0)
    {
        r = m_ssa[ i / K ];
        return true;
    }
    else
        return false;
}

// fetch the i-th value, if stored, return false otherwise.
//
template <uint32 K, typename Iterator>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE bool SSA_index_multiple_context<K,Iterator>::has(const index_type i) const
{
    return ((i & (K-1)) == 0);
}

} // namespace nvbio
