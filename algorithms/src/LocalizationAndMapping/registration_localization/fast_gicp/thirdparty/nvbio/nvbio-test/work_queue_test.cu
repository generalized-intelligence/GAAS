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

// work_queue_test.cu
//
#define NVBIO_CUDA_DEBUG

#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <algorithm>
#include <nvbio/basic/timer.h>
#include <nvbio/basic/console.h>
#include <nvbio/basic/strided_iterator.h>
#include <nvbio/basic/cuda/arch.h>
#include <nvbio/basic/cuda/work_queue.h>

namespace nvbio {
namespace wqtest {

struct TestWorkStream;

//
// A test work-unit to be used with cuda::WorkQueue.
// Odd work-units produce a continuation, the others don't.
//
struct TestWorkUnit
{
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    TestWorkUnit() {}

    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    TestWorkUnit(const uint32 _i, const uint32 _size, uint32* _output) : i(_i), size( _size ), offset( 0 ), output( _output ) {}

    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    bool run(const TestWorkStream& stream)
    {
        output[i] = i;

        if (i & 1)
        {
            const uint32 d = i - offset;
            offset += size;
            i       = offset + (d/2);
            size   /= 2;
            return true;
        }
        return false;
    }

    uint32  i;
    uint32  size;
    uint32  offset;
    uint32* output;
};

//
// A test work-stream to be used with cuda::WorkQueue
//
struct TestWorkStream
{
    typedef TestWorkUnit WorkUnit;

    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    TestWorkStream(const uint32 size, uint32* output) : m_size( size ), m_output( output ) {}

    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    uint32 size() const { return m_size; }

    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    void get(const uint32 i, TestWorkUnit* unit, const uint2 slot) const { *unit = TestWorkUnit(i,m_size,m_output); }

private:
    uint32  m_size;
    uint32* m_output;
};

template <typename WorkUnit>
struct BenchmarkWorkStream;

//
// A test work-unit to be used with cuda::WorkQueue.
// Odd work-units produce a continuation, the others don't.
// The work-units come with a payload, which is part of the work-unit itself.
//
template <uint32 PAYLOAD>
struct BenchmarkWorkUnit
{
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    BenchmarkWorkUnit() {}

    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    BenchmarkWorkUnit(
        const BenchmarkWorkStream<BenchmarkWorkUnit>& stream,
        const uint32 _i, const uint2 _slot) : i(_i)
    {
        // fill the payload with multiples of 2
        for (uint32 j = 0; j < PAYLOAD; ++j)
            m_payload[j] = j*2;
    }

    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    bool run(const BenchmarkWorkStream<BenchmarkWorkUnit>& stream)
    {
        // do something with the associated memory to simulate reading from it
        uint32 sum = 0;
        for (uint32 j = 0; j < PAYLOAD; ++j)
        {
            sum += m_payload[j];

            m_payload[j] *= 2;
        }

        assert( (sum & 1) == 0 );

        if ((i+sum) & 1)
        {
            i /= 2;
            return true;
        }
        return false;
    }

    uint32 i;
    uint32 m_payload[PAYLOAD];
};

//
// A test work-unit to be used with cuda::WorkQueue.
// Odd work-units produce a continuation, the others don't.
// The work-units come with a payload which is new'd and delete'd at run-time.
//
template <uint32 PAYLOAD>
struct BenchmarkDynMemWorkUnit
{
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    BenchmarkDynMemWorkUnit() : payload(NULL) {}

    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    ~BenchmarkDynMemWorkUnit() { }

    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    BenchmarkDynMemWorkUnit(
        const BenchmarkWorkStream<BenchmarkDynMemWorkUnit>& stream,
        const uint32 _i, const uint2 _slot) : i(_i), payload(NULL) {}

    NVBIO_FORCEINLINE NVBIO_DEVICE
    bool run(const BenchmarkWorkStream<BenchmarkDynMemWorkUnit>& stream);

    uint32  i;
    uint32* payload;
};

//
// A test work-unit to be used with cuda::WorkQueue.
// Odd work-units produce a continuation, the others don't.
// The work-units are bound to an external payload which is stored in the stream class
// in strided fashion.
// When continuations are moved from one execution slot to another by the work queue,
// an external mover class copies the payload to its new location.
//
template <uint32 PAYLOAD>
struct BenchmarkStridedWorkUnit
{
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    BenchmarkStridedWorkUnit() {}

    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    BenchmarkStridedWorkUnit(
        const BenchmarkWorkStream<BenchmarkStridedWorkUnit>& stream,
        const uint32 _i, const uint2 _slot);

    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    bool run(const BenchmarkWorkStream<BenchmarkStridedWorkUnit>& stream);

    uint32  i;
    uint2   slot;
};

//
// A work-unit mover class responsible for moving the strided payload bound to each
// work-unit. Its move method gets invoked when the work-queue changes the execution
// slot of a continuation relative to its parent.
//
template <uint32 PAYLOAD>
struct BenchmarkStridedWorkMover
{
    template <typename WorkUnit>
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    void move(
        const BenchmarkWorkStream<WorkUnit>& stream,
        const uint2 src_slot, WorkUnit* src_unit,
        const uint2 dst_slot, WorkUnit* dst_unit) const;
};

//
// A generic work-stream class for all the above work-units.
//
template <typename WorkUnitT>
struct BenchmarkWorkStream
{
    typedef WorkUnitT WorkUnit;

    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    BenchmarkWorkStream(const uint32 size, uint32* payloads = NULL, uint32 stride = 0) : m_size( size ), m_payloads( payloads ), m_stride( stride ) {}

    // set pool
    //
    void set_pool(uint32* pool_size, uint32* pool) { m_pool = pool; m_pool_size = pool_size; }

    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    uint32 size() const { return m_size; }

    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    void get(const uint32 i, WorkUnit* unit, const uint2 slot) const { *unit = WorkUnit( *this, i,slot); }

    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    uint32* payloads() const { return m_payloads; }

    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
    uint32 stride() const { return m_stride; }

    NVBIO_FORCEINLINE NVBIO_DEVICE
    uint32* alloc(const uint32 PAYLOAD) const
    {
        const uint32 slot = atomicSub( m_pool_size, 1u );
        return m_payloads + m_pool[slot-1] * PAYLOAD;
    }
    NVBIO_FORCEINLINE NVBIO_DEVICE
    void free(const uint32 PAYLOAD, const uint32* payload) const
    {
        const uint32 slot = atomicAdd( m_pool_size, 1u );
        m_pool[slot] = (payload - m_payloads) / PAYLOAD;
    }

private:
    uint32  m_size;
    uint32* m_payloads;
    uint32  m_stride;
    uint32* m_pool_size;
    uint32* m_pool;
};

template <uint32 PAYLOAD>
NVBIO_FORCEINLINE NVBIO_DEVICE
bool BenchmarkDynMemWorkUnit<PAYLOAD>::run(const BenchmarkWorkStream<BenchmarkDynMemWorkUnit>& stream)
{
    if (payload == NULL)
    {
        // alloc memory on first run
        payload = stream.alloc( PAYLOAD );
        assert( payload );

        // fill the payload with multiples of 2
        for (uint32 j = 0; j < PAYLOAD; ++j)
            payload[j] = j*2;
    }

    // do something with the associated memory to simulate reading from it
    uint32 sum = 0;
    for (uint32 j = 0; j < PAYLOAD; ++j)
    {
        sum += payload[j];

        payload[j] *= 2;
    }

    if ((i+sum) & 1)
    {
        i /= 2;
        return true;
    }
    
    // release memory
    stream.free( PAYLOAD, payload ); payload = NULL;
    return false;
}

template <uint32 PAYLOAD>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
BenchmarkStridedWorkUnit<PAYLOAD>::BenchmarkStridedWorkUnit(
    const BenchmarkWorkStream<BenchmarkStridedWorkUnit>& stream,
    const uint32 _i, const uint2 _slot) : i(_i), slot(_slot)
{
    uint32* payloads_buffer = (stream.payloads() + slot.y * stream.stride()*PAYLOAD);
    strided_iterator<uint32*> payload( payloads_buffer + slot.x, stream.stride() );

    // fill the payload with multiples of 2
    for (uint32 j = 0; j < PAYLOAD; ++j)
        payload[j] = j*2;
}

template <uint32 PAYLOAD>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
bool BenchmarkStridedWorkUnit<PAYLOAD>::run(const BenchmarkWorkStream<BenchmarkStridedWorkUnit>& stream)
{
    uint32* payloads_buffer = (stream.payloads() + slot.y * stream.stride()*PAYLOAD);
    strided_iterator<uint32*> payload( payloads_buffer + slot.x, stream.stride() );

    // do something with the associated memory to simulate reading from it
    uint32 sum = 0;
    for (uint32 j = 0; j < PAYLOAD; ++j)
    {
        sum += payload[j];

        payload[j] *= 2;
    }

    assert( (sum & 1) == 0 );

    if ((i+sum) & 1)
    {
        i /= 2;
        return true;
    }
    return false;
}

template <uint32 PAYLOAD>
template <typename WorkUnit>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
void BenchmarkStridedWorkMover<PAYLOAD>::move(
    const BenchmarkWorkStream<WorkUnit>& stream,
    const uint2 src_slot, WorkUnit* src_unit,
    const uint2 dst_slot, WorkUnit* dst_unit) const
{
    // copy the strided payloads
    const uint32  src_queue = src_slot.y;
    const uint32  dst_queue = dst_slot.y;

          uint32* src_payloads = stream.payloads() + src_queue * stream.stride() * PAYLOAD;
          uint32* dst_payloads = stream.payloads() + dst_queue * stream.stride() * PAYLOAD;

    strided_iterator<uint32*> src_payload( src_payloads + src_slot.x, stream.stride() );
    strided_iterator<uint32*> dst_payload( dst_payloads + dst_slot.x, stream.stride() );

    for (uint32 i = 0; i < PAYLOAD; ++i)
        dst_payload[i] = src_payload[i];

    // copy the unit
    *dst_unit = *src_unit;

    // fix destination's slot
    dst_unit->slot = dst_slot;
}

template <uint32 PAYLOAD, uint32 BLOCKDIM>
void benchmark(const uint32 n_tests, const uint32 min_size, const uint32 max_size)
{
    using namespace cuda;

    typedef BenchmarkWorkUnit<PAYLOAD>                                  FatWorkUnit;
    typedef BenchmarkWorkStream<FatWorkUnit>                            FatWorkStream;
    typedef WorkQueue<OrderedQueueTag,FatWorkUnit,BLOCKDIM>             FatWorkQueue;
    typedef WorkQueue<MultiPassQueueTag,FatWorkUnit,BLOCKDIM>           FatMKWorkQueue;
    typedef WorkQueue<PersistentWarpsQueueTag,FatWorkUnit,BLOCKDIM>     FatPWWorkQueue;
    typedef WorkQueue<PersistentThreadsQueueTag,FatWorkUnit,BLOCKDIM>   FatPTWorkQueue;

    const uint32 sz = uint32(sizeof(FatWorkUnit));
    const float  GB = float(1024*1024*1024);

    const uint32 base_stream_size = min_size;
    uint32 stream_doublings = 0;
    for (uint32 size = min_size; size <= max_size; size *= 2)
        ++stream_doublings;

    const uint64 bytes_copied = uint64(base_stream_size*2-1 - base_stream_size/2) * sz*2;

    float times[16];

    #if 0
    log_info( stderr, "    ordered work-queue, %u-byte payload", PAYLOAD*4u );
    for (uint32 m = 0; m < stream_doublings; ++m)
    {
        log_info_cont( stderr, "." );
        const uint32 n_stream_size = base_stream_size << m;

        FatWorkStream  stream( n_stream_size );
        FatWorkQueue   work_queue;

        // do one warm-up launch
        work_queue.consume( stream );
        cudaDeviceSynchronize();

        Timer timer;
        timer.start();

        for (uint32 i = 0; i < n_tests; ++i)
            work_queue.consume( stream );

        cudaDeviceSynchronize();
        timer.stop();

        times[m] = timer.seconds() / float(n_tests);
    }

    log_info_nl( stderr );
    log_info( stderr, "      runtime    (ms)  :" );
    for (uint32 i = 0; i < stream_doublings; ++i)
        log_info_cont( stderr, "  %7.2f", times[i] * 1.0e3f );
    log_info_nl( stderr );
    log_info( stderr, "      work-units (M/s) :" );
    for (uint32 i = 0; i < stream_doublings; ++i)
        log_info_cont( stderr, "  %7.2f", 1.0e-6f * (float((base_stream_size<<i)*2-1)/times[i]) );
    log_info_nl( stderr );
    log_info( stderr, "      bandwidth (GB/s) :" );
    for (uint32 i = 0; i < stream_doublings; ++i)
        log_info_cont( stderr, "  %7.2f", (float(bytes_copied<<i)/times[i]) / GB );
    log_info_nl( stderr );
    #endif

    log_info( stderr, "    multi-pass work-queue, %u-byte payload", PAYLOAD*4u );
    for (uint32 m = 0; m < stream_doublings; ++m)
    {
        log_info_cont( stderr, "." );
        const uint32 n_stream_size = base_stream_size << m;

        FatWorkStream  stream( n_stream_size );
        FatMKWorkQueue work_queue;

        // do one warm-up launch
        work_queue.consume( stream );
        cudaDeviceSynchronize();

        Timer timer;
        timer.start();

        for (uint32 i = 0; i < n_tests; ++i)
            work_queue.consume( stream );

        cudaDeviceSynchronize();
        timer.stop();

        times[m] = timer.seconds() / float(n_tests);
    }

    log_info_nl( stderr );
    log_info( stderr, "      runtime    (ms)  :" );
    for (uint32 i = 0; i < stream_doublings; ++i)
        log_info_cont( stderr, "  %7.2f", times[i] * 1.0e3f );
    log_info_nl( stderr );
    log_info( stderr, "      work-units (M/s) :" );
    for (uint32 i = 0; i < stream_doublings; ++i)
        log_info_cont( stderr, "  %7.2f", 1.0e-6f * (float((base_stream_size<<i)*2-1)/times[i]) );
    log_info_nl( stderr );
    log_info( stderr, "      bandwidth (GB/s) :" );
    for (uint32 i = 0; i < stream_doublings; ++i)
        log_info_cont( stderr, "  %7.2f", (float(bytes_copied<<i)/times[i]) / GB );
    log_info_nl( stderr );

    log_info( stderr, "    persistent-warps work-queue, %u-byte payload", PAYLOAD*4u );
    for (uint32 m = 0; m < stream_doublings; ++m)
    {
        log_info_cont( stderr, "." );
        const uint32 n_stream_size = base_stream_size << m;

        FatWorkStream  stream( n_stream_size );
        FatPWWorkQueue work_queue;

        // do one warm-up launch
        work_queue.consume( stream );
        cudaDeviceSynchronize();

        Timer timer;
        timer.start();

        for (uint32 i = 0; i < n_tests; ++i)
            work_queue.consume( stream );

        cudaDeviceSynchronize();
        timer.stop();

        times[m] = timer.seconds() / float(n_tests);
    }

    log_info_nl( stderr );
    log_info( stderr, "      runtime    (ms)  :" );
    for (uint32 i = 0; i < stream_doublings; ++i)
        log_info_cont( stderr, "  %7.2f", times[i] * 1.0e3f );
    log_info_nl( stderr );
    log_info( stderr, "      work-units (M/s) :" );
    for (uint32 i = 0; i < stream_doublings; ++i)
        log_info_cont( stderr, "  %7.2f", 1.0e-6f * (float((base_stream_size<<i)*2-1)/times[i]) );
    log_info_nl( stderr );
    log_info( stderr, "      bandwidth (GB/s) :" );
    for (uint32 i = 0; i < stream_doublings; ++i)
        log_info_cont( stderr, "  %7.2f", (float(bytes_copied<<i)/times[i]) / GB );
    log_info_nl( stderr );

    log_info( stderr, "    persistent-threads work-queue, %u-byte payload", PAYLOAD*4u );
    for (uint32 m = 0; m < stream_doublings; ++m)
    {
        log_info_cont( stderr, "." );
        const uint32 n_stream_size = base_stream_size << m;

        FatWorkStream  stream( n_stream_size );
        FatPTWorkQueue work_queue;

        // do one warm-up launch
        work_queue.consume( stream );
        cudaDeviceSynchronize();

        Timer timer;
        timer.start();

        for (uint32 i = 0; i < n_tests; ++i)
            work_queue.consume( stream );

        cudaDeviceSynchronize();
        timer.stop();

        times[m] = timer.seconds() / float(n_tests);
    }

    log_info_nl( stderr );
    log_info( stderr, "      runtime    (ms)  :" );
    for (uint32 i = 0; i < stream_doublings; ++i)
        log_info_cont( stderr, "  %7.2f", times[i] * 1.0e3f );
    log_info_nl( stderr );
    log_info( stderr, "      work-units (M/s) :" );
    for (uint32 i = 0; i < stream_doublings; ++i)
        log_info_cont( stderr, "  %7.2f", 1.0e-6f * (float((base_stream_size<<i)*2-1)/times[i]) );
    log_info_nl( stderr );
    log_info( stderr, "      bandwidth (GB/s) :" );
    for (uint32 i = 0; i < stream_doublings; ++i)
        log_info_cont( stderr, "  %7.2f", (float(bytes_copied<<i)/times[i]) / GB );
    log_info_nl( stderr );

    // alloc payloads storage, initialized to zero
    const uint32 capacity = 64*1024;
    thrust::device_vector<uint32> payloads( 2 * PAYLOAD * capacity, 0u );
    thrust::device_vector<uint32> payloads_pool( 1u + capacity, 0u );

    // fill the pool of payloads
    payloads_pool[capacity] = capacity;
    thrust::copy(
        thrust::make_counting_iterator( 0u ),
        thrust::make_counting_iterator( 0u ) + capacity,
        payloads_pool.begin() );

    typedef BenchmarkDynMemWorkUnit<PAYLOAD>                        DynMemWorkUnit;
    typedef BenchmarkWorkStream<DynMemWorkUnit>                     DynMemWorkStream;
    typedef WorkQueue<OrderedQueueTag,DynMemWorkUnit,BLOCKDIM>      DynMemWorkQueue;
    typedef WorkQueue<MultiPassQueueTag,DynMemWorkUnit,BLOCKDIM>    DynMemMKWorkQueue;

    #if 0
    log_info( stderr, "    ordered dyn-mem work-queue, %u-byte payload", PAYLOAD*4u );
    for (uint32 m = 0; m < stream_doublings; ++m)
    {
        log_info_cont( stderr, "." );
        const uint32 n_stream_size = base_stream_size << m;

        DynMemWorkStream  stream( n_stream_size, thrust::raw_pointer_cast( &payloads.front() ) );
        DynMemWorkQueue   work_queue;

        // setup the pool
        stream.set_pool(
            thrust::raw_pointer_cast( &payloads_pool.front() ) + capacity,
            thrust::raw_pointer_cast( &payloads_pool.front() ) );

        // do one warm-up launch
        work_queue.consume( stream );
        cudaDeviceSynchronize();

        Timer timer;
        timer.start();

        for (uint32 i = 0; i < n_tests; ++i)
            work_queue.consume( stream );

        cudaDeviceSynchronize();
        timer.stop();

        times[m] = timer.seconds() / float(n_tests);
    }

    log_info_nl( stderr );
    log_info( stderr, "      runtime    (ms)  :" );
    for (uint32 i = 0; i < stream_doublings; ++i)
        log_info_cont( stderr, "  %7.2f", times[i] * 1.0e3f );
    log_info_nl( stderr );
    log_info( stderr, "      work-units (M/s) :" );
    for (uint32 i = 0; i < stream_doublings; ++i)
        log_info_cont( stderr, "  %7.2f", 1.0e-6f * (float((base_stream_size<<i)*2-1)/times[i]) );
    log_info_nl( stderr );
    log_info( stderr, "      bandwidth (GB/s) :" );
    for (uint32 i = 0; i < stream_doublings; ++i)
        log_info_cont( stderr, "  %7.2f", (float(bytes_copied<<i)/times[i]) / GB );
    log_info_nl( stderr );
    #endif

    typedef BenchmarkStridedWorkUnit<PAYLOAD>                       StridedWorkUnit;
    typedef BenchmarkWorkStream<StridedWorkUnit>                    StridedWorkStream;
    typedef BenchmarkStridedWorkMover<PAYLOAD>                      StridedWorkMover;
    typedef WorkQueue<OrderedQueueTag,StridedWorkUnit,BLOCKDIM>     StridedWorkQueue;
    typedef WorkQueue<MultiPassQueueTag,StridedWorkUnit,BLOCKDIM>   StridedMKWorkQueue;

    #if 0
    log_info( stderr, "    ordered strided work-queue, %u-byte payload", PAYLOAD*4u );
    for (uint32 m = 0; m < stream_doublings; ++m)
    {
        log_info_cont( stderr, "." );
        const uint32 n_stream_size = base_stream_size << m;

        StridedWorkStream  stream( n_stream_size, thrust::raw_pointer_cast( &payloads.front() ), capacity );
        StridedWorkQueue   work_queue;

        // do one warm-up launch
        work_queue.consume( stream );
        cudaDeviceSynchronize();

        Timer timer;
        timer.start();

        for (uint32 i = 0; i < n_tests; ++i)
            work_queue.consume( stream, StridedWorkMover() );

        cudaDeviceSynchronize();
        timer.stop();

        times[m] = timer.seconds() / float(n_tests);
    }

    log_info_nl( stderr );
    log_info( stderr, "      runtime    (ms)  :" );
    for (uint32 i = 0; i < stream_doublings; ++i)
        log_info_cont( stderr, "  %7.2f", times[i] * 1.0e3f );
    log_info_nl( stderr );
    log_info( stderr, "      work-units (M/s) :" );
    for (uint32 i = 0; i < stream_doublings; ++i)
        log_info_cont( stderr, "  %7.2f", 1.0e-6f * (float((base_stream_size<<i)*2-1)/times[i]) );
    log_info_nl( stderr );
    log_info( stderr, "      bandwidth (GB/s) :" );
    for (uint32 i = 0; i < stream_doublings; ++i)
        log_info_cont( stderr, "  %7.2f", (float(bytes_copied<<i)/times[i]) / GB );
    log_info_nl( stderr );
    #endif

    log_info( stderr, "    multi-pass strided work-queue, %u-byte payload", PAYLOAD*4u );
    for (uint32 m = 0; m < stream_doublings; ++m)
    {
        log_info_cont( stderr, "." );
        const uint32 n_stream_size = base_stream_size << m;

        StridedWorkStream  stream( n_stream_size, thrust::raw_pointer_cast( &payloads.front() ), capacity );
        StridedMKWorkQueue work_queue;

        // do one warm-up launch
        work_queue.consume( stream );
        cudaDeviceSynchronize();

        Timer timer;
        timer.start();

        for (uint32 i = 0; i < n_tests; ++i)
            work_queue.consume( stream, StridedWorkMover() );

        cudaDeviceSynchronize();
        timer.stop();

        times[m] = timer.seconds() / float(n_tests);
    }

    log_info_nl( stderr );
    log_info( stderr, "      runtime    (ms)  :" );
    for (uint32 i = 0; i < stream_doublings; ++i)
        log_info_cont( stderr, "  %7.2f", times[i] * 1.0e3f );
    log_info_nl( stderr );
    log_info( stderr, "      work-units (M/s) :" );
    for (uint32 i = 0; i < stream_doublings; ++i)
        log_info_cont( stderr, "  %7.2f", 1.0e-6f * (float((base_stream_size<<i)*2-1)/times[i]) );
    log_info_nl( stderr );
    log_info( stderr, "      bandwidth (GB/s) :" );
    for (uint32 i = 0; i < stream_doublings; ++i)
        log_info_cont( stderr, "  %7.2f", (float(bytes_copied<<i)/times[i]) / GB );
    log_info_nl( stderr );
}

} // wqtest namespace

int work_queue_test(int argc, char* argv[])
{
    using namespace cuda;
    using namespace wqtest;

    log_info( stderr, "work_queue test... started\n" );

    uint32 n_tests  = 1;
    uint32 min_size = 512*1024;
    uint32 max_size = 1024*1024;
    uint32 max_payload = 32;

    for (int i = 0; i < argc; ++i)
    {
        if (strcmp( argv[i], "-n-tests" ) == 0)
            n_tests = atoi( argv[++i] );
        else if (strcmp( argv[i], "-min-size" ) == 0)
            min_size = atoi( argv[++i] ) * 1024;
        else if (strcmp( argv[i], "-max-size" ) == 0)
            min_size = atoi( argv[++i] ) * 1024;
        else if (strcmp( argv[i], "-max-payload" ) == 0)
            max_payload = atoi( argv[++i] );
    }

    NVBIO_VAR_UNUSED const uint32 BLOCKDIM = 128;

    typedef WorkQueue<OrderedQueueTag,TestWorkUnit,BLOCKDIM>            TestWorkQueue;
    typedef WorkQueue<MultiPassQueueTag,TestWorkUnit,BLOCKDIM>          TestMKWorkQueue;
    typedef WorkQueue<PersistentWarpsQueueTag,TestWorkUnit,BLOCKDIM>    TestPWWorkQueue;
    typedef WorkQueue<PersistentThreadsQueueTag,TestWorkUnit,BLOCKDIM>  TestPTWorkQueue;
    #if 0
    log_info( stderr, "  testing ordered work-queue:\n" );
    {
        const uint32 n_stream_size = 1024*1024;
        thrust::device_vector<uint32> output( n_stream_size*2, 0 );

        TestWorkStream stream( n_stream_size, thrust::raw_pointer_cast( &output.front() ) );
        TestWorkQueue  work_queue;

        // do one warm-up launch
        work_queue.consume( stream );
        cudaDeviceSynchronize();

        thrust::host_vector<uint32> h_output( output );
        for (uint32 i = 0; i < n_stream_size*2-1; ++i)
        {
            if (i != h_output[i])
            {
                log_error( stderr, "  found %u at position %u\n", h_output[i], i );
                return 1;
            }
        }
        log_info( stderr, "    correctness test passed\n" );
    }
    #endif
    log_info( stderr, "  testing multi-pass work-queue:\n" );
    {
        const uint32 n_stream_size = 1024*1024;
        thrust::device_vector<uint32> output( n_stream_size*2, 0 );

        TestWorkStream  stream( n_stream_size, thrust::raw_pointer_cast( &output.front() ) );
        TestMKWorkQueue work_queue;

        // do one warm-up launch
        work_queue.consume( stream );
        cudaDeviceSynchronize();

        thrust::host_vector<uint32> h_output( output );
        for (uint32 i = 0; i < n_stream_size*2-1; ++i)
        {
            if (i != h_output[i])
            {
                log_error( stderr, "  found %u at position %u\n", h_output[i], i );
                return 1;
            }
        }
        log_info( stderr, "    correctness test passed\n" );
    }
    log_info( stderr, "  testing persistent-warps work-queue:\n" );
    {
        const uint32 n_stream_size = 1024*1024;
        thrust::device_vector<uint32> output( n_stream_size*2, 0 );

        TestWorkStream  stream( n_stream_size, thrust::raw_pointer_cast( &output.front() ) );
        TestPWWorkQueue work_queue;

        // do one warm-up launch
        work_queue.consume( stream );
        cudaDeviceSynchronize();

        thrust::host_vector<uint32> h_output( output );
        for (uint32 i = 0; i < n_stream_size*2-1; ++i)
        {
            if (i != h_output[i])
            {
                log_error( stderr, "  found %u at position %u\n", h_output[i], i );
                return 1;
            }
        }
        log_info( stderr, "    correctness test passed\n" );
    }
    log_info( stderr, "  testing persistent-threads work-queue:\n" );
    {
        const uint32 n_stream_size = 1024*1024;
        thrust::device_vector<uint32> output( n_stream_size*2, 0 );

        TestWorkStream  stream( n_stream_size, thrust::raw_pointer_cast( &output.front() ) );
        TestPTWorkQueue work_queue;

        // do one warm-up launch
        work_queue.consume( stream );
        cudaDeviceSynchronize();

        thrust::host_vector<uint32> h_output( output );
        for (uint32 i = 0; i < n_stream_size*2-1; ++i)
        {
            if (i != h_output[i])
            {
                log_error( stderr, "  found %u at position %u\n", h_output[i], i );
                return 1;
            }
        }
        log_info( stderr, "    correctness test passed\n" );
    }
    log_info( stderr, "  benchmarking... started\n" );
    benchmark<1,BLOCKDIM>( n_tests, min_size, max_size );
    if (max_payload >= 32)
        benchmark<32,BLOCKDIM>( n_tests, min_size, max_size );
    if (max_payload >= 64)
        benchmark<64,BLOCKDIM>( n_tests, min_size, max_size );
    if (max_payload >= 128)
        benchmark<128,BLOCKDIM>( n_tests, min_size, max_size );
    if (max_payload >= 256)
        benchmark<256,BLOCKDIM>( n_tests, min_size, max_size );
    log_info( stderr, "  benchmarking... done\n" );
    log_info( stderr, "work_queue test... done\n" );
    return 0;
}

} // namespace nvbio
