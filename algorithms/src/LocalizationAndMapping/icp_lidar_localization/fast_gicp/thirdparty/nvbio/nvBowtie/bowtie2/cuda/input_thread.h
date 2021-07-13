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

#include <nvBowtie/bowtie2/cuda/defs.h>
#include <nvBowtie/bowtie2/cuda/stats.h>
#include <nvbio/basic/threads.h>
#include <nvbio/basic/timer.h>
#include <nvbio/io/sequence/sequence.h>
#include <stack>
#include <deque>

namespace nvbio {
namespace bowtie2 {
namespace cuda {

//
// A class implementing a background input thread, providing
// a set of input read-streams which are read in parallel to the
// operations performed by the main thread.
//

struct InputThreadSE : public Thread<InputThreadSE>
{
    static const uint32 BUFFERS = 4;

    InputThreadSE(io::SequenceDataStream* read_data_stream, Stats& _stats, const uint32 batch_size, const uint32 read_length) :
        m_read_data_stream( read_data_stream ), m_stats( _stats ), m_batch_size( batch_size ), m_read_length( read_length ), m_set(0), m_reads(0), m_done(false)
    {}

    void run();

    // get a batch
    //
    io::SequenceDataHost* next(uint32* offset = NULL);

    // release a batch
    //
    void release(io::SequenceDataHost* read_data);

    // return the batch size
    //
    uint32 batch_size() const { return m_batch_size; }

private:
    io::SequenceDataStream* m_read_data_stream;
    Stats&                  m_stats;
    uint32                  m_batch_size;
    uint32                  m_read_length;
    uint32                  m_set;
    uint32                  m_reads;

    io::SequenceDataHost                 m_read_data_storage[BUFFERS];

    Mutex                                m_free_pool_lock;
    std::stack<io::SequenceDataHost*>    m_free_pool;

    Mutex                                m_ready_pool_lock;
    std::deque<io::SequenceDataHost*>    m_ready_pool;
    std::deque<uint32>                   m_ready_poolN;

    volatile bool m_done;
};

//
// A class implementing a background input thread, providing
// a set of input read-streams which are read in parallel to the
// operations performed by the main thread.
//

struct InputThreadPE : public Thread<InputThreadPE>
{
    static const uint32 BUFFERS = 4;

    InputThreadPE(io::SequenceDataStream* read_data_stream1, io::SequenceDataStream* read_data_stream2, Stats& _stats, const uint32 batch_size, const uint32 read_length) :
        m_read_data_stream1( read_data_stream1 ), m_read_data_stream2( read_data_stream2 ), m_stats( _stats ), m_batch_size( batch_size ), m_read_length( read_length ), m_set(0), m_reads(0), m_done(false)
    {}

    void run();

    // get a batch
    //
    std::pair<io::SequenceDataHost*,io::SequenceDataHost*> next(uint32* offset = NULL);

    // release a batch
    //
    void release(std::pair<io::SequenceDataHost*,io::SequenceDataHost*> read_data);

    // return the batch size
    //
    uint32 batch_size() const { return m_batch_size; }

private:
    io::SequenceDataStream* m_read_data_stream1;
    io::SequenceDataStream* m_read_data_stream2;
    Stats&                  m_stats;
    uint32                  m_batch_size;
    uint32                  m_read_length;
    uint32                  m_set;
    uint32                  m_reads;

    io::SequenceDataHost    m_read_data_storage1[BUFFERS];
    io::SequenceDataHost    m_read_data_storage2[BUFFERS];

    Mutex                                m_free_pool_lock;
    std::stack<io::SequenceDataHost*>    m_free_pool1;
    std::stack<io::SequenceDataHost*>    m_free_pool2;

    Mutex                                m_ready_pool_lock;
    std::deque<io::SequenceDataHost*>    m_ready_pool1;
    std::deque<io::SequenceDataHost*>    m_ready_pool2;
    std::deque<uint32>                   m_ready_poolN;

    volatile bool m_done;
};

} // namespace cuda
} // namespace bowtie2
} // namespace nvbio
