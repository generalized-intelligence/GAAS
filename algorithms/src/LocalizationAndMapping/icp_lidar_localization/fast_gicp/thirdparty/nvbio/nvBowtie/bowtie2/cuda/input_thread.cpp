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

#include <nvBowtie/bowtie2/cuda/input_thread.h>
#include <nvBowtie/bowtie2/cuda/defs.h>
#include <nvBowtie/bowtie2/cuda/params.h>
#include <nvBowtie/bowtie2/cuda/stats.h>
#include <nvbio/io/output/output_utils.h>
#include <nvbio/basic/threads.h>
#include <nvbio/basic/atomics.h>
#include <nvbio/basic/timer.h>
#include <nvbio/basic/exceptions.h>

namespace nvbio {
namespace bowtie2 {
namespace cuda {

void InputThreadSE::run()
{
    log_verbose( stderr, "starting background input thread\n" );

    try
    {
        // fill up the free pool
        {
            ScopedLock lock( &m_free_pool_lock );
            for (uint32 i = 0; i < BUFFERS; ++i)
                m_free_pool.push( &m_read_data_storage[i] );
        }

        while (1u)
        {
            io::SequenceDataHost* read_data = NULL;

            // loop until the free pool gets filled
            while (read_data == NULL)
            {
                ScopedLock lock( &m_free_pool_lock );

                if (m_free_pool.empty() == false)
                {
                    read_data = m_free_pool.top();
                    m_free_pool.pop();
                }

                yield();
            }

            log_debug( stderr, "  reading input batch %u\n", m_set );

            Timer timer;
            timer.start();

            const int ret = io::next( DNA_N, read_data, m_read_data_stream, m_batch_size, m_batch_size*m_read_length );

            timer.stop();

            if (ret)
            {
                ScopedLock lock( &m_ready_pool_lock );
                m_ready_pool.push_front( read_data );
                m_ready_poolN.push_front( m_reads );

                m_reads += read_data->size();

                m_stats.read_io.add( read_data->size(), timer.seconds() );
            }
            else
            {
                // stop the thread
                m_done = true;
                host_release_fence();
                break;
            }

            // switch to the next set
            ++m_set;
        }
    }
    catch (nvbio::bad_alloc e)
    {
        log_error(stderr, "caught a nvbio::bad_alloc exception:\n");
        log_error(stderr, "  %s\n", e.what());
        exit(1);
    }
    catch (nvbio::logic_error e)
    {
        log_error(stderr, "caught a nvbio::logic_error exception:\n");
        log_error(stderr, "  %s\n", e.what());
        exit(1);
    }
    catch (nvbio::runtime_error e)
    {
        log_error(stderr, "caught a nvbio::runtime_error exception:\n");
        log_error(stderr, "  %s\n", e.what());
        exit(1);
    }
    catch (std::bad_alloc e)
    {
        log_error(stderr, "caught a std::bad_alloc exception:\n");
        log_error(stderr, "  %s\n", e.what());
        exit(1);
    }
    catch (std::logic_error e)
    {
        log_error(stderr, "caught a std::logic_error exception:\n");
        log_error(stderr, "  %s\n", e.what());
        exit(1);
    }
    catch (std::runtime_error e)
    {
        log_error(stderr, "caught a std::runtime_error exception:\n");
        log_error(stderr, "  %s\n", e.what());
        exit(1);
    }
    catch (...)
    {
        log_error(stderr, "caught an unknown exception!\n");
        exit(1);
    }
}

// get a batch
//
io::SequenceDataHost* InputThreadSE::next(uint32* offset)
{
    // loop until the ready pool gets filled
    while (1)
    {
        ScopedLock lock( &m_ready_pool_lock );

        if (m_ready_pool.empty() == false)
        {
            // pop from the ready pool
            io::SequenceDataHost* read_data = m_ready_pool.back();
            m_ready_pool.pop_back();
            if (offset) *offset = m_ready_poolN.back();
                                  m_ready_poolN.pop_back();
            return read_data;
        }
        else if (m_done)
        {
            if (offset) *offset = m_reads;
            return NULL;
        }

        yield();
    }
}

// release a batch
//
void InputThreadSE::release(io::SequenceDataHost* read_data)
{
    // push back to the free pool
    ScopedLock lock( &m_free_pool_lock );
    m_free_pool.push( read_data );
}

void InputThreadPE::run()
{
    log_verbose( stderr, "starting background paired-end input thread\n" );

    try
    {
        // fill up the free pool
        {
            ScopedLock lock( &m_free_pool_lock );
            for (uint32 i = 0; i < BUFFERS; ++i)
            {
                m_free_pool1.push( &m_read_data_storage1[i] );
                m_free_pool2.push( &m_read_data_storage2[i] );
            }
        }

        while (1u)
        {
            io::SequenceDataHost* read_data1 = NULL;
            io::SequenceDataHost* read_data2 = NULL;

            // loop until the free pool gets filled
            while (read_data1 == NULL || read_data2 == NULL)
            {
                ScopedLock lock( &m_free_pool_lock );

                if (m_free_pool1.empty() == false &&
                    m_free_pool2.empty() == false)
                {
                    read_data1 = m_free_pool1.top(); m_free_pool1.pop();
                    read_data2 = m_free_pool2.top(); m_free_pool2.pop();
                }

                yield();
            }

            log_debug( stderr, "  reading input batch %u\n", m_set );

            Timer timer;
            timer.start();

            const int ret1 = io::next( DNA_N, read_data1, m_read_data_stream1, m_batch_size, m_batch_size*m_read_length );
            const int ret2 = io::next( DNA_N, read_data2, m_read_data_stream2, read_data1->size() );

            timer.stop();

            if (ret1 && ret2)
            {
                ScopedLock lock( &m_ready_pool_lock );
                m_ready_pool1.push_front( read_data1 );
                m_ready_pool2.push_front( read_data2 );
                m_ready_poolN.push_front( m_reads );

                m_reads += read_data1->size();

                m_stats.read_io.add( read_data1->size(), timer.seconds() );
            }
            else
            {
                // stop the thread
                m_done = true;
                host_release_fence();
                break;
            }

            // switch to the next set
            ++m_set;
        }
    }
    catch (nvbio::bad_alloc e)
    {
        log_error(stderr, "caught a nvbio::bad_alloc exception:\n");
        log_error(stderr, "  %s\n", e.what());
        exit(1);
    }
    catch (nvbio::logic_error e)
    {
        log_error(stderr, "caught a nvbio::logic_error exception:\n");
        log_error(stderr, "  %s\n", e.what());
        exit(1);
    }
    catch (nvbio::runtime_error e)
    {
        log_error(stderr, "caught a nvbio::runtime_error exception:\n");
        log_error(stderr, "  %s\n", e.what());
        exit(1);
    }
    catch (std::bad_alloc e)
    {
        log_error(stderr, "caught a std::bad_alloc exception:\n");
        log_error(stderr, "  %s\n", e.what());
        exit(1);
    }
    catch (std::logic_error e)
    {
        log_error(stderr, "caught a std::logic_error exception:\n");
        log_error(stderr, "  %s\n", e.what());
        exit(1);
    }
    catch (std::runtime_error e)
    {
        log_error(stderr, "caught a std::runtime_error exception:\n");
        log_error(stderr, "  %s\n", e.what());
        exit(1);
    }
    catch (...)
    {
        log_error(stderr, "caught an unknown exception!\n");
        exit(1);
    }
}

// get a batch
//
std::pair<io::SequenceDataHost*,io::SequenceDataHost*> InputThreadPE::next(uint32* offset)
{
    // loop until the ready pool gets filled
    while (1)
    {
        ScopedLock lock( &m_ready_pool_lock );

        if (m_ready_pool1.empty() == false &&
            m_ready_pool2.empty() == false)
        {
            // pop from the ready pool
            std::pair<io::SequenceDataHost*,io::SequenceDataHost*> read_data;
            read_data.first  = m_ready_pool1.back(); m_ready_pool1.pop_back();
            read_data.second = m_ready_pool2.back(); m_ready_pool2.pop_back();
            if (offset) *offset = m_ready_poolN.back();
                                  m_ready_poolN.pop_back();
            return read_data;
        }
        else if (m_done)
        {
            if (offset) *offset = m_reads;
            return std::pair<io::SequenceDataHost*,io::SequenceDataHost*>( NULL, NULL );
        }

        yield();

    }
}

// release a batch
//
void InputThreadPE::release(std::pair<io::SequenceDataHost*,io::SequenceDataHost*> read_data)
{
    // push back to the free pool
    ScopedLock lock( &m_free_pool_lock );
    m_free_pool1.push( read_data.first );
    m_free_pool2.push( read_data.second );
}

} // namespace cuda
} // namespace bowtie2
} // namespace nvbio
