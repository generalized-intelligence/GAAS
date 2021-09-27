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

// pipeline_inl.h
//

#pragma once

namespace nvbio {

namespace priv {

struct PipelineThreadBase : public Thread<PipelineThreadBase>
{
    /// empty constructor
    ///
    PipelineThreadBase() : m_clients(0), m_id(0) {}

    /// virtual destructor
    ///
    virtual ~PipelineThreadBase() {}

    /// run this thread
    ///
    virtual void run() {}

    /// a client method to obtain the given object; once the client has
    /// finished using the object, it is responsible to call the release()
    /// method to signal completion.
    ///
    virtual void* fetch(const uint32 i) { return NULL; }

    /// a client method to release the latest batch
    ///
    virtual void release(const uint32 i) {}

    /// add a dependency on another node
    ///
    void add_dependency(PipelineThreadBase* dep) { m_deps.push_back( dep ); }

    /// add a client
    ///
    void add_client() { ++m_clients; }

    /// set the id
    ///
    void set_id(const uint32 id) { m_id = id; }

    std::vector<PipelineThreadBase*> m_deps;
    uint32                           m_clients;
    uint32                           m_id;
};

///
/// A class implementing a pipeline sink thread
///
/// \tparam SinkType   a class implementing the actual pipeline stage,
///                    must define the following interface:
///
///\code
/// interface Sink
/// {
///     typedef ... argument_type;
///
///     bool process(argument_type* src);
/// };
///\endcode
///
template <typename SinkType>
struct PipelineSinkThread : public PipelineThreadBase
{
    typedef typename SinkType::argument_type   argument_type;

    /// constructor
    ///
    PipelineSinkThread(SinkType* stage) : m_stage( stage ), m_counter(0)
    {}

    /// run the thread
    ///
    void run()
    {
        while (fill()) { yield(); }
    }

    /// fill the next batch
    ///
    bool fill()
    {
        // fetch the inputs from all sources
        PipelineContext context;
        for (uint32 i = 0; i < (uint32)m_deps.size(); ++i)
        {
            context.in[i] = m_deps[i]->fetch( m_counter );

            if (context.in[i] == NULL)
            {
                // release all inputs
                for (uint32 j = 0; j < i; ++j)
                    m_deps[j]->release( m_counter );

                // signal completion
                return false;
            }
        }

        // process
        bool ret = false;
        {
            ScopedTimer<float> timer( &m_time );

            // execute this stage
            ret = m_stage->process( context );
        }

        // release all inputs
        for (uint32 i = 0; i < (uint32)m_deps.size(); ++i)
            m_deps[i]->release( m_counter );

        // advance the counter
        m_counter++;

        return ret;
    }

    SinkType*           m_stage;
    uint32              m_counter;
    float               m_time;
};

///
/// A class implementing a multiple-buffered CPU pipeline thread
///
/// \tparam StageType   a class implementing the actual pipeline stage,
///                     must define the following interface:
///
///\code
/// interface Stage
/// {
///     typedef ... argument_type;
///     typedef ... return_type;
///
///     bool process(argument_type* src, return_type* dst);
/// };
///\endcode
///
template <typename StageType>
struct PipelineStageThread : public PipelineThreadBase
{
    static const uint32 EMPTY_SLOT = uint32(-1);

    typedef typename StageType::argument_type   argument_type;
    typedef typename StageType::return_type     return_type;

    /// constructor
    ///
    PipelineStageThread(StageType* stage, const uint32 buffers) : m_stage( stage ), m_buffers( buffers )
    {
        m_data.resize( m_buffers );

        for (uint32 i = 0; i < m_buffers; ++i)
        {
            m_data_ptr[i] = (return_type*)EMPTY_SLOT;
            m_data_id[i] = EMPTY_SLOT;
        }

        m_counter = 0;
    }

    /// run the thread
    ///
    void run()
    {
        while (fill()) { yield(); }
    }

    /// fill the next batch
    ///
    bool fill()
    {
        const uint32 slot = m_counter & (m_buffers-1);

        log_debug(stderr, "    [%u] polling for writing [%u:%u]... started\n", m_id, m_counter, slot);
        // poll until the set is done reading & ready to be reused
        while (m_data_id[ slot ] != EMPTY_SLOT)
        {
            yield();
        }
        log_debug(stderr, "    [%u] polling for writing [%u:%u]... done\n", m_id, m_counter, slot);

        PipelineContext context;

        // set the output
        context.out = &m_data[ slot ];

        // fetch the inputs from all sources
        for (uint32 i = 0; i < (uint32)m_deps.size(); ++i)
        {
            context.in[i] = m_deps[i]->fetch( m_counter );

            if (context.in[i] == NULL)
            {
                // release all inputs
                for (uint32 j = 0; j < i; ++j)
                    m_deps[j]->release( m_counter );

                // mark this as an invalid entry & return
                m_data_ptr[ slot ] = NULL;

                // make sure the other threads see this before the id is set
                host_release_fence();

                m_data_id[ slot ]  = m_counter;
                return false;
            }
        }

        bool ret = false;
        {
            ScopedTimer<float> timer( &m_time );

            // execute this stage
            ret = m_stage->process( context );
        }

        // release all inputs
        for (uint32 i = 0; i < (uint32)m_deps.size(); ++i)
            m_deps[i]->release( m_counter );

        if (ret)
        {
            // set the reference counter
            m_count[ slot ] = m_clients-1u;

            // mark the set as done
            m_data_ptr[ slot ] = &m_data[ slot ];

            // make sure the other threads see the reference count before the output is set
            host_release_fence();

            // mark the set as done
            m_data_id[ slot ] = m_counter;
        }
        else
        {
            // mark this as an invalid entry
            m_data_ptr[ slot ] = NULL;

            // make sure the other threads see this before the id is set
            host_release_fence();

            m_data_id[ slot ] = m_counter;
            return false;
        }

        // switch to the next set
        ++m_counter;
        return true;
    }

    /// a client method to obtain the next loaded batch; once the client has
    /// finished using the sequence, it is responsible to call the release()
    /// method to signal completion
    /// NOTE: this function will poll until the next batch is available, or
    /// return NULL if finished
    ///
    void* fetch(const uint32 i)
    {
        const uint32 slot = i & (m_buffers-1);

        log_debug(stderr, "    [%u] polling for reading [%u:%u]... started\n", m_id, i, slot);
        // poll until the set is ready to be consumed
        while (m_data_id[ slot ] != i)
        {
            yield();
        }

        // make sure the other writes are seen
        host_acquire_fence();

        log_debug(stderr, "    [%u] polling for reading [%u:%u]... done\n", m_id, i, slot);

        return (void*)m_data_ptr[ slot ];
    }

    /// a client method to release a given input
    ///
    void release(const uint32 i)
    {
        const uint32 slot = i & (m_buffers-1);

        const uint32 ref = atomic_sub( (uint32*)m_count + slot, 1u );
        if (ref == 0)
        {
            log_debug(stderr, "    [%u] release [%u:%u]\n", m_id, i, slot);
            // mark this set as free / ready to be written
            m_data_ptr[ slot ] = (return_type*)EMPTY_SLOT;
            m_data_id[ slot ]  = EMPTY_SLOT;

            // make sure the other threads see this change
            host_release_fence();
        }
    }

    StageType*                      m_stage;
    uint32                          m_buffers;
    std::vector<return_type>        m_data;
    return_type* volatile           m_data_ptr[64];
    uint32       volatile           m_data_id[64];
    uint32       volatile           m_count[64];
    volatile uint32                 m_counter;
    float                           m_time;
};

} // namespace priv

// run the pipeline to completion
//
inline Pipeline::~Pipeline()
{
    // start all threads
    for (size_t i = 0; i < m_stages.size(); ++i)
        delete m_stages[i];
}

// append a new pipeline stage
//
template <typename StageType>
uint32 Pipeline::append_stage(StageType* stage, const uint32 buffers)
{
    // create a new stage-thread
    priv::PipelineStageThread<StageType>* thread = new priv::PipelineStageThread<StageType>( stage, buffers );

    // append it
    m_stages.push_back( thread );

    const uint32 id = (uint32)m_stages.size()-1;
    thread->set_id( id );
    return id;
}

// append the pipeline sink
//
template <typename SinkType>
uint32 Pipeline::append_sink(SinkType* sink)
{
    // create a new stage-thread
    priv::PipelineSinkThread<SinkType>* thread = new priv::PipelineSinkThread<SinkType>( sink );

    // append it
    m_stages.push_back( thread );

    const uint32 id = (uint32)m_stages.size()-1;
    thread->set_id( id );
    return id;
}

// add a dependency
//
inline void Pipeline::add_dependency(const uint32 in, const uint32 out)
{
    m_stages[out]->add_dependency( m_stages[in] );
    m_stages[in]->add_client();
}

// run the pipeline to completion
//
inline void Pipeline::run()
{
    // start all threads
    for (size_t i = 0; i < m_stages.size(); ++i)
        m_stages[i]->create();

    // and join them
    for (size_t i = 0; i < m_stages.size(); ++i)
        m_stages[i]->join();
}

} // namespace nvbio
