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

#include <thrust/host_vector.h>
#include <thrust/device_vector.h>
#include <nvbio/basic/types.h>
#include <nvbio/basic/vector.h>
#include <cuda_runtime.h>

namespace nvbio {
namespace cuda {

template <typename T>
struct host_device_buffer
{
    virtual ~host_device_buffer() {}
    virtual uint32 size() const { return 0u; }
    virtual void resize(const uint32 size) {}
    virtual void fill(const T val) {}
    virtual void to_host() {}
    virtual const T* host_ptr()   const { return NULL; }
    virtual const T* device_ptr() const { return NULL; }
    virtual T* host_ptr()   { return NULL; }
    virtual T* device_ptr() { return NULL; }
};

template <typename T>
struct host_device_buffer_zero_copy : host_device_buffer<T>
{
     host_device_buffer_zero_copy() {}
     host_device_buffer_zero_copy(const uint32 size) { resize( size ); }
    ~host_device_buffer_zero_copy()
    {
        // unmap the memory
        cudaHostUnregister( host_ptr() );
    }

    uint32 size() const { return m_hvec.size(); }

    void resize(const uint32 size)
    {
        // resize the vector
        m_hvec.resize( size );

        // lock it and map it to the device
        cudaError_t error = cudaHostRegister( &m_hvec[0], sizeof(T) * size, cudaHostRegisterMapped );
        if (error)
        {
            log_error(stderr, "host_device_buffer_zero_copy::resize(): failed locking %llu bytes\n  %s\n", sizeof(T)*size, cudaGetErrorString(error));
            throw error;
        }
    }

    void fill(const T val)
    {
        thrust::fill( m_hvec.begin(), m_hvec.end(), val );
    }

    void to_host() {}

    virtual const T* host_ptr() const { return thrust::raw_pointer_cast( &m_hvec.front() ); }
    virtual const T* device_ptr() const
    {
        // get the mapped device pointer
        T* ptr_d;
        cudaError_t error = cudaHostGetDevicePointer( &ptr_d, const_cast<T*>(host_ptr()), 0u );
        if (error)
        {
            log_error(stderr, "host_device_buffer_zero_copy::device_ptr(): failed mapping %llu bytes\n  %s\n", sizeof(T)*m_hvec.size(), cudaGetErrorString(error));
            throw error;
        }
        return ptr_d;
    }
    virtual T* host_ptr() { return thrust::raw_pointer_cast( &m_hvec.front() ); }
    virtual T* device_ptr()
    {
        // get the mapped device pointer
        T* ptr_d;
        cudaError_t error = cudaHostGetDevicePointer( &ptr_d, host_ptr(), 0u );
        if (error)
        {
            log_error(stderr, "host_device_buffer_zero_copy::device_ptr(): failed mapping %llu bytes\n  %s\n", sizeof(T)*m_hvec.size(), cudaGetErrorString(error));
            throw error;
        }
        return ptr_d;
    }

private:
    thrust::host_vector<T>  m_hvec;
};

template <typename T>
struct host_device_buffer_sync : host_device_buffer<T>
{
     host_device_buffer_sync() {}
     host_device_buffer_sync(const uint32 size) { resize( size ); }
    ~host_device_buffer_sync() {}

    uint32 size() const { return m_hvec.size(); }

    void resize(const uint32 size)
    {
        m_hvec.resize( size );
        m_dvec.resize( size );
    }

    void fill(const T val)
    {
        thrust::fill( m_dvec.begin(), m_dvec.end(), val );
    }

    void to_host()
    {
        thrust_copy_vector(m_hvec, m_dvec);
    }

    virtual const T* host_ptr()   const { return thrust::raw_pointer_cast( &m_hvec.front() ); }
    virtual const T* device_ptr() const { return thrust::raw_pointer_cast( &m_dvec.front() ); }

    virtual T* host_ptr()   { return thrust::raw_pointer_cast( &m_hvec.front() ); }
    virtual T* device_ptr() { return thrust::raw_pointer_cast( &m_dvec.front() ); }

private:
    thrust::host_vector<T>   m_hvec;
    thrust::device_vector<T> m_dvec;
};

template <typename T>
void copy(
    const thrust::device_vector<T>& dvec,
          thrust::host_vector<T>&   hvec)
{
    hvec = dvec;
}

template <typename T>
void copy(
    host_device_buffer<T>&    dvec,
    thrust::host_vector<T>&   hvec)
{
    dvec.to_host();
    hvec.resize( dvec.size() );
    thrust::copy(
        dvec.host_ptr(),
        dvec.host_ptr() + dvec.size(),
        hvec.begin() );
}

template <typename T>
const T* device_pointer(const thrust::device_vector<T>& dvec)
{
    return thrust::raw_pointer_cast( &dvec.front() );
}

template <typename T>
T* device_pointer(thrust::device_vector<T>& dvec)
{
    return thrust::raw_pointer_cast( &dvec.front() );
}

template <typename T>
const T* device_pointer(const host_device_buffer<T>& dvec)
{
    return dvec.device_ptr();
}

template <typename T>
T* device_pointer(host_device_buffer<T>& dvec)
{
    return dvec.device_ptr();
}

} // namespace cuda
} // namespace nvbio
