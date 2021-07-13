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

#include <nvbio/basic/threads.h>

#ifdef WIN32
#include "windows.h"
#else
#include <pthread.h>
#include <unistd.h>
#include <string>
using namespace std;
#endif

#include <string.h>

namespace nvbio {

#if !defined(WIN32) && defined(PLATFORM_X86)
void cpuID(unsigned i, unsigned regs[4])
{
  asm volatile
    ("cpuid" : "=a" (regs[0]), "=b" (regs[1]), "=c" (regs[2]), "=d" (regs[3])
     : "a" (i), "c" (0));
  // ECX is set to zero for CPUID function 4
}
#endif

uint32 num_physical_cores()
{
  #ifdef WIN32
    SYSTEM_LOGICAL_PROCESSOR_INFORMATION buffer[32];
    DWORD returnLength = 32 * sizeof(SYSTEM_LOGICAL_PROCESSOR_INFORMATION);

    GetLogicalProcessorInformation(
        buffer,
        &returnLength );

    SYSTEM_LOGICAL_PROCESSOR_INFORMATION* ptr = buffer;

    uint32 processorCoreCount = 0;

    for (uint32 byteOffset = 0;
        byteOffset + sizeof(SYSTEM_LOGICAL_PROCESSOR_INFORMATION) <= returnLength;
        byteOffset += sizeof(SYSTEM_LOGICAL_PROCESSOR_INFORMATION))
    {
        switch (ptr->Relationship) 
        {
        case RelationProcessorCore:
            processorCoreCount++;
            break;
        }

        ptr++;
    }
    return processorCoreCount;
  #elif defined(PLATFORM_X86)
    unsigned regs[4];

    // Get vendor
    char vendor[12];
    cpuID(0, regs);
    memcpy(&vendor[0], &regs[1], sizeof(unsigned)); // EBX
    memcpy(&vendor[1], &regs[3], sizeof(unsigned)); // EDX
    memcpy(&vendor[2], &regs[2], sizeof(unsigned)); // ECX
    string cpuVendor = string(vendor, 12);

    // Get CPU features
    cpuID(1, regs);
//    unsigned cpuFeatures = regs[3]; // EDX

    // Logical core count per CPU
    cpuID(1, regs);
    unsigned logical = (regs[1] >> 16) & 0xff; // EBX[23:16]
    unsigned cores = logical;

    if (cpuVendor == "GenuineIntel")
    {
        // Get DCP cache info
        cpuID(4, regs);
        cores = ((regs[0] >> 26) & 0x3f) + 1; // EAX[31:26] + 1
    }
    else if (cpuVendor == "AuthenticAMD")
    {
        // Get NC: Number of CPU cores - 1
        cpuID(0x80000008, regs);
        cores = ((unsigned)(regs[2] & 0xff)) + 1; // ECX[7:0] + 1
    }
    return cores;
  #else
    return 0;
  #endif
}
uint32 num_logical_cores()
{
  #ifdef WIN32
    SYSTEM_INFO sysinfo;
    GetSystemInfo( &sysinfo );
    return uint32( sysinfo.dwNumberOfProcessors );
  #else
    return uint32( sysconf( _SC_NPROCESSORS_ONLN ) );
  #endif
}


#if NOTHREADS

struct ThreadBase::Impl
{
};

// constructor
ThreadBase::ThreadBase() : m_id( 0u ), m_impl( new Impl )
{
}

// destructor
ThreadBase::~ThreadBase()
{
}

// create the thread
void ThreadBase::create(void* (*func)(void*), void* arg)
{
    func( arg );
}
// join the thread
void ThreadBase::join()
{
}

/// Mutex class
struct Mutex::Impl
{
};

Mutex::Mutex() : m_impl( new Impl )
{
}
Mutex::~Mutex()
{
}

void Mutex::lock()   {}
void Mutex::unlock() {}

void yield() {}

#elif defined(WIN32)

namespace {

struct Func
{
    typedef void* (*FuncType)(void*);

    FuncType    m_func;
    void*       m_arg;
};
DWORD __stdcall CallFunc(void* arg)
{
    Func* func = reinterpret_cast<Func*>(arg);
    func->m_func( func->m_arg );
    return DWORD(0);
}

};

struct ThreadBase::Impl
{
     Impl() : m_handle(0), m_tid(0) {}
    ~Impl()
    {
        if (m_handle)
            CloseHandle( m_handle );
    }

    HANDLE m_handle;
    DWORD  m_tid;     // thread id
    Func   m_func_arg;
};

// constructor
ThreadBase::ThreadBase() : m_id( 0u ), m_impl( new Impl )
{
}

// destructor
ThreadBase::~ThreadBase()
{
}

// create the thread
void ThreadBase::create(void* (*func)(void*), void* arg)
{
    // fill in the function wrapper
    m_impl->m_func_arg.m_func = func;
    m_impl->m_func_arg.m_arg  = arg;

    // create the thread in a suspended state
    m_impl->m_handle = CreateThread(
        0, // Security attributes
        0, // Stack size
        CallFunc,
        &m_impl->m_func_arg,
        CREATE_SUSPENDED,
        &m_impl->m_tid);

    // and run it
    ResumeThread( m_impl->m_handle );
}
// join the thread
void ThreadBase::join()
{
    WaitForSingleObject( m_impl->m_handle, INFINITE );
}

/// Mutex class
struct Mutex::Impl
{
     Impl() { InitializeCriticalSection( &m_mutex ); }
    ~Impl() { DeleteCriticalSection( &m_mutex ); }

    CRITICAL_SECTION m_mutex;
};

Mutex::Mutex() : m_impl( new Impl )
{
}
Mutex::~Mutex()
{
}

void Mutex::lock()   { EnterCriticalSection( &m_impl->m_mutex ); }
void Mutex::unlock() { LeaveCriticalSection( &m_impl->m_mutex ); }

void yield() {}

#else

struct ThreadBase::Impl
{
    pthread_t m_thread;
};

// constructor
ThreadBase::ThreadBase() : m_id( 0u ), m_impl( new Impl )
{
}

// destructor
ThreadBase::~ThreadBase()
{
}

// create the thread
void ThreadBase::create(void* (*func)(void*), void* arg)
{
    pthread_create( &m_impl->m_thread, NULL, func, arg );
}
// join the thread
void ThreadBase::join()
{
    pthread_join( m_impl->m_thread, NULL );
}

/// Mutex class
struct Mutex::Impl
{
     Impl() { pthread_mutex_init( &m_mutex, NULL ); }
    ~Impl() { pthread_mutex_destroy( &m_mutex ); }

    pthread_mutex_t m_mutex;
};

Mutex::Mutex() : m_impl( new Impl )
{
}
Mutex::~Mutex()
{
}

void Mutex::lock()   { pthread_mutex_lock( &m_impl->m_mutex ); }
void Mutex::unlock() { pthread_mutex_unlock( &m_impl->m_mutex ); }

void yield() { pthread_yield(); }

#endif

} // namespace nvbio
