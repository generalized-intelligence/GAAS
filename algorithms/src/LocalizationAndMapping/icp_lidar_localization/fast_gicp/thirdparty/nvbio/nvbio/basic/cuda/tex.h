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

/*! \file scan.h
 *   \brief Define CUDA based scan primitives.
 */

#pragma once

#include <nvbio/basic/types.h>
#include <nvbio/basic/cuda/arch.h>

namespace nvbio {
namespace cuda {

#ifdef __CUDACC__

#define DECL_TEXTURE_WRAPPER_CLASS(NAME, TYPE) \
 texture<TYPE> NAME ## _tex; \
struct NAME ## _texture \
{ \
    typedef TYPE                            value_type; \
    typedef value_type                      reference; \
    typedef const value_type*               pointer; \
    typedef uint32                          difference_type; \
    typedef std::random_access_iterator_tag iterator_category; \
\
    NVBIO_FORCEINLINE NVBIO_DEVICE TYPE operator[] (const uint32 i) const; \
\
    static NVBIO_FORCEINLINE NVBIO_HOST void bind(const uint32* NAME, uint32 size); \
 \
    static NVBIO_FORCEINLINE NVBIO_HOST void unbind(); \
}; \

#define INST_TEXTURE_WRAPPER_CLASS(NAME, TYPE) \
NVBIO_FORCEINLINE NVBIO_DEVICE TYPE NAME ## _texture::operator[] (const uint32 i) const \
{ \
    return tex1Dfetch( NAME ## _tex, i ); \
} \
NVBIO_FORCEINLINE NVBIO_HOST void NAME ## _texture::bind(const uint32* ptr, uint32 size) \
{ \
    if (ptr == NULL) \
        return; \
\
    cudaChannelFormatDesc channel_desc = cudaCreateChannelDesc<TYPE>(); \
    NAME ## _tex.normalized = false; \
    NAME ## _tex.filterMode = cudaFilterModePoint; \
    cudaBindTexture( 0, &NAME ## _tex, ptr, &channel_desc, size*sizeof(uint32) ); \
    nvbio::cuda::check_error("NAME ## texture::bind"); \
} \
NVBIO_FORCEINLINE NVBIO_HOST void NAME ## _texture::unbind() \
{ \
    cudaUnbindTexture( &NAME ## _tex ); \
} \


#define DECL_TEXTURE_SELECTOR_WRAPPER_CLASS(NAME, TYPE) \
 texture<TYPE> NAME ## _tex1; \
 texture<TYPE> NAME ## _tex2; \
struct NAME ## _texture_selector \
{ \
    typedef TYPE                            value_type; \
    typedef value_type                      reference; \
    typedef const value_type*               pointer; \
    typedef uint32                          difference_type; \
    typedef std::random_access_iterator_tag iterator_category; \
\
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE NAME ## _texture_selector(const uint32 sel) : m_sel(sel) {} \
\
    NVBIO_FORCEINLINE NVBIO_DEVICE TYPE operator[] (const uint32 i) const; \
\
    static NVBIO_FORCEINLINE NVBIO_HOST void bind(const uint32* ptr1, const uint32* ptr2, uint32 size); \
\
    static NVBIO_FORCEINLINE NVBIO_HOST void unbind(); \
\
    uint32 m_sel; \
}; \

#define INST_TEXTURE_SELECTOR_WRAPPER_CLASS(NAME, TYPE) \
NVBIO_FORCEINLINE NVBIO_DEVICE TYPE NAME ## _texture_selector::operator[] (const uint32 i) const \
{ \
    if (m_sel) \
        return tex1Dfetch( NAME ## _tex2, i ); \
    else \
        return tex1Dfetch( NAME ## _tex1, i ); \
} \
NVBIO_FORCEINLINE NVBIO_HOST void NAME ## _texture_selector::bind(const uint32* ptr1, const uint32* ptr2, uint32 size) \
{ \
    cudaChannelFormatDesc channel_desc = cudaCreateChannelDesc<TYPE>(); \
    NAME ## _tex1.normalized = false; \
    NAME ## _tex1.filterMode = cudaFilterModePoint; \
    NAME ## _tex2.normalized = false; \
    NAME ## _tex2.filterMode = cudaFilterModePoint; \
    if (ptr1) cudaBindTexture( 0, &NAME ## _tex1, ptr1, &channel_desc, size*sizeof(uint32) ); \
    if (ptr2) cudaBindTexture( 0, &NAME ## _tex2, ptr2, &channel_desc, size*sizeof(uint32) ); \
    nvbio::cuda::check_error("NAME ## texture_selector::bind"); \
} \
NVBIO_FORCEINLINE NVBIO_HOST void NAME ## _texture_selector::unbind() \
{ \
    cudaUnbindTexture( &NAME ## _tex1 ); \
    cudaUnbindTexture( &NAME ## _tex2 ); \
} \


#define TEXTURE_WRAPPER_LOG_WIDTH 14
#define TEXTURE_WRAPPER_WIDTH     16384

#define DECL_TEXTURE_WRAPPER_CLASS_2D(NAME, TYPE) \
 texture<TYPE,2> NAME ## _tex_2d; \
struct NAME ## _texture_2d \
{ \
    typedef TYPE                            value_type; \
    typedef value_type                      reference; \
    typedef const value_type*               pointer; \
    typedef uint32                          difference_type; \
    typedef std::random_access_iterator_tag iterator_category; \
\
    NVBIO_FORCEINLINE NVBIO_DEVICE TYPE operator[] (const uint32 i) const; \
\
    static NVBIO_FORCEINLINE NVBIO_HOST void bind(const uint32* NAME, uint32 size); \
 \
    static NVBIO_FORCEINLINE NVBIO_HOST void unbind(); \
}; \

#define INST_TEXTURE_WRAPPER_CLASS_2D(NAME, TYPE) \
NVBIO_FORCEINLINE NVBIO_DEVICE TYPE NAME ## _texture_2d::operator[] (const uint32 i) const \
{ \
    return tex2D( NAME ## _tex_2d, i&(TEXTURE_WRAPPER_WIDTH-1u), i >> TEXTURE_WRAPPER_LOG_WIDTH ); \
} \
NVBIO_FORCEINLINE NVBIO_HOST void NAME ## _texture_2d::bind(const uint32* ptr, uint32 size) \
{ \
    if (ptr == NULL) \
        return; \
\
    cudaChannelFormatDesc channel_desc = cudaCreateChannelDesc<TYPE>(); \
    NAME ## _tex_2d.normalized = false; \
    NAME ## _tex_2d.filterMode = cudaFilterModePoint; \
\
    const uint32 comps = sizeof(TYPE)/sizeof(uint32); \
    const uint32 w = TEXTURE_WRAPPER_WIDTH; \
    const uint32 h = (size/comps + w-1) / w; \
\
    cudaBindTexture2D( 0, &NAME ## _tex_2d, ptr, &channel_desc, w, h, w*sizeof(TYPE) ); \
    nvbio::cuda::check_error("NAME ## texture_2d::bind"); \
} \
NVBIO_FORCEINLINE NVBIO_HOST void NAME ## _texture_2d::unbind() \
{ \
    cudaUnbindTexture( &NAME ## _tex_2d ); \
} \

#define DECL_TEXTURE_SELECTOR_WRAPPER_CLASS_2D(NAME, TYPE) \
 texture<TYPE,2> NAME ## _tex1_2d; \
 texture<TYPE,2> NAME ## _tex2_2d; \
struct NAME ## _texture_selector_2d \
{ \
    typedef TYPE                            value_type; \
    typedef value_type                      reference; \
    typedef const value_type*               pointer; \
    typedef uint32                          difference_type; \
    typedef std::random_access_iterator_tag iterator_category; \
\
    NVBIO_FORCEINLINE NVBIO_HOST_DEVICE NAME ## _texture_selector_2d(const uint32 sel) : m_sel(sel) {} \
\
    NVBIO_FORCEINLINE NVBIO_DEVICE TYPE operator[] (const uint32 i) const; \
\
    static NVBIO_FORCEINLINE NVBIO_HOST void bind(const uint32* ptr1, const uint32* ptr2, uint32 size); \
\
    static NVBIO_FORCEINLINE NVBIO_HOST void unbind(); \
\
    uint32 m_sel; \
}; \

#define INST_TEXTURE_SELECTOR_WRAPPER_CLASS_2D(NAME, TYPE) \
NVBIO_FORCEINLINE NVBIO_DEVICE TYPE NAME ## _texture_selector_2d::operator[] (const uint32 i) const \
{ \
    if (m_sel) \
        return tex2D( NAME ## _tex2_2d, i&(TEXTURE_WRAPPER_WIDTH-1u), i >> TEXTURE_WRAPPER_LOG_WIDTH ); \
    else \
        return tex2D( NAME ## _tex1_2d, i&(TEXTURE_WRAPPER_WIDTH-1u), i >> TEXTURE_WRAPPER_LOG_WIDTH ); \
} \
NVBIO_FORCEINLINE NVBIO_HOST void NAME ## _texture_selector_2d::bind(const uint32* ptr1, const uint32* ptr2, uint32 size) \
{ \
    cudaChannelFormatDesc channel_desc = cudaCreateChannelDesc<TYPE>(); \
    NAME ## _tex1_2d.normalized = false; \
    NAME ## _tex1_2d.filterMode = cudaFilterModePoint; \
    NAME ## _tex2_2d.normalized = false; \
    NAME ## _tex2_2d.filterMode = cudaFilterModePoint; \
\
    const uint32 comps = sizeof(TYPE)/sizeof(uint32); \
    const uint32 w = TEXTURE_WRAPPER_WIDTH; \
    const uint32 h = (size/comps + w-1) / w; \
\
    if (ptr1) cudaBindTexture2D( 0, &NAME ## _tex1_2d, ptr1, &channel_desc, w, h, w*sizeof(TYPE) ); \
    if (ptr2) cudaBindTexture2D( 0, &NAME ## _tex2_2d, ptr2, &channel_desc, w, h, w*sizeof(TYPE) ); \
    nvbio::cuda::check_error("NAME ## texture_selector_2d::bind"); \
} \
NVBIO_FORCEINLINE NVBIO_HOST void NAME ## _texture_selector_2d::unbind() \
{ \
    cudaUnbindTexture( &NAME ## _tex1_2d ); \
    cudaUnbindTexture( &NAME ## _tex2_2d ); \
} \

#else

#define DECL_TEXTURE_WRAPPER_CLASS(NAME, TYPE) \
struct NAME ## _texture \
{ \
    static NVBIO_FORCEINLINE NVBIO_HOST void bind(const uint32* NAME, uint32 size) {} \
\
    static NVBIO_FORCEINLINE NVBIO_HOST void unbind() {} \
};

#define INST_TEXTURE_WRAPPER_CLASS(NAME, TYPE)

#define DECL_TEXTURE_SELECTOR_WRAPPER_CLASS(NAME, TYPE) \
struct NAME ## _texture_selector \
{ \
    static NVBIO_FORCEINLINE NVBIO_HOST void bind(const uint32* NAME, uint32 size) {} \
\
    static NVBIO_FORCEINLINE NVBIO_HOST void unbind() {} \
};

#define INST_TEXTURE_SELECTOR_WRAPPER_CLASS(NAME, TYPE)

#define DECL_TEXTURE_WRAPPER_CLASS_2D(NAME, TYPE) \
struct NAME ## _texture_2d \
{ \
    static NVBIO_FORCEINLINE NVBIO_HOST void bind(const uint32* NAME, uint32 size) {} \
\
    static NVBIO_FORCEINLINE NVBIO_HOST void unbind() {} \
};

#define INST_TEXTURE_WRAPPER_CLASS_2D(NAME, TYPE)

#define DECL_TEXTURE_SELECTOR_WRAPPER_CLASS_2D(NAME, TYPE) \
struct NAME ## _texture_selector_2d \
{ \
    static NVBIO_FORCEINLINE NVBIO_HOST void bind(const uint32* NAME, uint32 size) {} \
\
    static NVBIO_FORCEINLINE NVBIO_HOST void unbind() {} \
};

#define INST_TEXTURE_SELECTOR_WRAPPER_CLASS_2D(NAME, TYPE)

#endif

} // namespace cuda
} // namespace nvbio
