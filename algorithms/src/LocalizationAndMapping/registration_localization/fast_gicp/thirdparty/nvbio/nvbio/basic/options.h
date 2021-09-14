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

#include <nvbio/basic/types.h>
#include <nvbio/basic/console.h>
#include <stdlib.h>
#include <string>

namespace nvbio {

template <typename options_type>
bool bool_option(const options_type& options, const char* name, const bool val)
{
    return ( (options.find( std::string(name) ) != options.end()) ?
        atoi( options.find(std::string(name))->second.c_str() ) :
        val ) ? true : false;
}

template <typename options_type>
bool bool_option(const options_type& options, const char* name1, const char* name2, const bool val)
{
    return (
        (options.find( std::string(name1) ) != options.end()) ?
            atoi( options.find(std::string(name1))->second.c_str() ) :
        (options.find( std::string(name2) ) != options.end()) ?
            atoi( options.find(std::string(name2))->second.c_str() ) :
            val ) ? true : false;
}

template <typename options_type>
uint32 uint_option(const options_type& options, const char* name, const uint32 val)
{
    return (options.find( std::string(name) ) != options.end()) ?
        atoi( options.find(std::string(name))->second.c_str() ) :
        val;
}

template <typename options_type>
uint32 uint_option(const options_type& options, const char* name1, const char* name2, const uint32 val)
{
    return
        (options.find( std::string(name1) ) != options.end()) ?
            atoi( options.find(std::string(name1))->second.c_str() ) :
        (options.find( std::string(name2) ) != options.end()) ?
            atoi( options.find(std::string(name2))->second.c_str() ) :
            val;
}

template <typename options_type>
int32 int_option(const options_type& options, const char* name, const int32 val)
{
    return (options.find( std::string(name) ) != options.end()) ?
        atoi( options.find(std::string(name))->second.c_str() ) :
        val;
}

template <typename options_type>
int32 int_option(const options_type& options, const char* name1, const char* name2, const uint32 val)
{
    return
        (options.find( std::string(name1) ) != options.end()) ?
            atoi( options.find(std::string(name1))->second.c_str() ) :
        (options.find( std::string(name2) ) != options.end()) ?
            atoi( options.find(std::string(name2))->second.c_str() ) :
            val;
}

template <typename options_type>
int64 int64_option(const options_type& options, const char* name, const int64 val)
{
    return (options.find( std::string(name) ) != options.end()) ?
        atoi( options.find(std::string(name))->second.c_str() ) :
        val;
}

template <typename options_type>
int64 int64_option(const options_type& options, const char* name1, const char* name2, const uint32 val)
{
    return
        (options.find( std::string(name1) ) != options.end()) ?
            atoi( options.find(std::string(name1))->second.c_str() ) :
        (options.find( std::string(name2) ) != options.end()) ?
            atoi( options.find(std::string(name2))->second.c_str() ) :
            val;
}

template <typename options_type>
float float_option(const options_type& options, const char* name, const float val)
{
    return (options.find( std::string(name) ) != options.end()) ?
        (float)atof( options.find(std::string(name))->second.c_str() ) :
        val;
}

template <typename options_type>
float float_option(const options_type& options, const char* name1, const char* name2, const uint32 val)
{
    return
        (options.find( std::string(name1) ) != options.end()) ?
            atof( options.find(std::string(name1))->second.c_str() ) :
        (options.find( std::string(name2) ) != options.end()) ?
            atof( options.find(std::string(name2))->second.c_str() ) :
            val;
}

template <typename options_type>
std::string string_option(const options_type& options, const char* name, const char* val)
{
    return (options.find( std::string(name) ) != options.end()) ?
        options.find(std::string(name))->second :
        std::string( val );
}

template <typename options_type>
std::string string_option(const options_type& options, const char* name1, const char* name2, const char* val)
{
    return
        (options.find( std::string(name1) ) != options.end()) ?
            options.find(std::string(name1))->second :
        (options.find( std::string(name2) ) != options.end()) ?
            options.find(std::string(name2))->second :
            std::string( val );
}

template <typename options_type>
int2 int2_option(const options_type& options, const char* name, const int2 val)
{
    if (options.find( std::string(name) ) != options.end())
    {
        const std::string str = options.find(std::string(name))->second;
        const size_t c = str.find(',');
        if (c == std::string::npos)
        {
            log_warning( stderr, "int2_option() : parsing error, missing comma\n" );
            return val;
        }
        const std::string num1 = str.substr( 0, c );
        const std::string num2 = str.substr( c + 1, str.size() );
        return make_int2( atoi( num1.c_str() ), atoi( num2.c_str() ) );
    }
    return val;
}

} // namespace nvbio
