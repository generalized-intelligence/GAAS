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

#include <stdio.h>

#if WIN32
extern unsigned int TEXT_BLUE;
extern unsigned int TEXT_RED;
extern unsigned int TEXT_GREEN;
extern unsigned int TEXT_BRIGHT;
#endif

///@addtogroup Basic
///@{

void textcolor(unsigned int color);

void log_visible(FILE* file, const char* string, ...);
void log_info(FILE* file, const char* string, ...);
void log_stats(FILE* file, const char* string, ...);
void log_verbose(FILE* file, const char* string, ...);
void log_debug(FILE* file, const char* string, ...);
void log_warning(FILE* file, const char* string, ...);
void log_error(FILE* file, const char* string, ...);

void log_visible_cont(FILE* file, const char* string, ...);
void log_info_cont(FILE* file, const char* string, ...);
void log_stats_cont(FILE* file, const char* string, ...);
void log_verbose_cont(FILE* file, const char* string, ...);
void log_debug_cont(FILE* file, const char* string, ...);
void log_warning_cont(FILE* file, const char* string, ...);
void log_error_cont(FILE* file, const char* string, ...);

void log_visible_nl(FILE* file);
void log_info_nl(FILE* file);
void log_stats_nl(FILE* file);
void log_verbose_nl(FILE* file);
void log_debug_nl(FILE* file);
void log_warning_nl(FILE* file);
void log_error_nl(FILE* file);

enum Verbosity
{
    V_ERROR   = 0,
    V_WARNING = 1,
    V_VISIBLE = 2,
    V_INFO    = 3,
    V_STATS   = 4,
    V_VERBOSE = 5,
    V_DEBUG   = 6,
};

void set_verbosity(Verbosity);

///@} Basic
