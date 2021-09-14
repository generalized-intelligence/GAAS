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

#include <nvbio/basic/console.h>
#include <string>

std::string retokenize(const char* format, const char* prefix)
{
    // check if the string starts with a carriage return
    bool        carriage_return = false;
    const char* start_p = format;

    for (const char* p = format; *p == '\r' || *p == ' '; ++p)
    {
        if (*p == '\r')
        {
            carriage_return = true;
            start_p = p;
            break;
        }
    }

    std::string new_format;
    if (carriage_return == false)
        new_format = std::string( prefix );

    for (const char* p = start_p; *p != '\0'; ++p)
    {
        if (*p == '\n')
        {
            new_format.append( 1u,'\n' );
            if (*(p+1) != '\0')
                new_format += prefix;
        }
        else if (*p == '\r')
        {
            new_format.append( 1u,'\r' );
            if (*(p+1) != '\0')
                new_format += prefix;
        }
        else
            new_format.append( 1u,*p );
    }
    return new_format;
}

#if WIN32
#include <nvbio/basic/threads.h>
#include <windows.h>

unsigned int TEXT_BLUE   = FOREGROUND_BLUE;
unsigned int TEXT_RED    = FOREGROUND_RED;
unsigned int TEXT_GREEN  = FOREGROUND_GREEN;
unsigned int TEXT_BRIGHT = FOREGROUND_INTENSITY;

namespace { nvbio::Mutex s_mutex; }

Verbosity s_verbosity = V_VERBOSE;

void set_verbosity(Verbosity level)
{
    s_verbosity = level;
}

static void textcolor(unsigned int color)
{
    HANDLE hConsole = GetStdHandle(STD_OUTPUT_HANDLE);  // Get handle to standard output
    SetConsoleTextAttribute(hConsole,color);  // set the text attribute of the previous handle
}

void log_visible(FILE* stream, const char* format, ...)
{
    nvbio::ScopedLock lock( &s_mutex );

    if (s_verbosity >= V_VISIBLE)
    {
        const std::string new_format = retokenize( format, "visible : " );
        textcolor( TEXT_RED | TEXT_GREEN | TEXT_BLUE | TEXT_BRIGHT );
        va_list args;
        va_start(args, format);
        vfprintf(stream, new_format.c_str(), args);
        va_end(args);
        textcolor( TEXT_RED | TEXT_GREEN | TEXT_BLUE );
    }
}
void log_info(FILE* stream, const char* format, ...)
{
    nvbio::ScopedLock lock( &s_mutex );

    if (s_verbosity >= V_INFO)
    {
        const std::string new_format = retokenize( format, "info    : " );
        textcolor( TEXT_RED | TEXT_GREEN | TEXT_BLUE );
        va_list args;
        va_start(args, format);
        vfprintf(stream, new_format.c_str(), args);
        va_end(args);
        textcolor( TEXT_RED | TEXT_GREEN | TEXT_BLUE );
    }
}
void log_stats(FILE* stream, const char* format, ...)
{
    if (s_verbosity >= V_STATS)
    {
        const std::string new_format = retokenize( format, "stats   : " );
        textcolor( TEXT_BLUE | TEXT_BRIGHT/*| TEXT_GREEN*/ );
        va_list args;
        va_start(args, format);
        vfprintf(stream, new_format.c_str(), args);
        va_end(args);
        textcolor( TEXT_RED | TEXT_GREEN | TEXT_BLUE );
    }
}
void log_verbose(FILE* stream, const char* format, ...)
{
    nvbio::ScopedLock lock( &s_mutex );

    if (s_verbosity >= V_VERBOSE)
    {
        const std::string new_format = retokenize( format, "verbose : " );
        textcolor( TEXT_GREEN );
        va_list args;
        va_start(args, format);
        vfprintf(stream, new_format.c_str(), args);
        va_end(args);
        textcolor( TEXT_RED | TEXT_GREEN | TEXT_BLUE );
    }
}
void log_debug(FILE* stream, const char* format, ...)
{
    nvbio::ScopedLock lock( &s_mutex );

    if (s_verbosity >= V_DEBUG)
    {
        const std::string new_format = retokenize( format, "debug   : " );
        textcolor( TEXT_RED );
        va_list args;
        va_start(args, format);
        vfprintf(stream, new_format.c_str(), args);
        va_end(args);
        textcolor( TEXT_RED | TEXT_GREEN | TEXT_BLUE );
    }
}
void log_warning(FILE* stream, const char* format, ...)
{
    nvbio::ScopedLock lock( &s_mutex );

    if (s_verbosity >= V_ERROR)
    {
        const std::string new_format = retokenize( format, "warning : " );
        textcolor( TEXT_RED | TEXT_BLUE | TEXT_BRIGHT );
        va_list args;
        va_start(args, format);
        vfprintf(stream, new_format.c_str(), args);
        va_end(args);
        textcolor( TEXT_RED | TEXT_GREEN | TEXT_BLUE );
    }
}
void log_error(FILE* stream, const char* format, ...)
{
    nvbio::ScopedLock lock( &s_mutex );

    if (s_verbosity >= V_ERROR)
    {
        const std::string new_format = retokenize( format, "error   : " );
        textcolor( TEXT_RED | TEXT_BRIGHT );
        va_list args;
        va_start(args, format);
        vfprintf(stream, new_format.c_str(), args);
        va_end(args);
        textcolor( TEXT_RED | TEXT_GREEN | TEXT_BLUE );
    }
}

void log_visible_cont(FILE* stream, const char* format, ...)
{
    nvbio::ScopedLock lock( &s_mutex );

    if (s_verbosity >= V_VISIBLE)
    {
        textcolor( TEXT_RED | TEXT_GREEN | TEXT_BLUE | TEXT_BRIGHT );
        va_list args;
        va_start(args, format);
        vfprintf(stream, format, args);
        va_end(args);
        textcolor( TEXT_RED | TEXT_GREEN | TEXT_BLUE );
    }
}
void log_info_cont(FILE* stream, const char* format, ...)
{
    nvbio::ScopedLock lock( &s_mutex );

    if (s_verbosity >= V_INFO)
    {
        textcolor( TEXT_RED | TEXT_GREEN | TEXT_BLUE );
        va_list args;
        va_start(args, format);
        vfprintf(stream, format, args);
        va_end(args);
        textcolor( TEXT_RED | TEXT_GREEN | TEXT_BLUE );
    }
}
void log_stats_cont(FILE* stream, const char* format, ...)
{
    nvbio::ScopedLock lock( &s_mutex );

    if (s_verbosity >= V_STATS)
    {
        const std::string new_format = retokenize( format, "stats   : " );
        textcolor( TEXT_BLUE | TEXT_BRIGHT/*| TEXT_GREEN*/ );
        va_list args;
        va_start(args, format);
        vfprintf(stream, new_format.c_str(), args);
        va_end(args);
        textcolor( TEXT_RED | TEXT_GREEN | TEXT_BLUE );
    }
}
void log_verbose_cont(FILE* stream, const char* format, ...)
{
    nvbio::ScopedLock lock( &s_mutex );

    if (s_verbosity >= V_VERBOSE)
    {
        textcolor( TEXT_GREEN );
        va_list args;
        va_start(args, format);
        vfprintf(stream, format, args);
        va_end(args);
        textcolor( TEXT_RED | TEXT_GREEN | TEXT_BLUE );
    }
}
void log_debug_cont(FILE* stream, const char* format, ...)
{
    nvbio::ScopedLock lock( &s_mutex );

    if (s_verbosity >= V_DEBUG)
    {
        textcolor( TEXT_RED );
        va_list args;
        va_start(args, format);
        vfprintf(stream, format, args);
        va_end(args);
        textcolor( TEXT_RED | TEXT_GREEN | TEXT_BLUE );
    }
}
void log_warning_cont(FILE* stream, const char* format, ...)
{
    nvbio::ScopedLock lock( &s_mutex );

    if (s_verbosity >= V_ERROR)
    {
        textcolor( TEXT_RED | TEXT_BLUE | TEXT_BRIGHT );
        va_list args;
        va_start(args, format);
        vfprintf(stream, format, args);
        va_end(args);
        textcolor( TEXT_RED | TEXT_GREEN | TEXT_BLUE );
    }
}
void log_error_cont(FILE* stream, const char* format, ...)
{
    nvbio::ScopedLock lock( &s_mutex );

    if (s_verbosity >= V_ERROR)
    {
        textcolor( TEXT_RED | TEXT_BRIGHT );
        va_list args;
        va_start(args, format);
        vfprintf(stream, format, args);
        va_end(args);
        textcolor( TEXT_RED | TEXT_GREEN | TEXT_BLUE );
    }
}

#else
#include <stdarg.h>

const char* TEXT_BRIGHT_BLUE    = "\033[01;34m";
const char* TEXT_BRIGHT_MAGENTA = "\033[01;35m";
const char* TEXT_BRIGHT_RED     = "\033[01;31m";
const char* TEXT_CYAN   = "\033[22;36m";
const char* TEXT_BLUE   = "\033[22;34m";
const char* TEXT_RED    = "\033[22;31m";
const char* TEXT_GREEN  = "\033[22;32m";
const char* TEXT_BRIGHT = "\033[01;37m";
const char* TEXT_NORMAL = "\033[22;37m";

Verbosity s_verbosity = V_VERBOSE;

void set_verbosity(Verbosity level)
{
    s_verbosity = level;
}

void log_visible(FILE* stream, const char* format, ...)
{
    if (s_verbosity >= V_VISIBLE)
    {
        const std::string new_format = retokenize( format, "visible : " );
        char col_format[2048];
        sprintf( col_format, "%s%s", TEXT_BRIGHT, new_format.c_str() );
        va_list args;
        va_start(args, format);
        vfprintf(stream, col_format, args);
        va_end(args);
    }
}
void log_info(FILE* stream, const char* format, ...)
{
    if (s_verbosity >= V_INFO)
    {
        const std::string new_format = retokenize( format, "info    : " );
        char col_format[2048];
        sprintf( col_format, "%s%s", TEXT_NORMAL, new_format.c_str() );
        va_list args;
        va_start(args, format);
        vfprintf(stream, col_format, args);
        va_end(args);
    }
}
void log_stats(FILE* stream, const char* format, ...)
{
    if (s_verbosity >= V_STATS)
    {
        const std::string new_format = retokenize( format, "stats   : " );
        char col_format[2048];
        sprintf( col_format, "%s%s", TEXT_CYAN, new_format.c_str() );
        va_list args;
        va_start(args, format);
        vfprintf(stream, col_format, args);
        va_end(args);
    }
}
void log_verbose(FILE* stream, const char* format, ...)
{
    if (s_verbosity >= V_VERBOSE)
    {
        const std::string new_format = retokenize( format, "verbose : " );
        char col_format[2048];
        sprintf( col_format, "%s%s", TEXT_GREEN, new_format.c_str() );
        va_list args;
        va_start(args, format);
        vfprintf(stream, col_format, args);
        va_end(args);
    }
}
void log_debug(FILE* stream, const char* format, ...)
{
    if (s_verbosity >= V_DEBUG)
    {
        const std::string new_format = retokenize( format, "debug   : " );
        char col_format[2048];
        sprintf( col_format, "%s%s", TEXT_RED, new_format.c_str() );
        va_list args;
        va_start(args, format);
        vfprintf(stream, col_format, args);
        va_end(args);
    }
}
void log_warning(FILE* stream, const char* format, ...)
{
    if (s_verbosity >= V_ERROR)
    {
        const std::string new_format = retokenize( format, "warning : " );
        char col_format[2048];
        sprintf( col_format, "%s%s", TEXT_BRIGHT_MAGENTA, new_format.c_str() );
        va_list args;
        va_start(args, format);
        vfprintf(stream, col_format, args);
        va_end(args);
    }
}
void log_error(FILE* stream, const char* format, ...)
{
    if (s_verbosity >= V_ERROR)
    {
        const std::string new_format = retokenize( format, "error   : " );
        char col_format[2048];
        sprintf( col_format, "%s%s", TEXT_BRIGHT_RED, new_format.c_str() );
        va_list args;
        va_start(args, format);
        vfprintf(stream, col_format, args);
        va_end(args);
    }
}

void log_visible_cont(FILE* stream, const char* format, ...)
{
    if (s_verbosity >= V_VISIBLE)
    {
        char col_format[2048];
        sprintf( col_format, "%s%s", TEXT_BRIGHT, format );
        va_list args;
        va_start(args, format);
        vfprintf(stream, col_format, args);
        va_end(args);
    }
}
void log_info_cont(FILE* stream, const char* format, ...)
{
    if (s_verbosity >= V_INFO)
    {
        char col_format[2048];
        sprintf( col_format, "%s%s", TEXT_NORMAL, format );
        va_list args;
        va_start(args, format);
        vfprintf(stream, col_format, args);
        va_end(args);
    }
}
void log_stats_cont(FILE* stream, const char* format, ...)
{
    if (s_verbosity >= V_STATS)
    {
        char col_format[2048];
        sprintf( col_format, "%s%s", TEXT_CYAN, format );
        va_list args;
        va_start(args, format);
        vfprintf(stream, col_format, args);
        va_end(args);
    }
}
void log_verbose_cont(FILE* stream, const char* format, ...)
{
    if (s_verbosity >= V_VERBOSE)
    {
        char col_format[2048];
        sprintf( col_format, "%s%s", TEXT_GREEN, format );
        va_list args;
        va_start(args, format);
        vfprintf(stream, col_format, args);
        va_end(args);
    }
}
void log_debug_cont(FILE* stream, const char* format, ...)
{
    if (s_verbosity >= V_DEBUG)
    {
        char col_format[2048];
        sprintf( col_format, "%s%s", TEXT_RED, format );
        va_list args;
        va_start(args, format);
        vfprintf(stream, col_format, args);
        va_end(args);
    }
}
void log_warning_cont(FILE* stream, const char* format, ...)
{
    if (s_verbosity >= V_ERROR)
    {
        char col_format[2048];
        sprintf( col_format, "%s%s", TEXT_BRIGHT_MAGENTA, format );
        va_list args;
        va_start(args, format);
        vfprintf(stream, col_format, args);
        va_end(args);
    }
}
void log_error_cont(FILE* stream, const char* format, ...)
{
    if (s_verbosity >= V_ERROR)
    {
        char col_format[2048];
        sprintf( col_format, "%s%s", TEXT_BRIGHT_RED, format );
        va_list args;
        va_start(args, format);
        vfprintf(stream, col_format, args);
        va_end(args);
    }
}

#endif

void log_visible_nl(FILE* stream)
{
    if (s_verbosity >= V_VISIBLE)
        fprintf(stream, "\n");
}
void log_info_nl(FILE* stream)
{
    if (s_verbosity >= V_INFO)
        fprintf(stream, "\n");
}
void log_stats_nl(FILE* stream)
{
    if (s_verbosity >= V_STATS)
        fprintf(stream, "\n");
}
void log_verbose_nl(FILE* stream)
{
    if (s_verbosity >= V_VERBOSE)
        fprintf(stream, "\n");
}
void log_debug_nl(FILE* stream)
{
    if (s_verbosity >= V_DEBUG)
        fprintf(stream, "\n");
}
void log_warning_nl(FILE* stream)
{
    if (s_verbosity >= V_ERROR)
        fprintf(stream, "\n");
}
void log_error_nl(FILE* stream)
{
    if (s_verbosity >= V_ERROR)
        fprintf(stream, "\n");
}
