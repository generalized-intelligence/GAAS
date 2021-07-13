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

#include "options.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

struct runtime_options command_line_options;

static void usage(void)
{
    fprintf(stderr, "usage: nvmem [-f|--file-ref] <genome> <input.fastq> <output-file>\n");
    fprintf(stderr, "\n");
    fprintf(stderr, "  -f, --file-ref        Read the genome from file directly (do not use mmap)\n");
    fprintf(stderr, "\n");
    exit(1);
}

#if defined(WIN32)

void parse_command_line(int argc, char **argv)
{
    for (int i = 0; i < argc-3; ++i)
    {
        if (strcmp( argv[i], "-f" ) == 0 ||
            strcmp( argv[i], "--file-ref" ) == 0)
            command_line_options.genome_use_mmap = false;
    }

    if (argc < 3)
    {
        // missing required arguments or too many arguments
        usage();
    }

    command_line_options.genome_file_name = argv[argc-3];
    command_line_options.input_file_name  = argv[argc-2];
    command_line_options.output_file_name = argv[argc-1];
}

#else

#include <unistd.h>
#include <getopt.h>

#include <nvbio/basic/console.h>

void parse_command_line(int argc, char **argv)
{
    static const char *options_short = "f:o";
    static struct option options_long[] = {
            { "file-ref",   no_argument,        NULL, 'f' },
            { NULL, 0, NULL, 0 },
    };

    int ch;
    while((ch = getopt_long(argc, argv, options_short, options_long, NULL)) != -1)
    {
        switch(ch)
        {
        case 'f':
            // -f, --file-ref
            command_line_options.genome_use_mmap = false;
            break;

        case '?':
        case ':':
        default:
            usage();
        }
    }

    if (optind != argc - 3)
    {
        // missing required arguments or too many arguments
        usage();
    }

    command_line_options.genome_file_name = argv[optind];
    command_line_options.input_file_name = argv[optind + 1];
    command_line_options.output_file_name = argv[optind + 2];
}

#endif