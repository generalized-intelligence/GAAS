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

#include <nvbio/sufsort/sufsort_utils.h>

namespace nvbio {

///@addtogroup Sufsort
///@{

/// open a string-set BWT file, returning a handler that can be used by the string-set BWT
/// construction functions.
///
/// The file type is specified by the extension of the output name; the following extensions
/// are supported:
///
/// <table>
/// <tr><td style="white-space: nowrap; vertical-align:text-top;">.txt</td><td style="vertical-align:text-top;">      ASCII</td></tr>
/// <tr><td style="white-space: nowrap; vertical-align:text-top;">.txt.gz</td><td style="vertical-align:text-top;">   ASCII, gzip compressed</td></tr>
/// <tr><td style="white-space: nowrap; vertical-align:text-top;">.txt.bgz</td><td style="vertical-align:text-top;">  ASCII, block-gzip compressed</td></tr>
/// <tr><td style="white-space: nowrap; vertical-align:text-top;">.bwt</td><td style="vertical-align:text-top;">      2-bit packed binary</td></tr>
/// <tr><td style="white-space: nowrap; vertical-align:text-top;">.bwt.gz</td><td style="vertical-align:text-top;">   2-bit packed binary, gzip compressed</td></tr>
/// <tr><td style="white-space: nowrap; vertical-align:text-top;">.bwt.bgz</td><td style="vertical-align:text-top;">  2-bit packed binary, block-gzip compressed</td></tr>
/// <tr><td style="white-space: nowrap; vertical-align:text-top;">.bwt4</td><td style="vertical-align:text-top;">     4-bit packed binary</td></tr>
/// <tr><td style="white-space: nowrap; vertical-align:text-top;">.bwt4.gz</td><td style="vertical-align:text-top;">  4-bit packed binary, gzip compressed</td></tr>
/// <tr><td style="white-space: nowrap; vertical-align:text-top;">.bwt4.bgz</td><td style="vertical-align:text-top;"> 4-bit packed binary, block-gzip compressed</td></tr>
/// </table>
///
/// Alongside with the main BWT file, a file containing the mapping between the primary
/// dollar tokens and their position in the BWT will be generated. This (.pri|.pri.gz|.pri.bgz)
/// file is a plain list of (position,string-id) pairs, either in ASCII or binary form.
/// The ASCII file has the form:
///\verbatim
///#PRI
///position[1] string[1]
///...
///position[n] string[n]
///\endverbatim
///
/// The binary file has the form:
///\verbatim
///char[4] header = "PRIB";
///struct { uint64 position; uint32 string_id; } pairs[n];
///\endverbatim
///
/// \param output_name      output name
/// \param params           additional compression parameters (e.g. "1R", "9", etc)
/// \return     a handler that can be used by the string-set BWT construction functions
///
SetBWTHandler* open_bwt_file(const char* output_name, const char* params);

///@}

} // namespace nvbio
