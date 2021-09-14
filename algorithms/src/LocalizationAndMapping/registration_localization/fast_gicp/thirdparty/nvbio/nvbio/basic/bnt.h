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

//
// NOTE: the code below is a derivative of bntseq.h, originally distributed
// under the MIT License, Copyright (c) 2008 Genome Research Ltd (GRL).
//

#pragma once

#include <nvbio/basic/types.h>
#include <string>
#include <vector>
#include <cstdio>

namespace nvbio {

struct BNTAnnData
{
    BNTAnnData() : offset(0), len(0), n_ambs(0), gi(0) {}

    int64   offset;
	int32   len;
	int32   n_ambs;
	uint32  gi;
};

struct BNTAnnInfo
{
    std::string name;
    std::string anno;
};

struct BNTAmb
{
	int64   offset;
	int32   len;
	char    amb;
};

struct BNTSeq
{
    BNTSeq() : l_pac(0), n_seqs(0), seed(0), n_holes(0) {}

    int64   l_pac;
	int32   n_seqs;
	uint32  seed;
	int32   n_holes;
    std::vector<BNTAnnInfo> anns_info;  // n_seqs elements
    std::vector<BNTAnnData> anns_data;  // n_seqs elements
    std::vector<BNTAmb>     ambs;       // n_holes elements
};

struct BNTInfo
{
    int64   l_pac;
	int32   n_seqs;
	uint32  seed;
	int32   n_holes;
};

struct BNTSeqLoader
{
    virtual void set_info(const BNTInfo info) {}
    virtual void read_ann(const BNTAnnInfo& info, BNTAnnData& data) {}
    virtual void read_amb(const BNTAmb& amb) {}
};

struct bns_fopen_failure {};
struct bns_files_mismatch {};

void save_bns(const BNTSeq& bns, const char *prefix);
void load_bns(BNTSeq& bns, const char *prefix);

void load_bns_info(BNTInfo& bns, const char *prefix);
void load_bns(BNTSeqLoader* bns, const char *prefix);

} // namespace nvbio
