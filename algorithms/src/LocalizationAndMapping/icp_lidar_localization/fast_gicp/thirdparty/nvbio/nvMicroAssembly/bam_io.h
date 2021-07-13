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

#include <htslib/sam.h>
#include <htslib/hts.h>
#include <htslib/bgzf.h>

#include "assembly_types.h"

using namespace nvbio;

/**------------- BAM Format -------------**/

// BAM header
struct BAM_header
{
	uint8 magic[4];  						 // BAM magic string
	int32 l_text;    						 // length of the header text
	std::string text; 						 // the header text itself
	int32 n_ref;     						 // number of reference sequences
	std::vector<std::string> sq_names; 		 // reference sequence names
	H_VectorU32 sq_lengths;					 // reference sequence lengths
};

// BAM alignment record header
struct BAM_alignment_header
{
	int32  block_size; // length of the remainder of the alignment record
	int32  refID;      // reference sequence ID, -1 <= refID < n_ref (-1 for a read without a mapping position)
	int32  pos;        // 0-based leftmost coordinate
	uint32 bin_mq_nl;  // bin << 16 | MAPQ << 8 | l_read_name
	uint32 flag_nc;    // FLAG << 16 | n_cigar_op
	int32  l_seq;      // length of the sequence
	int32  next_refID; // refID of the next segment (-1 <= next_refID < n_ref)
	int32  next_pos;   // 0-based leftmost pos of the next segment
	int32  tlen;       // template length

	uint32 bin(void) const { return bin_mq_nl >> 16; }
	uint32 mapq(void) const { return (bin_mq_nl & 0xff00) >> 8; }
	uint32 l_read_name(void) const { return bin_mq_nl & 0xff; }
	uint32 flags(void) const { return flag_nc >> 16; }
	uint32 num_cigar_ops(void) const { return flag_nc & 0xffff; }
};

// BAM alignment record
// consists of the alignment header followed by a variable-length
// data block; the auxiliary data is of block_size - ftell() bytes
struct BAM_alignment_record
{
	BAM_alignment_header header; 	// alignment info
	H_VectorU8 data; 	 			// qname + cigar + seq + qual + aux
};

// contiguous batch of raw alignment records
struct BAM_alignment_batch_raw
{
	H_VectorU8 recs; 		// raw byte array of alignment records
	H_VectorU64 offsets; 	// offsets into alignment records (num_recs + 1)
};

typedef enum
{
	BAM_NAMES       = 1,
	BAM_CIGARS   	= 2,
	BAM_READS      	= 4,
	BAM_QUALITIES 	= 8,
	BAM_FLAGS       = 16,
	BAM_POSITIONS  	= 32,
	BAM_REFIDS      = 64,
	BAM_MAPQ        = 128,
	BAM_AUX     	= 256,
	BAM_BIN     	= 512,
	BAM_ALL			= 0xFFFF
} BAM_field_masks;

typedef enum
{
	BAM_FLAGS_PAIRED        = 1,
	BAM_FLAGS_PROPER_PAIR   = 2,
	BAM_FLAGS_UNMAPPED      = 4,
	BAM_FLAGS_MATE_UNMAPPED = 8,
	BAM_FLAGS_REVERSE       = 16,
	BAM_FLAGS_MATE_REVERSE  = 32,
	BAM_FLAGS_READ_1        = 64,
	BAM_FLAGS_READ_2        = 128,
	BAM_FLAGS_SECONDARY     = 256,
	BAM_FLAGS_QC_FAILED     = 512,
	BAM_FLAGS_DUPLICATE     = 1024
} BAM_alignment_flags;


// ----- SoA Functionality ------

// CRQ: cigars, reads, qualities
struct BAM_CRQ_index
{
	uint64 cigar_start, cigar_len;
	uint64 read_start, read_len;
	uint64 qual_start, qual_len;

	BAM_CRQ_index(): cigar_start(0), cigar_len(0), read_start(0), read_len(0), qual_start(0), qual_len(0) { }
	BAM_CRQ_index(uint64 cigar_start, uint64 cigar_len, uint64 read_start, uint64 read_len, uint64 qual_start, uint64 qual_len)
	: cigar_start(cigar_start), cigar_len(cigar_len), read_start(read_start), read_len(read_len),
	  qual_start(qual_start), qual_len(qual_len) { }
};

struct BAM_NAUX_index
{
	uint64 aux_data_start;
	uint32 aux_data_len;
	uint64 name_start;

	BAM_NAUX_index() : aux_data_start(0), aux_data_len(0), name_start(0) { }
	BAM_NAUX_index(uint64 aux_data_start, uint32 aux_data_len, uint64 name_start)
	: aux_data_start(aux_data_start), aux_data_len(aux_data_len), name_start(name_start) { }
};

#define H_BATCH_SIZE_ALLOC 10000000U
#define ALNREC_SIZE_ALLOC 512

typedef nvbio::vector<device_tag, BAM_CRQ_index> D_VectorCRQIndex;
typedef nvbio::vector<host_tag, BAM_CRQ_index> H_VectorCRQIndex;

// batch of alignment records as SoA
// host-only now
struct BAM_alignment_batch_SoA
{
	BAM_alignment_batch_SoA(): field_mask(BAM_ALL), num_alns(0) { reserve(); }
	BAM_alignment_batch_SoA(uint32 fields): field_mask(fields), num_alns(0) { reserve(); }

	uint32 field_mask; // record fields stored
	uint64 num_alns;
	H_VectorCigarOp cigars;
	H_VectorDNA16 reads;
	H_VectorU16 bin;
	H_VectorU8 qualities;
	H_VectorU16 flags;
	H_VectorU32 positions;
	H_VectorU32 refIDs;
	H_VectorU8 mapq;
	H_VectorU8 aux_data;
	H_VectorU8 names;
	H_VectorU32 next_positions;
	H_VectorU32 next_refIDs;
	nvbio::vector<host_tag, BAM_CRQ_index> crq_index; // CRQ: cigars, reads, qualities
	nvbio::vector<host_tag, BAM_NAUX_index> aln_index; // names, aux

	void reserve()
	{
		if(field_mask & BAM_POSITIONS) positions.reserve(H_BATCH_SIZE_ALLOC);
		if(field_mask & BAM_REFIDS) refIDs.reserve(H_BATCH_SIZE_ALLOC);
		if(field_mask & BAM_BIN) bin.reserve(H_BATCH_SIZE_ALLOC);
		if(field_mask & BAM_MAPQ) mapq.reserve(H_BATCH_SIZE_ALLOC);
		if(field_mask & BAM_FLAGS) flags.reserve(H_BATCH_SIZE_ALLOC);
		if(field_mask & (BAM_CIGARS | BAM_QUALITIES | BAM_READS)) crq_index.reserve(H_BATCH_SIZE_ALLOC);
		if(field_mask & BAM_CIGARS) cigars.reserve(H_BATCH_SIZE_ALLOC * 32);
		if(field_mask & BAM_READS) reads.reserve(H_BATCH_SIZE_ALLOC * 125);
		if(field_mask & BAM_QUALITIES) qualities.reserve(H_BATCH_SIZE_ALLOC * 125);
		if(field_mask & (BAM_AUX | BAM_NAMES)) aln_index.reserve(H_BATCH_SIZE_ALLOC);
		if(field_mask & BAM_AUX) aux_data.reserve(H_BATCH_SIZE_ALLOC * 64);
		if(field_mask & BAM_NAMES) names.reserve(H_BATCH_SIZE_ALLOC * 256);
		if(field_mask == BAM_ALL) {
			next_positions.reserve(H_BATCH_SIZE_ALLOC);
			next_refIDs.reserve(H_BATCH_SIZE_ALLOC);
		}
	}

	void allocate(const BAM_alignment_batch_SoA& size_batch)
	{
		if(field_mask & BAM_POSITIONS) positions.resize(size_batch.positions.size());
		if(field_mask & BAM_REFIDS) refIDs.resize(size_batch.refIDs.size());
		if(field_mask & BAM_BIN) bin.resize(size_batch.bin.size());
		if(field_mask & BAM_MAPQ) mapq.resize(size_batch.mapq.size());
		if(field_mask & BAM_FLAGS) flags.resize(size_batch.flags.size());
		if(field_mask & (BAM_CIGARS | BAM_QUALITIES | BAM_READS)) crq_index.resize(size_batch.crq_index.size());
		if(field_mask & BAM_CIGARS) cigars.resize(size_batch.cigars.size());
		if(field_mask & BAM_READS) reads.resize(size_batch.reads.size());
		if(field_mask & BAM_QUALITIES) qualities.resize(size_batch.qualities.size());
		if(field_mask & (BAM_AUX | BAM_NAMES)) aln_index.resize(size_batch.aln_index.size());
		if(field_mask & BAM_AUX) aux_data.resize(size_batch.aux_data.size());
		if(field_mask & BAM_NAMES) names.resize(size_batch.names.size());
		if(field_mask == BAM_ALL) {
			next_positions.resize(size_batch.next_positions.size());
			next_refIDs.resize(size_batch.next_refIDs.size());
		}
	}

	void reset()
	{
		num_alns = 0;
		positions.clear();
		refIDs.clear();
		bin.clear();
		mapq.clear();
		flags.clear();
		crq_index.clear();
		cigars.clear();
		reads.clear();
		qualities.clear();
		aln_index.clear();
		aux_data.clear();
		names.clear();
		next_positions.clear();
		next_refIDs.clear();
	}

	void free()
	{
		// force the memory to be freed by swapping
		positions = H_VectorU32();
		refIDs = H_VectorU32();
		bin = H_VectorU16();
		mapq = H_VectorU8();
		flags = H_VectorU16();
		cigars = H_VectorCigarOp();
		reads = H_VectorDNA16();
		qualities = H_VectorU8();
		aux_data = H_VectorU8();
		names = H_VectorU8();
		crq_index = nvbio::vector<host_tag, BAM_CRQ_index>();
		aln_index = nvbio::vector<host_tag, BAM_NAUX_index>();
		next_positions = H_VectorU32();
		next_refIDs = H_VectorU32();
	}

	void shuffle(BAM_alignment_batch_SoA& batch_out, H_VectorU64 shuffle_idx) {
		batch_out.allocate(*this);

		if(field_mask & BAM_POSITIONS) {
			thrust::gather(shuffle_idx.begin(), shuffle_idx.end(), positions.begin(), batch_out.positions.begin());
		}
		if(field_mask & BAM_REFIDS) {
			thrust::gather(shuffle_idx.begin(), shuffle_idx.end(), refIDs.begin(), batch_out.refIDs.begin());
		}
		if(field_mask & BAM_BIN) {
			thrust::gather(shuffle_idx.begin(), shuffle_idx.end(), bin.begin(), batch_out.bin.begin());
		}
		if(field_mask & BAM_MAPQ) {
			thrust::gather(shuffle_idx.begin(), shuffle_idx.end(), mapq.begin(), batch_out.mapq.begin());
		}
		if(field_mask & BAM_FLAGS) {
			thrust::gather(shuffle_idx.begin(), shuffle_idx.end(), flags.begin(), batch_out.flags.begin());
		}
		if(field_mask & BAM_QUALITIES) {
			uint64 cigar_offset = 0;
			for(uint64 i = 0; i < num_alns; i++) {
				uint64 idx = shuffle_idx[i];
				thrust::copy_n(cigars.begin() + crq_index[idx].cigar_start, crq_index[idx].cigar_len,
						batch_out.cigars.begin() + cigar_offset);
				batch_out.crq_index[i].cigar_start = cigar_offset;
				batch_out.crq_index[i].cigar_len = crq_index[idx].cigar_len;
				cigar_offset += crq_index[idx].cigar_len;
			}
		}
		if(field_mask & BAM_READS) {
			uint64 read_offset = 0;
			for(uint64 i = 0; i < num_alns; i++) {
				uint64 idx = shuffle_idx[i];
				uint64* data_in = (uint64 *)reads.addrof(crq_index[idx].read_start);
				uint64* data_out = (uint64 *)batch_out.reads.addrof(read_offset);
				memcpy(data_out, data_in, crq_index[idx].read_len/2);
				batch_out.crq_index[i].read_start = read_offset;
				batch_out.crq_index[i].read_len = crq_index[idx].read_len;
				const uint32 padded_read_len_bp = ((crq_index[idx].read_len + 7) / 8) * 8;
				read_offset += padded_read_len_bp;
			}
		}
		if(field_mask & BAM_QUALITIES) {
			uint64 qual_offset = 0;
			for(uint64 i = 0; i < num_alns; i++) {
				uint64 idx = shuffle_idx[i];
				thrust::copy_n(qualities.begin() + crq_index[idx].qual_start, crq_index[idx].qual_len,
						batch_out.qualities.begin() + qual_offset);
				batch_out.crq_index[i].qual_start = qual_offset;
				batch_out.crq_index[i].qual_len = crq_index[idx].qual_len;
				qual_offset += crq_index[idx].qual_len;
			}
		}
		if(field_mask & BAM_AUX) {
			uint64 aux_offset = 0;
			for(uint64 i = 0; i < num_alns; i++) {
				uint64 idx = shuffle_idx[i];
				thrust::copy_n(aux_data.begin() + aln_index[idx].aux_data_start, aln_index[idx].aux_data_len, batch_out.aux_data.begin() + aux_offset);
				batch_out.aln_index[i].aux_data_start = aux_offset;
				batch_out.aln_index[i].aux_data_len = aln_index[idx].aux_data_len;
				aux_offset += aln_index[idx].aux_data_len;
			}
		}
		if(field_mask & BAM_NAMES){
			uint64 names_offset = 0;
			for(uint64 i = 0; i < num_alns; i++) {
				uint64 idx = shuffle_idx[i];
				uint32 len;
				if(idx < num_alns-1) {
					len = aln_index[idx+1].name_start-aln_index[idx].name_start;
				} else {
					len = names.size()-aln_index[idx].name_start;
				}
				thrust::copy_n(names.begin() + aln_index[idx].name_start, len, batch_out.names.begin() + names_offset);
				batch_out.aln_index[i].name_start = names_offset;
				names_offset += len;
			}
		}
		if(field_mask == BAM_ALL) {
			thrust::gather(shuffle_idx.begin(), shuffle_idx.end(), next_positions.begin(), batch_out.next_positions.begin());
			thrust::gather(shuffle_idx.begin(), shuffle_idx.end(), next_refIDs.begin(), batch_out.next_refIDs.begin());
		}
	}
};


/**------------------- BAM Reader -------------**/

// htslib-based BAM file reader
struct HTSBAMReader
{
public:
	bam_hdr_t* header;

private:
	samFile* fp;
	long fp_offset;

public:
	HTSBAMReader(const char *fname);
	~HTSBAMReader();

	bool read_aln_batch(std::vector<bam1_t*>& batch, const uint64 batch_size = 1000000);
	bool read_aln_batch(BAM_alignment_batch_SoA& batch, const uint64 batch_size = 1000000);
	bool read_aln_batch_intv(BAM_alignment_batch_SoA& batch, const uint32 contig = 1u, const uint64 start = 0u, const uint64 end = 1000000);
private:
	bool read_hdr(void);
	void parse_aln_rec(BAM_alignment_batch_SoA& batch, bam1_t* b, H_VectorU32& cigar_temp);
};

/**------------------- BAM Writer -------------**/

// htslib-based BAM file writer
struct HTSBAMWriter
{
private:
	samFile* fp;

public:
	HTSBAMWriter(const char *fname);
	~HTSBAMWriter();

	void write_hdr(bam_hdr_t* header);
	void write_aln_batch(std::vector<bam1_t*>& batch, H_VectorU64 shuffle_idx, bam_hdr_t* header);
	void write_aln_batch(BAM_alignment_batch_SoA& batch, H_VectorU64 shuffle_idx, bam_hdr_t* header);
};
