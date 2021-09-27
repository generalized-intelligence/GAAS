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

#include "assembly_types.h"
#include "bam_io.h"


using namespace nvbio;

/** ---- Read Functionality ---- **/

HTSBAMReader::HTSBAMReader(const char *fname)
{
	fp = sam_open(fname, "r");
	if (fp == NULL) {
		throw nvbio::runtime_error("Could not open %s", fname);
	}
	header = sam_hdr_read(fp);
	fp_offset = bgzf_tell(fp->fp.bgzf);
	if (header == NULL) {
		throw nvbio::runtime_error("Error parsing BAM file header");
	}
}

HTSBAMReader::~HTSBAMReader()
{
	bam_hdr_destroy(header);
	sam_close(fp);
}

void HTSBAMReader::parse_aln_rec(BAM_alignment_batch_SoA& batch, bam1_t* b, H_VectorU32& cigar_temp) {

	if(batch.field_mask & BAM_POSITIONS) {
			batch.positions.push_back(b->core.pos);
		}
		if(batch.field_mask & BAM_REFIDS) {
			batch.refIDs.push_back(b->core.tid);
		}
		if(batch.field_mask & BAM_MAPQ) {
			batch.bin.push_back(b->core.bin);
		}
		if(batch.field_mask & BAM_MAPQ) {
			batch.mapq.push_back(b->core.qual);
		}
		if(batch.field_mask & BAM_FLAGS) {
			batch.flags.push_back(b->core.flag);
		}
		if(batch.field_mask == BAM_ALL) {
			batch.next_refIDs.push_back(b->core.mtid);
			batch.next_positions.push_back(b->core.mpos);
		}

		// data portion
		uint32 offset = 0;
		const uint64 read_name_off = batch.names.size();
		const uint32 read_name_len = b->core.l_qname;
		if(batch.field_mask & BAM_NAMES) {
			batch.names.resize(read_name_off + read_name_len + 1);
			memcpy(&batch.names[read_name_off], b->data, read_name_len);
			batch.names[read_name_off + read_name_len] = '\0';
		}
		offset += read_name_len;

		BAM_CRQ_index crq_index(batch.cigars.size(), b->core.n_cigar, batch.reads.size(), b->core.l_qseq, batch.qualities.size(), b->core.l_qseq);
		if(batch.field_mask & (BAM_CIGARS | BAM_QUALITIES | BAM_READS)) {
			batch.crq_index.push_back(crq_index);
		}

		const uint32 cigar_len = b->core.n_cigar;
		if(batch.field_mask & BAM_CIGARS) {
			cigar_temp.resize(cigar_len);
			if (cigar_len) {
				memcpy(&cigar_temp[0], b->data + offset, sizeof(uint32) * cigar_len);
				for(uint32 c = 0; c < cigar_len; c++) {
					cigar_op op;
					op.op = cigar_temp[c] & 0xf;
					op.len = cigar_temp[c] >> 4;
					batch.cigars.push_back(op);
				}
			}
		}
		if (cigar_len) {
			offset += sizeof(uint32)*cigar_len;
		}

		if(batch.field_mask & BAM_READS) {
			// figure out the length of the sequence data, rounded up to reach a dword boundary
			const uint32 padded_read_len_bp = ((b->core.l_qseq + 7) / 8) * 8;
			//const uint64 required_words = util::divide_ri(b->core.l_qseq, 8);
			// make sure we have enough memory, then read in the sequence
			batch.reads.resize(crq_index.read_start + padded_read_len_bp);
			uint64 *storage = (uint64 *)batch.reads.addrof(crq_index.read_start);
			memcpy(storage, b->data + offset, b->core.l_qseq / 2);
		}
		offset += b->core.l_qseq / 2;

		if(batch.field_mask & BAM_QUALITIES) {
			batch.qualities.resize(crq_index.qual_start + b->core.l_qseq);
			memcpy(&batch.qualities[crq_index.qual_start], b->data + offset, b->core.l_qseq);
		}
		offset += b->core.l_qseq;

		const uint32 aux_len = b->l_data - offset;
		BAM_NAUX_index idx(batch.aux_data.size(), aux_len, read_name_off);
		if(batch.field_mask & (BAM_AUX | BAM_NAMES)) {
			batch.aln_index.push_back(idx);
		}
		if(batch.field_mask & BAM_AUX) {
			const uint64 aux_start = batch.aux_data.size();
			batch.aux_data.resize(aux_start + aux_len);
			memcpy(&batch.aux_data[aux_start], b->data + offset, aux_len);
		}
		offset += aux_len;
}

// returns false if no records were read
bool HTSBAMReader::read_aln_batch(BAM_alignment_batch_SoA& batch, const uint64 batch_size)
{
	batch.reset();
	H_VectorU32 cigar_temp(64);
	bam1_t *b = (bam1_t*)calloc(1, sizeof(bam1_t));
	for(uint64 aln_id = 0; aln_id < batch_size; aln_id++) {
		if (sam_read1(fp, header, b) < 0) break;
		parse_aln_rec(batch, b, cigar_temp);
		batch.num_alns++;
		if(batch.num_alns % 10000000 == 0) {
			printf("Loaded %llu records. \n", batch.num_alns);
		}
	}

	if (batch.num_alns == 0) {
		return false;
	}
	return true;
}

// loads reads overlapping the specified interval [s, e] (including the end points)
// the interval is 0-based (note: the record position is 0-based)
// this is a temporary hack -- should use bam index file instead
// returns false if no records were read
bool HTSBAMReader::read_aln_batch_intv(BAM_alignment_batch_SoA& batch, const uint32 contig, const uint64 start, const uint64 end)
{
	long new_offset = -1;

	H_VectorU32 cigar_temp(64);
	bam1_t *b = (bam1_t*)calloc(1, sizeof(bam1_t));
	while(1) {
		if (sam_read1(fp, header, b) < 0) break;
		if(b->core.tid > contig || (b->core.tid == contig && b->core.pos > end)) break; // reads one more record, should jump back
		if(b->core.flag & BAM_FUNMAP) continue;
		if(b->core.tid == contig && (bam_endpos(b) >= start)) {
			if(new_offset == -1) {
				new_offset = bgzf_tell(fp->fp.bgzf);
			}
			//printf("Read: %u %u %u\n", b->core.tid+1, b->core.pos+1, b->core.l_qseq);
			parse_aln_rec(batch, b, cigar_temp);
			batch.num_alns++;
		}
	}

	if(new_offset != -1) {
		fp_offset = new_offset;
	}

	bgzf_seek(fp->fp.bgzf, fp_offset, SEEK_SET);

	if (batch.num_alns == 0) {
		return false;
	}
	return true;
}

bool HTSBAMReader::read_aln_batch(std::vector<bam1_t*>& batch, const uint64 batch_size) {
	bam1_t *b = (bam1_t*)calloc(1, sizeof(bam1_t));
	uint64 aln_id = 0;
	for(aln_id = 0; aln_id < batch_size; aln_id++) {
		if (sam_read1(fp, header, b) < 0) break;
		batch.push_back(b);
	}
	if (aln_id == 0) {
		return false;
	}
	return true;
}


/** ---- Write Functionality ---- **/

HTSBAMWriter::HTSBAMWriter(const char *fname)
{
	fp = sam_open(fname, "wb");
	if (fp == NULL) {
		throw nvbio::runtime_error("Could not open %s for writing", fname);
	}
}

HTSBAMWriter::~HTSBAMWriter()
{
	if (fp) {
		sam_close(fp);
		fp = NULL;
	}
}

void HTSBAMWriter::write_hdr(bam_hdr_t* header)
{
	sam_hdr_write(fp, header);
}

// assumes the batch contains all the fields
void HTSBAMWriter::write_aln_batch(BAM_alignment_batch_SoA& batch, H_VectorU64 shuffle_idx, bam_hdr_t* header)
{
	bam1_t *b = (bam1_t*)calloc(1, sizeof(bam1_t));
	b->data = (uint8_t*)malloc(ALNREC_SIZE_ALLOC);
	b->m_data = ALNREC_SIZE_ALLOC;
	for(uint64 idx = 0; idx < batch.num_alns; idx++) {
		uint64 i = shuffle_idx[idx];
		BAM_CRQ_index crq = batch.crq_index[i];
		BAM_NAUX_index naux = batch.aln_index[i];
		bam1_core_t *c = &b->core;
		c->tid = batch.refIDs[i];
		c->pos = batch.positions[i];
		c->bin = batch.bin[i];
		c->qual = batch.mapq[i];
		if(i < batch.num_alns-1) {
			c->l_qname = batch.aln_index[i+1].name_start-naux.name_start-1; // remove the trailing \0
		} else {
			c->l_qname = batch.names.size()-naux.name_start-1;
		}
		c->flag = batch.flags[i];
		c->n_cigar = crq.cigar_len;
		c->l_qseq = crq.read_len;
		c->mtid = batch.next_refIDs[i];
		c->mpos = batch.next_positions[i];
		c->isize = 0; //TODO

		b->l_data = c->l_qname*sizeof(uint8) + crq.cigar_len*sizeof(cigar_op) + crq.qual_len*sizeof(uint8)
					+ (crq.read_len/2)*sizeof(uint8) + naux.aux_data_len*sizeof(uint8);
		if(b->l_data > b->m_data) {
			b->data = (uint8_t*)realloc(b->data, b->l_data);
			b->m_data = b->l_data;
		}
		//qname-cigar-seq-qual-aux
		memcpy(b->data, &batch.names[naux.name_start], c->l_qname);
		memcpy(b->data, &batch.cigars[crq.cigar_start], crq.cigar_len);
		memcpy(b->data, (uint64 *)batch.reads.addrof(crq.read_start), crq.read_len/2);
		memcpy(b->data, &batch.qualities[crq.qual_start], crq.qual_len);
		memcpy(b->data, &batch.aux_data[naux.aux_data_start], naux.aux_data_len);

		sam_write1(fp, header, b);
	}
	free(b->data);
	free(b);
}

void HTSBAMWriter::write_aln_batch(std::vector<bam1_t*>& batch, H_VectorU64 shuffle_idx, bam_hdr_t* header) {
	for(uint64 i = 0; i < batch.size(); i++) {
		sam_write1(fp, header, batch[i]);
	}
}
