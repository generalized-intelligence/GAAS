/*
 * Copyright (c) 2012-14, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 *
 *
 *
 *
 *
 *
 *
 *
 */

#include <stdio.h>
#include <zlib/zlib.h>

#include <nvbio/basic/types.h>
#include <nvbio/basic/exceptions.h>

#include "bam_io.h"


using namespace nvbio;

/** ---- Read Functionality ---- **/

BAMReader::BAMReader(const char *fname) : eof(false)
{
	fp = gzopen(fname, "rb");
	if (fp == NULL) {
		throw nvbio::runtime_error("Could not open %s", fname);
	}
	if (!read_hdr()) {
		throw nvbio::runtime_error("Error parsing BAM file header");
	}
}

BAMReader::~BAMReader()
{
	gzclose(fp);
}

#define GZREAD(field)	\
		if (read_data(&(field), sizeof(field), __LINE__) == false) {	\
			eof = true;	\
			return false;	\
		}

// parse the BAM header
bool BAMReader::read_hdr(void)
{
	GZREAD(header.magic);
	if (header.magic[0] != 'B' || header.magic[1] != 'A' ||
			header.magic[2] != 'M' || header.magic[3] != '\1') {
		throw nvbio::runtime_error("error parsing BAM file (invalid magic)");
	}
	GZREAD(header.l_text);
	header.text.resize(header.l_text);
	read_data(&header.text[0], header.l_text, __LINE__);

	// read reference sequence data
	GZREAD(header.n_ref);
	for(int c = 0; c < header.n_ref; c++) {
		std::string name;
		int32 l_name, l_ref;
		GZREAD(l_name);
		name.resize(l_name);
		read_data(&name[0], l_name, __LINE__);
		GZREAD(l_ref);
		header.sq_names.push_back(name);
		header.sq_lengths.push_back(l_ref);
	}
	data_start = gztell(fp);
	return true;
}

// load a batch of alignment records (AoS)
bool BAMReader::read_aln_batch(std::vector<BAM_alignment_record>& batch, const uint64 batch_size)
{
	uint64 aln_id;
	for(aln_id = 0; aln_id < batch_size; aln_id++) {
		if (eof) { break;}

		BAM_alignment_record record;
		GZREAD(record.header.block_size);
		z_off_t read_block_start = gztell(fp);
		GZREAD(record.header.refID);
		GZREAD(record.header.pos);
		GZREAD(record.header.bin_mq_nl);
		GZREAD(record.header.flag_nc);
		GZREAD(record.header.l_seq);
		GZREAD(record.header.next_refID);
		GZREAD(record.header.next_pos);
		GZREAD(record.header.tlen);

		// read the data section
		const uint32 data_len = record.header.block_size - (gztell(fp) - read_block_start);
		record.data.resize(data_len);
		read_data(&record.data[0], data_len, __LINE__);

		// store the record
		batch.push_back(record);
	}
	if (aln_id == 0) {
		return false;
	}
	return true;
}

// load a contiguous batch of raw byte alignment records (AoS)
bool BAMReader::read_aln_batch_raw(BAM_alignment_batch_raw& raw_batch, const uint64 batch_size)
{
	uint64 byte_offset = 0;
	uint64 aln_id;
	raw_batch.recs.resize(H_BATCH_SIZE_ALLOC*sizeof(BAM_alignment_record)); // size doesn't include the record data portion
	for(aln_id = 0; aln_id < batch_size; aln_id++) {
		if (eof) { break;}
		// read the size of the record
		uint32 record_size;
		if(!read_data(&record_size, sizeof(uint32), __LINE__)) {
			eof = true;
			break;
		}
		// allocate space
		raw_batch.recs.resize(byte_offset + record_size + sizeof(uint32));
		raw_batch.offsets.push_back(byte_offset);
		// store the size
		memcpy(&raw_batch.recs[byte_offset], &record_size, sizeof(uint32));
		byte_offset += sizeof(uint32);
		// read the data
		read_data(&raw_batch.recs[byte_offset], record_size, __LINE__);
		byte_offset += record_size;
	}
	if (byte_offset == 0) {
		return false;
	}
	raw_batch.offsets.push_back(byte_offset);
	return true;
}

// load a batch of alignment records (SoA)
// load only the fields specified by the batch field_mask
bool BAMReader::read_aln_batch(BAM_alignment_batch_SoA& batch, const uint64 batch_size)
{
	batch.reset();
	H_VectorU32 cigar_temp(64);
	BAM_alignment_header align;
	for(uint64 aln_id = 0; aln_id < batch_size; aln_id++) {
		if (eof) { break; }

		GZREAD(align.block_size);
		z_off_t read_block_start = gztell(fp);
		GZREAD(align.refID);
		GZREAD(align.pos);
		GZREAD(align.bin_mq_nl);
		GZREAD(align.flag_nc);
		GZREAD(align.l_seq);
		GZREAD(align.next_refID);
		GZREAD(align.next_pos);
		GZREAD(align.tlen);

		if(batch.field_mask & BAM_POSITIONS) {
			batch.positions.push_back(align.pos);
		}

		if(batch.field_mask & BAM_REFIDS) {
			if (align.refID < 0 || align.refID >= header.n_ref) {
				batch.refIDs.push_back(uint32(-1));
			} else {
				batch.refIDs.push_back(align.refID);
			}
		}

		if(batch.field_mask & BAM_MAPQ) {
			batch.mapq.push_back(align.mapq());
		}

		if(batch.field_mask & BAM_FLAGS) {
			batch.flags.push_back(align.flags());
		}

		const uint64 read_name_off = batch.names.size();
		const uint32 read_name_len = align.l_read_name();
		if(batch.field_mask & BAM_NAMES) {
			batch.names.resize(read_name_off + read_name_len + 1);
			read_data(&batch.names[read_name_off], read_name_len, __LINE__);
			batch.names[read_name_off + read_name_len] = '\0';
		} else {
			gzseek(fp, read_name_len, SEEK_CUR);
		}

		BAM_CRQ_index crq_index(batch.cigars.size(), align.num_cigar_ops(), batch.reads.size(), align.l_seq, batch.qualities.size(), align.l_seq);
		if(batch.field_mask & (BAM_CIGARS | BAM_QUALITIES | BAM_READS)) {
			batch.crq_index.push_back(crq_index);
		}

		const uint32 cigar_len = (align.flag_nc & 0xffff);
		if(batch.field_mask & BAM_CIGARS) {
			cigar_temp.resize(cigar_len);
			if (cigar_len) {
				read_data(&cigar_temp[0], sizeof(uint32) * cigar_len, __LINE__);
				for(uint32 c = 0; c < cigar_len; c++) {
					cigar_op op;
					op.op = cigar_temp[c] & 0xf;
					op.len = cigar_temp[c] >> 4;
					batch.cigars.push_back(op);
				}
			}
		} else {
			if (cigar_len) {
				gzseek(fp, sizeof(uint32) * cigar_len, SEEK_CUR);
			}
		}

		if(batch.field_mask & BAM_READS) {
			// figure out the length of the sequence data, rounded up to reach a dword boundary
			const uint32 padded_read_len_bp = ((align.l_seq + 7) / 8) * 8;
			// make sure we have enough memory, then read in the sequence
			batch.reads.resize(crq_index.read_start + padded_read_len_bp);
			uint64 *storage = (uint64 *)batch.reads.addrof(crq_index.read_start);
			read_data(storage, align.l_seq / 2, __LINE__);
		} else {
			gzseek(fp, align.l_seq / 2, SEEK_CUR);
		}

		if(batch.field_mask & BAM_QUALITIES) {
			batch.qualities.resize(crq_index.read_start + align.l_seq);
			read_data(&batch.qualities[crq_index.read_start], align.l_seq, __LINE__);
		} else {
			gzseek(fp, align.l_seq, SEEK_CUR);
		}

		const uint32 aux_len = align.block_size - (gztell(fp) - read_block_start);
		// align index
		if(batch.field_mask & (BAM_AUX | BAM_NAMES)) {
			BAM_NAUX_index idx(batch.aux_data.size(), aux_len, read_name_off);
			batch.aln_index.push_back(idx);
		}
		if(batch.field_mask & BAM_AUX) {
			const uint64 aux_start = batch.aux_data.size();
			batch.aux_data.resize(aux_start + aux_len);
			read_data(&batch.aux_data[aux_start], aux_len, __LINE__);
		} else {
			gzseek(fp, aux_len, SEEK_CUR);
		}

		batch.num_alns++;
	}

	if (batch.num_alns == 0) {
		return false;
	}
	return true;
}

bool BAMReader::toSoA(const BAM_alignment_batch_raw& batch, BAM_alignment_batch_SoA& soa_batch)
{
	soa_batch.reset();
	BAM_alignment_header align;
	H_VectorU32 cigar_temp(64);
	int offset;
	for(uint64 idx = 0; idx < batch.offsets.size()-1; idx++) {
		uint32 len = batch.offsets[idx + 1] - batch.offsets[idx];
		const uint8* data = &batch.recs.data()[batch.offsets[idx]];

		offset = sizeof(uint32);
		memcpy(&align.refID, data + offset, sizeof(uint32));
		offset += sizeof(uint32);
		memcpy(&align.pos, data + offset, sizeof(uint32));
		offset += sizeof(uint32);
		memcpy(&align.bin_mq_nl, data + offset, sizeof(uint32));
		offset += sizeof(uint32);
		memcpy(&align.flag_nc, data + offset, sizeof(uint32));
		offset += sizeof(uint32);
		memcpy(&align.l_seq, data + offset, sizeof(uint32));
		offset += sizeof(uint32);
		// skip remainder
		offset += 3*sizeof(int32);

		if(soa_batch.field_mask & BAM_REFIDS) {
			if (align.refID < 0 || align.refID >= header.n_ref) {
				soa_batch.refIDs.push_back(uint32(-1));
			} else {
				soa_batch.refIDs.push_back(align.refID);
			}
		}

		if(soa_batch.field_mask & BAM_POSITIONS) {
			soa_batch.positions.push_back(align.pos);
		}

		if(soa_batch.field_mask & BAM_MAPQ) {
			soa_batch.mapq.push_back(align.mapq());
		}

		if(soa_batch.field_mask & BAM_FLAGS) {
			soa_batch.flags.push_back(align.flags());
		}

		const uint32 read_name_off = soa_batch.names.size();
		const uint32 read_name_len = align.l_read_name();
		if(soa_batch.field_mask & BAM_NAMES) {
			soa_batch.names.resize(read_name_off + read_name_len + 1);
			memcpy(&soa_batch.names[read_name_off], data + offset, read_name_len);
			soa_batch.names[read_name_off + read_name_len] = '\0';
		}
		offset += read_name_len;

		BAM_CRQ_index crq_index(soa_batch.cigars.size(), align.num_cigar_ops(),
				soa_batch.reads.size(), align.l_seq, soa_batch.qualities.size(), align.l_seq);
		if(soa_batch.field_mask & (BAM_CIGARS | BAM_QUALITIES | BAM_READS)) {
			soa_batch.crq_index.push_back(crq_index);
		}

		const uint32 cigar_len = (align.flag_nc & 0xffff);
		if(soa_batch.field_mask & BAM_CIGARS) {
			cigar_temp.resize(cigar_len);
			if (cigar_len) {
				memcpy(&cigar_temp[0], data + offset, sizeof(uint32) * cigar_len);
				for(uint32 c = 0; c < cigar_len; c++) {
					cigar_op op;
					op.op = cigar_temp[c] & 0xf;
					op.len = cigar_temp[c] >> 4;
					soa_batch.cigars.push_back(op);
				}
			}
		}
		if (cigar_len) {
			offset += sizeof(uint32)*cigar_len;
		}

		if(soa_batch.field_mask & BAM_READS) {
			// figure out the length of the sequence data, rounded up to reach a dword boundary
			const uint32 padded_read_len_bp = ((align.l_seq + 7) / 8) * 8;
			// make sure we have enough memory, then read in the sequence
			soa_batch.reads.resize(crq_index.read_start + padded_read_len_bp);
			uint64 *storage = (uint64 *)soa_batch.reads.addrof(crq_index.read_start);
			memcpy(storage, data + offset, align.l_seq / 2);
		}
		offset += align.l_seq / 2;

		if(soa_batch.field_mask & BAM_QUALITIES) {
			soa_batch.qualities.resize(crq_index.read_start + align.l_seq);
			memcpy(&soa_batch.qualities[crq_index.read_start], data + offset, align.l_seq);
		}
		offset += align.l_seq;

		const uint32 aux_len = len - offset;
		// align index
		if(soa_batch.field_mask & (BAM_AUX | BAM_NAMES)) {
			BAM_NAUX_index idx(soa_batch.aux_data.size(), aux_len, read_name_off);
			soa_batch.aln_index.push_back(idx);
		}
		if(soa_batch.field_mask & BAM_AUX) {
			const uint64 aux_start = soa_batch.aux_data.size();
			soa_batch.aux_data.resize(aux_start + aux_len);
			memcpy(&soa_batch.aux_data[aux_start], data + offset, aux_len);
		}
		offset += aux_len;

		soa_batch.num_alns++;
	}

	if (soa_batch.num_alns == 0) {
		return false;
	}
	return true;
}

bool BAMReader::read_data(void *output, unsigned int len, int line)
{
	if(len == 0) {
		return true;
	}
	if (eof) {
		return false;
	}
	int ret = gzread(fp, output, len);
	if (ret > 0) {
		return true;
	} else { // check for EOF separately; zlib will not always return Z_STREAM_END at EOF below
		if (gzeof(fp)) {
			eof = true;
		} else { // ask zlib what happened and inform the user
			int err;
			const char *msg = gzerror(fp, &err);
			assert(err != Z_STREAM_END); // we're making the assumption that we never see Z_STREAM_END here
			if (err == 0) {
				ret = gzread(fp, output, len);
				if (ret > 0) {
					return true;
				} else {
					throw nvbio::runtime_error("Error processing BAM file (line %d): zlib error %d (%s) ret = %d", line, err, msg, ret);
				}
			}
		}
		return false;
	}
}

HTSBAMReader::HTSBAMReader(const char *fname)
{
	fp = sam_open(fname, "r");
	if (fp == NULL) {
		throw nvbio::runtime_error("Could not open %s", fname);
	}
	header = sam_hdr_read(fp);
	if (header == NULL) {
		throw nvbio::runtime_error("Error parsing BAM file header");
	}
}

HTSBAMReader::~HTSBAMReader()
{
	bam_hdr_destroy(header);
	sam_close(fp);
}

// returns false if no records were read
bool HTSBAMReader::read_aln_batch(BAM_alignment_batch_SoA& batch, const uint64 batch_size)
{
	batch.reset();
	H_VectorU32 cigar_temp(64);
	bam1_t *b = (bam1_t*)calloc(1, sizeof(bam1_t));
	for(uint64 aln_id = 0; aln_id < batch_size; aln_id++) {
		if (sam_read1(fp, header, b) < 0) break;

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

BAMWriter::BAMWriter(const char *fname)
{
	fp = fopen(fname, "wt");
	if (fp == NULL) {
		throw nvbio::runtime_error("Could not open %s for writing", fname);
	}

	// set a 256kb output buffer on fp and make sure it's not line buffered
	// this makes sure small fwrites do not land on disk straight away
	// (256kb was chosen based on the default stripe size for Linux mdraid RAID-5 volumes)
	setvbuf(fp, NULL, _IOFBF, 256 * 1024);
}

BAMWriter::~BAMWriter()
{
	if (fp) {
		fclose(fp);
		fp = NULL;
	}
}

void BAMWriter::write_header(BAM_header& header) {
	data_buffer.append_data(&header.magic[0], 4);
	data_buffer.append_int32(header.l_text);
	// header text might be larger than the buffer size
	// cannot append the entire string
	if(header.l_text <= data_buffer.get_remaining_size()) {
		data_buffer.append_string(header.text.c_str());
	} else {
		const char* header_text = header.text.c_str();
		for(int i = 0; i < header.l_text; i++) {
			if (data_buffer.is_full()) {
				write_block(data_buffer);
			}
			data_buffer.append_int8(header_text[i]);
		}
	}
	if (data_buffer.is_full()) {
		write_block(data_buffer);
	}

	data_buffer.append_int32(header.n_ref);
	for(int i = 0; i < header.n_ref; i++) {
		data_buffer.append_int32((int32) header.sq_names[i].length());

		if(header.sq_names[i].length() <= data_buffer.get_remaining_size()) {
			data_buffer.append_string(header.sq_names[i].c_str());
		} else {
			const char* seq_text = header.sq_names[i].c_str();
			for(uint32 j = 0; j < header.sq_names[i].length(); j++) {
				if (data_buffer.is_full()) {
					write_block(data_buffer);
				}
				data_buffer.append_int8(seq_text[j]);
			}
		}
		if (data_buffer.is_full()) {
			write_block(data_buffer);
		}
		data_buffer.append_int8(0);
		data_buffer.append_int32(header.sq_lengths[i]);
	}

	if (data_buffer.get_pos()) {
		write_block(data_buffer);
	}
}

// write out raw byte record data in the order specified by shuffle_idx
void BAMWriter::write_aln_batch_raw(BAM_alignment_batch_raw& batch, H_VectorU64 shuffle_idx)
{
	for(uint64 i = 0; i < shuffle_idx.size(); i++) {
		uint64 idx = shuffle_idx[i];
		uint32 len = batch.offsets[idx + 1] - batch.offsets[idx];
		// records can be of varying size, must ensure sufficient buffer space
		if(len <= data_buffer.get_remaining_size()) {
			data_buffer.append_data(&batch.recs[batch.offsets[idx]], len);
		} else {
			for(uint32 j = 0; j < len; j++) {
				if (data_buffer.is_full()) {
					write_block(data_buffer);
				}
				data_buffer.append_int8(batch.recs[batch.offsets[idx + j]]);
			}
		}
		if (data_buffer.is_full()) {
			write_block(data_buffer);
		}
	}
	if (data_buffer.get_pos()) {
		write_block(data_buffer);
	}
}

void BAMWriter::write_block(io::DataBuffer& block)
{
	io::DataBuffer compressed;
	bgzf.start_block(compressed);
	bgzf.compress(compressed, block);
	bgzf.end_block(compressed);
	fwrite(compressed.get_base_ptr(), compressed.pos, 1, fp);
	block.rewind();
}

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
