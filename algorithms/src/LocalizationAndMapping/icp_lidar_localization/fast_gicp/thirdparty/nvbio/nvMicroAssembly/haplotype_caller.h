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

#include "assembly_types.h"
#include "bam_io.h"
#include "kmers.h"
#include "regions.h"
#include "assembly_graph.h"

using namespace nvbio;

/** HaplotypeCaller Pipeline Structures **/

struct assembly_pipeline
{
	// ---- Parameters ----

	const static uint32 kmer_size = 10;
	const static uint32 k_best_haplotypes = 12;
	const static uint32 active_region_size = 100;
	const static bool increaze_kmer_size_for_cycles = false;
	const static uint32 max_reads_in_region_per_sample = 1000;
	const static uint32 min_reads_per_alignment_start = 5;
	const static uint32 max_ref_size = 1000;
	const static uint32 ref_padding = 100;

	// ---- Data ----

	// reference genome
	io::SequenceDataDevice d_ref;
	RefSequenceSet d_ref_seqs;

	// active regions
	D_VectorU32	d_active_region_ids;
	D_VectorU32 d_active_region_offsets;

	// read sequences
	D_SequenceSet d_seq_set;
	D_VectorDNA d_set_seqs_data;
	D_VectorU64 d_set_offsets;

	// read correction

	// assembly results


	// load the reference genome on the device
	void load_ref(io::SequenceDataHost& h_ref) {
		d_ref = h_ref;
		const io::SequenceDataAccess<DNA> d_ref_access(d_ref);
		d_ref_seqs = d_ref_access.sequence_string_set();
	}

	void prep_assembly_sequences(const io::SequenceDataHost& h_ref,
			const BAM_alignment_batch_SoA& read_batch,
			const H_VectorActiveRegions active_regions)
	{
		// prepare the ref haplotypes and the reads overlapping the regions
		const io::SequenceDataAccess<DNA_N> h_ref_access(h_ref);
		const io::SequenceDataAccess<DNA_N>::sequence_string_set_type h_ref_seqs = h_ref_access.sequence_string_set();
		uint64 n_seqs = read_batch.num_alns + active_regions.size(); // includes the ref sequences
		H_VectorDNA h_seqs;
		H_VectorU64 h_offsets(n_seqs+1);
		H_VectorU32 h_active_region_ids(n_seqs);
		H_VectorU32 h_active_region_offsets(active_regions.size());
		uint64 offset = 0u;

		// load each active region
		uint64 n_seqs_loaded = 0;
		for(uint32 r = 0; r < active_regions.size(); r++) {
			h_active_region_offsets[r] = n_seqs_loaded;
			// reference haplotype
			active_region region = active_regions[r];
			const io::SequenceDataAccess<DNA_N>::sequence_string_set_type::string_type seq = h_ref_seqs[region.genome_loc.contig];
			for(uint32 i = region.genome_loc.start; i <= region.genome_loc.stop; i++) {
				h_seqs.push_back(char_to_dna(nvbio::to_char<DNA_N>(seq[i])));
			}
			h_active_region_ids[n_seqs_loaded] = r;
			h_offsets[n_seqs_loaded] = offset;
			offset += region.genome_loc.stop - region.genome_loc.start + 1;
			n_seqs_loaded++;

			// reads overlapping the region
			for(uint64 i = region.read_batch_offset; i < region.read_batch_offset + region.n_reads; i++) {
				for(uint32 j = 0; j < read_batch.crq_index[i].read_len; j += 2) {
					uint8 c1 = read_batch.reads[read_batch.crq_index[i].read_start + j];
					uint8 c2 = read_batch.reads[read_batch.crq_index[i].read_start + j + 1];
					h_seqs.push_back(char_to_dna(iupac16_to_char(c2)));
					h_seqs.push_back(char_to_dna(iupac16_to_char(c1)));
				}
				h_offsets[n_seqs_loaded] = offset;
				h_active_region_ids[n_seqs_loaded] = r;
				offset += read_batch.crq_index[i].read_len;
				n_seqs_loaded++;
			}
		}
		h_offsets[n_seqs_loaded] = offset;

		cuda::Timer timer;
		timer.start();
		d_active_region_ids = h_active_region_ids;
		//d_active_region_offsets = h_active_region_offsets;
		d_set_seqs_data = h_seqs;
		D_StreamDNA stream((uint32*) plain_view(d_set_seqs_data.m_storage));
		d_set_offsets = h_offsets;
		d_seq_set = D_SequenceSet(n_seqs_loaded, stream, nvbio::plain_view(d_set_offsets));
		timer.stop();
		printf("Total input data transfer time  : %.8fs\n", timer.seconds());
		printf("Loaded %llu sequences \n", n_seqs_loaded);
	}
};
