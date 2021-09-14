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

#include <nvbio/basic/cuda/timer.h>

#include "assembly_types.h"
#include "haplotype_caller.h"
#include "bam_io.h"
#include "assembly.h"

// loads reads overlapping multiple active regions from a BAM file
// BAM index files should be used instead for better performance
// note: start_pos is 0-based
void load_active_regions_temp(const char* bam_fname,
		const uint32 n_active_regions,
		const uint32 active_region_size,
		uint32 start_pos,
		H_VectorActiveRegions& active_regions,
		BAM_alignment_batch_SoA& h_batch)
{
	HTSBAMReader bam_reader(bam_fname);
	for(uint32 i = 0; i < n_active_regions; i++) {
		active_region r;
		r.genome_loc.contig = 0u;
		r.genome_loc.start = start_pos;
		r.genome_loc.stop = start_pos + active_region_size - 1;
		r.read_batch_offset = h_batch.num_alns;
		if(!bam_reader.read_aln_batch_intv(h_batch, r.genome_loc.contig, r.genome_loc.start, r.genome_loc.stop)) {
			printf("No reads were loaded for region %u starting at %u. \n", i, start_pos);
		}
		r.n_reads = h_batch.num_alns - r.read_batch_offset;
		active_regions.push_back(r);
		start_pos += active_region_size;
	}
}

// main local de-novo assemply engine
void haplotype_caller_pipeline(const char* ref_fname, const char* bam_fname,
		const uint32 n_regions, const uint32 start_pos /*0-based*/)
{
	cuda::Timer timer;
	assembly_pipeline pipeline;

	// 1. load the genome reference
	io::SequenceDataHost h_ref;
	if (!io::load_sequence_file(DNA_N, &h_ref, ref_fname)) {
		printf("Failed to load the FASTA reference file \"%s\"\n", ref_fname);
		exit(1);
	}

	// 2. load BAM alignments (BAM must be sorted)
	BAM_alignment_batch_SoA h_read_batch(BAM_READS);
	H_VectorActiveRegions h_active_regions;
	load_active_regions_temp(bam_fname, n_regions, pipeline.active_region_size, start_pos, h_active_regions, h_read_batch);

	// 3. prepare the active region reference haplotypes and the read sequences for assembly
	pipeline.prep_assembly_sequences(h_ref, h_read_batch, h_active_regions);

	// 4. de-novo local assembly
	timer.start();
	local_assembly(&pipeline);
	timer.stop();
	printf("Total assembly time  : %.8fs\n", timer.seconds());

}

int main(int argc, char **argv)
{
	if(argc < 3) {
		printf("Usage: ./assembly [options] <ref_file> <bam_file> <n_regions> <start_pos>\n");
		exit(1);
	}

	try {
		haplotype_caller_pipeline(argv[1], argv[2], atoi(argv[3]), atoi(argv[4])-1 /*1-based */);
	}
	catch (nvbio::runtime_error& e) {
		printf("%s\n", e.what());
		exit(1);
	}
	return 0;
}
