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

using namespace nvbio;

#define ASSEMBLY_MIN_BASE_QUALITY 6

struct debruijn_graph
{
	uint32 kmer_size; // kmer_size used to build the graph
	uint32 n_nodes;
	uint32 n_edges;
	uint32 n_subgraphs;

	// global CSR representation
	D_VectorSetKmerCoord nodes;
	D_VectorU32 node_adjancency_map;
	D_VectorU32 node_adj_offests;
	D_VectorU32 node_out_degrees;
	D_VectorU32 node_in_degrees;
	D_VectorU32 edge_counts;	// edge multiplicities
	D_VectorU32 edge_weights;	// transitions probabilities
	D_VectorU8 edge_ref_flags; 	// flags each edge if the reference haplotype contains it

	D_VectorU32 node_in_adjancency_map;
	D_VectorU32 node_in_adj_offests;

	// local active region subgraphs
	D_VectorU32 subgraph_source_ids;
	D_VectorU32 subgraph_sink_ids;
	D_VectorU32 subgraph_n_nodes;

	// topological sort
	D_VectorU32 topo_sorted_uids; // topologically sorted nodes
	D_VectorU32 topo_sorted_offsets;

	// properties
	D_VectorU8 low_complexity;
	D_VectorU8 cycle_presence; // each sub-graph is marked 1 if it contains a cycle


	// kernel view
	struct view
	{
		D_VectorU32::plain_view_type node_adjancency_map;
		D_VectorU32::plain_view_type node_adj_offests;
		D_VectorU32::plain_view_type node_out_degrees;
		D_VectorU32::plain_view_type node_in_degrees;
		D_VectorU32::plain_view_type edge_counts;
		D_VectorU32::plain_view_type edge_weights;

		D_VectorU32::plain_view_type subgraph_source_ids;
		D_VectorU32::plain_view_type subgraph_sink_ids;
		D_VectorU32::plain_view_type subgraph_n_nodes;

		D_VectorU32::plain_view_type topo_sorted_uids;
		D_VectorU32::plain_view_type topo_sorted_offsets;

		D_VectorU8::plain_view_type cycle_presence;

		D_VectorU32::plain_view_type node_in_adjancency_map;
		D_VectorU32::plain_view_type node_in_adj_offests;
	};

	operator view() {
		view v = {
				plain_view(node_adjancency_map),
				plain_view(node_adj_offests),
				plain_view(node_out_degrees),
				plain_view(node_in_degrees),
				plain_view(edge_counts),
				plain_view(edge_weights),
				plain_view(subgraph_source_ids),
				plain_view(subgraph_sink_ids),
				plain_view(subgraph_n_nodes),
				plain_view(topo_sorted_uids),
				plain_view(topo_sorted_offsets),
				plain_view(cycle_presence),
				plain_view(node_in_adjancency_map),
				plain_view(node_in_adj_offests)
		};
		return v;
	}


	// construction
	bool construct_graph(const D_SequenceSet& sequence_set, const D_VectorU32& d_active_region_ids,
			const uint32 kmer_size, const bool allow_low_complexity);
	void compute_edge_weights();
	void compute_in_degrees();
	void compute_in_adjacencies();
	void compute_subgraph_n_nodes();
	void compute_complexity();

	// validation
	bool is_low_complexity(const uint32 subgraph_id) { return low_complexity[subgraph_id];}
	bool has_cycles(const uint32 subgraph_id) { return cycle_presence[subgraph_id];}

	// search
	void topological_sort();
	void find_k_best_paths(const uint32 k);

	// debugging
	void print_dot_graph(const D_SequenceSet& sequence_set);
};

#include "assembly_graph_inl.h"

