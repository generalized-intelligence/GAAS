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

#include <stdio.h>
#include <fstream>
#include "kmers.h"

#define TOPO_SORT_LOCAL_NODE_SET_SIZE 200
#define MAX_IN_DEGREE 20

struct graph_functor
{
	debruijn_graph::view graph;
	NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
	graph_functor(debruijn_graph::view _graph) : graph(_graph) { }
};

struct topological_sort_functor : public graph_functor
{
	NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
	topological_sort_functor(debruijn_graph::view _graph) : graph_functor(_graph) { }

	NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
	void operator() (const uint32 graph_id)
	{
		const uint32 source = graph.subgraph_source_ids[graph_id];
		const uint32 n_nodes = graph.subgraph_n_nodes[graph_id];
		uint32 L_offset = graph.topo_sorted_offsets[graph_id]; // ordered UID output offset
		uint32 n_sorted = 0; // number of nodes sorted so far

		uint32 S[TOPO_SORT_LOCAL_NODE_SET_SIZE];  // set of nodes with in-degree 0
		S[0] = source;
		uint32 s_first = 0;
		uint32 s_last = 1;

		while(s_last - s_first != 0) {
			const uint32 n = S[s_first];
			// remove from S
			s_first++;

			// add to ordered list
			graph.topo_sorted_uids[L_offset + n_sorted] = n;
			n_sorted++;

			// traverse the out neighbors
			uint32 out_degree = graph.node_out_degrees[n];
			uint32 nbr_offset = graph.node_adj_offests[n];
			for(uint32 i = 0; i < out_degree; i++) {
				uint32 m = graph.node_adjancency_map[nbr_offset + i];
				graph.node_in_degrees[m]--;
				if(graph.node_in_degrees[m] == 0) {
					// add to S
					if(s_first > 0) {
						s_first--;
						S[s_first] = m;
					} else {
						if(s_last >= TOPO_SORT_LOCAL_NODE_SET_SIZE) {
							printf("ERROR: Topological sort exceeded max locally allocated set size!\n");
							graph.cycle_presence[graph_id] = true; //TODO: handle this case
						}
						S[s_last] = m;
						s_last++;
					}
				}
			}
		}

		if(n_sorted != n_nodes) {
			printf("Found cycle! %u %u \n", n_sorted, n_nodes);
			graph.cycle_presence[graph_id] = true; // found a cycle!
		}
		graph.cycle_presence[graph_id] = false;
	}
};

struct find_k_best_paths_functor : public graph_functor
{
	const uint32 k;
	uint32* topk_scores;

	NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
	find_k_best_paths_functor(debruijn_graph::view _graph, const uint32 _k, uint32* _topk_scores) :
	graph_functor(_graph), k(_k), topk_scores(_topk_scores) { }

	NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
	void operator() (const uint32 graph_id)
	{
		const uint32 n_nodes = graph.subgraph_n_nodes[graph_id];
		const uint32 order_offset = graph.topo_sorted_offsets[graph_id];

		for(uint32 i = 0; i < n_nodes; i++) { // for each node in topological order
			const uint32 n = graph.topo_sorted_uids[order_offset + i];
			const uint32 in_degree = graph.node_in_degrees[n];
			const uint32 in_nbr_offset = graph.node_in_adj_offests[n];

			// compute the top k paths to n
			uint8 nbr_top_offsets[MAX_IN_DEGREE]; // keep track of how many paths consumed per in-neighbor
			for(uint32 j = 0; j < k; j++) {
				uint32 max_score = 0;
				bool set_score = false;
				for(uint32 v = 0; v < in_degree; v++) {
					const uint32 m = graph.node_in_adjancency_map[in_nbr_offset + v];
					uint32 j_score_offset = nbr_top_offsets[v]*n_nodes + m;
					if(topk_scores[j_score_offset] == 0) continue;
					set_score = true;

					// find edge weight (TODO: pre-compute this)
					uint32 edge_weight = 0;
					for(uint32 l = 0; l < graph.node_out_degrees[m]; l++) {
						if(graph.node_adjancency_map[graph.node_adj_offests[m + l]] == m) {
							edge_weight = graph.edge_weights[graph.node_adj_offests[m + l]];
							break;
						}
					}

					uint32 new_score = topk_scores[j_score_offset] + edge_weight;
					if(new_score > max_score) {
						max_score = new_score;
						nbr_top_offsets[v]++;
					}
				}
				if(set_score) {
					topk_scores[j*n_nodes + n] = max_score;
					// TODO: keep pointers
				} else {
					break; // consumed all the paths to this node
				}
			}
		}
	}
};

struct edge_weights_functor : public graph_functor
{
	NVBIO_HOST_DEVICE
	edge_weights_functor(debruijn_graph::view _graph) : graph_functor(_graph) { }

	NVBIO_DEVICE
	void operator() (const uint32 node_uid)
	{
		const uint32 nbr_offset = graph.node_adjancency_map[node_uid];
		const uint32 out_degree = graph.node_in_degrees[node_uid];

		uint32 total_support = 0; // TODO: compute as a reduction
		for(uint32 i = 0; i < out_degree; i++) {
			total_support += graph.edge_counts[nbr_offset + i];
		}

		for(uint32 i = 0; i < out_degree; i++) {

			graph.edge_weights[nbr_offset + i] = log10((float)graph.edge_counts[nbr_offset + i]) - log10((float)total_support);
		}

	}
};

struct flip_edges : public graph_functor
{
	NVBIO_HOST_DEVICE
	flip_edges(debruijn_graph::view _graph) : graph_functor(_graph) { }

	NVBIO_DEVICE // TODO: add atomicCAS to atomics.h for both host and device
	void operator() (const uint32 node_uid)
	{
		const uint32 nbr_offset = graph.node_adjancency_map[node_uid];
		const uint32 n_nbrs = graph.node_in_degrees[node_uid];

		for(uint32 i = 0; i < n_nbrs; i++) {
			const uint32 nbr_uid = graph.node_adjancency_map[nbr_offset + i];
			const uint32 offset = graph.node_in_adj_offests[nbr_uid];
			// atomically get an index
			uint32 idx = 0;
			uint32 val = atomicCAS(&graph.node_in_adjancency_map[idx], (uint32) -1, node_uid);
			while(val != (uint32) -1) {
				idx++;
				val = atomicCAS(&graph.node_in_adjancency_map[idx], (uint32) -1, node_uid);
			}
		}

	}
};

// converts a coordinate to the corresponding sequence -- used for debugging
template <typename string_set_type>
struct coord_to_seq
{
	typedef typename string_set_type::string_type sequence;
	const string_set_type   string_set;
	const uint32            kmer_size;
	const SequenceSetKmerCoord* nodes;
	uint8* sequences;

	NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
	coord_to_seq(const uint32 _kmer_size, const string_set_type _string_set, const SequenceSetKmerCoord* _nodes, uint8* _sequences) :
		kmer_size(_kmer_size), string_set(_string_set), nodes(_nodes),
		sequences(_sequences) {}

	NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
	void operator() (const uint64 node_idx) const
	{
		const SequenceSetKmerCoord node_coord = nodes[node_idx];
		const sequence seq = string_set[node_coord.x];
		const uint32 seq_pos = node_coord.y;
		const uint64 offset = kmer_size * node_idx;

		for (uint32 i = 0; i < kmer_size; i++) {
			uint8 c = dna_to_char(seq[seq_pos + i]);
			sequences[offset + i] = c;
		}
	}
};



template <typename string_set_type>
struct extract_active_region_source_sink
{
	const string_set_type   string_set;
	const SequenceSetKmerCoord* coords;
	const uint32* seq_active_region_ids;
	const uint32* global_to_UID_map;
	const uint32* global_to_sorted_map;
	uint32* source_ids;
	uint32* sink_ids;
	uint32 kmer_size;

	NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
	extract_active_region_source_sink(const string_set_type _string_set, const uint32 _kmer_size,
			const SequenceSetKmerCoord* _coords,
			const uint32* _seq_active_region_ids, const uint32* _global_to_UID_map, const uint32* _global_to_sorted_map,
			uint32* _source_ids, uint32* _sink_ids):
			string_set(_string_set), kmer_size(_kmer_size),
			coords(_coords), seq_active_region_ids(_seq_active_region_ids),
			global_to_UID_map(_global_to_UID_map), global_to_sorted_map(_global_to_sorted_map),
			source_ids(_source_ids), sink_ids(_sink_ids) { }

	NVBIO_FORCEINLINE NVBIO_HOST_DEVICE
	void operator() (const uint32 global_coord_idx) const
	{
		const SequenceSetKmerCoord source_coord = coords[global_to_sorted_map[global_coord_idx]];

		// determine if the coord is the source coord of a reference sequence (first seq in each region)
		if(global_coord_idx != 0 && coords[global_to_sorted_map[global_coord_idx - 1]].w == source_coord.w) return;
		// set the source
		source_ids[source_coord.w] = global_to_UID_map[source_coord.z];

		// find and set the sink
		const uint32 seq_len = string_set[source_coord.x].length();
		const uint32 n_kmers = seq_len - kmer_size + 1;
		const SequenceSetKmerCoord sink_coord = coords[global_to_sorted_map[global_coord_idx + n_kmers - 1]]; // last coord of the ref
		sink_ids[sink_coord.w] = global_to_UID_map[sink_coord.z];
	}
};

// construct a debruijn assembly graph with the given kmer length
bool debruijn_graph::construct_graph(const D_SequenceSet& d_sequence_set, const D_VectorU32& d_active_region_ids,
		const uint32 _kmer_size, bool allow_low_complexity)
{
	kmer_size = _kmer_size;
	D_KmerSet<D_SequenceSet> graph_kmer_set(d_sequence_set, d_active_region_ids);

	// --- NODE extraction ---
	graph_kmer_set.kmer_size = kmer_size;
	graph_kmer_set.gen_kmer_coords();
	graph_kmer_set.init_alloc_temp_space();
	graph_kmer_set.gen_kmer_64b_keys();
	graph_kmer_set.sort_kmers_by_64b_keys();
	graph_kmer_set.partition_kmers_by_uniqueness();
	graph_kmer_set.gen_global_to_sorted_id_map();
	graph_kmer_set.mark_unique_kmers();
	graph_kmer_set.extract_super_kmers(); 	// handle kmers that are repeats
	segmented_sort_super_kmers_lexicographic(
			graph_kmer_set.string_set,
			graph_kmer_set.n_super_coords,
			graph_kmer_set.super_coords,
			graph_kmer_set.super_prefix_uids,
			graph_kmer_set.super_suffix_uids);

	// extract repeat kmer nodes (as well as U-R, R-R, and R-U repeat edges)
	D_VectorU32 repeat_edges_from_uids;
	D_VectorU32 repeat_edges_to_uids;
	D_VectorU32 repeat_edge_counts;
	graph_kmer_set.collapse_and_extract_non_overlaps(nodes, repeat_edges_from_uids, repeat_edges_to_uids, repeat_edge_counts);

	// extract unique kmer nodes
	thrust::copy_n(
			thrust::make_permutation_iterator(
					thrust::make_permutation_iterator( // unique coordinates
							graph_kmer_set.coords.begin(),
							graph_kmer_set.kmers_64b_unique_idxs.begin()),
					thrust::make_permutation_iterator(
							graph_kmer_set.global_to_UID_map.begin(), // global id -> UID
							thrust::make_transform_iterator( // coord -> global ids
									thrust::make_permutation_iterator( // unique coordinates
											graph_kmer_set.coords.begin(),
											graph_kmer_set.kmers_64b_unique_idxs.begin()),
									get_global_id()))),
			graph_kmer_set.n_unique,
			nodes.begin());

	// extract the source and sink subgraph nodes (for each active region)
	/*n_subgraphs = d_active_region_ids[d_active_region_ids.size()-1] + 1;
	subgraph_source_ids.resize(n_subgraphs);
	subgraph_sink_ids.resize(n_subgraphs);
	thrust::for_each(
			thrust::make_counting_iterator(0u),
			thrust::make_counting_iterator(0u) + graph_kmer_set.n_kmers,
			extract_active_region_source_sink<D_SequenceSet>(d_sequence_set, kmer_size,
					plain_view(graph_kmer_set.coords), plain_view(d_active_region_ids),
					plain_view(graph_kmer_set.global_to_UID_map), plain_view(graph_kmer_set.global_to_sorted_id_map),
					plain_view(subgraph_source_ids), plain_view(subgraph_sink_ids)));*/

	printf("Number of kmers %u \n", graph_kmer_set.n_kmers);
	printf("Number distinct kmers %u \n", graph_kmer_set.n_distinct);
	printf("Number unique kmers %u \n", graph_kmer_set.n_unique);

	// --- ADJACENCY MAP construction ---
	uint32 n_unique_nodes = graph_kmer_set.n_unique;
	n_nodes = graph_kmer_set.n_unique + graph_kmer_set.n_repeat;
	node_adj_offests.resize(n_nodes + 1);
	node_out_degrees.resize(n_nodes); // number of outgoing edges per nodes

	// generate edge (k+1)mers
	graph_kmer_set.kmer_size = kmer_size + 1;
	graph_kmer_set.gen_kmer_coords();
	graph_kmer_set.filter_coords_by_prefix_uniqueness(graph_kmer_set.global_unique_flag_map); // keep only the edges starting with a unique kmer
	graph_kmer_set.gen_kmer_64b_keys();
	graph_kmer_set.segmented_sort_kmers_by_64b_keys(); // to guarantee same prefix edges are consecutive by region
	graph_kmer_set.count_kmers(); // U-U and U-R edge counts

	// get the prefix node UID of each U-U and U-R edge
	D_VectorU32 unique_prefix_node_uids(graph_kmer_set.n_unique);
	thrust::transform(
			thrust::make_transform_iterator(
					graph_kmer_set.kmers_64b_unique_idxs.begin(),
					get_prefix_global_id_by_idx(plain_view(graph_kmer_set.coords))),
			thrust::make_transform_iterator(
					graph_kmer_set.kmers_64b_unique_idxs.begin() + graph_kmer_set.n_unique,
					get_prefix_global_id_by_idx(plain_view(graph_kmer_set.coords))),
			unique_prefix_node_uids.begin(),
			global_to_uid(plain_view(graph_kmer_set.global_to_UID_map)));

	D_VectorU32 unique_suffix_node_uids(graph_kmer_set.n_unique); // TODO: fuse with above
	thrust::transform(
			thrust::make_transform_iterator(
					graph_kmer_set.kmers_64b_unique_idxs.begin(),
					get_suffix_global_id_by_idx(plain_view(graph_kmer_set.coords))),
			thrust::make_transform_iterator(
					graph_kmer_set.kmers_64b_unique_idxs.begin() + graph_kmer_set.n_unique,
					get_suffix_global_id_by_idx(plain_view(graph_kmer_set.coords))),
			unique_suffix_node_uids.begin(),
			global_to_uid(plain_view(graph_kmer_set.global_to_UID_map)));

	// count the number of outgoing edges per unique kmer (i.e. of type U-U and U-R)
	// reduce by UID (edges starting with the same unique kmer will be consecutive)
	D_VectorU32 unique_prefix_uids_idx(n_unique_nodes);
	D_VectorU32 unique_prefix_counts(n_unique_nodes);
	uint32 n_unique_prefix_uids = thrust::reduce_by_key(
			thrust::make_counting_iterator(0),
			thrust::make_counting_iterator(0) + graph_kmer_set.n_unique,
			thrust::constant_iterator<uint32>(1),
			unique_prefix_uids_idx.begin(),
			unique_prefix_counts.begin(),
			kmer_uid_eq(plain_view(unique_prefix_node_uids))).first - unique_prefix_uids_idx.begin();

	// scatter counts based on UIDs
	thrust::scatter(
			unique_prefix_counts.begin(),
			unique_prefix_counts.begin() + n_unique_prefix_uids,
			thrust::make_permutation_iterator(
					unique_prefix_node_uids.begin(),
					unique_prefix_uids_idx.begin()),
			node_out_degrees.begin());

	// count the number of outgoing edges per repeat kmer (i.e. of type U-R, R-R and R-U)
	thrust::sort_by_key(
			repeat_edges_from_uids.begin(),
			repeat_edges_from_uids.end(),
			thrust::make_zip_iterator(thrust::make_tuple(repeat_edges_to_uids.begin(), repeat_edge_counts.begin())));

	D_VectorU32 repeat_prefix_uids_idx(repeat_edges_from_uids.size());
	D_VectorU32 repeat_prefix_counts(repeat_edges_from_uids.size());
	uint32 n_repeat_prefix_uids = thrust::reduce_by_key(
			thrust::make_counting_iterator(0),
			thrust::make_counting_iterator(0) + repeat_edges_from_uids.size(),
			thrust::constant_iterator<uint32>(1),
			repeat_prefix_uids_idx.begin(),
			repeat_prefix_counts.begin(),
			kmer_uid_eq (plain_view(repeat_edges_from_uids))).first - repeat_prefix_uids_idx.begin();

	// we need to discard the beginning of this vector corresponding to U-R edges
	// by counting the number of unique prefixes
	thrust::device_vector<uint8> temp_storage;
	uint32 n_unique_prefixes = cuda::reduce(
			n_repeat_prefix_uids,
			thrust::make_transform_iterator(
					thrust::make_permutation_iterator(
							repeat_edges_from_uids.begin(),
							repeat_prefix_uids_idx.begin()),
					is_unique_uid(n_unique_nodes)),
			thrust::plus<uint32>(),
			temp_storage);

	// scatter counts based on UIDs
	thrust::scatter(
			repeat_prefix_counts.begin() + n_unique_prefixes,
			repeat_prefix_counts.begin() + n_repeat_prefix_uids,
			thrust::make_permutation_iterator(
					repeat_edges_from_uids.begin(),
					repeat_prefix_uids_idx.begin() + n_unique_prefixes),
			node_out_degrees.begin());

	// prefix sum to get the offsets
	cuda::inclusive_scan(
			n_nodes,
			node_out_degrees.begin(),
			node_adj_offests.begin() + 1,
			thrust::plus<uint32>(),
			temp_storage);

	// fill out the adjacency map: U-R, R-R, R-U edges
	n_edges = node_adj_offests[n_nodes];
	node_adjancency_map.resize(n_edges);
	edge_counts.resize(n_edges);
	thrust::for_each(
			repeat_prefix_uids_idx.begin(),
			repeat_prefix_uids_idx.begin() + n_repeat_prefix_uids,
			extract_repeat_adjacencies(
					plain_view(node_adj_offests),
					plain_view(repeat_edges_from_uids),
					plain_view(repeat_edges_to_uids),
					plain_view(repeat_edge_counts),
					plain_view(node_adjancency_map),
					plain_view(edge_counts)));

	// fill out U-U edges
	thrust::for_each(
			unique_prefix_uids_idx.begin(),
			unique_prefix_uids_idx.begin() + n_unique_prefix_uids,
			extract_unique_adjacencies(
					plain_view(node_adj_offests),
					plain_view(unique_prefix_node_uids),
					plain_view(unique_suffix_node_uids),
					plain_view(graph_kmer_set.kmers_counts),
					plain_view(node_adjancency_map),
					plain_view(edge_counts)));

//	// TODO: prune low weight chains
//	// TODO: merge chains

	return true;
}

// --- note: the functionality below has not yet been fully tested

void debruijn_graph::compute_edge_weights()
{
	edge_weights.resize(n_edges);
	thrust::for_each(
		thrust::make_counting_iterator(0u),
		thrust::make_counting_iterator(0u) + n_nodes,
		edge_weights_functor(*this));
}

// compute the in-degree of each node in the graph
// the number of times a node UID occurs in the adjacency map
// is the node's in-degree
void debruijn_graph::compute_in_degrees()
{
	// sort the adjacency map
	D_VectorU32 adj_map(node_adjancency_map);
	thrust::sort(adj_map.begin(), adj_map.end());

	D_VectorU32 distinct_node_UIDs(n_nodes);
	D_VectorU32 counts(n_nodes);
	uint32 n_distinct = thrust::reduce_by_key(
			adj_map.begin(),
			adj_map.end(),
			thrust::make_constant_iterator<uint32>(1),
			distinct_node_UIDs.begin(),
			counts.begin()).first - distinct_node_UIDs.begin();

	node_in_degrees.resize(n_nodes);
	thrust::scatter(
			counts.begin(),
			counts.begin() + n_distinct,
			distinct_node_UIDs.begin(),
			node_in_degrees.begin());

	// prefix sum to get the offsets
	thrust::device_vector<uint8> temp_storage;
	node_in_adj_offests.resize(n_nodes + 1);
	cuda::inclusive_scan(
			n_nodes,
			node_in_degrees.begin(),
			node_in_adj_offests.begin() + 1,
			thrust::plus<uint32>(),
			temp_storage);
}

void debruijn_graph::compute_in_adjacencies()
{
	// sort the adjacency map
	node_in_adjancency_map.resize(n_edges);
	thrust::fill(node_in_adjancency_map.begin(), node_in_adjancency_map.end(), (uint32) -1);

	thrust::for_each(
			thrust::make_counting_iterator(0u),
			thrust::make_counting_iterator(0u) + n_nodes,
			flip_edges(*this));
}

void debruijn_graph::compute_subgraph_n_nodes()
{
	// sort the adjacency map
	D_VectorU32 active_region_ids(n_nodes);
	thrust::transform(
			nodes.begin(),
			nodes.begin() + n_nodes,
			active_region_ids.begin(),
			get_kmer_reg_id());

	thrust::sort(active_region_ids.begin(), active_region_ids.end());

	D_VectorU32 active_regions(n_subgraphs);
	D_VectorU32 counts(n_subgraphs);
	uint32 n_distinct = thrust::reduce_by_key(
			active_region_ids.begin(),
			active_region_ids.end(),
			thrust::make_constant_iterator<uint32>(1),
			active_regions.begin(),
			counts.begin()).first - active_regions.begin();

	subgraph_n_nodes.resize(n_subgraphs);
	thrust::scatter(
			counts.begin(),
			counts.begin() + n_distinct,
			active_regions.begin(),
			subgraph_n_nodes.begin());

	// prefix sum to get the offsets
	thrust::device_vector<uint8> temp_storage;
	topo_sorted_offsets.resize(n_subgraphs + 1);
	cuda::inclusive_scan(
			n_subgraphs,
			subgraph_n_nodes.begin(),
			topo_sorted_offsets.begin() + 1,
			thrust::plus<uint32>(),
			temp_storage);
}

void debruijn_graph::compute_complexity()
{
	//returns true if n_repeats * 4 > n_unique
}

// performs a topological sort of the nodes in each subgraph
// for each subgraph: all nodes are on a path from source to sink
void debruijn_graph::topological_sort()
{
	compute_in_degrees();
	compute_subgraph_n_nodes();

	topo_sorted_uids.resize(n_nodes);
	cycle_presence.resize(n_subgraphs);
	thrust::for_each(
			thrust::make_counting_iterator(0u),
			thrust::make_counting_iterator(0u) + n_subgraphs,
			topological_sort_functor(*this));
}

void debruijn_graph::find_k_best_paths(const uint32 k)
{
	compute_in_degrees();
	compute_in_adjacencies();

	// allocate top k space for each node
	D_VectorU32 topk_scores(k*n_nodes);
	thrust::for_each(
			thrust::make_counting_iterator(0u),
			thrust::make_counting_iterator(0u) + n_subgraphs,
			find_k_best_paths_functor(*this, k, plain_view(topk_scores)));
}

// writes a DOT graph representation
void debruijn_graph::print_dot_graph(const D_SequenceSet& string_set)
{
	D_VectorU8 d_labels(kmer_size*n_nodes);
	thrust::for_each(
			thrust::make_counting_iterator<uint64>(0u),
			thrust::make_counting_iterator<uint64>(0u) + n_nodes,
			coord_to_seq<D_SequenceSet>(kmer_size, string_set, plain_view(nodes), plain_view(d_labels)));
	H_VectorU8 labels = d_labels;

	std::ofstream dot_file;
	dot_file.open("cuda_test_graph.txt");
	for(uint64 i = 0; i < n_nodes; i++) {
		uint32 n_out_edges = node_adj_offests[i+1] - node_adj_offests[i];
		uint64 offset = node_adj_offests[i];

		uint64 label_offset = i*kmer_size;
		std::string node_seq(labels.begin() + label_offset, labels.begin() + label_offset + kmer_size);

		for(uint32 j = 0; j < n_out_edges; j++) {
			uint64 nbr_uid = node_adjancency_map[offset + j];
			//uint32 count = edge_counts[offset + j];

			label_offset = nbr_uid*kmer_size;
			std::string nbr_seq(labels.begin() + label_offset, labels.begin() + label_offset + kmer_size);

			dot_file << node_seq << "_" << i << "\t" << nbr_seq << "_" << nbr_uid << "\n";
		}
	}
	dot_file.close();

	dot_file.open("cuda_test_graph.dot");

	dot_file << "digraph assemblyGraphs { \n";
	for(uint64 i = 0; i < n_nodes; i++) {
		uint32 n_out_edges = node_adj_offests[i+1] - node_adj_offests[i];
		uint64 offset = node_adj_offests[i];

		uint64 label_offset = i*kmer_size;
		std::string node_seq(labels.begin() + label_offset, labels.begin() + label_offset + kmer_size);

		for(uint32 j = 0; j < n_out_edges; j++) {
			uint64 nbr_uid = node_adjancency_map[offset + j];
			uint32 count = edge_counts[offset + j];

			label_offset = nbr_uid*kmer_size;
			std::string nbr_seq(labels.begin() + label_offset, labels.begin() + label_offset + kmer_size);

			dot_file << node_seq << "_" << i << " -> " << nbr_seq << "_" << nbr_uid << " [label=\"" << count<< "\"];\n";
		}
	}

	for(uint64 i = 0; i < n_nodes; i++) {
		uint64 offset = i*kmer_size;
		std::string node_seq(labels.begin() + offset, labels.begin() + offset + kmer_size);
		dot_file << node_seq << "_" << i << " [label=\"" << node_seq << "\",shape=box];\n";
	}
	dot_file << "}";
	dot_file.close();
}
