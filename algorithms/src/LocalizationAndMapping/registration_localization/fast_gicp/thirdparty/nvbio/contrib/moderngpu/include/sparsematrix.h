/******************************************************************************
 * Copyright (c) 2013, NVIDIA CORPORATION.  All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the NVIDIA CORPORATION nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
 * ARE DISCLAIMED. IN NO EVENT SHALL NVIDIA CORPORATION BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************/

/******************************************************************************
 *
 * Code and text by Sean Baxter, NVIDIA Research
 * See http://nvlabs.github.io/moderngpu for repository and documentation.
 *
 ******************************************************************************/

#pragma once

#include "util/static.h"

namespace mgpu {

struct SparseMatrix {
	int height, width, nz;
	std::vector<int> csr;				// height
	std::vector<int> cols;				// nz
	std::vector<double> matrix;			// nz
};

bool ReadSparseMatrix(FILE* f, std::auto_ptr<SparseMatrix>* ppMatrix,
	std::string& err);

bool ReadSparseMatrix(const char* filename,
	std::auto_ptr<SparseMatrix>* ppMatrix, std::string& err);

bool LoadBinaryMatrix(const char* filename,
	std::auto_ptr<SparseMatrix>* ppMatrix);

bool StoreBinaryMatrix(const char* filename, const SparseMatrix& matrix);

bool LoadCachedMatrix(const char* filename, 
	std::auto_ptr<SparseMatrix>* ppMatrix, std::string& err);

// Multiply the matrix by a vector of 1s.
template<typename T>
void SpmvTest(const SparseMatrix& m, T* results) {
	memset(results, 0, sizeof(T) * m.height);
	for(int row = 0; row < m.height; ++row) {
		T product = 0;
		int begin = m.csr[row];
		int end = (row + 1 < m.height) ? m.csr[row + 1] : m.nz;
		for(int i = begin; i < end; ++i)
			product += (T)m.matrix[i];

		results[row] = product;
	}		
}

template<typename T>
void CompareVecs(const T* test, const T* ref, int count) {
	for(int i = 0; i < count; ++i) {
		double x = ref[i];
		double y = test[i];
		double diff = fabs(x - y);

		if(diff > 1.0e-5) {
			if(y > 0) {
				if(1.01 * x < y || 0.99 * x > y) {
					printf("BAD OUTPUT AT COMPONENT %d: %8.5e vs %8.5e\n", i,
						x, y);
				//	exit(0);
					return;
				}
			} else {
				if(1.01 * x > y || 0.99 * x < y) {
					printf("BAD OUTPUT AT COMPONENT %d: %8.5e vs %8.5e\n", i, 
						x, y);
				//	exit(0);
					return;
				}
			}
		}
	}
}

struct MatrixStats {
	int height, width, nz;
	
	// Row density moments:
	double mean;
	double stddev;
	double skewness;
};

MatrixStats ComputeMatrixStats(const SparseMatrix& m);

int64 MulSparseMatrices(const SparseMatrix& A, const SparseMatrix& B,
	std::auto_ptr<SparseMatrix>* ppC);


int64 ComputeProductCount(const SparseMatrix& A, const SparseMatrix& B);

void ComputeColRanges(const SparseMatrix& A, const SparseMatrix& B,
	int* colMin, int* colMax);

} // namespace mgpu
