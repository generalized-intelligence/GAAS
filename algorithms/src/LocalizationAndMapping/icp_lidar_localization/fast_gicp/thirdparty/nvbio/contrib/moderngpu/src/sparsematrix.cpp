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

#include <algorithm>
#include <sstream>
#include <map>
#include "sparsematrix.h"
#include "mmio.h"

namespace mgpu {

struct MatrixElement {
	int row, col;
	double value;
	bool operator<(const MatrixElement& b) const {
		if(row < b.row) return true;
		if(b.row < row) return false;
		return col < b.col;
	}
};

bool ReadSparseMatrix(FILE* f, std::auto_ptr<SparseMatrix>* ppMatrix,
	std::string& err) {

	MM_typecode matcode;
	int code = mm_read_banner(f, &matcode);
	if(code) {
		err = "Not a sparse matrix.\n";
		return false;
	}

	bool isPattern = mm_is_pattern(matcode);
	bool isReal = mm_is_real(matcode);

	if(!(isPattern || isReal) || !mm_is_matrix(matcode) || 
		!mm_is_coordinate(matcode) || !mm_is_sparse(matcode)) {
		err = "Not a real matrix.\n";
		return false;
	}

	int nz;
	int height, width;
	code = mm_read_mtx_crd_size(f, &height, &width, &nz);
	std::vector<MatrixElement> elements;
	elements.reserve(nz);

	for(int i = 0; i < nz; ++i) {
		MatrixElement e;
		int x, y;

		if(isReal) {
			int count = fscanf(f, "%d %d %lf", &y, &x, &e.value);
			if(3 != count) {
				std::ostringstream oss;
				oss<< "Error parsing line "<< (i + 1);
				err = oss.str();
				return false;
			}
		} else if(isPattern) {
			int count = fscanf(f, "%d %df", &y, &x);
			if(2 != count) {
				std::ostringstream oss;
				oss<< "Error parsing line "<< (i + 1);
				err = oss.str();
				return false;
			}
			e.value = 1;
		}
		e.row = y - 1;
		e.col = x - 1;
		elements.push_back(e);

		if((mm_is_symmetric(matcode) || mm_is_skew(matcode)) && (x != y)) {
			std::swap(e.row, e.col);
			if(mm_is_skew(matcode)) e.value = -e.value;
			elements.push_back(e);
		}
	}

	std::sort(elements.begin(), elements.end());

	std::auto_ptr<SparseMatrix> m(new SparseMatrix);
	
	nz = (int)elements.size();
	m->height = height;
	m->width = width;
	m->nz = nz;
	m->csr.resize(height + 1);
	m->cols.resize(nz);
	m->matrix.resize(nz);
	for(int i = 0; i < nz; ++i) {
		MatrixElement e = elements[i];
		++m->csr[e.row];
		m->cols[i] = e.col;
		m->matrix[i] = e.value;
	}

	// scan the counts into CSR.
	int scan = 0;
	for(int i = 0; i < height; ++i) {
		int count = m->csr[i];
		m->csr[i] = scan;
		scan += count;
	}
	m->csr[height] = scan;

	*ppMatrix = m;

	return true;
}

bool ReadSparseMatrix(const char* filename,
	std::auto_ptr<SparseMatrix>* ppMatrix, std::string& err) {

	FILE* f = fopen(filename, "r");
	if(!f) {
		err = "Could not open file.";
		return false;
	}

	bool success = ReadSparseMatrix(f, ppMatrix, err);
	
	fclose(f);
	return success;
}

bool LoadBinaryMatrix(const char* filename,
	std::auto_ptr<SparseMatrix>* ppMatrix) {

	FILE* f = fopen(filename, "rb");
	if(!f) return false;

	std::auto_ptr<SparseMatrix> m(new SparseMatrix);
	fread(&m->height, 4, 1, f);
	fread(&m->width, 4, 1, f);
	fread(&m->nz, 4, 1, f);

	m->csr.resize(m->height + 1);
	m->cols.resize(m->nz);
	m->matrix.resize(m->nz);

	fread(&m->csr[0], 4, m->height + 1, f);
	fread(&m->cols[0], 4, m->nz, f);
	fread(&m->matrix[0], 8, m->nz, f);

	fclose(f);

	*ppMatrix = m;
	return true;
}

bool StoreBinaryMatrix(const char* filename, const SparseMatrix& m) {

	FILE* f = fopen(filename, "wb");
	if(!f) return false;

	fwrite(&m.height, 4, 1, f);
	fwrite(&m.width, 4, 1, f);
	fwrite(&m.nz, 4, 1, f);

	fwrite(&m.csr[0], 4, m.height + 1, f);
	fwrite(&m.cols[0], 4, m.nz, f);
	fwrite(&m.matrix[0], 8, m.nz, f);

	fclose(f);
	return true;
}

bool LoadCachedMatrix(const char* filename, 
	std::auto_ptr<SparseMatrix>* ppMatrix, std::string& err) {

	// Attempt to load the binary matrix. If that fails, load the Matrix Market
	// ASCII matrix and store as binary.
	std::auto_ptr<SparseMatrix> m;
	char s[260];
	sprintf(s, "%s.bin", filename);
	bool success = LoadBinaryMatrix(s, &m);
	if(!success) {
		success = ReadSparseMatrix(filename, &m, err);
		if(!success) return false;

		printf("Creating temp file %s...\n", s);
		StoreBinaryMatrix(s, *m);
	}
	*ppMatrix = m;
	return true;
}

////////////////////////////////////////////////////////////////////////////////

MatrixStats ComputeMatrixStats(const SparseMatrix& m) {
	MatrixStats stats;
	stats.height = m.height;
	stats.width = m.width;
	stats.nz = m.nz;
	stats.mean = (double)m.nz / m.height;

	// Compute the second moment.
	double variance = 0;
	for(int i = 0; i < m.height; ++i) {
		int count = m.csr[i + 1] - m.csr[i];
		double x = count - stats.mean;
		variance += x * x;
	}
	stats.stddev = sqrt(variance / m.height);

	// Compute the third moment.
	double skewness = 0;
	for(int i = 0; i < m.height; ++i) {
		int count = m.csr[i + 1] - m.csr[i];
		double x = count - stats.mean;
		skewness += x * x * x;
	}
	stats.skewness = skewness / m.height / pow(stats.stddev, 3.0);

	return stats;
}

////////////////////////////////////////////////////////////////////////////////
// MulSparseMatrices

int64 MulSparseMatrices(const SparseMatrix& A, const SparseMatrix& B,
	std::auto_ptr<SparseMatrix>* ppC) {

	std::auto_ptr<SparseMatrix> C(new SparseMatrix);
	C->height = A.height;
	C->width = B.width;
	C->nz = 0;

	int64 numProducts = 0;
	for(int row = 0; row < A.height; ++row) {
		std::map<int, double> rowMap;
		int aCsr0 = A.csr[row];
		int aCsr1 = (row + 1 < A.height) ? A.csr[row + 1] : A.nz;
		
		for(int i = aCsr0; i < aCsr1; ++i) {
			int aCol = A.cols[i];
			double x = A.matrix[i];

			int bCsr0 = B.csr[aCol];
			int bCsr1 = (aCol + 1 < B.height) ? B.csr[aCol + 1] : B.nz;
			
			numProducts += bCsr1 - bCsr0;

			for(int j = bCsr0; j < bCsr1; ++j)
				rowMap[B.cols[j]] += x * B.matrix[j];
		}

		// Read out the row data.
		std::map<int, double>::const_iterator i = rowMap.begin();
		C->csr.push_back(C->nz);
		while(i != rowMap.end()) {
			C->cols.push_back(i->first);
			C->matrix.push_back(i->second);
			++i;
		}
		C->nz += (int)rowMap.size();
	}

	*ppC = C;
	return numProducts;
}


int64 ComputeProductCount(const SparseMatrix& A, const SparseMatrix& B) {
	int64 numProducts = 0;
	for(int ai = 0; ai < A.nz; ++ai) {
		int aCol = A.cols[ai];
		int bCsr0 = B.csr[aCol];
		int bCsr1 = (aCol + 1 < B.width) ? B.csr[aCol + 1] : B.nz;
		numProducts += bCsr1 - bCsr0;
	}
	return numProducts;
}

void ComputeColRanges(const SparseMatrix& A, const SparseMatrix& B,
	int* colMin, int* colMax) {

	// Loop over all NZ elements in A and do two lookups into B.
	for(int row = 0; row < A.height; ++row) {
		int cMin = B.width;
		int cMax = 0;

		int csr0 = A.csr[row];
		int csr1 = A.csr[row + 1];
		for(int i = csr0; i < csr1; ++i) {
			int aCol = A.cols[i];
			cMin = std::min(cMin, B.cols[B.csr[aCol]]);
			cMax = std::max(cMax, B.cols[B.csr[aCol + 1] - 1]);
		}
		colMin[row] = cMin;
		colMax[row] = cMax;
	}
}

} // namespace mgpu
