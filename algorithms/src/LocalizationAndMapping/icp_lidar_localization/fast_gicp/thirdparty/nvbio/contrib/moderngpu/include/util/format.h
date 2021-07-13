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

#include "static.h"
#include <vector>
#include <cstdio>
#include <algorithm>
#include <string>

namespace mgpu {

// Like sprintf but dynamically allocates sufficient output to hold the entire
// text.
std::string stringprintf(const char* format, ...);

// Returns xxx.xx(K|M|B)
std::string FormatInteger(int64 x);

const char* TypeIdString(const std::type_info& ti);

template<typename T>
const char* TypeIdName() {
	return TypeIdString(typeid(T));
}

struct FormatOpPrintf {
	const char* format;
	FormatOpPrintf(const char* f) : format(f) { }

	template<typename T>
	std::string operator()(int index, T x) const {
		return stringprintf(format, x);
	}
};

struct FormatOpMaskBit {
	const char* format;
	FormatOpMaskBit(const char* f) : format(f) { }

	std::string operator()(int index, int x) const {
		return stringprintf(format, (0x80000000 & x) ? '*' : ' ', 
			0x7fffffff & x);
	}
};

struct FormatOpMarkArray {
	const char* format;
	const int* marks;
	int numMarks;

	FormatOpMarkArray(const char* f, const int* m, int n) :
		format(f), marks(m), numMarks(n) { }

	std::string operator()(int index, int x) const {
		// Search for index in the array of marks.
		bool mark = std::binary_search(marks, marks + numMarks, index);
		return stringprintf(format, mark ? '*' : ' ', x);
	}
};

template<typename T, typename Op>
std::string FormatArrayOp(const T* data, size_t count, Op op, int numCols) {
	std::string s;
	size_t numRows = MGPU_DIV_UP(count, numCols);
	for(size_t row(0); row < numRows; ++row) { 
		size_t left = row * numCols;
		s.append(stringprintf("%5d:  ", left));

		for(size_t col(left); col < std::min(left + numCols, count); ++col) {
			s.append(op(col, data[col]));
			s.push_back(' ');
		}
		s.push_back('\n');
	}
	return s;
}

template<typename T>
std::string FormatArray(const T* data, size_t count, const char* format, 
	int numCols) {
	return FormatArrayOp(data, count, FormatOpPrintf(format), numCols);
}

template<typename T>
std::string FormatArray(const std::vector<T>& data, const char* format, 
	int numCols) {
	return FormatArray(&data[0], (int)data.size(), format, numCols);
}
template<typename T, typename Op>
std::string FormatArrayOp(const std::vector<T>& data, Op op, int numCols) {
	return FormatArrayOp(&data[0], (int)data.size(), op, numCols);
}

template<typename T>
void PrintArray(const T* data, size_t count, const char* format, int numCols) {
	std::string s = FormatArray(data, count, format, numCols);
	printf("%s", s.c_str());
}

template<typename T>
void PrintArray(const std::vector<T>& data, const char* format, int numCols) {
	std::string s = FormatArray(data, format, numCols);
	printf("%s", s.c_str());
}

template<typename T, typename Op>
void PrintArrayOp(const std::vector<T>& data, Op op, int numCols) {
	std::string s = FormatArrayOp(data, op, numCols);
	printf("%s", s.c_str());
}




} // namespace mgpu
