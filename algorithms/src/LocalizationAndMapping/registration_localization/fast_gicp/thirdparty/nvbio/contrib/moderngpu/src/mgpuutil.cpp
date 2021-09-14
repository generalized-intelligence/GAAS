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

#include "util/format.h"
#include <vector_types.h>
#include <cstdarg>
#include <map>

#define MGPU_RAND_NS std::tr1

#ifdef _MSC_VER
#include <random>
#else
#include <tr1/random>
#endif

namespace mgpu {

////////////////////////////////////////////////////////////////////////////////
// String formatting utilities.

std::string stringprintf(const char* format, ...) {
	va_list args;
	va_start(args, format);
	int len = vsnprintf(0, 0, format, args);
	va_end(args);

	// allocate space.
	std::string text;
	text.resize(len);

	va_start(args, format);
	vsnprintf(&text[0], len + 1, format, args);
	va_end(args);

	return text;
}

std::string FormatInteger(int64 x) {
	std::string s;
	if(x < 1000)
		s = stringprintf("%6d", (int)x);
	else if(x < 1000000) {
		if(0 == (x % 1000))
			s = stringprintf("%5dK", (int)(x / 1000));
		else
			s = stringprintf("%5.1lfK", x / 1.0e3);
	} else if(x < 1000000000ll) {
		if(0 == (x % 1000000ll))
			s = stringprintf("%5dM", (int)(x / 1000000));
		else
			s = stringprintf("%5.1lfM", x / 1.0e6);
	} else {
		if(0 == (x % 1000000000ll))
			s = stringprintf("%5dB", (int)(x / 1000000000ll));
		else
			s = stringprintf("%5.1lfB", x / 1.0e9);
	}
	return s;
}

class TypeIdMap {
	typedef std::map<std::string, const char*> Map;
	Map _map;

	void Insert(const std::type_info& ti, const char* name) {
		_map[ti.name()] = name;
	}
public:
	TypeIdMap() {
		Insert(typeid(char), "char");
		Insert(typeid(byte), "byte");
		Insert(typeid(short), "short");
		Insert(typeid(ushort), "ushort");
		Insert(typeid(int), "int");
		Insert(typeid(int64), "int64");
		Insert(typeid(uint), "uint");
		Insert(typeid(uint64), "uint64");
		Insert(typeid(float), "float");
		Insert(typeid(double), "double");
		Insert(typeid(int2), "int2");
		Insert(typeid(int3), "int3");
		Insert(typeid(int4), "int4");
		Insert(typeid(uint2), "uint2");
		Insert(typeid(uint3), "uint3");
		Insert(typeid(uint4), "uint4");
		Insert(typeid(float2), "float2");
		Insert(typeid(float3), "float3");
		Insert(typeid(float4), "float4");
		Insert(typeid(double2), "double2");
		Insert(typeid(double3), "double3");
		Insert(typeid(double4), "double4");
		Insert(typeid(char*), "char*");
	}
	const char* name(const std::type_info& ti) {
		const char* n = ti.name();
		Map::iterator it = _map.find(n);
		if(it != _map.end()) 
			n = it->second;
		return n;
	}
};

const char* TypeIdString(const std::type_info& ti) {
	static TypeIdMap typeIdMap;
	return typeIdMap.name(ti);
}

////////////////////////////////////////////////////////////////////////////////
// Random number generators.

MGPU_RAND_NS::mt19937 mt19937;

int Rand(int min, int max) {
	MGPU_RAND_NS::uniform_int<int> r(min, max);
	return r(mt19937);
}
int64 Rand(int64 min, int64 max) {
	MGPU_RAND_NS::uniform_int<int64> r(min, max);
	return r(mt19937);
}
uint Rand(uint min, uint max) {
	MGPU_RAND_NS::uniform_int<uint> r(min, max);
	return r(mt19937);
}
uint64 Rand(uint64 min, uint64 max) {
	MGPU_RAND_NS::uniform_int<uint64> r(min, max);
	return r(mt19937);
}
float Rand(float min, float max) {
	MGPU_RAND_NS::uniform_real<float> r(min, max);
	return r(mt19937);
}
double Rand(double min, double max) {
	MGPU_RAND_NS::uniform_real<double> r(min, max);
	return r(mt19937);
}

} // namespace mgpu
