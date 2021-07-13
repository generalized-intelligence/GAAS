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

namespace mgpu {

extern int Rand(int min, int max);
extern int64 Rand(int64 min, int64 max);
extern uint Rand(uint min, uint max);
extern uint64 Rand(uint64 min, uint64 max);
extern float Rand(float min, float max);
extern double Rand(double min, double max);


////////////////////////////////////////////////////////////////////////////////
// intrusive_ptr

// boost::noncopyable, moved here so we don't have dependency on boost
class noncopyable {
protected:
	noncopyable() {}
	~noncopyable() {}
private:
	noncopyable(const noncopyable&) { }
	const noncopyable& operator=(const noncopyable&) { return *this; }
};

class CudaBase : public noncopyable {
public:
	CudaBase() : _ref(0) { }
	virtual ~CudaBase() { }
	virtual long AddRef() {
	//	return BOOST_INTERLOCKED_INCREMENT(&_ref);
		return ++_ref;
	}
	virtual void Release() {
	//	if(!BOOST_INTERLOCKED_DECREMENT(&_ref)) delete this;
		if(!--_ref) delete this;		
	}
private:
	long _ref;
};

inline long intrusive_ptr_add_ref(CudaBase* base) {
	return base->AddRef();
}

inline void intrusive_ptr_release(CudaBase* base) {
	base->Release();
}

template<typename T>
class intrusive_ptr {
public:
	intrusive_ptr() : _p(0) { }
	explicit intrusive_ptr(T* p) : _p(p) {
		if(p) intrusive_ptr_add_ref(p);
	}
	intrusive_ptr(const intrusive_ptr<T>& rhs) : _p(rhs._p) {
		if(_p) intrusive_ptr_add_ref(_p);
	}
	~intrusive_ptr() {
		if(_p) intrusive_ptr_release(_p);
	}
	intrusive_ptr& operator=(const intrusive_ptr& rhs) {
		intrusive_ptr(rhs.get()).swap(*this);
		return *this;
	}

	void reset(T* p = 0) {
		intrusive_ptr(p).swap(*this);
	}
	T* release() {
		T* p = _p;
		_p = 0;
		return p;
	}

	T* get() const { return _p; }
	operator T*() const { return _p; }
	T* operator->() const { return _p; }
	
	void swap(intrusive_ptr& rhs) {
		std::swap(_p, rhs._p);
	}
private:
	T* _p;
};

} // namespace mgpu
