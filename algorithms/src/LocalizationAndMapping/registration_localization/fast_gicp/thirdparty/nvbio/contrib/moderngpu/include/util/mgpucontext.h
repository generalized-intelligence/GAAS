#pragma once

#include "util/util.h"
#include "util/format.h"
#include "mgpualloc.h"
#include <cuda.h>

namespace mgpu {


#ifdef _DEBUG
#define MGPU_SYNC_CHECK(s) {												\
	cudaError_t error = cudaDeviceSynchronize();							\
	if(cudaSuccess != error) {												\
		printf("CUDA ERROR %d %s\n%s:%d.\n%s\n",							\
			error, cudaGetErrorString(error), __FILE__, __LINE__, s);		\
		exit(0);															\
	}																		\
}
#else
#define MGPU_SYNC_CHECK(s)
#endif

template<typename T>
void copyDtoH(T* dest, const T* source, int count) {
	cudaMemcpy(dest, source, sizeof(T) * count, cudaMemcpyDeviceToHost);
}
template<typename T>
void copyDtoD(T* dest, const T* source, int count, cudaStream_t stream = 0) {
	cudaMemcpyAsync(dest, source, sizeof(T) * count, cudaMemcpyDeviceToDevice,
		stream);
}
template<typename T>
void copyDtoH(std::vector<T>& dest, const T* source, int count) {
	dest.resize(count);
	if(count) 
		copyDtoH(&dest[0], source, count);
}

template<typename T>
void copyHtoD(T* dest, const T* source, int count) {
	cudaMemcpy(dest, source, sizeof(T) * count, cudaMemcpyHostToDevice);
}
template<typename T>
void copyHtoD(T* dest, const std::vector<T>& source) {
	if(source.size())
		copyHtoD(dest, &source[0], source.size());
}


////////////////////////////////////////////////////////////////////////////////

class CudaContext;
typedef intrusive_ptr<CudaContext> ContextPtr;
typedef intrusive_ptr<CudaAlloc> AllocPtr;

class CudaException : public std::exception {
public:
	cudaError_t error;

	CudaException() throw() { }
	CudaException(cudaError_t e) throw() : error(e) { }
	CudaException(const CudaException& e) throw() : error(e.error) { }

	virtual const char* what() const throw() {
		return "CUDA runtime error";
	}
};


////////////////////////////////////////////////////////////////////////////////
// CudaEvent and CudaTimer. 
// Exception-safe wrappers around cudaEvent_t.

class CudaEvent : public noncopyable {
public:
	CudaEvent() { 
		cudaEventCreate(&_event);
	}
	explicit CudaEvent(int flags) {
		cudaEventCreateWithFlags(&_event, flags);
	}
	~CudaEvent() {
		cudaEventDestroy(_event);
	}
	operator cudaEvent_t() { return _event; }
	void Swap(CudaEvent& rhs) {
		std::swap(_event, rhs._event);
	}
private:
	cudaEvent_t _event;
};

class CudaTimer : noncopyable {
	CudaEvent start, end;
public:
	void Start();
	double Split();
	double Throughput(int count, int numIterations);
};


////////////////////////////////////////////////////////////////////////////////

struct DeviceGroup;

class CudaDevice : public noncopyable {
	friend struct DeviceGroup;
public:
	static int DeviceCount();
	static CudaDevice& ByOrdinal(int ordinal);
	static CudaDevice& Selected();

	// Device properties.
	const cudaDeviceProp& Prop() const { return _prop; }
	int Ordinal() const { return _ordinal; }
	int NumSMs() const { return _prop.multiProcessorCount; }
	int ArchVersion() const { return 100 * _prop.major + 10 * _prop.minor; }

	// LaunchBox properties.
	int PTXVersion() const { return _ptxVersion; }

	std::string DeviceString() const;

	// Set this device as the active device on the thread.
	void SetActive();

private:
	CudaDevice() { }		// hide the destructor.
	int _ordinal;
	int _ptxVersion;
	cudaDeviceProp _prop;
};

////////////////////////////////////////////////////////////////////////////////
// CudaDeviceMem
// Exception-safe CUDA device memory container. Use the MGPU_MEM(T) macro for
// the type of the reference-counting container.
// CudaDeviceMem AddRefs the allocator that returned the memory, releasing the
// pointer when the object is destroyed.

template<typename T>
class CudaDeviceMem : public CudaBase {
	friend class CudaMemSupport;
public:
	~CudaDeviceMem();

	const T* get() const { return _p; }
	T* get() { return _p; }

	operator const T*() const { return get(); }
	operator T*() { return get(); }

	// Size is in units of T, not bytes.
	size_t Size() const { return _size; }

	// Copy from this to the argument array.
	cudaError_t ToDevice(T* data, size_t count) const;
	cudaError_t ToDevice(size_t srcOffest, size_t bytes, void* data) const;
	cudaError_t ToHost(T* data, size_t count) const;
	cudaError_t ToHost(std::vector<T>& data) const;
	cudaError_t ToHost(std::vector<T>& data, size_t count) const;
	cudaError_t ToHost(size_t srcOffset, size_t bytes, void* data) const;

	// Copy from the argument array to this.
	cudaError_t FromDevice(const T* data, size_t count);
	cudaError_t FromDevice(size_t dstOffset, size_t bytes, const void* data);
	cudaError_t FromHost(const std::vector<T>& data);
	cudaError_t FromHost(const std::vector<T>& data, size_t count);
	cudaError_t FromHost(const T* data, size_t count);
	cudaError_t FromHost(size_t destOffset, size_t bytes, const void* data);

private:
	friend class CudaContext;
	CudaDeviceMem(CudaAlloc* alloc) : _p(0), _size(0), _alloc(alloc) { }

	AllocPtr _alloc;
	T* _p; 
	size_t _size;
};

typedef intrusive_ptr<CudaAlloc> AllocPtr;
#define MGPU_MEM(type) mgpu::intrusive_ptr< mgpu::CudaDeviceMem< type > >  

////////////////////////////////////////////////////////////////////////////////
// CudaMemSupport
// Convenience functions for allocating device memory and copying to it from
// the host. These functions are factored into their own class for clarity.
// The class is derived by CudaContext.

class CudaMemSupport : public CudaBase {
	friend class CudaDevice;
	friend class CudaContext;
public:
	CudaDevice& Device() { return _alloc->Device(); }

	// Swap out the associated allocator.
	void SetAllocator(CudaAlloc* alloc) { 
		assert(alloc->Device().Ordinal() == _alloc->Device().Ordinal());
		_alloc.reset(alloc);
	}

	// Access the associated allocator.
	CudaAlloc* GetAllocator() { return _alloc.get(); }	

	// Support for creating arrays.
	template<typename T>
	MGPU_MEM(T) Malloc(size_t count);

	template<typename T>
	MGPU_MEM(T) Malloc(const T* data, size_t count);

	template<typename T>
	MGPU_MEM(T) Malloc(const std::vector<T>& data);

	template<typename T>
	MGPU_MEM(T) Fill(size_t count, T fill);

	template<typename T>
	MGPU_MEM(T) FillAscending(size_t count, T first, T step);

	template<typename T>
	MGPU_MEM(T) GenRandom(size_t count, T min, T max);

	template<typename T>
	MGPU_MEM(T) SortRandom(size_t count, T min, T max);

	template<typename T, typename Func>
	MGPU_MEM(T) GenFunc(size_t count, Func f);

protected:
	CudaMemSupport() { }
	AllocPtr _alloc;
};

////////////////////////////////////////////////////////////////////////////////

class CudaContext;
typedef mgpu::intrusive_ptr<CudaContext> ContextPtr;

// Create a context on the default stream (0).
ContextPtr CreateCudaDevice(int ordinal);
ContextPtr CreateCudaDevice(int argc, char** argv, bool printInfo = false);

// Create a context on a new stream.
ContextPtr CreateCudaDeviceStream(int ordinal);
ContextPtr CreateCudaDeviceStream(int argc, char** argv, 
	bool printInfo = false);

// Create a context and attach to an existing stream.
ContextPtr CreateCudaDeviceAttachStream(cudaStream_t stream);
ContextPtr CreateCudaDeviceAttachStream(int ordinal, cudaStream_t stream);

struct ContextGroup;

class CudaContext : public CudaMemSupport {
	friend struct ContextGroup;

	friend ContextPtr CreateCudaDevice(int ordinal);
	friend ContextPtr CreateCudaDeviceStream(int ordinal);
	friend ContextPtr CreateCudaDeviceAttachStream(int ordinal, 
		cudaStream_t stream);
public:
	static CudaContext& StandardContext(int ordinal = -1);

	// 4KB of page-locked memory per context.
	int* PageLocked() { return _pageLocked; }
	cudaStream_t AuxStream() const { return _auxStream; }

	int NumSMs() { return Device().NumSMs(); }
	int ArchVersion() { return Device().ArchVersion(); }
	int PTXVersion() { return Device().PTXVersion(); }
	std::string DeviceString() { return Device().DeviceString(); }

	cudaStream_t Stream() const { return _stream; }

	// Set this device as the active device on the thread.
	void SetActive() { Device().SetActive(); }

	// Access the included event.
	CudaEvent& Event() { return _event; }

	// Use the included timer.
	CudaTimer& Timer() { return _timer; }
	void Start() { _timer.Start(); }
	double Split() { return _timer.Split(); }
	double Throughput(int count, int numIterations) {
		return _timer.Throughput(count, numIterations);
	}

	virtual long AddRef() {
		return _noRefCount ? 1 : CudaMemSupport::AddRef();
	}
	virtual void Release() {
		if(!_noRefCount) CudaMemSupport::Release();
	}
private:
	CudaContext(CudaDevice& device, bool newStream, bool standard);
	~CudaContext();

	AllocPtr CreateDefaultAlloc(CudaDevice& device);

	bool _ownStream;
	cudaStream_t _stream;
	cudaStream_t _auxStream;
	CudaEvent _event;
	CudaTimer _timer;
	bool _noRefCount;
	int* _pageLocked;
};

////////////////////////////////////////////////////////////////////////////////
// CudaDeviceMem method implementations

template<typename T>
cudaError_t CudaDeviceMem<T>::ToDevice(T* data, size_t count) const {
	return ToDevice(0, sizeof(T) * count, data);
}
template<typename T>
cudaError_t CudaDeviceMem<T>::ToDevice(size_t srcOffset, size_t bytes, 
	void* data) const {
	cudaError_t error = cudaMemcpy(data, (char*)_p + srcOffset, bytes, 
		cudaMemcpyDeviceToDevice);
	if(cudaSuccess != error) {
		printf("CudaDeviceMem::ToDevice copy error %d\n", error);
		exit(0);
	}
	return error;
}

template<typename T>
cudaError_t CudaDeviceMem<T>::ToHost(T* data, size_t count) const {
	return ToHost(0, sizeof(T) * count, data);
}
template<typename T>
cudaError_t CudaDeviceMem<T>::ToHost(std::vector<T>& data, size_t count) const {
	data.resize(count);
	cudaError_t error = cudaSuccess;
	if(_size) error = ToHost(&data[0], count);
	return error;
}
template<typename T>
cudaError_t CudaDeviceMem<T>::ToHost(std::vector<T>& data) const {
	return ToHost(data, _size);
}
template<typename T>
cudaError_t CudaDeviceMem<T>::ToHost(size_t srcOffset, size_t bytes, 
	void* data) const {

	cudaError_t error = cudaMemcpy(data, (char*)_p + srcOffset, bytes,
		cudaMemcpyDeviceToHost);
	if(cudaSuccess != error) {
		printf("CudaDeviceMem::ToHost copy error %d\n", error);
		exit(0);
	}
	return error;
}

template<typename T>
cudaError_t CudaDeviceMem<T>::FromDevice(const T* data, size_t count) {
	return FromDevice(0, sizeof(T) * count, data);
}
template<typename T>
cudaError_t CudaDeviceMem<T>::FromDevice(size_t dstOffset, size_t bytes,
	const void* data) {
	if(dstOffset + bytes > sizeof(T) * _size)
		return cudaErrorInvalidValue;
	cudaMemcpy(_p + dstOffset, data, bytes, cudaMemcpyDeviceToDevice);
	return cudaSuccess;
}
template<typename T>
cudaError_t CudaDeviceMem<T>::FromHost(const std::vector<T>& data,
	size_t count) {
	cudaError_t error = cudaSuccess;
	if(data.size()) error = FromHost(&data[0], count);
	return error;
}
template<typename T>
cudaError_t CudaDeviceMem<T>::FromHost(const std::vector<T>& data) {
	return FromHost(data, data.size());
}
template<typename T>
cudaError_t CudaDeviceMem<T>::FromHost(const T* data, size_t count) {
	return FromHost(0, sizeof(T) * count, data);
}
template<typename T>
cudaError_t CudaDeviceMem<T>::FromHost(size_t dstOffset, size_t bytes,
	const void* data) {
	if(dstOffset + bytes > sizeof(T) * _size)
		return cudaErrorInvalidValue;
	cudaMemcpy(_p + dstOffset, data, bytes, cudaMemcpyHostToDevice);
	return cudaSuccess;
}
template<typename T>
CudaDeviceMem<T>::~CudaDeviceMem() {
	_alloc->Free(_p);
}

////////////////////////////////////////////////////////////////////////////////
// CudaMemSupport method implementations

template<typename T>
MGPU_MEM(T) CudaMemSupport::Malloc(size_t count) {
	MGPU_MEM(T) mem(new CudaDeviceMem<T>(_alloc.get()));
	mem->_size = count;
	cudaError_t error = _alloc->Malloc(sizeof(T) * count, (void**)&mem->_p);
	if(cudaSuccess != error) {
		printf("cudaMalloc error %d\n", error);		
		exit(0);
		throw CudaException(cudaErrorMemoryAllocation);
	}
#ifdef DEBUG
	// Initialize the memory to -1 in debug mode.
//	cudaMemset(mem->get(), -1, count);
#endif

	return mem;
}

template<typename T>
MGPU_MEM(T) CudaMemSupport::Malloc(const T* data, size_t count) {
	MGPU_MEM(T) mem = Malloc<T>(count);
	mem->FromHost(data, count);
	return mem;
}

template<typename T>
MGPU_MEM(T) CudaMemSupport::Malloc(const std::vector<T>& data) {
	MGPU_MEM(T) mem = Malloc<T>(data.size());
	if(data.size()) mem->FromHost(&data[0], data.size());
	return mem;
}

template<typename T>
MGPU_MEM(T) CudaMemSupport::Fill(size_t count, T fill) {
	std::vector<T> data(count, fill);
	return Malloc(data);
}

template<typename T>
MGPU_MEM(T) CudaMemSupport::FillAscending(size_t count, T first, T step) {
	std::vector<T> data(count);
	for(size_t i = 0; i < count; ++i)
		data[i] = first + i * step;
	return Malloc(data);
}

template<typename T>
MGPU_MEM(T) CudaMemSupport::GenRandom(size_t count, T min, T max) {
	std::vector<T> data(count);
	for(size_t i = 0; i < count; ++i)
		data[i] = Rand(min, max);
	return Malloc(data);
}

template<typename T>
MGPU_MEM(T) CudaMemSupport::SortRandom(size_t count, T min, T max) {
	std::vector<T> data(count);
	for(size_t i = 0; i < count; ++i)
		data[i] = Rand(min, max);
	std::sort(data.begin(), data.end());
	return Malloc(data);
}

template<typename T, typename Func>
MGPU_MEM(T) CudaMemSupport::GenFunc(size_t count, Func f) {
	std::vector<T> data(count);
	for(size_t i = 0; i < count; ++i)
		data[i] = f(i);

	MGPU_MEM(T) mem = Malloc<T>(count);
	mem->FromHost(data, count);
	return mem;
}

////////////////////////////////////////////////////////////////////////////////
// Format methods that operate directly on device mem.

template<typename T, typename Op>
std::string FormatArrayOp(const CudaDeviceMem<T>& mem, int count, Op op,
	int numCols) {
	std::vector<T> host;
	mem.ToHost(host, count);
	return FormatArrayOp(host, op, numCols);
}

template<typename T, typename Op>
std::string FormatArrayOp(const CudaDeviceMem<T>& mem, Op op, int numCols) {
	return FormatArrayOp(mem, mem.Size(), op, numCols);
}

template<typename T>
void PrintArray(const CudaDeviceMem<T>& mem, int count, const char* format, 
	int numCols) {
	std::string s = FormatArrayOp(mem, count, FormatOpPrintf(format), numCols);
	printf("%s", s.c_str());
}

template<typename T>
void PrintArray(const CudaDeviceMem<T>& mem, const char* format, int numCols) {
	PrintArray(mem, mem.Size(), format, numCols);
}
template<typename T, typename Op>
void PrintArrayOp(const CudaDeviceMem<T>& mem, Op op, int numCols) {
	std::string s = FormatArrayOp(mem, op, numCols);
	printf("%s", s.c_str());
}


////////////////////////////////////////////////////////////////////////////////


} // namespace mgpu
