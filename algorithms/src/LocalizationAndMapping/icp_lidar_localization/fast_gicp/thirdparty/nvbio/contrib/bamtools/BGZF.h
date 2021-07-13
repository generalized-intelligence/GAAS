// ***************************************************************************
// BGZF.h (c) 2009 Derek Barnett, Michael Str�mberg
// Marth Lab, Department of Biology, Boston College
// All rights reserved.
// ---------------------------------------------------------------------------
// Last modified: 8 December 2009 (DB)
// ---------------------------------------------------------------------------
// BGZF routines were adapted from the bgzf.c code developed at the Broad
// Institute.
// ---------------------------------------------------------------------------
// Provides the basic functionality for reading & writing BGZF files
// ***************************************************************************

#ifndef BGZF_H
#define BGZF_H

#include <nvbio/basic/atomics.h>

// 'C' includes
#include <cstdio>
#include <cstdlib>
#include <cstring>

// C++ includes
#include <string>

// zlib includes
#include <zlib/zlib.h>

// Platform-specific type definitions
#ifdef _MSC_VER
        typedef char                 int8_t;
        typedef unsigned char       uint8_t;
        typedef short               int16_t;
        typedef unsigned short     uint16_t;
        typedef int                 int32_t;
        typedef unsigned int       uint32_t;
        typedef long long           int64_t;
        typedef unsigned long long uint64_t;
#else
        #include <stdint.h>
#endif

namespace BamTools {

// zlib constants
const int GZIP_ID1   = 31;
const int GZIP_ID2   = 139;
const int CM_DEFLATE = 8;
const int FLG_FEXTRA = 4;
const int OS_UNKNOWN = 255;
const int BGZF_XLEN  = 6;
const int BGZF_ID1   = 66;
const int BGZF_ID2   = 67;
const int BGZF_LEN   = 2;
const int GZIP_WINDOW_BITS    = -15;
const int Z_DEFAULT_MEM_LEVEL = 8;

// BZGF constants
const int BLOCK_HEADER_LENGTH = 18;
const int BLOCK_FOOTER_LENGTH = 8;
const int MAX_BLOCK_SIZE      = 65536;
const int DEFAULT_BLOCK_SIZE  = 65536;

struct BgzfThread;

struct BgzfData {

    // data members
    unsigned int UncompressedBlockSize;
    unsigned int CompressedBlockSize;
    unsigned int BlockLength;
    unsigned int BlockOffset;
    uint64_t BlockAddress;
    bool     IsOpen;
    bool     IsWriteOnly;
    FILE*    Stream;
    char*    UncompressedBlock;
    char*    CompressedBlock;

    nvbio::AtomicInt32      BackgroundThreads;
    nvbio::AtomicInt32      ActiveThreads;
    volatile unsigned int   CurrentBlockSize;
    nvbio::AtomicInt32      WorkCounter;
    uint32_t                ThreadCount;
    int volatile*           BlockLengths;
    BgzfThread*             ThreadPool;

    // constructor & destructor
    BgzfData(const uint32_t threads = uint32_t(-1));
    ~BgzfData(void);

    // closes BGZF file
    void Close(void);
    // opens the BGZF file for reading (mode is either "rb" for reading, or "wb" for writing
    void Open(const std::string& filename, const char* mode);
    // reads BGZF data into a byte buffer
    int Read(char* data, const unsigned int dataLength);
    // reads BGZF block
    int ReadBlock(void);
    // seek to position in BAM file
    bool Seek(int64_t position);
    // get file position in BAM file
    int64_t Tell(void);
    // writes the supplied data into the BGZF buffer
    unsigned int Write(const char* data, const unsigned int dataLen);

    // checks BGZF block header
    static inline bool CheckBlockHeader(char* header);
    // packs an unsigned integer into the specified buffer
    static inline void PackUnsignedInt(char* buffer, unsigned int value);
    // packs an unsigned short into the specified buffer
    static inline void PackUnsignedShort(char* buffer, unsigned short value);
    // unpacks a buffer into a signed int
    static inline signed int UnpackSignedInt(char* buffer);
    // unpacks a buffer into a unsigned int
    static inline unsigned int UnpackUnsignedInt(char* buffer);
    // unpacks a buffer into a unsigned short
    static inline unsigned short UnpackUnsignedShort(char* buffer);

    // compresses the given block
    int DeflateBlock(int32_t id, const unsigned int blockSize);
    // compresses the current block
    int DeflateBlocks(void);
    // flushes the data in the BGZF block
    void FlushBlocks(void);
    // de-compresses the current block
    int InflateBlock(const int& blockLength);
};

// -------------------------------------------------------------

inline
bool BgzfData::CheckBlockHeader(char* header) {

    return (header[0] == GZIP_ID1 &&
            header[1] == (char)GZIP_ID2 &&
            header[2] == Z_DEFLATED &&
            (header[3] & FLG_FEXTRA) != 0 &&
            BgzfData::UnpackUnsignedShort(&header[10]) == BGZF_XLEN &&
            header[12] == BGZF_ID1 &&
            header[13] == BGZF_ID2 &&
            BgzfData::UnpackUnsignedShort(&header[14]) == BGZF_LEN );
}

// packs an unsigned integer into the specified buffer
inline
void BgzfData::PackUnsignedInt(char* buffer, unsigned int value) {
    buffer[0] = (char)value;
    buffer[1] = (char)(value >> 8);
    buffer[2] = (char)(value >> 16);
    buffer[3] = (char)(value >> 24);
}

// packs an unsigned short into the specified buffer
inline
void BgzfData::PackUnsignedShort(char* buffer, unsigned short value) {
    buffer[0] = (char)value;
    buffer[1] = (char)(value >> 8);
}

// unpacks a buffer into a signed int
inline
signed int BgzfData::UnpackSignedInt(char* buffer) {
    union { signed int value; unsigned char valueBuffer[sizeof(signed int)]; } un;
    un.value = 0;
    un.valueBuffer[0] = buffer[0];
    un.valueBuffer[1] = buffer[1];
    un.valueBuffer[2] = buffer[2];
    un.valueBuffer[3] = buffer[3];
    return un.value;
}

// unpacks a buffer into an unsigned int
inline
unsigned int BgzfData::UnpackUnsignedInt(char* buffer) {
    union { unsigned int value; unsigned char valueBuffer[sizeof(unsigned int)]; } un;
    un.value = 0;
    un.valueBuffer[0] = buffer[0];
    un.valueBuffer[1] = buffer[1];
    un.valueBuffer[2] = buffer[2];
    un.valueBuffer[3] = buffer[3];
    return un.value;
}

// unpacks a buffer into an unsigned short
inline
unsigned short BgzfData::UnpackUnsignedShort(char* buffer) {
    union { unsigned short value; unsigned char valueBuffer[sizeof(unsigned short)];} un;
    un.value = 0;
    un.valueBuffer[0] = buffer[0];
    un.valueBuffer[1] = buffer[1];
    return un.value;
}

} // namespace BamTools

#endif // BGZF_H
