// ***************************************************************************
// BGZF.cpp (c) 2009 Derek Barnett, Michael Str�mberg
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

#include "BGZF.h"
#include <nvbio/basic/threads.h>
#include <nvbio/basic/console.h>
#include <algorithm>

using namespace BamTools;
using namespace nvbio;
using std::string;
using std::min;

namespace BamTools {

struct BgzfThread : public Thread<BgzfThread>
{
    void set_data(BgzfData* data) { m_data = data; }

    void run()
    {
        const uint32 tid = ++m_data->BackgroundThreads;
        log_verbose( stderr, "starting BAM output thread %u\n", tid );

        while (m_data->BackgroundThreads != 0)
        {
            if (m_data->WorkCounter > 0u)
            {
                const int32 id = --m_data->WorkCounter;

                if (id >= 0)
                {
                    m_data->BlockLengths[id] = m_data->DeflateBlock( id, m_data->CurrentBlockSize );

                    --m_data->ActiveThreads;
                }
            }
        }
    }

    BgzfData* m_data;
};

BgzfData::BgzfData(const uint32_t threads)
    : UncompressedBlockSize(DEFAULT_BLOCK_SIZE)
    , CompressedBlockSize(MAX_BLOCK_SIZE)
    , BlockLength(0)
    , BlockOffset(0)
    , BlockAddress(0)
    , IsOpen(false)
    , IsWriteOnly(false)
    , Stream(NULL)
    , UncompressedBlock(NULL)
    , CompressedBlock(NULL)
    , BackgroundThreads( 0 )
    , ActiveThreads( 0 )
    , WorkCounter( 0 )
    , ThreadCount( threads == uint32(-1) ? nvbio::num_physical_cores() : 4 )
{
    try {
        CompressedBlock   = new char[CompressedBlockSize * ThreadCount * 4];
        UncompressedBlock = new char[UncompressedBlockSize * ThreadCount];
        BlockLengths      = new int [ThreadCount];
        ThreadPool        = new BgzfThread[ThreadCount];

        // launch background threads
        for (uint32_t i = 0; i < ThreadCount; ++i)
        {
            ThreadPool[i].set_data( this );
            ThreadPool[i].create();
        }

        // poll until all background threads are ready
        while (BackgroundThreads < ThreadCount) {}
    }
    catch( std::bad_alloc& ) {
        printf("ERROR: Unable to allocate memory for our BGZF object.\n");
        exit(1);
    }
}

// destructor
BgzfData::~BgzfData(void)
{
    // stop background threads
    BackgroundThreads = 0;

    for (uint32_t i = 0; i < ThreadCount; ++i)
        ThreadPool[i].join();

    if(CompressedBlock)   delete [] CompressedBlock;
    if(UncompressedBlock) delete [] UncompressedBlock;
    if(BlockLengths) delete [] BlockLengths;
    if(ThreadPool) delete [] ThreadPool;
}

// closes BGZF file
void BgzfData::Close(void) {

    if (!IsOpen) { return; }
    IsOpen = false;

    // flush the BGZF block
    if ( IsWriteOnly ) { FlushBlocks(); }

    // flush and close
    fflush(Stream);
    fclose(Stream);
}

// compresses the current block
int BgzfData::DeflateBlocks(void)
{
    // count the number of blocks to deflate
    int32_t blockSize = UncompressedBlockSize;
    while (1)
    {
        const int n_blocks = (BlockOffset + blockSize-1) / blockSize;

    #if 0
        // single threaded implementation
        for (int i = 0; i < n_blocks; ++i)
            BlockLengths[i] = DeflateBlock( i, blockSize );
    #else
        CurrentBlockSize  = blockSize;   // set current block size
        WorkCounter       = n_blocks;    // set number of blocks to process
        ActiveThreads     = n_blocks;    // activate background threads

        // poll until the blocks are being processed
        while (ActiveThreads > 0) {}
    #endif

        // check whether compression failed for any of the blocks
        bool ok = true;
        for (int i = 0; i < n_blocks; ++i)
        {
            if (BlockLengths[i] < 0)
                ok = false;
        }

        if (ok)
            return n_blocks;

        // compression failed: reduce block size
        blockSize -= 1024;
        if (blockSize <= 0)
        {
            printf("ERROR: input reduction failed.\n");
            exit(1);
        }
    }
    return 0;
}

// compresses the current block
int BgzfData::DeflateBlock(int32_t id, const unsigned int uncompressedBlockSize)
{
    // check if the block is within the proper range
    if (uncompressedBlockSize * id >= BlockOffset)
        return 0;

    // initialize the gzip header
    char* buffer = CompressedBlock + id * CompressedBlockSize;
    unsigned int bufferSize = CompressedBlockSize;

    memset(buffer, 0, 18);
    buffer[0]  = GZIP_ID1;
    buffer[1]  = (char)GZIP_ID2;
    buffer[2]  = CM_DEFLATE;
    buffer[3]  = FLG_FEXTRA;
    buffer[9]  = (char)OS_UNKNOWN;
    buffer[10] = BGZF_XLEN;
    buffer[12] = BGZF_ID1;
    buffer[13] = BGZF_ID2;
    buffer[14] = BGZF_LEN;

    // loop to retry for blocks that do not compress enough
    int inputLength = std::min( uncompressedBlockSize, BlockOffset - uncompressedBlockSize * id );
    int compressedLength = 0;

    z_stream zs;
    zs.zalloc    = NULL;
    zs.zfree     = NULL;
    zs.next_in   = (Bytef*)UncompressedBlock + id * uncompressedBlockSize;
    zs.avail_in  = inputLength;
    zs.next_out  = (Bytef*)&buffer[BLOCK_HEADER_LENGTH];
    zs.avail_out = bufferSize - BLOCK_HEADER_LENGTH - BLOCK_FOOTER_LENGTH;

    // initialize the zlib compression algorithm
    if(deflateInit2(&zs, Z_DEFAULT_COMPRESSION, Z_DEFLATED, GZIP_WINDOW_BITS, Z_DEFAULT_MEM_LEVEL, Z_DEFAULT_STRATEGY) != Z_OK) {
        printf("ERROR: zlib deflate initialization failed.\n");
        exit(1);
    }

    // compress the data
    int status = deflate(&zs, Z_FINISH);
    if(status != Z_STREAM_END) {

        deflateEnd(&zs);

        // reduce the input length and try again
        if(status == Z_OK)
            return -1;
    }

    // finalize the compression routine
    if(deflateEnd(&zs) != Z_OK) {
        printf("ERROR: deflate end failed.\n");
        exit(1);
    }

    compressedLength = zs.total_out;
    compressedLength += BLOCK_HEADER_LENGTH + BLOCK_FOOTER_LENGTH;

    if(compressedLength > MAX_BLOCK_SIZE) {
        printf("ERROR: deflate overflow.\n");
        exit(1);
    }

    // store the compressed length
    BgzfData::PackUnsignedShort(&buffer[16], (unsigned short)(compressedLength - 1));

    // store the CRC32 checksum
    unsigned int crc = crc32(0, NULL, 0);
    crc = crc32(crc, (Bytef*)UncompressedBlock + id * uncompressedBlockSize, inputLength);
    BgzfData::PackUnsignedInt(&buffer[compressedLength - 8], crc);
    BgzfData::PackUnsignedInt(&buffer[compressedLength - 4], inputLength);

    return compressedLength;
}

// flushes the data in the BGZF block
void BgzfData::FlushBlocks(void) {

    // compress the data block
    int n_blocks = DeflateBlocks();

    // flush the data to our output stream
    for (int i = 0; i < n_blocks; ++i)
    {
        int blockLength = BlockLengths[i];

        int numBytesWritten = (int)fwrite(CompressedBlock + i * CompressedBlockSize, 1, blockLength, Stream);

        if(numBytesWritten != blockLength) {
            printf("ERROR: Expected to write %u bytes during flushing, but wrote %u bytes.\n", blockLength, numBytesWritten);
            exit(1);
        }

        BlockAddress += blockLength;
    }
    BlockOffset = 0;
}

// de-compresses the current block
int BgzfData::InflateBlock(const int& blockLength) {

    // Inflate the block in m_BGZF.CompressedBlock into m_BGZF.UncompressedBlock
    z_stream zs;
    zs.zalloc    = NULL;
    zs.zfree     = NULL;
    zs.next_in   = (Bytef*)CompressedBlock + 18;
    zs.avail_in  = blockLength - 16;
    zs.next_out  = (Bytef*)UncompressedBlock;
    zs.avail_out = UncompressedBlockSize;

    int status = inflateInit2(&zs, GZIP_WINDOW_BITS);
    if (status != Z_OK) {
        printf("inflateInit failed\n");
        exit(1);
    }

    status = inflate(&zs, Z_FINISH);
    if (status != Z_STREAM_END) {
        inflateEnd(&zs);
        printf("inflate failed\n");
        exit(1);
    }

    status = inflateEnd(&zs);
    if (status != Z_OK) {
        printf("inflateEnd failed\n");
        exit(1);
    }

    return zs.total_out;
}

void BgzfData::Open(const string& filename, const char* mode) {

    if ( strcmp(mode, "rb") == 0 ) {
        IsWriteOnly = false;
    } else if ( strcmp(mode, "wb") == 0) {
        IsWriteOnly = true;
    } else {
        printf("ERROR: Unknown file mode: %s\n", mode);
        exit(1);
    }

    Stream = fopen(filename.c_str(), mode);
    if(!Stream) {
        printf("ERROR: Unable to open the BAM file %s\n", filename.c_str() );
        exit(1);
    }
    IsOpen = true;
}

int BgzfData::Read(char* data, const unsigned int dataLength) {

   if (dataLength == 0) { return 0; }

   char* output = data;
   unsigned int numBytesRead = 0;
   while (numBytesRead < dataLength) {

       int bytesAvailable = BlockLength - BlockOffset;
       if (bytesAvailable <= 0) {
           if ( ReadBlock() != 0 ) { return -1; }
           bytesAvailable = BlockLength - BlockOffset;
           if ( bytesAvailable <= 0 ) { break; }
       }

       char* buffer   = UncompressedBlock;
       int copyLength = min( (int)(dataLength-numBytesRead), bytesAvailable );
       memcpy(output, buffer + BlockOffset, copyLength);

       BlockOffset  += copyLength;
       output       += copyLength;
       numBytesRead += copyLength;
   }

   if ( BlockOffset == BlockLength ) {
       BlockAddress = ftell(Stream);
       BlockOffset  = 0;
       BlockLength  = 0;
   }

   return numBytesRead;
}

int BgzfData::ReadBlock(void) {

    char    header[BLOCK_HEADER_LENGTH];
    int64_t blockAddress = ftell(Stream);

    int count = (int)fread(header, 1, sizeof(header), Stream);
    if (count == 0) {
        BlockLength = 0;
        return 0;
    }

    if (count != sizeof(header)) {
        printf("read block failed - count != sizeof(header)\n");
        return -1;
    }

    if (!BgzfData::CheckBlockHeader(header)) {
        printf("read block failed - CheckBlockHeader() returned false\n");
        return -1;
    }

    int blockLength = BgzfData::UnpackUnsignedShort(&header[16]) + 1;
    char* compressedBlock = CompressedBlock;
    memcpy(compressedBlock, header, BLOCK_HEADER_LENGTH);
    int remaining = blockLength - BLOCK_HEADER_LENGTH;

    count = (int)fread(&compressedBlock[BLOCK_HEADER_LENGTH], 1, remaining, Stream);
    if (count != remaining) {
        printf("read block failed - count != remaining\n");
        return -1;
    }

    count = InflateBlock(blockLength);
    if (count < 0) { return -1; }

    if ( BlockLength != 0 ) {
        BlockOffset = 0;
    }

    BlockAddress = blockAddress;
    BlockLength  = count;
    return 0;
}

bool BgzfData::Seek(int64_t position) {

    int     blockOffset  = (position & 0xFFFF);
    int64_t blockAddress = (position >> 16) & 0xFFFFFFFFFFFFLL;

    if (fseek(Stream, (long)blockAddress, SEEK_SET) != 0) {
        printf("ERROR: Unable to seek in BAM file\n");
        exit(1);
    }

    BlockLength  = 0;
    BlockAddress = blockAddress;
    BlockOffset  = blockOffset;
    return true;
}

int64_t BgzfData::Tell(void) {
    return ( (BlockAddress << 16) | (BlockOffset & 0xFFFF) );
}

// writes the supplied data into the BGZF buffer
unsigned int BgzfData::Write(const char* data, const unsigned int dataLen) {

    // initialize
    unsigned int numBytesWritten = 0;
    const char* input = data;
    unsigned int blockLength = UncompressedBlockSize * ThreadCount;

    // copy the data to the buffer
    while(numBytesWritten < dataLen) {
        unsigned int copyLength = min(blockLength - BlockOffset, dataLen - numBytesWritten);
        char* buffer = UncompressedBlock;
        memcpy(buffer + BlockOffset, input, copyLength);

        BlockOffset     += copyLength;
        input           += copyLength;
        numBytesWritten += copyLength;

        if(BlockOffset == blockLength) {
            FlushBlocks();
        }
    }

    return numBytesWritten;
}

} // namespace BamTools