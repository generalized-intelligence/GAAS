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

#include <nvbio/sufsort/file_bwt.h>
#include <nvbio/sufsort/file_bwt_bgz.h>
#include <nvbio/sufsort/sufsort_priv.h>
#include <zlib/zlib.h>
#ifdef _OPENMP
#include <omp.h>
#endif

namespace nvbio {

namespace { // anonymous namespace

/// convert a DNA+N+$ symbol to its ASCII character
///
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE char dna6_to_char(const uint8 c)
{
    return c == 0u   ? 'A' :
           c == 1u   ? 'C' :
           c == 2u   ? 'G' :
           c == 3u   ? 'T' :
           c == 255u ? '$' :
                       'N';
}

/// convert a DNA+N+$ string to an ASCII string
///
template <typename SymbolIterator>
NVBIO_FORCEINLINE NVBIO_HOST_DEVICE void dna6_to_string(
    const SymbolIterator    begin,
    const uint32            n,
    char*                   string)
{
    for (uint32 i = 0; i < n; ++i)
        string[i] = dna6_to_char( begin[i] );

    string[n] = '\0';
}

// utility function to convert an unsigned int to a base-10 string representation
template <typename T> uint32 itoa(char *buf, T in)
{
    uint32 len = 0;

    // convert to base10
    do
    {
        buf[len] = "0123456789"[in % 10];
        in /= 10;
        len++;
    } while(in);

    // reverse
    for(uint32 c = 0; c < len / 2; c++)
    {
        char tmp;
        tmp = buf[c];
        buf[c] = buf[len - c - 1];
        buf[len - c - 1] = tmp;
    }

    // terminate
    buf[len] = 0;
    return len;
}

} // anonymous namespace

/// A class to output the BWT to a packed host string
///
template <typename BWTWriter, uint32 SYMBOL_SIZE, bool BIG_ENDIAN, typename word_type>
struct FileBWTHandler : public SetBWTHandler, public BWTWriter
{
    static const uint32 WORD_SIZE = uint32( 8u * sizeof(word_type) );
    static const uint32 SYMBOLS_PER_WORD = WORD_SIZE / SYMBOL_SIZE;

    /// constructor
    ///
    FileBWTHandler() : offset(0) {}

    /// destructor
    ///
    virtual ~FileBWTHandler() {}

    /// write header
    ///
    void write_header()
    {
        const char* magic = "PRIB";         // PRImary-Binary
        BWTWriter::index_write( 4, magic );
    }

    /// process a batch of BWT symbols
    ///
    template <uint32 IN_SYMBOL_SIZE>
    void write_bwt(
        const uint32  n_suffixes,
        const uint32* bwt_storage)
    {
        typedef PackedStream<const uint32*,uint8,IN_SYMBOL_SIZE,true> input_stream_type;

        input_stream_type bwt( bwt_storage );

        const uint32 n_words = util::round_i( n_suffixes, SYMBOLS_PER_WORD );

        // expand our cache if needed
        if (cache.size() < n_words+2 ) // 2 more guardband words to avoid out-of-bounds accesses
            cache.resize( n_words+2 );

        const uint32 word_offset = offset & (SYMBOLS_PER_WORD-1);
              uint32 word_rem    = 0;
              uint32 cache_idx   = 0;

        if (word_offset)
        {
            // compute how many symbols we still need to encode to fill the current word
            word_rem = SYMBOLS_PER_WORD - word_offset;

            // fetch the word in question
            word_type word = cache_word;

            for (uint32 i = 0; i < word_rem; ++i)
            {
                const uint32       bit_idx = (word_offset + i) * SYMBOL_SIZE;
                const uint32 symbol_offset = BIG_ENDIAN ? (WORD_SIZE - SYMBOL_SIZE - bit_idx) : bit_idx;
                const word_type     symbol = word_type(bwt[i]) << symbol_offset;

                // set bits
                word |= symbol;
            }

            // write out the cached word
            cache[0]  = word;
            cache_idx = 1;
        }

        #pragma omp parallel for
        for (int i = word_rem; i < int( n_suffixes ); i += SYMBOLS_PER_WORD)
        {
            // encode a word's worth of characters
            word_type word = 0u;

            const uint32 n_symbols = nvbio::min( SYMBOLS_PER_WORD, n_suffixes - i );

            for (uint32 j = 0; j < n_symbols; ++j)
            {
                const uint32       bit_idx = j * SYMBOL_SIZE;
                const uint32 symbol_offset = BIG_ENDIAN ? (WORD_SIZE - SYMBOL_SIZE - bit_idx) : bit_idx;
                const word_type     symbol = word_type(bwt[i + j]) << symbol_offset;

                // set bits
                word |= symbol;
            }

            // write out the word and advance word_idx
            const uint32 word_idx = (i - word_rem) / SYMBOLS_PER_WORD;

            cache[ cache_idx + word_idx ] = word;
        }

        // compute how many words we can actually write out
        const uint32 n_full_words = cache_idx + (n_suffixes - word_rem) / SYMBOLS_PER_WORD;

        // write out the cache buffer
        {
            const uint32 n_bytes   = uint32( sizeof(word_type) * n_full_words );
            const uint32 n_written = BWTWriter::bwt_write( n_bytes, &cache[0] );
            if (n_written != n_bytes)
                throw nvbio::runtime_error("FileBWTHandler::process() : bwt write failed! (%u/%u bytes written)", n_written, n_bytes);
        }

        // save the last (possibly partial) word (hence the +2 guardband)
        cache_word = cache[ n_full_words ];
    }

    /// process a batch of BWT symbols
    ///
    void process(
        const uint32  n_suffixes,
        const uint32  bits_per_symbol,
        const uint32* bwt,
        const uint32  n_dollars,
        const uint64* dollar_pos,
        const uint64* dollar_ids)
    {
        if (bits_per_symbol == 2)
            write_bwt<2>( n_suffixes, bwt );
        else if (bits_per_symbol == 4)
            write_bwt<4>( n_suffixes, bwt );
        else if (bits_per_symbol == 8)
            write_bwt<8>( n_suffixes, bwt );
        else
            throw nvbio::runtime_error("FileBWTHandler::process() : unsupported input format! (%u bits per symbol)", bits_per_symbol);

        // and write the list to the output
        if (n_dollars)
        {
            if (dollars.size() < n_dollars)
                dollars.resize( n_dollars );

            #pragma omp parallel for
            for (int32 i = 0; i < int32( n_dollars ); ++i)
                dollars[i] = std::make_pair( dollar_pos[i], dollar_ids[i] );

            const uint32 n_bytes   = uint32( sizeof(uint64) * 2 * n_dollars );
            const uint32 n_written = BWTWriter::index_write( n_bytes, &dollars[0] );
            if (n_written != n_bytes)
                throw nvbio::runtime_error("FileBWTHandler::process() : index write failed! (%u/%u bytes written)", n_written, n_bytes);
        }

        // advance the offset
        offset += n_suffixes;
    }

    /// process a batch of BWT symbols
    ///
    void process(
        const uint32  n_suffixes,
        const uint8*  bwt,
        const uint32  n_dollars,
        const uint64* dollar_pos,
        const uint64* dollar_ids)
    {
        const uint32 n_words = util::round_i( n_suffixes, SYMBOLS_PER_WORD );

        // expand our cache if needed
        if (cache.size() < n_words+2 ) // 2 more guardband words to avoid out-of-bounds accesses
            cache.resize( n_words+2 );

        const uint32 word_offset = offset & (SYMBOLS_PER_WORD-1);
              uint32 word_rem    = 0;
              uint32 cache_idx   = 0;

        if (word_offset)
        {
            // compute how many symbols we still need to encode to fill the current word
            word_rem = SYMBOLS_PER_WORD - word_offset;

            // fetch the word in question
            word_type word = cache_word;

            for (uint32 i = 0; i < word_rem; ++i)
            {
                const uint32       bit_idx = (word_offset + i) * SYMBOL_SIZE;
                const uint32 symbol_offset = BIG_ENDIAN ? (WORD_SIZE - SYMBOL_SIZE - bit_idx) : bit_idx;
                const word_type     symbol = word_type(bwt[i]) << symbol_offset;

                // set bits
                word |= symbol;
            }

            // write out the cached word
            cache[0]  = word;
            cache_idx = 1;
        }

        #pragma omp parallel for
        for (int i = word_rem; i < int( n_suffixes ); i += SYMBOLS_PER_WORD)
        {
            // encode a word's worth of characters
            word_type word = 0u;

            const uint32 n_symbols = nvbio::min( SYMBOLS_PER_WORD, n_suffixes - i );

            for (uint32 j = 0; j < n_symbols; ++j)
            {
                const uint32       bit_idx = j * SYMBOL_SIZE;
                const uint32 symbol_offset = BIG_ENDIAN ? (WORD_SIZE - SYMBOL_SIZE - bit_idx) : bit_idx;
                const word_type     symbol = word_type(bwt[i + j]) << symbol_offset;

                // set bits
                word |= symbol;
            }

            // write out the word and advance word_idx
            const uint32 word_idx = (i - word_rem) / SYMBOLS_PER_WORD;

            cache[ cache_idx + word_idx ] = word;
        }

        // compute how many words we can actually write out
        const uint32 n_full_words = cache_idx + (n_suffixes - word_rem) / SYMBOLS_PER_WORD;

        // write out the cache buffer
        {
            const uint32 n_bytes   = uint32( sizeof(word_type) * n_full_words );
            const uint32 n_written = BWTWriter::bwt_write( n_bytes, &cache[0] );
            if (n_written != n_bytes)
                throw nvbio::runtime_error("FileBWTHandler::process() : bwt write failed! (%u/%u bytes written)", n_written, n_bytes);
        }

        // save the last (possibly partial) word (hence the +2 guardband)
        cache_word = cache[ n_full_words ];

        // and write the list to the output
        if (n_dollars)
        {
            priv::alloc_storage( dollars, n_dollars );

            #pragma omp parallel for
            for (int32 i = 0; i < int32( n_dollars ); ++i)
                dollars[i] = std::make_pair( dollar_pos[i], dollar_ids[i] );

            const uint32 n_bytes   = uint32( sizeof(uint64) * 2 * n_dollars );
            const uint32 n_written = BWTWriter::index_write( n_bytes, &dollars[0] );
            if (n_written != n_bytes)
                throw nvbio::runtime_error("FileBWTHandler::process() : index write failed! (%u/%u bytes written)", n_written, n_bytes);
        }

        // advance the offset
        offset += n_suffixes;
    }

    uint64                  offset;
    std::vector<word_type>  cache;
    word_type               cache_word;
    std::vector< std::pair<uint64,uint64> > dollars;
};

/// A class to output the BWT to a ASCII file
///
template <typename BWTWriter>
struct ASCIIFileBWTHandler : public SetBWTHandler, public BWTWriter
{
    /// constructor
    ///
    ASCIIFileBWTHandler() : offset(0) {}

    /// destructor
    ///
    virtual ~ASCIIFileBWTHandler() {}

    /// write header
    ///
    void write_header()
    {
        const char* magic = "#PRI\n";       // PRImary-ASCII
        BWTWriter::index_write( 5, magic );
    }

    void write_dollars(
        const uint32        n_dollars,
        const uint64*       dollar_pos,
        const uint64*       dollar_ids)
    {
        if (n_dollars)
        {
            // reserve enough storage to encode 2 very large numbers in base 10 (up to 15 digits), plus a space and a newline
            priv::alloc_storage( dollar_buffer, n_dollars * 32 );

            uint32 output_size = 0;

            for (uint32 i = 0; i < n_dollars; ++i)
            {
                char* buf = &dollar_buffer[ output_size ];

                const uint32 len1 = itoa( buf,            dollar_pos[i] );  buf[len1]            = ' ';
                const uint32 len2 = itoa( buf + len1 + 1, dollar_ids[i] ); buf[len1 + len2 + 1] = '\n';
                const uint32 len  = len1 + len2 + 2;

                output_size += len;
            }

            const uint32 n_bytes   = output_size;
            const uint32 n_written = BWTWriter::index_write( n_bytes, &dollar_buffer[0] );
            if (n_written != n_bytes)
                throw nvbio::runtime_error("FileBWTHandler::process() : index write failed! (%u/%u bytes written)", n_written, n_bytes);
        }
    }

    /// process a batch of BWT symbols
    ///
    template <typename bwt_iterator>
    void write(
        const uint32        n_suffixes,
        const bwt_iterator  bwt,
        const uint32        n_dollars,
        const uint64*       dollar_pos,
        const uint64*       dollar_ids)
    {
        // write out the cache buffer
        priv::alloc_storage( ascii, n_suffixes + 1 );

        // convert to ASCII
        dna6_to_string( bwt, n_suffixes, &ascii[0] );
        {
            const uint32 n_bytes   = uint32( n_suffixes );
            const uint32 n_written = BWTWriter::bwt_write( n_bytes, &ascii[0] );
            if (n_written != n_bytes)
                throw nvbio::runtime_error("FileBWTHandler::process() : bwt write failed! (%u/%u bytes written)", n_written, n_bytes);
        }

        // and write the list to the output
        write_dollars( n_dollars, dollar_pos, dollar_ids );

        // advance the offset
        offset += n_suffixes;
    }

    /// process a batch of BWT symbols
    ///
    void process(
        const uint32  n_suffixes,
        const uint32  bits_per_symbol,
        const uint32* bwt,
        const uint32  n_dollars,
        const uint64* dollar_pos,
        const uint64* dollar_ids)
    {
        // convert to ASCII
        if (bits_per_symbol == 2)
            write( n_suffixes, PackedStream<const uint32*,uint8,2,true>( bwt ), n_dollars, dollar_pos, dollar_ids );
        else if (bits_per_symbol == 4)
            write( n_suffixes, PackedStream<const uint32*,uint8,4,true>( bwt ), n_dollars, dollar_pos, dollar_ids );
        else if (bits_per_symbol == 8)
            write( n_suffixes, PackedStream<const uint32*,uint8,8,true>( bwt ), n_dollars, dollar_pos, dollar_ids );
        else
            throw nvbio::runtime_error("FileBWTHandler::process() : unsupported input format! (%u bits per symbol)", bits_per_symbol);
    }
        
    /// process a batch of BWT symbols
    ///
    void process(
        const uint32  n_suffixes,
        const uint8*  bwt,
        const uint32  n_dollars,
        const uint64* dollar_pos,
        const uint64* dollar_ids)
    {
        write( n_suffixes, bwt, n_dollars, dollar_pos, dollar_ids );
    }

    uint64                  offset;
    std::vector<char>       ascii;
    std::vector<char>       dollar_buffer;
};

/// A class to output the BWT to a binary file
///
struct RawBWTWriter
{
    /// constructor
    ///
    RawBWTWriter();

    /// destructor
    ///
    ~RawBWTWriter();

    void open(const char* output_name, const char* index_name);

    /// write to the bwt
    ///
    uint32 bwt_write(const uint32 n_bytes, const void* buffer);

    /// write to the index
    ///
    uint32 index_write(const uint32 n_bytes, const void* buffer);

    /// return whether the file is in a good state
    ///
    bool is_ok() const;

private:
    FILE*   output_file;
    FILE*   index_file;
};

/// A class to output the BWT to a gzipped binary file
///
struct BWTGZWriter
{
    /// constructor
    ///
    BWTGZWriter();

    /// destructor
    ///
    ~BWTGZWriter();

    void open(const char* output_name, const char* index_name, const char* compression);

    /// write to the bwt
    ///
    uint32 bwt_write(const uint32 n_bytes, const void* buffer);

    /// write to the index
    ///
    uint32 index_write(const uint32 n_bytes, const void* buffer);

    /// return whether the file is in a good state
    ///
    bool is_ok() const;

private:
    void*   output_file;
    void*   index_file;
};

// constructor
//
RawBWTWriter::RawBWTWriter() :
    output_file(NULL),
    index_file(NULL)
{}

// destructor
//
RawBWTWriter::~RawBWTWriter()
{
    fclose( output_file );
    fclose( index_file );
}

void RawBWTWriter::open(const char* output_name, const char* index_name)
{
    log_verbose(stderr,"  opening bwt file \"%s\"\n", output_name);
    log_verbose(stderr,"  opening index file \"%s\"\n", index_name);
    output_file = fopen( output_name, "wb" );
    index_file  = fopen( index_name,  "wb" );
}

// write to the bwt
//
uint32 RawBWTWriter::bwt_write(const uint32 n_bytes, const void* buffer)
{
    return fwrite( buffer, sizeof(uint8), n_bytes, output_file );
}

// write to the index
//
uint32 RawBWTWriter::index_write(const uint32 n_bytes, const void* buffer)
{
    return fwrite( buffer, sizeof(uint8), n_bytes, index_file );
}

// return whether the file is in a good state
//
bool RawBWTWriter::is_ok() const { return output_file != NULL || index_file != NULL; }


// constructor
//
BWTGZWriter::BWTGZWriter() :
    output_file(NULL),
    index_file(NULL)
{}

// destructor
//
BWTGZWriter::~BWTGZWriter()
{
    gzclose( output_file );
    gzclose( index_file );
}

void BWTGZWriter::open(const char* output_name, const char* index_name, const char* compression)
{
    char comp_string[5];
    sprintf( comp_string, "wb%s", compression );

    log_verbose(stderr,"  opening bwt file \"%s\" (compression level: %s)\n", output_name, compression);
    log_verbose(stderr,"  opening index file \"%s\" (compression level: %s)\n", index_name, compression);
    output_file = gzopen( output_name, comp_string );
    index_file  = gzopen( index_name,  comp_string );
}

// write to the bwt
//
uint32 BWTGZWriter::bwt_write(const uint32 n_bytes, const void* buffer)
{
    return gzwrite( output_file, buffer, n_bytes );
}

// write to the index
//
uint32 BWTGZWriter::index_write(const uint32 n_bytes, const void* buffer)
{
    return gzwrite( index_file, buffer, n_bytes );
}

// return whether the file is in a good state
//
bool BWTGZWriter::is_ok() const { return output_file != NULL || index_file != NULL; }


// open a BWT file
//
SetBWTHandler* open_bwt_file(const char* output_name, const char* params)
{
    enum OutputFormat
    {
        UNKNOWN = 0,
        TXT     = 1,
        TXTGZ   = 2,
        TXTBGZ  = 3,
        TXTLZ4  = 4,
        BWT2    = 5,
        BWT2GZ  = 6,
        BWT2BGZ = 7,
        BWT2LZ4 = 8,
        BWT4    = 9,
        BWT4GZ  = 10,
        BWT4BGZ = 11,
        BWT4LZ4 = 12,
    };
    OutputFormat format = UNKNOWN;
    std::string  index_string = output_name;

    // detect the file format from the suffix
    {
        const uint32 len = (uint32)strlen( output_name );

        //
        // detect BWT2* variants
        //
        if (len >= strlen(".bwt.bgz"))
        {
            if (strcmp(&output_name[len - strlen(".bwt.bgz")], ".bwt.bgz") == 0)
            {
                format = BWT2BGZ;
                index_string.replace( index_string.find(".bwt.bgz"), 8u, ".pri.bgz" );
            }
        }
        if (len >= strlen(".bwt.gz"))
        {
            if (strcmp(&output_name[len - strlen(".bwt.gz")], ".bwt.gz") == 0)
            {
                format = BWT2GZ;
                index_string.replace( index_string.find(".bwt.gz"), 7u, ".pri.gz" );
            }
        }
        if (len >= strlen(".bwt"))
        {
            if (strcmp(&output_name[len - strlen(".bwt")], ".bwt") == 0)
            {
                format = BWT2;
                index_string.replace( index_string.find(".bwt"), 4u, ".pri" );
            }
        }

        //
        // detect BWT4* variants
        //
        if (len >= strlen(".bwt4.bgz"))
        {
            if (strcmp(&output_name[len - strlen(".bwt4.bgz")], ".bwt4.bgz") == 0)
            {
                format = BWT4BGZ;
                index_string.replace( index_string.find(".bwt4.bgz"), 9u, ".pri.bgz" );
            }
        }
        if (len >= strlen(".bwt4.gz"))
        {
            if (strcmp(&output_name[len - strlen(".bwt4.gz")], ".bwt4.gz") == 0)
            {
                format = BWT4GZ;
                index_string.replace( index_string.find(".bwt4.gz"), 8u, ".pri.gz" );
            }
        }
        if (len >= strlen(".bwt4"))
        {
            if (strcmp(&output_name[len - strlen(".bwt4")], ".bwt4") == 0)
            {
                format = BWT4;
                index_string.replace( index_string.find(".bwt4"), 5u, ".pri" );
            }
        }

        //
        // detect TXT* variants
        //
        if (len >= strlen(".txt.gz"))
        {
            if (strcmp(&output_name[len - strlen(".txt.gz")], ".txt.gz") == 0)
            {
                format = TXTGZ;
                index_string.replace( index_string.find(".txt.gz"), 7u, ".pri.gz" );
            }
        }
        if (len >= strlen(".txt.bgz"))
        {
            if (strcmp(&output_name[len - strlen(".txt.bgz")], ".txt.bgz") == 0)
            {
                format = TXTGZ;
                index_string.replace( index_string.find(".txt.bgz"), 8u, ".pri.bgz" );
            }
        }
        if (len >= strlen(".txt"))
        {
            if (strcmp(&output_name[len - strlen(".txt")], ".txt") == 0)
            {
                format = TXT;
                index_string.replace( index_string.find(".txt"), 4u, ".pri" );
            }
        }
    }

    if (format == BWT2)
    {
        // build an output handler
        FileBWTHandler<RawBWTWriter,2,true,uint32>* file_handler = new FileBWTHandler<RawBWTWriter,2,true,uint32>();

        file_handler->open( output_name, index_string.c_str() );
        if (file_handler->is_ok() == false)
        {
            log_error(stderr,"  unable to open output file \"%s\"\n", output_name);
            return NULL;
        }
        file_handler->write_header();
        return file_handler;
    }
    else if (format == BWT2BGZ)
    {
        // build an output handler
        FileBWTHandler<BWTBGZWriter,2,true,uint32>* file_handler = new FileBWTHandler<BWTBGZWriter,2,true,uint32>();

        file_handler->open( output_name, index_string.c_str(), params );
        if (file_handler->is_ok() == false)
        {
            log_error(stderr,"  unable to open output file \"%s\"\n", output_name);
            return NULL;
        }
        file_handler->write_header();
        return file_handler;
    }
    else if (format == BWT2GZ)
    {
        // build an output handler
        FileBWTHandler<BWTGZWriter,2,true,uint32>* file_handler = new FileBWTHandler<BWTGZWriter,2,true,uint32>();

        file_handler->open( output_name, index_string.c_str(), params );
        if (file_handler->is_ok() == false)
        {
            log_error(stderr,"  unable to open output file \"%s\"\n", output_name);
            return NULL;
        }
        file_handler->write_header();
        return file_handler;
    }
    else if (format == BWT4)
    {
        // build an output handler
        FileBWTHandler<RawBWTWriter,4,true,uint32>* file_handler = new FileBWTHandler<RawBWTWriter,4,true,uint32>();

        file_handler->open( output_name, index_string.c_str() );
        if (file_handler->is_ok() == false)
        {
            log_error(stderr,"  unable to open output file \"%s\"\n", output_name);
            return NULL;
        }
        file_handler->write_header();
        return file_handler;
    }
    else if (format == BWT4BGZ)
    {
        // build an output handler
        FileBWTHandler<BWTBGZWriter,4,true,uint32>* file_handler = new FileBWTHandler<BWTBGZWriter,4,true,uint32>();

        file_handler->open( output_name, index_string.c_str(), params );
        if (file_handler->is_ok() == false)
        {
            log_error(stderr,"  unable to open output file \"%s\"\n", output_name);
            return NULL;
        }
        file_handler->write_header();
        return file_handler;
    }
    else if (format == BWT4GZ)
    {
        // build an output handler
        FileBWTHandler<BWTGZWriter,4,true,uint32>* file_handler = new FileBWTHandler<BWTGZWriter,4,true,uint32>();

        file_handler->open( output_name, index_string.c_str(), params );
        if (file_handler->is_ok() == false)
        {
            log_error(stderr,"  unable to open output file \"%s\"\n", output_name);
            return NULL;
        }
        file_handler->write_header();
        return file_handler;
    }
    else if (format == TXT)
    {
        // build an output handler
        ASCIIFileBWTHandler<RawBWTWriter>* file_handler = new ASCIIFileBWTHandler<RawBWTWriter>();

        file_handler->open( output_name, index_string.c_str() );
        if (file_handler->is_ok() == false)
        {
            log_error(stderr,"  unable to open output file \"%s\"\n", output_name);
            return NULL;
        }
        file_handler->write_header();
        return file_handler;
    }
    else if (format == TXTGZ)
    {
        // build an output handler
        ASCIIFileBWTHandler<BWTGZWriter>* file_handler = new ASCIIFileBWTHandler<BWTGZWriter>();

        file_handler->open( output_name, index_string.c_str(), params );
        if (file_handler->is_ok() == false)
        {
            log_error(stderr,"  unable to open output file \"%s\"\n", output_name);
            return NULL;
        }
        file_handler->write_header();
        return file_handler;
    }
    else if (format == TXTBGZ)
    {
        // build an output handler
        ASCIIFileBWTHandler<BWTBGZWriter>* file_handler = new ASCIIFileBWTHandler<BWTBGZWriter>();

        file_handler->open( output_name, index_string.c_str(), params );
        if (file_handler->is_ok() == false)
        {
            log_error(stderr,"  unable to open output file \"%s\"\n", output_name);
            return NULL;
        }
        file_handler->write_header();
        return file_handler;
    }

    log_error(stderr,"  unknown output format \"%s\"\n", output_name);
    return NULL;
}

} // namespace nvbio
