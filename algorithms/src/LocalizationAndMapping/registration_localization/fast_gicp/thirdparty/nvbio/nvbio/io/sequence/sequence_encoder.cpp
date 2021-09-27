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

#include <nvbio/io/sequence/sequence_encoder.h>
#include <stdio.h>

namespace nvbio {
namespace io {

namespace { // anonymous

// converts ASCII characters for amino-acids into
// a 5 letter alphabet for { A, C, G, T, N }.
inline unsigned char nst_nt4_encode(unsigned char c)
{
    static unsigned char nst_nt4_table[256] = {
        4, 4, 4, 4,  4, 4, 4, 4,  4, 4, 4, 4,  4, 4, 4, 4,
        4, 4, 4, 4,  4, 4, 4, 4,  4, 4, 4, 4,  4, 4, 4, 4,
        4, 4, 4, 4,  4, 4, 4, 4,  4, 4, 4, 4,  4, 5 /*'-'*/, 4, 4,
        4, 4, 4, 4,  4, 4, 4, 4,  4, 4, 4, 4,  4, 4, 4, 4,
        4, 0, 4, 1,  4, 4, 4, 2,  4, 4, 4, 4,  4, 4, 4, 4,
        4, 4, 4, 4,  3, 4, 4, 4,  4, 4, 4, 4,  4, 4, 4, 4,
        4, 0, 4, 1,  4, 4, 4, 2,  4, 4, 4, 4,  4, 4, 4, 4,
        4, 4, 4, 4,  3, 4, 4, 4,  4, 4, 4, 4,  4, 4, 4, 4,
        4, 4, 4, 4,  4, 4, 4, 4,  4, 4, 4, 4,  4, 4, 4, 4,
        4, 4, 4, 4,  4, 4, 4, 4,  4, 4, 4, 4,  4, 4, 4, 4,
        4, 4, 4, 4,  4, 4, 4, 4,  4, 4, 4, 4,  4, 4, 4, 4,
        4, 4, 4, 4,  4, 4, 4, 4,  4, 4, 4, 4,  4, 4, 4, 4,
        4, 4, 4, 4,  4, 4, 4, 4,  4, 4, 4, 4,  4, 4, 4, 4,
        4, 4, 4, 4,  4, 4, 4, 4,  4, 4, 4, 4,  4, 4, 4, 4,
        4, 4, 4, 4,  4, 4, 4, 4,  4, 4, 4, 4,  4, 4, 4, 4,
        4, 4, 4, 4,  4, 4, 4, 4,  4, 4, 4, 4,  4, 4, 4, 4
    };

    return nst_nt4_table[c];
}

// convert a quality value in one of the supported encodings to Phred
template <QualityEncoding encoding>
inline unsigned char convert_to_phred_quality(const uint8 q)
{
    // this table maps Solexa quality values to Phred scale
    static unsigned char s_solexa_to_phred[] = {
        0, 1, 1, 1, 1, 1, 1, 2, 2, 3,
        3, 4, 4, 5, 5, 6, 7, 8, 9, 10,
        10, 11, 12, 13, 14, 15, 16, 17, 18, 19,
        20, 21, 22, 23, 24, 25, 26, 27, 28, 29,
        30, 31, 32, 33, 34, 35, 36, 37, 38, 39,
        40, 41, 42, 43, 44, 45, 46, 47, 48, 49,
        50, 51, 52, 53, 54, 55, 56, 57, 58, 59,
        60, 61, 62, 63, 64, 65, 66, 67, 68, 69,
        70, 71, 72, 73, 74, 75, 76, 77, 78, 79,
        80, 81, 82, 83, 84, 85, 86, 87, 88, 89,
        90, 91, 92, 93, 94, 95, 96, 97, 98, 99,
        100, 101, 102, 103, 104, 105, 106, 107, 108, 109,
        110, 111, 112, 113, 114, 115, 116, 117, 118, 119,
        120, 121, 122, 123, 124, 125, 126, 127, 128, 129,
        130, 131, 132, 133, 134, 135, 136, 137, 138, 139,
        140, 141, 142, 143, 144, 145, 146, 147, 148, 149,
        150, 151, 152, 153, 154, 155, 156, 157, 158, 159,
        160, 161, 162, 163, 164, 165, 166, 167, 168, 169,
        170, 171, 172, 173, 174, 175, 176, 177, 178, 179,
        180, 181, 182, 183, 184, 185, 186, 187, 188, 189,
        190, 191, 192, 193, 194, 195, 196, 197, 198, 199,
        200, 201, 202, 203, 204, 205, 206, 207, 208, 209,
        210, 211, 212, 213, 214, 215, 216, 217, 218, 219,
        220, 221, 222, 223, 224, 225, 226, 227, 228, 229,
        230, 231, 232, 233, 234, 235, 236, 237, 238, 239,
        240, 241, 242, 243, 244, 245, 246, 247, 248, 249,
        250, 251, 252, 253, 254, 255
    };

    switch(encoding)
    {
    case Phred:
        return q;

    case Phred33:
        return q - 33;

    case Phred64:
        return q - 64;

    case Solexa:
        return s_solexa_to_phred[q];

    default:
        break;
    }

    // gcc is dumb
    return q;
}

} // anonymous namespace

// a small sequence class supporting REVERSE | COMPLEMENT operations
//
template <Alphabet ALPHABET, SequenceDataEncoder::StrandOp FLAGS>
struct sequence_string
{
    // constructor
    sequence_string(const uint32 len, const uint8* sequence, const uint8* qual) : m_len(len), m_sequence(sequence), m_qual(qual) {}

    // string length
    uint32 length() const { return m_len; }

    // indexing operator
    uint8 operator[] (const uint32 i) const
    {
        const uint32 index = (FLAGS & SequenceDataEncoder::REVERSE_OP) ? m_len - i - 1u : i;

        // fetch the bp
        const uint8 c = m_sequence[index];

        // and convert it to the proper alphabet
        if (ALPHABET == DNA || ALPHABET == DNA_N)
        {
            const uint8 bp = nst_nt4_encode( c );

            if (FLAGS & SequenceDataEncoder::COMPLEMENT_OP)
                return bp < 4u ? 3u - bp : 4u;
    
            return bp;
        }
        else
        {
            const uint8 bp = from_char<ALPHABET>( c );

            // TODO: implement complementing!
            //if (FLAGS & SequenceDataEncoder::COMPLEMENT_OP)
            //    return ???;

            return bp;
        }
    }

    // quality operator
    uint8 quality(const uint32 i) const
    {
        const uint32 index = (FLAGS & REVERSE) ? m_len - i - 1u : i;

        return m_qual[index];
    }

    const uint32 m_len;
    const uint8* m_sequence;
    const uint8* m_qual;
};



// encode a sequence according to a compile-time quality-encoding
//
template <Alphabet ALPHABET, QualityEncoding quality_encoding, typename sequence_type>
void encode(
    const sequence_type                                                            sequence,
    typename SequenceDataEdit<ALPHABET,SequenceDataView>::sequence_stream_type     stream,
    char*                                                                          qual_stream)
{
  #if 1
    const uint32 len = sequence.length();

    // use the custom PackedStream assign() method
    assign( len, sequence, stream  );

    // naive serial implementation
    for (uint32 i = 0; i < len; i++)
        qual_stream[i] = convert_to_phred_quality<quality_encoding>(sequence.quality(i));
  #else
    // naive serial implementation
    for (uint32 i = 0; i < sequence.length(); i++)
    {
        stream[i] = sequence[i];
        qual_stream[i] = convert_to_phred_quality<quality_encoding>(sequence.quality(i));
    }
  #endif
}

// encode a sequence according to some compile-time flags and run-time quality-encoding
//
template <Alphabet ALPHABET, typename sequence_type>
void encode(
    const QualityEncoding                                                           quality_encoding,
    const sequence_type                                                             sequence,
    typename SequenceDataEdit<ALPHABET,SequenceDataView>::sequence_stream_type      stream,
    char*                                                                           qual_stream)
{
    switch (quality_encoding)
    {
    case Phred:
        encode<ALPHABET,Phred>( sequence, stream, qual_stream );
        break;
    case Phred33:
        encode<ALPHABET,Phred33>( sequence, stream, qual_stream );
        break;
    case Phred64:
        encode<ALPHABET,Phred64>( sequence, stream, qual_stream );
        break;
    case Solexa:
        encode<ALPHABET,Solexa>( sequence, stream, qual_stream );
        break;

    default:
        break;
    }
}

// encode a sequence according to some given run-time flags and quality-encoding
//
template <Alphabet ALPHABET>
void encode(
    const SequenceDataEncoder::StrandOp                                             conversion_flags,
    const QualityEncoding                                                           quality_encoding,
    const uint32                                                                    sequence_len,
    const uint8*                                                                    sequence,
    const uint8*                                                                    quality,
    typename SequenceDataEdit<ALPHABET,SequenceDataView>::sequence_stream_type      stream,
    char*                                                                           qual_stream)
{
    const sequence_string<ALPHABET,SequenceDataEncoder::REVERSE_OP>              r_sequence( sequence_len, sequence, quality );
    const sequence_string<ALPHABET,SequenceDataEncoder::REVERSE_COMPLEMENT_OP>   rc_sequence( sequence_len, sequence, quality );
    const sequence_string<ALPHABET,SequenceDataEncoder::COMPLEMENT_OP>           fc_sequence( sequence_len, sequence, quality );
    const sequence_string<ALPHABET,SequenceDataEncoder::NO_OP>                   f_sequence( sequence_len, sequence, quality );

    if (conversion_flags & SequenceDataEncoder::REVERSE_OP)
    {
        if (conversion_flags & SequenceDataEncoder::COMPLEMENT_OP)
            encode<ALPHABET>( quality_encoding, rc_sequence, stream, qual_stream );
        else
            encode<ALPHABET>( quality_encoding, r_sequence,  stream, qual_stream );
    }
    else
    {
        if (conversion_flags & SequenceDataEncoder::COMPLEMENT_OP)
            encode<ALPHABET>( quality_encoding, fc_sequence, stream, qual_stream );
        else
            encode<ALPHABET>( quality_encoding, f_sequence,  stream, qual_stream );
    }
}

///
/// Concrete class to encode a host-side SequenceData object.
///
template <Alphabet SEQUENCE_ALPHABET>
struct SequenceDataEncoderImpl : public SequenceDataEncoder
{
    // symbol size for sequences
    static const uint32 SEQUENCE_BITS             = SequenceDataTraits<SEQUENCE_ALPHABET>::SEQUENCE_BITS;
    static const bool   SEQUENCE_BIG_ENDIAN       = SequenceDataTraits<SEQUENCE_ALPHABET>::SEQUENCE_BIG_ENDIAN;
    static const uint32 SEQUENCE_SYMBOLS_PER_WORD = SequenceDataTraits<SEQUENCE_ALPHABET>::SEQUENCE_SYMBOLS_PER_WORD;

    /// constructor
    ///
    SequenceDataEncoderImpl(SequenceDataHost* data, bool append = false) :
        SequenceDataEncoder( SEQUENCE_ALPHABET ),
        m_data( data ),
        m_append( append ) {}

    /// reserve enough storage for a given number of sequences and bps
    ///
    void reserve(const uint32 n_sequences, const uint32 n_bps)
    {
        if (m_append)
            m_data->reserve( m_data->size() + n_sequences, m_data->bps() + n_bps );
        else
            m_data->reserve( n_sequences, n_bps );
    }

    /// signals that the batch is to begin
    ///
    void begin_batch(void)
    {
        if (m_append == false)
        {
            // reset the batch
            m_data->SequenceDataInfo::operator=( SequenceDataInfo() );

            //m_data->m_sequence_vec.resize( 0 );
            //m_data->m_qual_vec.resize( 0 );
            //m_data->m_name_vec.resize( 0 );

            if (m_data->m_sequence_index_vec.size() < 1)
                m_data->m_sequence_index_vec.resize( 1 );
            m_data->m_sequence_index_vec[0] = 0;

            if (m_data->m_name_index_vec.size() < 1)
                m_data->m_name_index_vec.resize( 1 );
            m_data->m_name_index_vec[0] = 0;
        }

        // assign the alphabet
        m_data->m_alphabet = SEQUENCE_ALPHABET;
        m_data->m_has_qualities = true;
    }

    /// add a sequence to the end of this batch
    ///
    /// \param sequence_len                 input sequence length
    /// \param name                         sequence name
    /// \param base_pairs                   list of base pairs
    /// \param quality                      list of base qualities
    /// \param quality_encoding             quality encoding scheme
    /// \param truncate_sequence_len        truncate the sequence if longer than this
    /// \param conversion_flags             conversion operators applied to the strand
    ///
    void push_back(
        const uint32             in_sequence_len,
        const char*              name,
        const uint8*             base_pairs,
        const uint8*             quality,
        const QualityEncoding    quality_encoding,
        const uint32             max_sequence_len,
        const uint32             trim3,
        const uint32             trim5,
        const StrandOp           conversion_flags)
    {
        const uint32 trimmed_len = in_sequence_len > trim3 + trim5 ?
                                   in_sequence_len - trim3 - trim5 : 0u;

        // truncate sequence
        const uint32 sequence_len = nvbio::min( trimmed_len, max_sequence_len );

        base_pairs += trim5;
        quality    += trim5;

        assert(sequence_len);

        // resize the sequences & quality buffers
        {
            static const uint32 bps_per_word = 32u / SEQUENCE_BITS;
            const uint32 stream_len = m_data->m_sequence_stream_len + sequence_len;
            const uint32 words      = util::divide_ri( stream_len, bps_per_word );

            if (m_data->m_sequence_vec.size() < words)
                m_data->m_sequence_vec.resize( words*2 );
            if (m_data->m_qual_vec.size() < stream_len)
                m_data->m_qual_vec.resize( stream_len*2 );

            m_data->m_sequence_stream_words = words;
        }

        // encode the sequence data
        typename SequenceDataEdit<SEQUENCE_ALPHABET,SequenceDataView>::sequence_stream_type stream( nvbio::raw_pointer( m_data->m_sequence_vec ) );
        encode<SEQUENCE_ALPHABET>(
            conversion_flags,
            quality_encoding,
            sequence_len,
            base_pairs,
            quality,
            stream + m_data->m_sequence_stream_len,
            nvbio::raw_pointer( m_data->m_qual_vec ) + m_data->m_sequence_stream_len );

        // update sequence and bp counts
        m_data->m_n_seqs++;
        m_data->m_sequence_stream_len += sequence_len;

        if (m_data->m_sequence_index_vec.size() < m_data->m_n_seqs + 1u)
            m_data->m_sequence_index_vec.resize( (m_data->m_n_seqs + 1u)*2 );
        m_data->m_sequence_index_vec[ m_data->m_n_seqs ] = m_data->m_sequence_stream_len;

        m_data->m_min_sequence_len = nvbio::min( m_data->m_min_sequence_len, sequence_len );
        m_data->m_max_sequence_len = nvbio::max( m_data->m_max_sequence_len, sequence_len );

        // store the sequence name
        const uint32 name_len = uint32(strlen(name));
        const uint32 name_offset = m_data->m_name_stream_len;

        if (m_data->m_name_vec.size() < name_offset + name_len + 1)
            m_data->m_name_vec.resize( (name_offset + name_len + 1)*2 );
        memcpy( nvbio::raw_pointer( m_data->m_name_vec ) + name_offset, name, name_len + 1 );

        m_data->m_name_stream_len += name_len + 1;

        if (m_data->m_name_index_vec.size() < m_data->m_n_seqs + 1u)
            m_data->m_name_index_vec.resize( (m_data->m_n_seqs + 1u)*2 );
        m_data->m_name_index_vec[ m_data->m_n_seqs ] = m_data->m_name_stream_len;
    }

    /// signals that the batch is complete
    ///
    void end_batch(void)
    {
        assert( m_data->m_sequence_stream_words == util::divide_ri( m_data->m_sequence_stream_len, SEQUENCE_SYMBOLS_PER_WORD ) );

        m_data->m_avg_sequence_len = (uint32) ceilf(float(m_data->m_sequence_stream_len) / float(m_data->m_n_seqs));
    }

    /// return the sequence data info
    ///
    const SequenceDataInfo* info() const { return m_data; }

private:
    SequenceDataHost* m_data;
    bool              m_append;
};

// create a sequence encoder
//
SequenceDataEncoder* create_encoder(const Alphabet alphabet, SequenceDataHost* data)
{
    switch (alphabet)
    {
    case DNA:
        return new SequenceDataEncoderImpl<DNA>( data );
        break;
    case DNA_N:
        return new SequenceDataEncoderImpl<DNA_N>( data );
        break;
    case PROTEIN:
        return new SequenceDataEncoderImpl<PROTEIN>( data );
        break;
    case RNA:
        return new SequenceDataEncoderImpl<RNA>( data );
        break;
    case RNA_N:
        return new SequenceDataEncoderImpl<RNA_N>( data );
        break;
    case ASCII:
        return new SequenceDataEncoderImpl<ASCII>( data );
        break;

    default:
        break;
    }
    return NULL;
}

// next batch
//
int next(const Alphabet alphabet, SequenceDataHost* data, SequenceDataStream* stream, const uint32 batch_size, const uint32 batch_bps)
{
    switch (alphabet)
    {
    case DNA:
        {
            SequenceDataEncoderImpl<DNA> encoder( data );
            return stream->next( &encoder, batch_size, batch_bps );
        }
        break;
    case DNA_N:
        {
            SequenceDataEncoderImpl<DNA_N> encoder( data );
            return stream->next( &encoder, batch_size, batch_bps );
        }
        break;
    case PROTEIN:
        {
            SequenceDataEncoderImpl<PROTEIN> encoder( data );
            return stream->next( &encoder, batch_size, batch_bps );
        }
        break;
    case RNA:
        {
            SequenceDataEncoderImpl<RNA> encoder( data );
            return stream->next( &encoder, batch_size, batch_bps );
        }
        break;
    case RNA_N:
        {
            SequenceDataEncoderImpl<RNA_N> encoder( data );
            return stream->next( &encoder, batch_size, batch_bps );
        }
        break;
    case ASCII:
        {
            SequenceDataEncoderImpl<ASCII> encoder( data );
            return stream->next( &encoder, batch_size, batch_bps );
        }
        break;

    default:
        break;
    }
    return 0;
}

// next batch
//
int append(const Alphabet alphabet, SequenceDataHost* data, SequenceDataStream* stream, const uint32 batch_size, const uint32 batch_bps)
{
    switch (alphabet)
    {
    case DNA:
        {
            SequenceDataEncoderImpl<DNA> encoder( data, true );
            return stream->next( &encoder, batch_size, batch_bps );
        }
        break;
    case DNA_N:
        {
            SequenceDataEncoderImpl<DNA_N> encoder( data, true );
            return stream->next( &encoder, batch_size, batch_bps );
        }
        break;
    case PROTEIN:
        {
            SequenceDataEncoderImpl<PROTEIN> encoder( data, true );
            return stream->next( &encoder, batch_size, batch_bps );
        }
        break;
    case RNA:
        {
            SequenceDataEncoderImpl<DNA> encoder( data, true );
            return stream->next( &encoder, batch_size, batch_bps );
        }
        break;
    case RNA_N:
        {
            SequenceDataEncoderImpl<RNA_N> encoder( data, true );
            return stream->next( &encoder, batch_size, batch_bps );
        }
        break;
    case ASCII:
        {
            SequenceDataEncoderImpl<ASCII> encoder( data, true );
            return stream->next( &encoder, batch_size, batch_bps );
        }
        break;

    default:
        break;
    }
    return 0;
}

// next batch
//
int skip(SequenceDataStream* stream, const uint32 batch_size, const uint32 batch_bps)
{
    SequenceDataEncoder encoder( PROTEIN );
    return stream->next( &encoder, batch_size );
}

} // namespace io
} // namespace nvbio
