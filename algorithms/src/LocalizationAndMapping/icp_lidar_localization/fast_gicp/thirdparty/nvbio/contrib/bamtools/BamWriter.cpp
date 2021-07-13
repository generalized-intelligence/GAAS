// ***************************************************************************
// BamWriter.cpp (c) 2009 Michael Strömberg, Derek Barnett
// Marth Lab, Department of Biology, Boston College
// All rights reserved.
// ---------------------------------------------------------------------------
// Last modified: 8 December 2009 (DB)
// ---------------------------------------------------------------------------
// Uses BGZF routines were adapted from the bgzf.c code developed at the Broad
// Institute.
// ---------------------------------------------------------------------------
// Provides the basic functionality for producing BAM files
// ***************************************************************************

// BGZF includes
#include "BGZF.h"
#include "BamWriter.h"
using namespace BamTools;
using namespace std;

struct BamWriter::BamWriterPrivate {

    // data members
    BgzfData mBGZF;

    string mpackedCigar;
    string mencodedQuery;
    string mbaseQualities;

    // constructor / destructor
    BamWriterPrivate(void) { }
    ~BamWriterPrivate(void) {
        mBGZF.Close();
    }

    // "public" interface
    void Close(void);
    void Open(const std::string& filename, const std::string& samHeader, const BamTools::RefVector& referenceSequences);
    void SaveAlignment(const BamTools::BamAlignment& al);

    // internal methods
    void CreatePackedCigar(const std::vector<CigarOp>& cigarOperations, std::string& packedCigar);
    void EncodeQuerySequence(const std::string& query, std::string& encodedQuery);
};

// -----------------------------------------------------
// BamWriter implementation
// -----------------------------------------------------

// constructor
BamWriter::BamWriter(void) {
    d = new BamWriterPrivate;
}

// destructor
BamWriter::~BamWriter(void) {
    delete d;
    d = 0;
}

// closes the alignment archive
void BamWriter::Close(void) {
    d->Close();
}

// opens the alignment archive
void BamWriter::Open(const string& filename, const string& samHeader, const RefVector& referenceSequences) {
    d->Open(filename, samHeader, referenceSequences);
}

// saves the alignment to the alignment archive
void BamWriter::SaveAlignment(const BamAlignment& al) {
    d->SaveAlignment(al);
}

// -----------------------------------------------------
// BamWriterPrivate implementation
// -----------------------------------------------------

// closes the alignment archive
void BamWriter::BamWriterPrivate::Close(void) {
    mBGZF.Close();
}

// creates a cigar string from the supplied alignment
void BamWriter::BamWriterPrivate::CreatePackedCigar(const vector<CigarOp>& cigarOperations, string& packedCigar) {

    // initialize
    const unsigned int numCigarOperations = (unsigned int)cigarOperations.size();
    packedCigar.resize(numCigarOperations * BT_SIZEOF_INT);

    // pack the cigar data into the string
    unsigned int* pPackedCigar = (unsigned int*)packedCigar.data();

    unsigned int cigarOp;
    vector<CigarOp>::const_iterator coIter;
    for(coIter = cigarOperations.begin(); coIter != cigarOperations.end(); coIter++) {

        switch(coIter->Type) {
            case 'M':
                  cigarOp = BAM_CMATCH;
                  break;
            case 'I':
                  cigarOp = BAM_CINS;
                  break;
            case 'D':
                  cigarOp = BAM_CDEL;
                  break;
            case 'N':
                  cigarOp = BAM_CREF_SKIP;
                  break;
            case 'S':
                  cigarOp = BAM_CSOFT_CLIP;
                  break;
            case 'H':
                  cigarOp = BAM_CHARD_CLIP;
                  break;
            case 'P':
                  cigarOp = BAM_CPAD;
                  break;
            default:
                  printf("ERROR: Unknown cigar operation found: %c\n", coIter->Type);
                  exit(1);
        }

        *pPackedCigar = coIter->Length << BAM_CIGAR_SHIFT | cigarOp;
        pPackedCigar++;
    }
}

// encodes the supplied query sequence into 4-bit notation
void BamWriter::BamWriterPrivate::EncodeQuerySequence(const string& query, string& encodedQuery) {

    // prepare the encoded query string
    const unsigned int queryLen = (unsigned int)query.size();
    const unsigned int encodedQueryLen = (unsigned int)((queryLen / 2.0) + 0.5);
    encodedQuery.resize(encodedQueryLen);
    char* pEncodedQuery = (char*)encodedQuery.data();
    const char* pQuery = (const char*)query.data();

    unsigned char nucleotideCode;
    bool useHighWord = true;

    while(*pQuery) {

        switch(*pQuery) {
            case '=':
                    nucleotideCode = 0;
                    break;
            case 'A':
                    nucleotideCode = 1;
                    break;
            case 'C':
                    nucleotideCode = 2;
                    break;
            case 'G':
                    nucleotideCode = 4;
                    break;
            case 'T':
                    nucleotideCode = 8;
                    break;
            case 'N':
                    nucleotideCode = 15;
                    break;
            default:
                    printf("ERROR: Only the following bases are supported in the BAM format: {=, A, C, G, T, N}. Found [%c]\n", *pQuery);
                    exit(1);
        }

        // pack the nucleotide code
        if(useHighWord) {
            *pEncodedQuery = nucleotideCode << 4;
            useHighWord = false;
        } else {
            *pEncodedQuery |= nucleotideCode;
            pEncodedQuery++;
            useHighWord = true;
        }

        // increment the query position
        pQuery++;
    }
}

// opens the alignment archive
void BamWriter::BamWriterPrivate::Open(const string& filename, const string& samHeader, const RefVector& referenceSequences) {

    // open the BGZF file for writing
    mBGZF.Open(filename, "wb");

    // ================
    // write the header
    // ================

    // write the BAM signature
    const unsigned char SIGNATURE_LENGTH = 4;
    const char* BAM_SIGNATURE = "BAM\1";
    mBGZF.Write(BAM_SIGNATURE, SIGNATURE_LENGTH);

    // write the SAM header text length
    const unsigned int samHeaderLen = (unsigned int)samHeader.size();
    mBGZF.Write((char*)&samHeaderLen, BT_SIZEOF_INT);

    // write the SAM header text
    if(samHeaderLen > 0) {
        mBGZF.Write(samHeader.data(), samHeaderLen);
    }

    // write the number of reference sequences
    const unsigned int numReferenceSequences = (unsigned int)referenceSequences.size();
    mBGZF.Write((char*)&numReferenceSequences, BT_SIZEOF_INT);

    // =============================
    // write the sequence dictionary
    // =============================

    RefVector::const_iterator rsIter;
    for(rsIter = referenceSequences.begin(); rsIter != referenceSequences.end(); rsIter++) {

        // write the reference sequence name length
        const unsigned int referenceSequenceNameLen = (unsigned int)rsIter->RefName.size() + 1;
        mBGZF.Write((char*)&referenceSequenceNameLen, BT_SIZEOF_INT);

        // write the reference sequence name
        mBGZF.Write(rsIter->RefName.c_str(), referenceSequenceNameLen);

        // write the reference sequence length
        mBGZF.Write((char*)&rsIter->RefLength, BT_SIZEOF_INT);
    }
}

// saves the alignment to the alignment archive
void BamWriter::BamWriterPrivate::SaveAlignment(const BamAlignment& al) {

    // initialize
    const unsigned int nameLen            = (unsigned int)al.Name.size() + 1;
    const unsigned int queryLen           = (unsigned int)al.QueryBases.size();
    const unsigned int numCigarOperations = (unsigned int)al.CigarData.size();

    // create our packed cigar string
    string& packedCigar = mpackedCigar;
    packedCigar.erase( packedCigar.begin(), packedCigar.end() );
    CreatePackedCigar(al.CigarData, packedCigar);
    const unsigned int packedCigarLen = (unsigned int)packedCigar.size();

    // encode the query
    string& encodedQuery = mencodedQuery;
    encodedQuery.erase( encodedQuery.begin(), encodedQuery.end() );
    EncodeQuerySequence(al.QueryBases, encodedQuery);
    const unsigned int encodedQueryLen = (unsigned int)encodedQuery.size();

    // store the tag data length
    const unsigned int tagDataLength = (unsigned int)al.TagData.size() + 1;

    // assign the BAM core data
    unsigned int buffer[8];
    buffer[0] = al.RefID;
    buffer[1] = al.Position;
    buffer[2] = (al.Bin << 16) | (al.MapQuality << 8) | nameLen;
    buffer[3] = (al.AlignmentFlag << 16) | numCigarOperations;
    buffer[4] = queryLen;
    buffer[5] = al.MateRefID;
    buffer[6] = al.MatePosition;
    buffer[7] = al.InsertSize;

    // write the block size
    const unsigned int dataBlockSize = nameLen + packedCigarLen + encodedQueryLen + queryLen + tagDataLength;
    const unsigned int blockSize = BAM_CORE_SIZE + dataBlockSize;
    mBGZF.Write((char*)&blockSize, BT_SIZEOF_INT);

    // write the BAM core
    mBGZF.Write((char*)&buffer, BAM_CORE_SIZE);

    // write the query name
    mBGZF.Write(al.Name.c_str(), nameLen);

    // write the packed cigar
    mBGZF.Write(packedCigar.data(), packedCigarLen);

    // write the encoded query sequence
    mBGZF.Write(encodedQuery.data(), encodedQueryLen);

    // write the base qualities
    mbaseQualities = al.Qualities;
    char* pBaseQualities = (char*)mbaseQualities.data();
    for(unsigned int i = 0; i < queryLen; i++) { pBaseQualities[i] -= 33; }
    mBGZF.Write(pBaseQualities, queryLen);

    // write the read group tag
    mBGZF.Write(al.TagData.data(), tagDataLength);
}
