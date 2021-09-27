// ***************************************************************************
// BamAux.h (c) 2009 Derek Barnett, Michael Str�mberg
// Marth Lab, Department of Biology, Boston College
// All rights reserved.
// ---------------------------------------------------------------------------
// Last modified: 8 December 2009 (DB)
// ---------------------------------------------------------------------------
// Provides the basic constants, data structures, etc. for using BAM files
// ***************************************************************************

#ifndef BAMAUX_H
#define BAMAUX_H

#include "BGZF.h"

// C inclues
#include <cstdlib>
#include <cstring>

// C++ includes
#include <exception>
#include <map>
#include <string>
#include <utility>
#include <vector>

namespace BamTools {

// BAM constants
const int BAM_CORE_SIZE   = 32;
const int BAM_CMATCH      = 0;
const int BAM_CINS        = 1;
const int BAM_CDEL        = 2;
const int BAM_CREF_SKIP   = 3;
const int BAM_CSOFT_CLIP  = 4;
const int BAM_CHARD_CLIP  = 5;
const int BAM_CPAD        = 6;
const int BAM_CIGAR_SHIFT = 4;
const int BAM_CIGAR_MASK  = ((1 << BAM_CIGAR_SHIFT) - 1);

// BAM index constants
const int MAX_BIN           = 37450;	// =(8^6-1)/7+1
const int BAM_MIN_CHUNK_GAP = 32768;
const int BAM_LIDX_SHIFT    = 14;

// Explicit variable sizes
const int BT_SIZEOF_INT = 4;

struct CigarOp;
struct RefEdit;

struct BamAlignment {

    // Queries against alignment flag
    public:
        // Returns true if this read is a PCR duplicate (determined by external app)
        bool IsDuplicate(void) const { return ( (AlignmentFlag & DUPLICATE) != 0 ); }
        // Returns true if this read failed quality control (determined by external app)
        bool IsFailedQC(void) const { return ( (AlignmentFlag & QC_FAILED) != 0 ); }
        // Returns true if alignment is first mate on read
        bool IsFirstMate(void) const { return ( (AlignmentFlag & READ_1) != 0 ); }
        // Returns true if alignment is mapped
        bool IsMapped(void) const { return ( (AlignmentFlag & UNMAPPED) == 0 ); }
        // Returns true if alignment's mate is mapped
        bool IsMateMapped(void) const { return ( (AlignmentFlag & MATE_UNMAPPED) == 0 ); }
        // Returns true if alignment's mate mapped to reverse strand
        bool IsMateReverseStrand(void) const { return ( (AlignmentFlag & MATE_REVERSE)  != 0 ); }
        // Returns true if alignment part of paired-end read
        bool IsPaired(void) const { return ( (AlignmentFlag & PAIRED) != 0 ); }
        // Returns true if this position is primary alignment (determined by external app)
        bool IsPrimaryAlignment(void) const  { return ( (AlignmentFlag & SECONDARY) == 0 ); }
        // Returns true if alignment is part of read that satisfied paired-end resolution (determined by external app)
        bool IsProperPair(void) const { return ( (AlignmentFlag & PROPER_PAIR) != 0 ); }
        // Returns true if alignment mapped to reverse strand
        bool IsReverseStrand(void) const { return ( (AlignmentFlag & REVERSE) != 0 ); }
        // Returns true if alignment is second mate on read
        bool IsSecondMate(void) const { return ( (AlignmentFlag & READ_2) != 0 ); }

    public:

        // get "RG" tag data
        bool GetReadGroup(std::string& readGroup) const
        {
            const char* rg = GetTag("RG");
            if (rg==NULL || !rg)
            {
                return false;
            }
            // assign the read group
            const unsigned int readGroupLen = (unsigned int)std::strlen(rg);
            readGroup = std::string(rg, readGroupLen);
            //readGroup.resize(readGroupLen);
            //std::memcpy((char*)readGroup.data, rg, readGroupLen );
            return true;
        }

        // get "NM" tag data - contributed by Aaron Quinlan
        bool GetEditDistance(uint8_t& editDistance) const { return GetTag( "NM", editDistance ); }

        // get generic tag data - NVBIO
        template <typename T>
        bool GetTag(const char* tagName, T& tagVal) const
        {
            if ( TagData.empty() ) { return false; }

            // localize the tag data
            const char* pTagData = (const char*)TagData.c_str();
            const char* pTagStorageType = NULL;
            const unsigned int tagDataLen = (unsigned int)TagData.size() + 1;
            unsigned int numBytesParsed = 0;

            bool foundTag = false;
            while( numBytesParsed < tagDataLen )
            {
                // skip white spaces
                while ( (*pTagData == ' ' || *pTagData == '\n') && numBytesParsed < tagDataLen)
                {
                    ++numBytesParsed;
                    ++pTagData;
                }
                if (*pTagData == '\0' || numBytesParsed == tagDataLen)
                    break;

                const char* pTagType = pTagData;
                pTagStorageType = pTagData + 2;
                pTagData       += 3;
                numBytesParsed += 3;

                // check the current tag
                if (strncmp(pTagType, tagName, 2) == 0) {
                    foundTag = true;
                    break;
                }

                // get the storage class and find the next tag
                SkipToNextTag( pTagType, *pTagStorageType, pTagData, numBytesParsed );
            }
            // return if the edit distance tag was not present
            if ( !foundTag ) { return false; }

            // assign the tag value
            return GetTagValue( tagName, pTagData, *pTagStorageType, &tagVal );
        }

        // get generic tag data - NVBIO
        const char* GetTag(const char* tagName) const
        {
            if ( TagData.empty() ) { return NULL; }

            // localize the tag data
            const char* pTagData = (const char*)TagData.c_str();
            const unsigned int tagDataLen = (unsigned int)TagData.size() + 1;
            unsigned int numBytesParsed = 0;

            while( numBytesParsed < tagDataLen )
            {
                // skip white spaces
                while ( (*pTagData == ' ' || *pTagData == '\n') && numBytesParsed < tagDataLen)
                {
                    ++numBytesParsed;
                    ++pTagData;
                }
                if (*pTagData == '\0' || numBytesParsed == tagDataLen)
                    break;

                const char* pTagType = pTagData;
                const char* pTagStorageType = pTagData + 2;
                pTagData       += 3;
                numBytesParsed += 3;

                // check the current tag
                if (strncmp(pTagType, tagName, 2) == 0)
                    return pTagData;

                // get the storage class and find the next tag
                SkipToNextTag( pTagType, *pTagStorageType, pTagData, numBytesParsed );
            }
            // return if the edit distance tag was not present
            return NULL;
        }

    private:
        static bool CheckType(const char storageType,  int8_t)  { return (storageType == 'A') || (storageType == 'c'); }
        static bool CheckType(const char storageType, uint8_t)  { return (storageType == 'C'); }
        static bool CheckType(const char storageType,  int16_t) { return (storageType == 's') || (storageType == 'c') || (storageType == 'C'); }
        static bool CheckType(const char storageType, uint16_t) { return (storageType == 'S') || (storageType == 'C'); }
        static bool CheckType(const char storageType,  int32_t) { return (storageType == 'i') || (storageType == 'c') || (storageType == 'C') || (storageType == 's') || (storageType == 'S'); }
        static bool CheckType(const char storageType, uint32_t) { return (storageType == 'I') || (storageType == 'C') || (storageType == 'S'); }
        static bool CheckType(const char storageType, float)    { return (storageType == 'f'); }

        template <typename T>
        static bool GetTagValue(const char* tagName, const char *pTagData, const char storageType, T* tagVal)
        {
            switch(storageType) {
                case 'A':
                    if (CheckType(storageType,T()) == false)
                        return false;

                    *tagVal = T( *(const char*)(pTagData) );
                    break;
                case 'c':
                    if (CheckType(storageType,T()) == false)
                        return false;

                    *tagVal = T( *(const int8_t*)(pTagData) );
                    break;
                case 'C':
                    if (CheckType(storageType,T()) == false)
                        return false;

                    *tagVal = T( *(const uint8_t*)(pTagData) );
                    break;
                case 's':
                    if (CheckType(storageType,T()) == false)
                        return false;

                    *tagVal = T( *(const int16_t*)(pTagData) );
                    break;
                case 'S':
                    if (CheckType(storageType,T()) == false)
                        return false;

                    *tagVal = T( *(const uint16_t*)(pTagData) );
                    break;

                case 'i':
                case 'f':
                case 'I':
                    if (CheckType(storageType,T()) == false)
                        return false;

                    *tagVal = T( *(const T*)(pTagData) );
                    break;

                case 'B':
                    {
                        const char type    = *pTagData++;
                        const uint8_t len0 = *pTagData++;
                        const uint8_t len1 = *pTagData++;
                        const uint8_t len2 = *pTagData++;
                        const uint8_t len3 = *pTagData++;
                        const uint32_t len = len0 | (len1 << 8) | (len2 << 16) | (len3 << 24);

                        if (CheckType(type,T()) == false)
                            return false;

                        const T* tagData = (const T*)pTagData;

                        switch (type)
                        {
                        case 'c':
                        case 'C':
                            for (uint32_t i = 0; i < len; ++i)
                                *tagVal++ = *(const T*)(tagData++);
                            break;
                        case 's':
                        case 'S':
                            for (uint32_t i = 0; i < len; ++i)
                                *tagVal++ = *(const T*)(tagData++);
                            break;
                        case 'i':
                        case 'I':
                        case 'f':
                            for (uint32_t i = 0; i < len; ++i)
                                *tagVal++ = *(const T*)(tagData++);
                            break;
                        }
                    }

                case 'Z':
                    if (CheckType(storageType,T()) == false)
                        return false;

                    while(*pTagData != '\0')
                        *tagVal++ = *pTagData++;

                    break;

                case 'H':
                    // TODO!
                    {
                        char tagString[3];
                        tagString[0] = tagName[0];
                        tagString[1] = tagName[1];
                        tagString[2] = '\0';

                        fprintf(stderr,"WARNING: tag storage class 'H' is not yet supported, tag[%s:H]\n", tagString);
                        return false;
                    }
                    //break;

                default:
                    {
                        //char tagString[3];
                        //tagString[0] = tagName[0];
                        //tagString[1] = tagName[1];
                        //tagString[2] = '\0';

                        //fprintf(stderr,"ERROR: Unknown tag storage class encountered, tag[%s:%c]\n", tagString, storageType);
                        //exit(1);
                    }
            }
            return true;
        }

        static void SkipToNextTag(const char* tagName, const char storageType, const char* &pTagData, unsigned int& numBytesParsed)
        {
            switch(storageType) {
                case 'A':
                case 'c':
                case 'C':
                        ++numBytesParsed;
                        ++pTagData;
                        break;

                case 'B':
                    {
                        const char type    = *pTagData++;
                        const uint8_t len0 = *pTagData++;
                        const uint8_t len1 = *pTagData++;
                        const uint8_t len2 = *pTagData++;
                        const uint8_t len3 = *pTagData++;
                        const uint32_t len = len0 | (len1 << 8) | (len2 << 16) | (len3 << 24);

                        switch (type)
                        {
                        case 'c':
                        case 'C':
                            pTagData += len;
                            break;
                        case 's':
                        case 'S':
                            pTagData += len * 2;
                            break;
                        case 'i':
                        case 'I':
                        case 'f':
                            pTagData += len * 4;
                            break;
                        }
                        break;
                    }

                case 's':
                case 'S':
                        numBytesParsed += 2;
                        pTagData       += 2;
                        break;

                case 'i':
                case 'f':
                case 'I':
                        numBytesParsed += 4;
                        pTagData       += 4;
                        break;

                case 'Z':
                case 'H':
                        while(*pTagData != '\0') {
                            ++numBytesParsed;
                            ++pTagData;
                        }
                        if(*pTagData == '\0')
                        {
                            ++numBytesParsed;
                            ++pTagData;
                        }
                        break;

                default:
                    {
                        //char tagString[3];
                        //tagString[0] = tagName[0];
                        //tagString[1] = tagName[1];
                        //tagString[2] = '\0';

                        //fprintf(stderr,"ERROR: Unknown tag storage class encountered: [%s:%c]\n", tagString, storageType);
                        //exit(1);
                    }
            }
        }

    // Data members
    public:
        std::string  Name;              // Read name
        int32_t      Length;            // Query length
        std::string  QueryBases;        // 'Original' sequence (as reported from sequencing machine)
        std::string  AlignedBases;      // 'Aligned' sequence (includes any indels, padding, clipping)
        std::string  Qualities;         // FASTQ qualities (ASCII characters, not numeric values)
        std::string  TagData;           // Tag data (accessor methods will pull the requested information out)
        int32_t      RefID;             // ID number for reference sequence
        int32_t      Position;          // Position (0-based) where alignment starts
        uint16_t     Bin;               // Bin in BAM file where this alignment resides
        uint16_t     MapQuality;        // Mapping quality score
        uint32_t     AlignmentFlag;     // Alignment bit-flag - see Is<something>() methods for available queries
        std::vector<CigarOp> CigarData; // CIGAR operations for this alignment
        std::vector<RefEdit> DeltaData; // Delta operations from reference to the query sequence
        int32_t      MateRefID;         // ID number for reference sequence where alignment's mate was aligned
        int32_t      MatePosition;      // Position (0-based) where alignment's mate starts
        int32_t      InsertSize;        // Mate-pair insert size

    // Alignment flag query constants
        enum { PAIRED        = 1,
               PROPER_PAIR   = 2,
               UNMAPPED      = 4,
               MATE_UNMAPPED = 8,
               REVERSE       = 16,
               MATE_REVERSE  = 32,
               READ_1        = 64,
               READ_2        = 128,
               SECONDARY     = 256,
               QC_FAILED     = 512,
               DUPLICATE     = 1024
             };
};

// ----------------------------------------------------------------
// Auxiliary data structs & typedefs

struct CigarOp {
    char     Type;   // Operation type (MIDNSHP)
    uint32_t Length; // Operation length (number of bases)
};

struct RefEdit
{
    char Type; //Op type (MID etc.)
    uint32_t L; //parameter - length for ID, Sub Code for X
    uint32_t pos; //position in the query sequence
};
struct RefData {
    // data members
    std::string RefName;          // Name of reference sequence
    uint32_t    RefLength;        // Length of reference sequence
    bool        RefHasAlignments; // True if BAM file contains alignments mapped to reference sequence
    // constructor
    RefData(void)
        : RefLength(0)
        , RefHasAlignments(false)
    { }
};

typedef std::vector<RefData> RefVector;
typedef std::vector<BamAlignment> BamAlignmentVector;

// ----------------------------------------------------------------
// Indexing structs & typedefs

struct Chunk {
    // data members
    uint64_t Start;
    uint64_t Stop;
    // constructor
    Chunk(const uint64_t& start = 0, const uint64_t& stop = 0)
        : Start(start)
        , Stop(stop)
    { }
};

inline
bool ChunkLessThan(const Chunk& lhs, const Chunk& rhs) {
    return lhs.Start < rhs.Start;
}

typedef std::vector<Chunk> ChunkVector;
typedef std::map<uint32_t, ChunkVector> BamBinMap;
typedef std::vector<uint64_t> LinearOffsetVector;

struct ReferenceIndex {
    // data members
    BamBinMap Bins;
    LinearOffsetVector Offsets;
    // constructor
    ReferenceIndex(const BamBinMap& binMap = BamBinMap(),
                   const LinearOffsetVector& offsets = LinearOffsetVector())
        : Bins(binMap)
        , Offsets(offsets)
    { }
};

typedef std::vector<ReferenceIndex> BamIndex;

} // namespace BamTools

#endif // BAMAUX_H
