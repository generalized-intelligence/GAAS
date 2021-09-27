// ***************************************************************************
// BamReader.h (c) 2009 Derek Barnett, Michael Strömberg
// Marth Lab, Department of Biology, Boston College
// All rights reserved.
// ---------------------------------------------------------------------------
// Last modified: 8 December 2009 (DB)
// ---------------------------------------------------------------------------
// Uses BGZF routines were adapted from the bgzf.c code developed at the Broad
// Institute.
// ---------------------------------------------------------------------------
// Provides the basic functionality for reading BAM files
// ***************************************************************************

#ifndef BAMREADER_H
#define BAMREADER_H

// C++ includes
#include <string>

// BamTools includes
#include "BamAux.h"

namespace BamTools {

class BamReader {

    // constructor / destructor
    public:
        BamReader(void);
        ~BamReader(void);

    // public interface
    public:

        // ----------------------
        // BAM file operations
        // ----------------------

        // close BAM file
        void Close(void);
        // performs random-access jump to reference, position
        bool Jump(int refID, int position = 0);
        // opens BAM file (and optional BAM index file, if provided)
        void Open(const std::string& filename, const std::string& indexFilename = "");
        // returns file pointer to beginning of alignments
        bool Rewind(void);

        // ----------------------
        // access alignment data
        // ----------------------

        // retrieves next available alignment (returns success/fail)
        bool GetNextAlignment(BamAlignment& bAlignment);

        // ----------------------
        // access auxiliary data
        // ----------------------

        // returns SAM header text
        const std::string GetHeaderText(void) const;
        // returns number of reference sequences
        const int GetReferenceCount(void) const;
        // returns vector of reference objects
        const BamTools::RefVector GetReferenceData(void) const;
        // returns reference id (used for BamReader::Jump()) for the given reference name
        const int GetReferenceID(const std::string& refName) const;

        // ----------------------
        // BAM index operations
        // ----------------------

        // creates index for BAM file, saves to file (default = bamFilename + ".bai")
        bool CreateIndex(void);

    // private implementation
    private:
        struct BamReaderPrivate;
        BamReaderPrivate* d;
};

} // namespace BamTools

#endif // BAMREADER_H
