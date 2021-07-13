// ***************************************************************************
// BamWriter.h (c) 2009 Michael Strömberg, Derek Barnett
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

#ifndef BAMWRITER_H
#define BAMWRITER_H

// C++ includes
#include <string>

// BamTools includes
#include "BamAux.h"

namespace BamTools {

class BamWriter {

    // constructor/destructor
    public:
        BamWriter(void);
        ~BamWriter(void);

    // public interface
    public:
        // closes the alignment archive
        void Close(void);
        // opens the alignment archive
        void Open(const std::string& filename, const std::string& samHeader, const BamTools::RefVector& referenceSequences);
        // saves the alignment to the alignment archive
        void SaveAlignment(const BamTools::BamAlignment& al);

    // private implementation
    private:
        struct BamWriterPrivate;
        BamWriterPrivate* d;
};

} // namespace BamTools

#endif // BAMWRITER_H
