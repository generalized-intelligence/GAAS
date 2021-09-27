nvbio
=====

NVBIO is a library of reusable components designed by NVIDIA Corporatin to accelerate bioinformatics applications using CUDA.


Though it is specifically designed to unleash the power of NVIDIA GPUs,
most of its components are completely cross-platform and can be used both from host C++ and device CUDA code.

The purpose of NVBIO is twofold: it can be thought of both as a solid basis to build new, modern applications targeting
GPUs, which deferring the core computations to a library will always automatically and transparently benefit from new
advances in GPU computing, and as example material to design novel bioinformatics algorithms for massively parallel architectures.

Additionally, NVBIO contains a suite of applications built on top of it, including a re-engineered implementation of the famous
Bowtie2 short read aligner.
Unlike many prototypes, nvBowtie is an attempt to build an industrial strength aligner, reproducing most of Bowtie2's
original features as well as adding a few more, such as efficient support for direct BAM (and soon CRAM) output.


NVBIO is hosted on GitHub at:

  http://nvlabs.github.io/nvbio/


To compile, you can perform the following easy steps from within the installation directory:

  1. mkdir build;
  2. cd build;
  3. cmake ..;
  4. make -j8;
  
By default, NVBIO will be compiled for sm_35. For newer architectures, please use the cmake option -DGPU_ARCHITECTURE=sm_XX.

The main contributors of NVBIO are:

  Jacopo Pantaleoni  -  jpantaleoni@nvidia.com
  Nuno Subtil        -  nsubtil@nvidia.com
