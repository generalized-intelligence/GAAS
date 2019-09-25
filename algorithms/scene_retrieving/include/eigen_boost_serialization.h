/*
Copyright (c) 2015 Michael Tao

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
*/


#ifndef EIGEN_BOOST_SERIALIZATION
#define EIGEN_BOOST_SERIALIZATION
#include <Eigen/Sparse>
#include <Eigen/Dense>
#include <boost/serialization/split_free.hpp>
#include <boost/serialization/vector.hpp>

namespace boost{
  namespace serialization{
    
    template <class Archive, typename _Scalar, int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols>
        void save(Archive & ar, const Eigen::Matrix<_Scalar,_Rows,_Cols,_Options,_MaxRows,_MaxCols> & m, const unsigned int version) {
            int rows=m.rows(),cols=m.cols();
            ar & rows;
            ar & cols;
            ar & boost::serialization::make_array(m.data(), rows*cols);
        }
    template <class Archive, typename _Scalar, int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols>
        void load(Archive & ar, Eigen::Matrix<_Scalar,_Rows,_Cols,_Options,_MaxRows,_MaxCols> & m, const unsigned int version) {
            int rows,cols;
            ar & rows;
            ar & cols;
            m.resize(rows,cols);
            ar & boost::serialization::make_array(m.data(), rows*cols);
        }

    template <class Archive, typename _Scalar, int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols>
        void serialize(Archive & ar, Eigen::Matrix<_Scalar,_Rows,_Cols,_Options,_MaxRows,_MaxCols> & m, const unsigned int version) {
            split_free(ar,m,version);
        }


    template <class Archive, typename _Scalar>
        void save(Archive & ar, const Eigen::Triplet<_Scalar> & m, const unsigned int version) {
            ar & m.row();
            ar & m.col();
            ar & m.value();
        }
    template <class Archive, typename _Scalar>
        void load(Archive & ar, Eigen::Triplet<_Scalar> & m, const unsigned int version) {
            int row,col;
            _Scalar value;
            ar & row;
            ar & col;
            ar & value;
            m = Eigen::Triplet<_Scalar>(row,col,value);
        }

    template <class Archive, typename _Scalar>
        void serialize(Archive & ar, Eigen::Triplet<_Scalar> & m, const unsigned int version) {
            split_free(ar,m,version);
        }


    template <class Archive, typename _Scalar, int _Options,typename _Index>
        void save(Archive & ar, const Eigen::SparseMatrix<_Scalar,_Options,_Index> & m, const unsigned int version) {
            int innerSize=m.innerSize();
            int outerSize=m.outerSize();
            typedef typename Eigen::Triplet<_Scalar> Triplet;
            std::vector<Triplet> triplets;
            for(int i=0; i < outerSize; ++i) {
                for(typename Eigen::SparseMatrix<_Scalar,_Options,_Index>::InnerIterator it(m,i); it; ++it) {
                triplets.push_back(Triplet(it.row(), it.col(), it.value()));
                }
            }
            ar & innerSize;
            ar & outerSize;
            ar & triplets;
        }
    template <class Archive, typename _Scalar, int _Options, typename _Index>
        void load(Archive & ar, Eigen::SparseMatrix<_Scalar,_Options,_Index>  & m, const unsigned int version) {
            int innerSize;
            int outerSize;
            ar & innerSize;
            ar & outerSize;
            int rows = m.IsRowMajor?outerSize:innerSize;
            int cols = m.IsRowMajor?innerSize:outerSize;
            m.resize(rows,cols);
            typedef typename Eigen::Triplet<_Scalar> Triplet;
            std::vector<Triplet> triplets;
            ar & triplets;
            m.setFromTriplets(triplets.begin(), triplets.end());

        }
    template <class Archive, typename _Scalar, int _Options, typename _Index>
        void serialize(Archive & ar, Eigen::SparseMatrix<_Scalar,_Options,_Index> & m, const unsigned int version) {
            split_free(ar,m,version);
        }

}}
#endif
