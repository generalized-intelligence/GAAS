#ifndef SERIALIZATION_H
#define SERIALIZATION_H



#include <boost/serialization/split_member.hpp>
#include <boost/serialization/serialization.hpp>

#include <boost/serialization/map.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/shared_ptr.hpp>
#include <boost/serialization/weak_ptr.hpp>

#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>

//boost serialiazaion support for cv::Mat and cv::KeyPoint
#include "Cv_mat.h"

//boost serialization support for eigen
#include "eigen_boost_serialization.h"




#endif
