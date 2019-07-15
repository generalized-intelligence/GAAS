#include <gtest/gtest.h>
#include <okvis/timing/NsecTimeUtilities.hpp>

TEST( NsetTimeTestSuite, testChronoConversion ) {

  std::chrono::system_clock::time_point tp1 = std::chrono::system_clock::now();
  okvis::timing::NsecTime ns1 = okvis::timing::chronoToNsec( tp1 );
  std::chrono::system_clock::time_point tp2 = okvis::timing::nsecToChrono( ns1 );
  ASSERT_TRUE(tp1 == tp2);
  
}


TEST( NsetTimeTestSuite, testSecConversion ) {

  okvis::timing::NsecTime ns1 = okvis::timing::nsecNow();
  double s2 = okvis::timing::nsecToSec(ns1);
  okvis::timing::NsecTime ns2 = okvis::timing::secToNsec(s2);
  
  ASSERT_LT(abs(ns1-ns2), 1000000);
  
}
