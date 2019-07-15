#include <okvis/DenseMatcher.hpp>
#include <math.h>
#include <gtest/gtest.h>

class TestMatchingAlgorithm : public okvis::MatchingAlgorithm {
 public:
  TestMatchingAlgorithm() {
  }
  virtual ~TestMatchingAlgorithm() {
  }

  /// \brief this will be called exactly once for each call to DenseMatcher::match()
  virtual void doSetup() {
  }

  /// \brief what is the size of list A?
  virtual size_t sizeA() const {
    return listA.size();
  }
  /// \brief what is the size of list B?
  virtual size_t sizeB() const {
    return listB.size();
  }

  /// distances above this threshold will not be returned as matches.
  virtual float distanceThreshold() const {
    return 4.0f;
  }

  /// by which factor does the first best match has to be better than the second best one.
  virtual float distanceRatioThreshold() const {
    return 3.0f;
  }

  /// \brief Should we skip the item in list A? This will be called once for each item in the list
  virtual bool skipA(size_t indexA) const {
    return indexA == 0;
  }

  /// \brief Should we skip the item in list B? This will be called many times.
  virtual bool skipB(size_t /* indexB */) const {
    return false;
  }

  /// \brief the "distance" between the two points.
  ///        For points that absolutely don't match. Please use float max.
  virtual float distance(size_t indexA, size_t indexB) const {
    double diff = listA[indexA] - listB[indexB];
    return fabs(diff);
  }

  /// \brief a function that tells you how many times setMatching() will be called.
  virtual void reserveMatches(size_t numMatches) {
    matches.clear();
    matches.reserve(numMatches);
  }

  /// \brief At the end of the matching step, this function is called once
  ///        for each pair of matches discovered.
  virtual void setBestMatch(size_t indexA, size_t indexB, double /* distance */) {
    matches.push_back(std::make_pair(indexA, indexB));
  }

  std::vector<double> listA;
  std::vector<double> listB;
  std::vector<std::pair<int, int> > matches;
};

TEST(DenseMatcherTestSuite, denseMatcherTest)
{
  TestMatchingAlgorithm tma;

  tma.listA.push_back(1.0);
  tma.listA.push_back(3.0);
  tma.listA.push_back(2.0);
  tma.listA.push_back(0.9);

  /// This shouldn't be matched because 18 - 4 > 4.0
  tma.listB.push_back(18.0);
  tma.listB.push_back(2.1);
  tma.listB.push_back(4.0);
  // This shouldn't be matched with listA[0] because of skipping listA[0]
  // So, it will be matches with listA[3]
  tma.listB.push_back(1.0);

  // We should have 1 --> 2 and 2 --> 1

  okvis::DenseMatcher matcher;

  matcher.match(tma);

  ASSERT_EQ(3u, tma.matches.size());

  for(size_t i = 0; i < tma.matches.size(); ++i)
  {
    switch(tma.matches[i].first)
    {
      case 1:
      ASSERT_EQ(2, tma.matches[i].second);
      break;
      case 2:
      ASSERT_EQ(1, tma.matches[i].second);
      break;
      case 3:
      ASSERT_EQ(3, tma.matches[i].second);
      break;
      default:
      FAIL() << "Unexpected match " << tma.matches[i].first << " --> " << tma.matches[i].second;
    }
  }
}

TEST(DenseMatcherTestSuite, denseMatcherDistanceRatioTest)
{
  TestMatchingAlgorithm tma;

  tma.listA.push_back(8.0);
  tma.listA.push_back(1.0);
  tma.listA.push_back(3.0);
  tma.listA.push_back(2.0);
  tma.listA.push_back(0.9);

  /// This shouldn't be matched because 16/15 < 4
  tma.listB.push_back(18.0);
  tma.listB.push_back(2.1);
  /// This shouldn't be matched because 2/1 < 4
  tma.listB.push_back(4.0);
  tma.listB.push_back(1.0);

  // This shouldn't be matched with listA[0] because of skipping listA[0]
  tma.listB.push_back(7.0);

  // We should have 1 --> 2 and 2 --> 1
  bool useDistanceRatioThreshold = true;
  okvis::DenseMatcher matcher(4, 4, useDistanceRatioThreshold);

  matcher.match(tma);

  ASSERT_EQ(2u, tma.matches.size());

  for(size_t i = 0; i < tma.matches.size(); ++i)
  {
    switch(tma.matches[i].first)
    {
      case 1:
      ASSERT_EQ(3, tma.matches[i].second);
      break;
      case 3:
      ASSERT_EQ(1, tma.matches[i].second);
      break;
      default:
      FAIL() << "Unexpected match " << tma.matches[i].first << " --> " << tma.matches[i].second;
    }
  }
}
