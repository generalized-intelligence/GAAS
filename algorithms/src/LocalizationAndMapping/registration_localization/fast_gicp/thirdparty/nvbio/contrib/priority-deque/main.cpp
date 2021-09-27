//#define NDEBUG 1

#ifndef __cplusplus
#error A C++ compiler is required.
#endif

#include <vector>
#include <queue>
#include <iostream>
#include <stdint.h>
#include <time.h>
#include <algorithm>

#include "priority_deque.hpp"
#include "priority_deque_verify.hpp"

int main();

//! @todo More tests!

#define BENCHMARK
#define VERIFY_HEAP

template <typename pq_t>
void benchmark_priority_queue (unsigned benchmark_elements) {
  pq_t pq;
  clock_t bench_begin, bench_mid, bench_end;
  std::cout << "Benchmark results: ";
//  First, fill the queue with most of the elements, to get a better idea of the average case.
  for (unsigned n = benchmark_elements / 2; n--;)
    pq.push(rand());
//  Test how long it takes to fill the rest of the way.
  bench_begin = clock();
  for (unsigned n = benchmark_elements / 2; n--;)
    pq.push(rand());
  bench_mid = clock();
  std::cout << "Push: " << (bench_mid - bench_begin) << " (" << static_cast<double>(bench_mid - bench_begin) / CLOCKS_PER_SEC << "s)";

//  Test time required to remove the same number of elements.
  bench_mid = clock();
  for (unsigned n = benchmark_elements / 2; n--;)
    pq.pop();
  bench_end = clock();
  std::cout << ", Pop: " << (bench_end - bench_mid) << " (" << static_cast<double>(bench_end - bench_mid) / CLOCKS_PER_SEC << "s)\n";
}

int main() {
  std::cout << "__cplusplus = " << __cplusplus << "\n";
#ifndef NDEBUG
	std::cout << "Debug mode (asserts active).\n";
#endif
  srand(clock());

  using boost::container::priority_deque;
  using boost::heap::is_interval_heap;
  using std::priority_queue;

//  Begin tests
{
  priority_deque<int> test_deque;
//  Sufficient sample size for observing all potential interesting behaviors.
  for (int i = 0; i < 256; ++i) {
    try {
      test_deque.push(i);
      if (std::find(test_deque.begin(), test_deque.end(), i) == test_deque.end()) {
        std::cerr << "Element lost <push>.\n";
        break;
      }
      if (!is_interval_heap(test_deque.begin(), test_deque.end(), std::less<int>())) {
        std::cerr << "Heap corrupted <push>.\n";
        break;
      }
      if (test_deque.maximum() != i)
        std::cerr << "Incorrect maximum. i = " << i << "\n";
    } catch (...) {
      std::cerr << "Exception thrown <push>.\n";
      break;
    }
  }
  std::vector<int> test_vector;
  for (int i = 0; i < 256; ++i) {
    test_vector.push_back(i);
    std::random_shuffle(test_vector.begin(), test_vector.end());
    try {
      test_deque.clear();
      test_deque.merge(test_vector);
      if (test_deque.size() != test_vector.size())
        std::cerr << "Merge changed number of elements.\n";
      for (int j = 0; j <= i; ++j) {
        if (std::find(test_deque.begin(), test_deque.end(), j) == test_deque.end()) {
          std::cerr << "Element removed <merge>.\n";
          break;
        }
      }
      if (!is_interval_heap(test_deque.begin(), test_deque.end(), std::less<int>())) {
        std::cerr << "Heap corrupted <merge>.\n";
        break;
      }
    } catch (...) {
      std::cerr << "Exception thrown <merge>.\n";
      break;
    }
  }
}
//  Benchmark performance relative to std::priority_queue.
#ifdef BENCHMARK
{
  typedef intmax_t benchmark_type;
  const unsigned benchmark_elements = 20000000;
  std::cout << "PD: ";
  benchmark_priority_queue<priority_deque<benchmark_type> >(benchmark_elements);
  std::cout << "PQ: ";
  benchmark_priority_queue<priority_queue<benchmark_type> >(benchmark_elements);
}
#endif

#ifdef VERIFY_HEAP
  std::cout << "\n\nTesting heap integrity after:\n";
  const int element_total = 3001;//8192 * 2000;
	priority_deque<uint32_t> pd;

  std::cout << "'push' (" << element_total << "x)\n";
	for (int n = 1; n <= element_total; ++n) {
		pd.push(rand());
		if (is_valid_until(pd) != pd.end()) {
		  std::cout << "Failed push (error in replace_leaf)\n";
      break;
		}
	}

  srand(25346);
  std::vector<uint32_t> v;
  for (int n = 1; n <= 5; ++n)
		v.push_back(rand());
  for (int n = 1; n <= 1; ++n) {
    std::cout << "'merge' (" << v.size() << " elements)\n";

    /*for (auto it = v.begin(); it != v.end(); ++it)
      std::cout << *it << ", ";
    std::cout << "\n";*/
    boost::heap::make_interval_heap(v.begin(), v.end(), std::less<uint32_t>());
    if (!boost::heap::is_interval_heap(v.begin(), v.end(), std::less<uint32_t>())) {
      std::cout << "Failed\n";
      for (auto it = v.begin(); it != v.end(); ++it)
        std::cout << *it << ", ";
      std::cout << "\n";
    }
    std::random_shuffle(v.begin(), v.end());
    v.push_back(rand());
  }
  /*for (int n = 1; n < 64; ++n) {
    std::cout << "'merge' (" << v.size() << " elements)\n";
    pd.clear();
    pd.merge(v.begin(), v.end());
    if (is_valid_until(pd) != pd.end())
      std::cout << "Failed merge (error in make_valid)\n";
    v.push_back(rand());
  }*/

  std::cout << "'set' (" << element_total << "x)\n";
  for (int t = 0; t < element_total; ++t) {
    pd.update(pd.begin() + rand() % pd.size(), rand());
		if (is_valid_until(pd) != pd.end()) {
		  std::cout << "Failed random-access set (error in replace)\n";
      break;
		}
  }

  std::cout << "'pop_minimum' (" << pd.size() << "x)\n";
	while (!pd.empty()) {
		pd.pop_minimum();
		if (is_valid_until(pd) != pd.end()) {
		  std::cout << "Failed pop_minimum (error in replace)\n";
      break;
		}
	}
  pd.clear();

  std::cout << "'push', 'pop_maximum', 'pop_minimum', 'erase', 'set' (indefinitely)\n";
  boost::container::priority_deque<int> test_pd;
  for (int n = 0; n < 13; ++n) {
    test_pd.push(rand());
    if (is_valid_until(pd) != pd.end())
      std::cout << "Failed push (error in replace_leaf)\n";
  }
  int last_rand = -1;
  while (true) {
    if (test_pd.size() < 3000) {
      last_rand = -1;
      test_pd.push(rand());
    } else {
      last_rand = rand() % 8;
      switch (last_rand) {
        case 0: test_pd.pop_maximum(); break;
        case 1: test_pd.pop_minimum(); break;
        case 2: test_pd.erase(test_pd.begin() + rand() % test_pd.size()); break;
        default: test_pd.update(test_pd.begin() + rand() % test_pd.size(), rand());
      }
    }
    if (is_valid_until(pd) != pd.end()) {
      std::cout << "An error has occurred! (code " << last_rand << ").\n";
      break;
    }
  }
#endif
  return 0;
}
