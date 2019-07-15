/*********************************************************************************
 *  OKVIS - Open Keyframe-based Visual-Inertial SLAM
 *  Copyright (c) 2015, Autonomous Systems Lab / ETH Zurich
 *
 *  Copyright (c) 2011-2013, Paul Furgale and others.
 *  All rights reserved.
 *
 *  Unlike otherwise stated in source files, the code in this repository is
 *  published under the Revised BSD (New BSD) license.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *      * Redistributions of source code must retain the above copyright
 *        notice, this list of conditions and the following disclaimer.
 *      * Redistributions in binary form must reproduce the above copyright
 *        notice, this list of conditions and the following disclaimer in the
 *        documentation and/or other materials provided with the distribution.
 *      * Neither the name of the <organization> nor the
 *        names of its contributors may be used to endorse or promote products
 *        derived from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 *  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 *  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
 *  DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 *  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 *  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 *  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************************/

 /*
  * Modified: Stefan Leutenegger (s.leutenegger@imperial.ac.uk)
  * Modified: Andreas Forster (an.forster@gmail.com)
  */

#ifndef INCLUDE_OKVIS_TIMING_TIMER_HPP_
#define INCLUDE_OKVIS_TIMING_TIMER_HPP_

#ifndef BOOST_DATE_TIME_NO_LOCALE
#define BOOST_DATE_TIME_NO_LOCALE
#include <boost/date_time/posix_time/posix_time.hpp>
#undef BOOST_DATE_TIME_NO_LOCALE
#else
#include <boost/date_time/posix_time/posix_time.hpp>
#endif

#include <mutex>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics.hpp>
#include <boost/accumulators/statistics/rolling_mean.hpp>
#include <unordered_map>
#include <vector>

#include <okvis/assert_macros.hpp>

#ifdef _WIN32
#define OKVIS_USE_HIGH_PERF_TIMER
#include <windows.h>
#endif


namespace okvis {
namespace timing {
  
  OKVIS_DEFINE_EXCEPTION(TimerException, std::runtime_error)
  
  struct TimerMapValue {
    // Initialize the window size for the rolling mean.
    TimerMapValue() : m_acc(boost::accumulators::tag::rolling_window::window_size = 50){}
    boost::accumulators::accumulator_set<
      double, 
      boost::accumulators::features<
	boost::accumulators::tag::lazy_variance,
	boost::accumulators::tag::sum,
	boost::accumulators::tag::min,
	boost::accumulators::tag::max,
	boost::accumulators::tag::rolling_mean,
	boost::accumulators::tag::mean
	>
      > m_acc;
  };
  
  // A class that has the timer interface but does nothing.
  // Swapping this in in place of the Timer class (say with a 
  // typedef) should allow one to disable timing. Because all
  // of the functions are inline, they should just disappear.
  class DummyTimer {
  public:
    DummyTimer(size_t /* handle */, bool /* constructStopped */ ){}
    DummyTimer(size_t /* handle */){}
    DummyTimer(std::string const & /* tag */){}
    DummyTimer(std::string const & /* tag */, bool /* constructStopped */){}
    ~DummyTimer(){}
    
    void start(){}
    void stop(){}
    void discardTiming(){}
    bool isTiming(){ return false; }
  };
  
  class Timer {
  public:
    Timer(size_t handle, bool constructStopped = false);
    Timer(std::string const & tag, bool constructStopped = false);
    ~Timer();
    
    void start();
    void stop();
    bool isTiming();
    void discardTiming();
  private:
#ifdef OKVIS_USE_HIGH_PERF_TIMER
    LARGE_INTEGER m_time;
#else
    boost::posix_time::ptime m_time;
#endif
    bool m_timing;
    size_t m_handle;
  };
  
  class Timing{
  public:
    friend class Timer;
    // Static funcitons to query the timers:
    static  size_t getHandle(std::string const & tag);
    static  std::string getTag(size_t handle);
    static  double getTotalSeconds(size_t handle);
    static  double getTotalSeconds(std::string const & tag);
    static  double getMeanSeconds(size_t handle);
    static  double getMeanSeconds(std::string const & tag);
    static  size_t getNumSamples(size_t handle);
    static  size_t getNumSamples(std::string const & tag);
    static  double getVarianceSeconds(size_t handle);
    static  double getVarianceSeconds(std::string const & tag);
    static  double getMinSeconds(size_t handle);
    static  double getMinSeconds(std::string const & tag);
    static  double getMaxSeconds(size_t handle);
    static  double getMaxSeconds(std::string const & tag);
    static  double getHz(size_t handle);
    static  double getHz(std::string const & tag);
    static  void print(std::ostream & out);
    static  void reset(size_t handle);
    static  void reset(std::string const & tag);
    static  std::string print();
    static  std::string secondsToTimeString(double seconds);
    
  private:
    void addTime(size_t handle, double seconds);
    
    static Timing & instance();
    
    // Singleton design pattern
    Timing();
    ~Timing();
    
    typedef std::unordered_map<std::string,size_t> map_t;
    typedef std::vector<TimerMapValue> list_t;
    
    std::mutex addNewHandleMutex_;

    // Static members
    list_t m_timers;
    map_t m_tagMap;
#ifdef OKVIS_USE_HIGH_PERF_TIMER
    double m_clockPeriod;
#endif
    size_t m_maxTagLength;
    
  }; // end class timer
  
#ifdef NDEBUG
  typedef DummyTimer DebugTimer;
#else
  typedef Timer DebugTimer;
#endif
  
} // namespace timing
} // end namespace sm

#endif // INCLUDE_OKVIS_TIMING_TIMER_HPP_
