#include <vector>
#include <iostream>
#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <fast/fast.h>

int main (int argc, char * argv[]) {
   const int n_trials = 1000;
   std::vector<fast::fast_xy> corners;
   cv::Mat img = cv::imread(std::string(TEST_DATA_DIR) + "/test1.png", 0);
   cv::Mat downSampled; 
   cv::resize(img, downSampled, cv::Size(752, 480));
   img = downSampled;

   printf("\nTesting PLAIN version\n");
   double time_accumulator = 0;
   for (int i = 0; i < n_trials; ++i) {
      corners.clear();
      double t = (double)cv::getTickCount();
      fast::fast_corner_detect_10((fast::fast_byte *)(img.data), img.cols, img.rows, img.cols, 75, corners);
      time_accumulator +=  ((cv::getTickCount() - t) / cv::getTickFrequency());
   }
   printf("PLAIN took %f ms (average over %d trials).\n", time_accumulator/((double)n_trials)*1000.0, n_trials );
   printf("PLAIN version extracted %zu features.\n", corners.size());

#if __NEON__
   printf("\nTesting NEON version\n");
   time_accumulator = 0;
   for (int i = 0; i < n_trials; ++i) {
     corners.clear();
      double t = (double)cv::getTickCount();
      fast::fast_corner_detect_9_neon((fast::fast_byte *)(img.data), img.cols, img.rows, img.cols, 75, corners);
      time_accumulator +=  ((cv::getTickCount() - t) / cv::getTickFrequency());
   }
   printf("NEON version took %f ms (average over %d trials).\n", time_accumulator/((double)n_trials)*1000.0, n_trials);
   printf("NEON version extracted %zu features.\n", corners.size());
#endif
   
#if __SSE2__
   printf("\nTesting SSE2 version\n");
   time_accumulator = 0;
   for (int i = 0; i < n_trials; ++i) {
     corners.clear();
      double t = (double)cv::getTickCount();
      fast::fast_corner_detect_10_sse2((fast::fast_byte *)(img.data), img.cols, img.rows, img.cols, 75, corners);
      time_accumulator +=  ((cv::getTickCount() - t) / cv::getTickFrequency());
   }
   printf("SSE2 version took %f ms (average over %d trials).\n", time_accumulator/((double)n_trials)*1000.0, n_trials);
   printf("SSE2 version extracted %zu features.\n", corners.size());
#endif

   printf("\nBENCHMARK version extracted 167 features.\n");  
   return 0;
}
