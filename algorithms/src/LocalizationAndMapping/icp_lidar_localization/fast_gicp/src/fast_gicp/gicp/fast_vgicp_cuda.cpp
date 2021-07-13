#include <fast_gicp/gicp/fast_vgicp_cuda.hpp>
#include <fast_gicp/gicp/impl/fast_vgicp_cuda_impl.hpp>

template class fast_gicp::FastVGICPCuda<pcl::PointXYZ, pcl::PointXYZ>;
template class fast_gicp::FastVGICPCuda<pcl::PointXYZI, pcl::PointXYZI>;
