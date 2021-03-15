#ifndef CPU_NDT_H_
#define CPU_NDT_H_

#include "Registration.h"
#include "VoxelGrid.h"
#include <eigen3/Eigen/Geometry>

namespace cpu
{

template <typename PointSourceType, typename PointTargetType>
class NormalDistributionsTransform : public Registration<PointSourceType, PointTargetType>
{
public:
	NormalDistributionsTransform();

	// 拷贝构造
	NormalDistributionsTransform(const NormalDistributionsTransform &other);

	void setStepSize(double step_size);

	// 设置 ndt 中 voxel 的大小
	void setResolution(float resolution);

	// 设置离群点的比例，用于计算混合分布中均值和高斯的权重
	void setOutlierRatio(double olr);

	double getStepSize() const;

	float getResolution() const;

	double getOutlierRatio() const;

	// TODO: 如何度量的？实际意义是什么？
	double getTransformationProbability() const;

	int getRealIterations();

	/* Set the input map points */
	void setInputTarget(typename pcl::PointCloud<PointTargetType>::Ptr input);

	// Euclidean fitness score，output 点云与 target 中最近点距离的平方和。感觉作用不是很大，在环境变化明显的时候，即使位姿比较准确，这个值应该也会挺大的。而且有一定计算量，最好不好经常调用
	/* Compute and get fitness score */
	double getFitnessScore(double max_range = DBL_MAX);

	// 调用 voxel_grid_ 的 update 进行更新
	void updateVoxelGrid(typename pcl::PointCloud<PointTargetType>::Ptr new_cloud);

protected:
	// 给定初始位姿估计，牛顿迭代计算位姿
	void computeTransformation(const Eigen::Matrix<float, 4, 4> &guess);

	using Registration<PointSourceType, PointTargetType>::transformation_epsilon_;
	using Registration<PointSourceType, PointTargetType>::max_iterations_;
	using Registration<PointSourceType, PointTargetType>::source_cloud_;
	using Registration<PointSourceType, PointTargetType>::trans_cloud_;
	using Registration<PointSourceType, PointTargetType>::converged_;
	using Registration<PointSourceType, PointTargetType>::nr_iterations_;
	using Registration<PointSourceType, PointTargetType>::final_transformation_;
	using Registration<PointSourceType, PointTargetType>::transformation_;
	using Registration<PointSourceType, PointTargetType>::previous_transformation_;
	using Registration<PointSourceType, PointTargetType>::target_cloud_updated_;
	using Registration<PointSourceType, PointTargetType>::target_cloud_;

private:
	//Copied from ndt.h
	double auxilaryFunction_PsiMT(double a, double f_a, double f_0, double g_0, double mu = 1.e-4);

	//Copied from ndt.h
	double auxilaryFunction_dPsiMT(double g_a, double g_0, double mu = 1.e-4);

	double updateIntervalMT(double &a_l, double &f_l, double &g_l,
													double &a_u, double &f_u, double &g_u,
													double a_t, double f_t, double g_t);

	double trialValueSelectionMT(double a_l, double f_l, double g_l,
															 double a_u, double f_u, double g_u,
															 double a_t, double f_t, double g_t);

	void computeAngleDerivatives(Eigen::Matrix<double, 6, 1> pose, bool compute_hessian = true);

	double computeStepLengthMT(const Eigen::Matrix<double, 6, 1> &x, Eigen::Matrix<double, 6, 1> &step_dir,
														 double step_init, double step_max, double step_min, double &score,
														 Eigen::Matrix<double, 6, 1> &score_gradient, Eigen::Matrix<double, 6, 6> &hessian,
														 typename pcl::PointCloud<PointSourceType> &trans_cloud);

	void computeHessian(Eigen::Matrix<double, 6, 6> &hessian, typename pcl::PointCloud<PointSourceType> &trans_cloud, Eigen::Matrix<double, 6, 1> &p);

	double computeDerivatives(Eigen::Matrix<double, 6, 1> &score_gradient, Eigen::Matrix<double, 6, 6> &hessian,
														typename pcl::PointCloud<PointSourceType> &trans_cloud,
														Eigen::Matrix<double, 6, 1> pose, bool compute_hessian = true);
	void computePointDerivatives(Eigen::Vector3d &x, Eigen::Matrix<double, 3, 6> &point_gradient, Eigen::Matrix<double, 18, 6> &point_hessian, bool computeHessian = true);
	double updateDerivatives(Eigen::Matrix<double, 6, 1> &score_gradient, Eigen::Matrix<double, 6, 6> &hessian,
													 Eigen::Matrix<double, 3, 6> point_gradient, Eigen::Matrix<double, 18, 6> point_hessian,
													 Eigen::Vector3d &x_trans, Eigen::Matrix3d &c_inv, bool compute_hessian = true);
	void updateHessian(Eigen::Matrix<double, 6, 6> &hessian,
										 Eigen::Matrix<double, 3, 6> point_gradient, Eigen::Matrix<double, 18, 6> point_hessian,
										 Eigen::Vector3d &x_trans, Eigen::Matrix3d &c_inv);

	double gauss_d1_, gauss_d2_;
	double outlier_ratio_;
	Eigen::Vector3d j_ang_a_, j_ang_b_, j_ang_c_, j_ang_d_, j_ang_e_, j_ang_f_, j_ang_g_, j_ang_h_;

	Eigen::Vector3d h_ang_a2_, h_ang_a3_,
			h_ang_b2_, h_ang_b3_,
			h_ang_c2_, h_ang_c3_,
			h_ang_d1_, h_ang_d2_, h_ang_d3_,
			h_ang_e1_, h_ang_e2_, h_ang_e3_,
			h_ang_f1_, h_ang_f2_, h_ang_f3_;

	// [x,y,z,roll,pitch,yaw] 的最小变化量(m, rad)，当小于这个值时就停止 align
        //double transformation_epsilon;
	// More-Thuente line search 的最大步长，大一些可以更快的下降，但也可能 overshoot 导致陷入局部最优
	double step_size_;
	// ndt 中 voxel 的大小，每个 voxel 中会保存 mean，covariance 和 点云，这个值是最 scale dependent 的，应该足够大（一个 voxel 至少包含 6 个点），也不能太大（要反映局部空间的特征）
	float resolution_;
	// 还不知道怎么度量的
	double trans_probability_;

	int real_iterations_;

	VoxelGrid<PointSourceType> voxel_grid_;
};
} // namespace cpu

#endif
