#ifndef OCTREE_H_
#define OCTREE_H_

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <float.h>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>

/* The octree is built on top of a voxel grid to fasten the nearest neighbor search */
namespace cpu {
template <typename PointSourceType>
class Octree {
public:

	Octree();

	/* Input is a vector of boundaries and ptsum of the voxel grid
	 * Those boundaries is needed since the number of actually occupied voxels may be
	 * much smaller than reserved number of voxels */

	/**
	 * @brief Set the Input object 虽然 Octree 是在 VoxelGrid 的基础上构造的，但还是需要 point_cloud 以计算每个 OctreeNode 的点云 bounding box
	 * 一个 OctreeNode 包含 leaf_size 个 voxel（会向上/向下取整），自底向上进行构造 Octree
	 * @param occupied_voxels point_cloud 中每个点对应的 voxel 坐标
	 * @param point_cloud 
	 */
	void setInput(std::vector<Eigen::Vector3i> occupied_voxels, typename pcl::PointCloud<PointSourceType>::Ptr point_cloud);

	/**
	 * @brief 使用新增的 new_voxels 更新 Octree
	 * 先 updateBoundaries，扩充边界，修改数组大小和维度，从 old_tree 自底向上往 new_tree 拷贝。updateOctreeContent 再根据 new_voxels 对 new_tree 进行更新
	 * @param new_voxels 
	 * @param new_cloud 
	 */
	void update(std::vector<Eigen::Vector3i> new_voxels, typename pcl::PointCloud<PointSourceType>::Ptr new_cloud);

	/**
	 * @brief 返回距离查询点 q 最近（距离 centroid）的 OctreeNode 的 bounding box，查询过程与 KDTree 十分类似
	 * 1. 先从根节点到叶节点找到距离 q 最近的 OctreeNode
	 * 2. 自底向上到根节点（goUp）检查在其兄弟节点中是否存在更近的 OctreeNode，对每个兄弟节点向下（goDown）遍历至叶节点找更近的 OctreeNode，会计算 q 与 OctreeNode 的 bounding box 的距离，如果更大则剪枝，如果更小则向下到叶节点
	 * @param q 查询点
	 * @return Eigen::Matrix<float, 6, 1> 距离查询点 q 最近的 OctreeNode 的 bounding box
	 */
	Eigen::Matrix<float, 6, 1> nearestOctreeNode(PointSourceType q);

private:
	// Octree 的一个节点
	typedef struct {
    // OctreeNode 包含的点云的 bounding box
		float lx, ux;
		float ly, uy;
		float lz, uz;
    // OctreeNode 包含的点云的中心位置
		Eigen::Vector3d centroid;
    // OctreeNode 包含的点数
		int point_num;
	} OctreeNode;

  // Octree 每层的边界，用于计算每一层节点的 index
	typedef struct {
		int lower_x, upper_x;
		int lower_y, upper_y;
		int lower_z, upper_z;
	} OctreeLevelBoundaries;

  // Octree 每层在 x,y,z 维度上数量，与 OctreeLevelBoundaries 配合计算每层 OctreeNode 的 index
	typedef struct {
		int x, y, z;
	} OctreeLevelDim;

	// 根据 OctreeNode 的三维 index 计算实际存储在 vector 中的 index
	// Convert from 3d indexes and level of the tree node to the actual index in the array
	int index2id(int idx, int idy, int idz, int level);
	int index2id(int idx, int idy, int idz, OctreeLevelBoundaries bounds, OctreeLevelDim dims);

	// Convert from the index in the array to the 3d indexes of the tree node
	Eigen::Vector3i id2index(int id, int level);
	Eigen::Vector3i id2index(int id, OctreeLevelBoundaries bounds, OctreeLevelDim dims);

	// 根据下一层的子 OctreeNode 构造上一层的父 OctreeNode
	void buildLevel(int level);

	// 给定 node_id 和所在层（最底层为 0），根据 (*occupancy_check)[level] 判断是否被 occupied，即是否包含点云
	bool isOccupied(int node_id, int level);

	bool isOccupied(std::vector<unsigned int> occupancy, int node_id);

	void setOccupied(int node_id, int level);

	void setOccupied(std::vector<unsigned int> &occupancy, int node_id);

	// 根据 new_voxels 更新每层的 boundaries
	void updateBoundaries(std::vector<Eigen::Vector3i> new_voxels);

	// 以 factor 为单位向上/向下取整
	int roundUp(int input, int factor);
	int roundDown(int input, int factor);

	int div(int input, int divisor);

	// 根据新点云自底向上更新 octree
	void updateOctreeContent(std::vector<Eigen::Vector3i> new_voxels, typename pcl::PointCloud<PointSourceType>::Ptr new_cloud);

	// 计算点 q 距离 node 的 bounding box 的距离
	double dist(OctreeNode node, PointSourceType q);

	/* Three functions to search for the nearest neighbor of a point */
	// 参数名起的太烂了，跟 voxel 没什么关系，就是找距离 q 最近的 OctreeNode
	void initRange(PointSourceType q, double &min_range, int &current_nn_voxel);

	void goUp(Eigen::Matrix<int, 4, 1 > tree_node, PointSourceType q, double &min_range, int &current_nn_voxel);

	void goDown(Eigen::Matrix<int, 4, 1> tree_node, PointSourceType q, double &min_range, int &current_nn_voxel);

	// 实际的 Octree，每层 OctreeNode 存储在一个 vector 中，最终形成了一个二维数组
	boost::shared_ptr<std::vector<std::vector<OctreeNode> > > octree_;
	// 保存每一层 OctreeNode 的边界，以便 3d index 与 1d index 的转换
	boost::shared_ptr<std::vector<OctreeLevelBoundaries> > reserved_size_;
	// 保存每一层 OctreeNode 的维度，以便 3d index 与 1d index 的转换
	boost::shared_ptr<std::vector<OctreeLevelDim> > dimension_;

	/* Used for checking if an octree node is occupied or not
	 * If an octree node is occupied (containing some points),
	 * then the corresponding bit is set
	 */
	/**
	 * @brief 用于检查一个 OctreeNode 是否被 occupied，可以用于构造树阶段 OctreeNode 的初始化，也可以用来查询阶段的提前剪枝
	 * 每一层 OctreeNode 的 occupied 情况被保存在一个 vector 中，每个 OctreeNode 的 occupied 情况用 1bit 保存。相较于直接使用 bool 占据的空间更小。
	 */
	boost::shared_ptr<std::vector<std::vector<unsigned int> > > occupancy_check_;

	// 每个维度保存的 voxel 数量（代码里的“维度”有时候指的是 x,y,z，有时候指的是 x,y,z 方向上的 size，注意区分）
	int leaf_x_, leaf_y_, leaf_z_;		// Number of voxels contained in each leaf

	// 取整用的，不太明白这么做的好处，是为了避免频繁更新吗？
	static const int MAX_BX_ = 8;
	static const int MAX_BY_ = 8;
	static const int MAX_BZ_ = 4;
};
}

#endif
