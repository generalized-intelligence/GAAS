/*********************************************************************************
 *  OKVIS - Open Keyframe-based Visual-Inertial SLAM
 *  Copyright (c) 2015, Autonomous Systems Lab / ETH Zurich
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 * 
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *   * Neither the name of Autonomous Systems Lab / ETH Zurich nor the names of
 *     its contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *  Created on: Sep 8, 2013
 *      Author: Stefan Leutenegger (s.leutenegger@imperial.ac.uk)
 *********************************************************************************/

/**
 * @file Map.hpp
 * @brief Header file for the Map class. This essentially encapsulates the ceres::Problem.
 * @author Stefan Leutenegger
 */

#ifndef INCLUDE_OKVIS_CERES_MAP_HPP_
#define INCLUDE_OKVIS_CERES_MAP_HPP_

#include <memory>

#include <ceres/ceres.h>
#include <okvis/ceres/ParameterBlock.hpp>
#include <okvis/ceres/HomogeneousPointLocalParameterization.hpp>
#include <okvis/ceres/PoseLocalParameterization.hpp>
#include <okvis/Time.hpp>
#include <okvis/FrameTypedefs.hpp>
#include <okvis/assert_macros.hpp>
#include <unordered_map>
#include <okvis/ceres/ErrorInterface.hpp>

/// \brief okvis Main namespace of this package.
namespace okvis {
/// \brief ceres Namespace for ceres-related functionality implemented in okvis.
namespace ceres {

/// @brief The Map class. This keeps track of how parameter blocks are connected to residual blocks.
///        In essence, it encapsulates the ceres::Problem. This way, we can easily manipulate the optimisation
///        problem. You could argue why not use cere's internal mechanisms to do that. We found that our
///        implementation was faster...
class Map {
 public:
  OKVIS_DEFINE_EXCEPTION(Exception,std::runtime_error)

  /// @brief Constructor.
  Map();

  // definitions
  /// @brief Struct to store some infos about a residual.
  struct ResidualBlockSpec {
    ResidualBlockSpec()
        : residualBlockId(0),
          lossFunctionPtr(0),
          errorInterfacePtr(std::shared_ptr<ErrorInterface>()) {
    }

    /// @brief Constructor
    /// @param[in] residualBlockId ID of residual block.
    /// @param[in] lossFunctionPtr The m-estimator.
    /// @param[in] errorInterfacePtr The pointer to the error interface of the respective residual block.
    ResidualBlockSpec(::ceres::ResidualBlockId residualBlockId,
                      ::ceres::LossFunction* lossFunctionPtr,
                      std::shared_ptr<ErrorInterface> errorInterfacePtr)
        : residualBlockId(residualBlockId),
          lossFunctionPtr(lossFunctionPtr),
          errorInterfacePtr(errorInterfacePtr) {
    }

    ::ceres::ResidualBlockId residualBlockId;           ///< ID of residual block.
    ::ceres::LossFunction* lossFunctionPtr;             ///< The m-estimator.
    std::shared_ptr<ErrorInterface> errorInterfacePtr;  ///< The pointer to the error interface of the respective residual block.
  };
  typedef std::pair<uint64_t, std::shared_ptr<okvis::ceres::ParameterBlock> > ParameterBlockSpec;

  typedef std::vector<ResidualBlockSpec> ResidualBlockCollection;
  typedef std::vector<ParameterBlockSpec> ParameterBlockCollection;

  /// @brief The Parameterisation enum
  enum Parameterization {
    HomogeneousPoint,     ///< Use okvis::ceres::HomogeneousPointLocalParameterization.
    Pose6d,               ///< Use okvis::ceres::PoseLocalParameterization.
    Pose3d,               ///< Use okvis::ceres::PoseLocalParameterization3d (orientation varying).
    Pose4d,               ///< Use okvis::ceres::PoseLocalParameterization4d (position and yaw varying).
    Pose2d,               ///< Use okvis::ceres::PoseLocalParameterization2d (roll/pitch varying).
    Trivial               ///< No local parameterisation.
  };

  /**
   * @brief Check whether a certain parameter block is part of the map.
   * @param parameterBlockId ID of parameter block to find.
   * @return True if parameter block is part of map.
   */
  bool parameterBlockExists(uint64_t parameterBlockId) const;

  /// @name Print info
  /// @{

  /// @brief Log information on a parameter block.
  void printParameterBlockInfo(uint64_t parameterBlockId) const;

  /// @brief Log information on a residual block.
  void printResidualBlockInfo(::ceres::ResidualBlockId residualBlockId) const;

  /// @}

  // for quality assessment
  /**
   * @brief Obtain the Hessian block for a specific parameter block.
   * @param[in] parameterBlockId Parameter block ID of interest.
   * @param[out] H the output Hessian block.
   */
  void getLhs(uint64_t parameterBlockId, Eigen::MatrixXd& H);

  /// @name add/remove
  /// @{

  /**
   * @brief Add a parameter block to the map.
   * @param parameterBlock    Parameter block to insert.
   * @param parameterization  okvis::ceres::Parameterization to tell how to do the local parameterisation.
   * @param group             Schur elimination group -- currently unused.
   * @return True if successful.
   */
  bool addParameterBlock(
      std::shared_ptr<okvis::ceres::ParameterBlock> parameterBlock,
      int parameterization = Parameterization::Trivial, const int group = -1);

  /**
   * @brief Remove a parameter block from the map.
   * @param parameterBlockId ID of block to remove.
   * @return True if successful.
   */
  bool removeParameterBlock(uint64_t parameterBlockId);

  /**
   * @brief Remove a parameter block from the map.
   * @param parameterBlock Pointer to the block to remove.
   * @return True if successful.
   */
  bool removeParameterBlock(
      std::shared_ptr<okvis::ceres::ParameterBlock> parameterBlock);

  /**
   * @brief Adds a residual block.
   * @param[in] cost_function The error term to be used.
   * @param[in] loss_function Use an m-estimator? NULL, if not needed.
   * @param[in] parameterBlockPtrs A vector that contains all the parameter blocks the error term relates to.
   * @return
   */
  ::ceres::ResidualBlockId addResidualBlock(
      std::shared_ptr< ::ceres::CostFunction> cost_function,
      ::ceres::LossFunction* loss_function,
      std::vector<std::shared_ptr<okvis::ceres::ParameterBlock> >& parameterBlockPtrs);

  /**
   * @brief Replace the parameters connected to a residual block ID.
   * @param[in] residualBlockId The ID of the residual block the parameter blocks of which are to be to be replaced.
   * @param[in] parameterBlockPtrs A vector containing the parameter blocks to be replaced.
   */
  void resetResidualBlock(
      ::ceres::ResidualBlockId residualBlockId,
      std::vector<std::shared_ptr<okvis::ceres::ParameterBlock> >& parameterBlockPtrs);

  /**
   * @brief Add a residual block. See respective ceres docu. If more are needed, see other interface.
   * @param[in] cost_function The error term to be used.
   * @param[in] loss_function Use an m-estimator? NULL, if not needed.
   * @param[in] x0 The first parameter block.
   * @param[in] x1 The second parameter block (if existent).
   * @param[in] x2 The third parameter block (if existent).
   * @param[in] x3 The 4th parameter block (if existent).
   * @param[in] x4 The 5th parameter block (if existent).
   * @param[in] x5 The 6th parameter block (if existent).
   * @param[in] x6 The 7th parameter block (if existent).
   * @param[in] x7 The 8th parameter block (if existent).
   * @param[in] x8 The 9th parameter block (if existent).
   * @param[in] x9 The 10th parameter block (if existent).
   * @return The residual block ID, i.e. what cost_function points to.
   */
  ::ceres::ResidualBlockId addResidualBlock(
      std::shared_ptr< ::ceres::CostFunction> cost_function,
      ::ceres::LossFunction* loss_function,
      std::shared_ptr<okvis::ceres::ParameterBlock> x0,
      std::shared_ptr<okvis::ceres::ParameterBlock> x1 = std::shared_ptr<
          okvis::ceres::ParameterBlock>(),
      std::shared_ptr<okvis::ceres::ParameterBlock> x2 = std::shared_ptr<
          okvis::ceres::ParameterBlock>(),
      std::shared_ptr<okvis::ceres::ParameterBlock> x3 = std::shared_ptr<
          okvis::ceres::ParameterBlock>(),
      std::shared_ptr<okvis::ceres::ParameterBlock> x4 = std::shared_ptr<
          okvis::ceres::ParameterBlock>(),
      std::shared_ptr<okvis::ceres::ParameterBlock> x5 = std::shared_ptr<
          okvis::ceres::ParameterBlock>(),
      std::shared_ptr<okvis::ceres::ParameterBlock> x6 = std::shared_ptr<
          okvis::ceres::ParameterBlock>(),
      std::shared_ptr<okvis::ceres::ParameterBlock> x7 = std::shared_ptr<
          okvis::ceres::ParameterBlock>(),
      std::shared_ptr<okvis::ceres::ParameterBlock> x8 = std::shared_ptr<
          okvis::ceres::ParameterBlock>(),
      std::shared_ptr<okvis::ceres::ParameterBlock> x9 = std::shared_ptr<
          okvis::ceres::ParameterBlock>());

  /**
   * @brief Remove a residual block.
   * @param[in] id The residual block ID of the residual block to be removed.
   * @return True on success.
   */
  bool removeResidualBlock(::ceres::ResidualBlockId id);

  /// @}

  /// @name Set constant/variable/local parameterization
  /// @{

  /**
   * @brief Do not optimise a certain parameter block.
   * @param[in] parameterBlockId The parameter block ID of the parameter block to set fixed.
   * @return True on success.
   */
  bool setParameterBlockConstant(uint64_t parameterBlockId);

  /**
   * @brief Optimise a certain parameter block (this is the default).
   * @param[in] parameterBlockId The parameter block ID of the parameter block to set fixed.
   * @return True on success.
   */
  bool setParameterBlockVariable(uint64_t parameterBlockId);

  /**
   * @brief Do not optimise a certain parameter block.
   * @param[in] parameterBlock Pointer to the parameter block that should be constant.
   * @return True on success.
   */
  bool setParameterBlockConstant(
      std::shared_ptr<okvis::ceres::ParameterBlock> parameterBlock) {
    return setParameterBlockConstant(parameterBlock->id());
  }

  /**
   * @brief Optimise a certain parameter block (this is the default).
   * @param[in] parameterBlock Pointer to the parameter block that should be optimised.
   * @return True on success.
   */
  bool setParameterBlockVariable(
      std::shared_ptr<okvis::ceres::ParameterBlock> parameterBlock) {
    return setParameterBlockVariable(parameterBlock->id());
  }

  /**
   * @brief Reset the (local) parameterisation of a parameter block.
   * @param[in] parameterBlockId The ID of the parameter block in question.
   * @param[in] parameterization okvis::ceres::Parameterization to tell how to do the local parameterisation.
   * @return True on success.
   */
  bool resetParameterization(uint64_t parameterBlockId, int parameterization);

  /**
   * @brief Set the (local) parameterisation of a parameter block.
   * @param[in] parameterBlockId The ID of the parameter block in question.
   * @param[in] local_parameterization Give it an actual local parameterisation object.
   * @return True on success.
   */
  bool setParameterization(
      uint64_t parameterBlockId,
      ::ceres::LocalParameterization* local_parameterization);

  /**
   * @brief Set the (local) parameterisation of a parameter block.
   * @param[in] parameterBlock The pointer to the parameter block in question.
   * @param[in] local_parameterization Give it an actual local parameterisation object.
   * @return True on success.
   */
  bool setParameterization(
      std::shared_ptr<okvis::ceres::ParameterBlock> parameterBlock,
      ::ceres::LocalParameterization* local_parameterization) {
    return setParameterization(parameterBlock->id(), local_parameterization);
  }

  /// @}

  /// @name Getters
  /// @{

  /// @brief Get a shared pointer to a parameter block.
  std::shared_ptr<okvis::ceres::ParameterBlock> parameterBlockPtr(
      uint64_t parameterBlockId);  // get a vertex

  /// @brief Get a shared pointer to a parameter block.
  std::shared_ptr<const okvis::ceres::ParameterBlock> parameterBlockPtr(
      uint64_t parameterBlockId) const;  // get a vertex

  /// @brief Get a shared pointer to an error term.
  std::shared_ptr<okvis::ceres::ErrorInterface> errorInterfacePtr(
      ::ceres::ResidualBlockId residualBlockId);  // get a vertex

  /// @brief Get a shared pointer to an error term.
  std::shared_ptr<const okvis::ceres::ErrorInterface> errorInterfacePtr(
      ::ceres::ResidualBlockId residualBlockId) const;  // get a vertex

  /// @brief Get the residual blocks of a parameter block.
  /// @param[in] parameterBlockId The ID of the parameter block in question.
  /// @return Infos about all the residual blocks connected.
  ResidualBlockCollection residuals(uint64_t parameterBlockId) const;

  /// @brief Get the parameters of a residual block.
  /// @param[in] residualBlockId The ID of the residual block in question.
  /// @return Infos about all the parameter blocks connected.
  ParameterBlockCollection parameters(
      ::ceres::ResidualBlockId residualBlockId) const;  // get the parameter blocks connected

  /// @}

  // Jacobian checker
  /**
   * @brief Check a Jacobian with numeric differences.
   * @warning Checks the minimal version only.
   * @param[in] residualBlockId The ID of the residual block to be checked.
   * @param[in] relTol Relative numeric tolerance.
   * @return True if correct.
   */
  bool isJacobianCorrect(::ceres::ResidualBlockId residualBlockId,
                         double relTol = 1e-6) const;

  // access to the map as such
  /// \brief The actual map from Id to parameter block pointer.
  typedef std::unordered_map<uint64_t,
      std::shared_ptr<okvis::ceres::ParameterBlock> > Id2ParameterBlock_Map;

  /// \brief The actual map from Id to residual block specs.
  typedef std::unordered_map< ::ceres::ResidualBlockId, ResidualBlockSpec> ResidualBlockId2ResidualBlockSpec_Map;

  /// @brief Get map connecting parameter block IDs to parameter blocks
  const Id2ParameterBlock_Map& id2parameterBlockMap() const {
    return id2ParameterBlock_Map_;
  }
  /// @brief Get the actual map from Id to residual block specs.
  const ResidualBlockId2ResidualBlockSpec_Map& residualBlockId2ResidualBlockSpecMap() const {
    return residualBlockId2ResidualBlockSpec_Map_;
  }

  // these are public for convenient manipulation
  /// \brief Ceres options
  ::ceres::Solver::Options options;

  /// \brief Ceres optimization summary
  ::ceres::Solver::Summary summary;

  /// @brief Solve the optimization problem.
  void solve() {
    Solve(options, problem_.get(), &summary);
  }

 protected:

  /// \brief count the inserted residual blocks.
  uint64_t residualCounter_;

  // member variables related to optimization
  /// \brief The ceres problem
  std::shared_ptr< ::ceres::Problem> problem_;

  // the actual maps
  /// \brief Go from Id to residual block pointer.
  typedef std::unordered_multimap<uint64_t, ResidualBlockSpec> Id2ResidualBlock_Multimap;

  /// \brief Go from residual block id to its parameter blocks.
  typedef std::unordered_map< ::ceres::ResidualBlockId,
      ParameterBlockCollection> ResidualBlockId2ParameterBlockCollection_Map;

  /// \brief The map connecting parameter block ID's and parameter blocks
  Id2ParameterBlock_Map id2ParameterBlock_Map_;

  /// \brief Go from residual ID to specs.
  ResidualBlockId2ResidualBlockSpec_Map residualBlockId2ResidualBlockSpec_Map_;

  /// \brief Go from Id to residual block pointer.
  Id2ResidualBlock_Multimap id2ResidualBlock_Multimap_;

  /// \brief Go from residual block id to its parameter blocks.
  ResidualBlockId2ParameterBlockCollection_Map residualBlockId2ParameterBlockCollection_Map_;

  /// \brief Store parameterisation locally.
  okvis::ceres::HomogeneousPointLocalParameterization homogeneousPointLocalParameterization_;

  /// \brief Store parameterisation locally.
  okvis::ceres::PoseLocalParameterization poseLocalParameterization_;

  /// \brief Store parameterisation locally.
  okvis::ceres::PoseLocalParameterization2d poseLocalParameterization2d_;

  /// \brief Store parameterisation locally.
  okvis::ceres::PoseLocalParameterization3d poseLocalParameterization3d_;

  /// \brief Store parameterisation locally.
  okvis::ceres::PoseLocalParameterization4d poseLocalParameterization4d_;

};

}  //namespace okvis
}  //namespace ceres

#endif /* INCLUDE_OKVIS_CERES_MAP_HPP_ */
