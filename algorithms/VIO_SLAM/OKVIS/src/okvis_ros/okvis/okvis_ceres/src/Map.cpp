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
 *    Modified: Andreas Forster (an.forster@gmail.com)
 *********************************************************************************/

/**
 * @file Map.cpp
 * @brief Source file for the Map class.
 * @author Stefan Leutenegger
 * @author Andreas Forster
 */

#include <okvis/ceres/Map.hpp>
#include <ceres/ordered_groups.h>
#include <okvis/ceres/HomogeneousPointParameterBlock.hpp>
#include <okvis/ceres/MarginalizationError.hpp>

/// \brief okvis Main namespace of this package.
namespace okvis {
/// \brief ceres Namespace for ceres-related functionality implemented in okvis.
namespace ceres {

// Constructor.
Map::Map()
    : residualCounter_(0) {
  ::ceres::Problem::Options problemOptions;
  problemOptions.local_parameterization_ownership =
      ::ceres::Ownership::DO_NOT_TAKE_OWNERSHIP;
  problemOptions.loss_function_ownership =
      ::ceres::Ownership::DO_NOT_TAKE_OWNERSHIP;
  problemOptions.cost_function_ownership =
      ::ceres::Ownership::DO_NOT_TAKE_OWNERSHIP;
  //problemOptions.enable_fast_parameter_block_removal = true;
  problem_.reset(new ::ceres::Problem(problemOptions));
  //options.linear_solver_ordering = new ::ceres::ParameterBlockOrdering;
}

// Check whether a certain parameter block is part of the map.
bool Map::parameterBlockExists(uint64_t parameterBlockId) const {
  if (id2ParameterBlock_Map_.find(parameterBlockId)
      == id2ParameterBlock_Map_.end())
    return false;
  return true;
}

// Log information on a parameter block.
void Map::printParameterBlockInfo(uint64_t parameterBlockId) const {
  ResidualBlockCollection residualCollection = residuals(parameterBlockId);
  LOG(INFO) << "parameter info" << std::endl << "----------------------------"
            << std::endl << " - block Id: " << parameterBlockId << std::endl
            << " - type: " << parameterBlockPtr(parameterBlockId)->typeInfo()
            << std::endl << " - residuals (" << residualCollection.size()
            << "):";
  for (size_t i = 0; i < residualCollection.size(); ++i) {
    LOG(INFO)
        << "   - id: "
        << residualCollection.at(i).residualBlockId
        << std::endl
        << "   - type: "
        << errorInterfacePtr(residualCollection.at(i).residualBlockId)->typeInfo();
  }
  LOG(INFO) << "============================";
}

// Log information on a residual block.
void Map::printResidualBlockInfo(
    ::ceres::ResidualBlockId residualBlockId) const {
  LOG(INFO) << "   - id: " << residualBlockId << std::endl << "   - type: "
            << errorInterfacePtr(residualBlockId)->typeInfo();
}

// Obtain the Hessian block for a specific parameter block.
void Map::getLhs(uint64_t parameterBlockId, Eigen::MatrixXd& H) {
  OKVIS_ASSERT_TRUE_DBG(Exception,parameterBlockExists(parameterBlockId),"parameter block not in map.");
  ResidualBlockCollection res = residuals(parameterBlockId);
  H.setZero();
  for (size_t i = 0; i < res.size(); ++i) {

    // parameters:
    ParameterBlockCollection pars = parameters(res[i].residualBlockId);

    double** parametersRaw = new double*[pars.size()];
    Eigen::VectorXd residualsEigen(res[i].errorInterfacePtr->residualDim());
    double* residualsRaw = residualsEigen.data();

    double** jacobiansRaw = new double*[pars.size()];
    std::vector<
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>,
        Eigen::aligned_allocator<
            Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic,
                Eigen::RowMajor> > > jacobiansEigen(pars.size());

    double** jacobiansMinimalRaw = new double*[pars.size()];
    std::vector<
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>,
        Eigen::aligned_allocator<
            Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic,
                Eigen::RowMajor> > > jacobiansMinimalEigen(pars.size());

    int J = -1;
    for (size_t j = 0; j < pars.size(); ++j) {
      // determine which is the relevant block
      if (pars[j].second->id() == parameterBlockId)
        J = j;
      parametersRaw[j] = pars[j].second->parameters();
      jacobiansEigen[j].resize(res[i].errorInterfacePtr->residualDim(),
                               pars[j].second->dimension());
      jacobiansRaw[j] = jacobiansEigen[j].data();
      jacobiansMinimalEigen[j].resize(res[i].errorInterfacePtr->residualDim(),
                                      pars[j].second->minimalDimension());
      jacobiansMinimalRaw[j] = jacobiansMinimalEigen[j].data();
    }

    // evaluate residual block
    res[i].errorInterfacePtr->EvaluateWithMinimalJacobians(parametersRaw,
                                                           residualsRaw,
                                                           jacobiansRaw,
                                                           jacobiansMinimalRaw);

    // get block
    H += jacobiansMinimalEigen[J].transpose() * jacobiansMinimalEigen[J];

    // cleanup
    delete[] parametersRaw;
    delete[] jacobiansRaw;
    delete[] jacobiansMinimalRaw;
  }
}

// Check a Jacobian with numeric differences.
bool Map::isJacobianCorrect(::ceres::ResidualBlockId residualBlockId,
                            double relTol) const {
  std::shared_ptr<const okvis::ceres::ErrorInterface> errorInterface_ptr =
      errorInterfacePtr(residualBlockId);
  ParameterBlockCollection parametersBlocks = parameters(residualBlockId);

  // set up data structures for storage
  std::vector<
      Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>,
      Eigen::aligned_allocator<
          Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> > > J(
      parametersBlocks.size());
  std::vector<
      Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>,
      Eigen::aligned_allocator<
          Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> > > J_min(
      parametersBlocks.size());
  std::vector<
      Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>,
      Eigen::aligned_allocator<
          Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> > > J_numDiff(
      parametersBlocks.size());
  std::vector<
      Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>,
      Eigen::aligned_allocator<
          Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> > > J_min_numDiff(
      parametersBlocks.size());
  double **parameters, **jacobians, **jacobiansMinimal;
  parameters = new double*[parametersBlocks.size()];
  jacobians = new double*[parametersBlocks.size()];
  jacobiansMinimal = new double*[parametersBlocks.size()];
  for (size_t i = 0; i < parametersBlocks.size(); ++i) {
    // set up the analytic Jacobians
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> Ji(
        errorInterface_ptr->residualDim(),
        parametersBlocks[i].second->dimension());
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> Ji_min(
        errorInterface_ptr->residualDim(),
        parametersBlocks[i].second->minimalDimension());

    // set up the numeric ones
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> Ji_numDiff(
        errorInterface_ptr->residualDim(),
        parametersBlocks[i].second->dimension());
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> Ji_min_numDiff(
        errorInterface_ptr->residualDim(),
        parametersBlocks[i].second->minimalDimension());

    // fill in
    J[i].resize(errorInterface_ptr->residualDim(),
                parametersBlocks[i].second->dimension());
    J_min[i].resize(errorInterface_ptr->residualDim(),
                    parametersBlocks[i].second->minimalDimension());
    J_numDiff[i].resize(errorInterface_ptr->residualDim(),
                        parametersBlocks[i].second->dimension());
    J_min_numDiff[i].resize(errorInterface_ptr->residualDim(),
                            parametersBlocks[i].second->minimalDimension());
    parameters[i] = parametersBlocks[i].second->parameters();
    jacobians[i] = J[i].data();
    jacobiansMinimal[i] = J_min[i].data();
  }

  // calculate num diff Jacobians
  const double delta = 1e-8;
  for (size_t i = 0; i < parametersBlocks.size(); ++i) {
    for (size_t j = 0; j < parametersBlocks[i].second->minimalDimension();
        ++j) {
      Eigen::VectorXd residuals_p(errorInterface_ptr->residualDim());
      Eigen::VectorXd residuals_m(errorInterface_ptr->residualDim());

      // apply positive delta
      Eigen::VectorXd parameters_p(parametersBlocks[i].second->dimension());
      Eigen::VectorXd parameters_m(parametersBlocks[i].second->dimension());
      Eigen::VectorXd plus(parametersBlocks[i].second->minimalDimension());
      plus.setZero();
      plus[j] = delta;
      parametersBlocks[i].second->plus(parameters[i], plus.data(),
                                       parameters_p.data());
      parameters[i] = parameters_p.data();
      errorInterface_ptr->EvaluateWithMinimalJacobians(parameters,
                                                       residuals_p.data(), NULL,
                                                       NULL);
      parameters[i] = parametersBlocks[i].second->parameters();  // reset
      // apply negative delta
      plus.setZero();
      plus[j] = -delta;
      parametersBlocks[i].second->plus(parameters[i], plus.data(),
                                       parameters_m.data());
      parameters[i] = parameters_m.data();
      errorInterface_ptr->EvaluateWithMinimalJacobians(parameters,
                                                       residuals_m.data(), NULL,
                                                       NULL);
      parameters[i] = parametersBlocks[i].second->parameters();  // reset
      // calculate numeric difference
      J_min_numDiff[i].col(j) = (residuals_p - residuals_m) * 1.0
          / (2.0 * delta);
    }
  }

  // calculate analytic Jacobians and compare
  bool isCorrect = true;
  Eigen::VectorXd residuals(errorInterface_ptr->residualDim());
  for (size_t i = 0; i < parametersBlocks.size(); ++i) {
    // calc
    errorInterface_ptr->EvaluateWithMinimalJacobians(parameters,
                                                     residuals.data(),
                                                     jacobians,
                                                     jacobiansMinimal);
    // check
    double norm = J_min_numDiff[i].norm();
    Eigen::MatrixXd J_diff = J_min_numDiff[i] - J_min[i];
    double maxDiff = std::max(-J_diff.minCoeff(), J_diff.maxCoeff());
    if (maxDiff / norm > relTol) {
      LOG(INFO) << "Jacobian inconsistent: " << errorInterface_ptr->typeInfo();
      LOG(INFO) << "num diff Jacobian[" << i << "]:";
      LOG(INFO) << J_min_numDiff[i];
      LOG(INFO) << "provided Jacobian[" << i << "]:";
      LOG(INFO) << J_min[i];
      LOG(INFO) << "relative error: " << maxDiff / norm
                << ", relative tolerance: " << relTol;
      isCorrect = false;
    }

  }

  delete[] parameters;
  delete[] jacobians;
  delete[] jacobiansMinimal;

  return isCorrect;
}

// Add a parameter block to the map
bool Map::addParameterBlock(
    std::shared_ptr<okvis::ceres::ParameterBlock> parameterBlock,
    int parameterization, const int /*group*/) {

  // check Id availability
  if (parameterBlockExists(parameterBlock->id())) {
    return false;
  }

  id2ParameterBlock_Map_.insert(
      std::pair<uint64_t, std::shared_ptr<okvis::ceres::ParameterBlock> >(
          parameterBlock->id(), parameterBlock));

  // also add to ceres problem
  switch (parameterization) {
    case Parameterization::Trivial: {
      problem_->AddParameterBlock(parameterBlock->parameters(),
                                  parameterBlock->dimension());
      break;
    }
    case Parameterization::HomogeneousPoint: {
      problem_->AddParameterBlock(parameterBlock->parameters(),
                                  parameterBlock->dimension(),
                                  &homogeneousPointLocalParameterization_);
      parameterBlock->setLocalParameterizationPtr(
          &homogeneousPointLocalParameterization_);
      break;
    }
    case Parameterization::Pose6d: {
      problem_->AddParameterBlock(parameterBlock->parameters(),
                                  parameterBlock->dimension(),
                                  &poseLocalParameterization_);
      parameterBlock->setLocalParameterizationPtr(&poseLocalParameterization_);
      break;
    }
    case Parameterization::Pose3d: {
      problem_->AddParameterBlock(parameterBlock->parameters(),
                                  parameterBlock->dimension(),
                                  &poseLocalParameterization3d_);
      parameterBlock->setLocalParameterizationPtr(&poseLocalParameterization3d_);
      break;
    }
    case Parameterization::Pose4d: {
      problem_->AddParameterBlock(parameterBlock->parameters(),
                                  parameterBlock->dimension(),
                                  &poseLocalParameterization4d_);
      parameterBlock->setLocalParameterizationPtr(&poseLocalParameterization4d_);
      break;
    }
    case Parameterization::Pose2d: {
      problem_->AddParameterBlock(parameterBlock->parameters(),
                                  parameterBlock->dimension(),
                                  &poseLocalParameterization2d_);
      parameterBlock->setLocalParameterizationPtr(&poseLocalParameterization2d_);
      break;
    }
    default: {
      return false;
      break;  // just for consistency...
    }
  }

  /*const okvis::ceres::LocalParamizationAdditionalInterfaces* ptr =
      dynamic_cast<const okvis::ceres::LocalParamizationAdditionalInterfaces*>(
      parameterBlock->localParameterizationPtr());
  if(ptr)
    std::cout<<"verify local size "<< parameterBlock->localParameterizationPtr()->LocalSize() << " = "<<
            int(ptr->verify(parameterBlock->parameters()))<<
            std::endl;*/

  return true;
}

// Remove a parameter block from the map.
bool Map::removeParameterBlock(uint64_t parameterBlockId) {
  if (!parameterBlockExists(parameterBlockId))
    return false;

  // remove all connected residuals
  const ResidualBlockCollection res = residuals(parameterBlockId);
  for (size_t i = 0; i < res.size(); ++i) {
    removeResidualBlock(res[i].residualBlockId);  // remove in ceres and book-keeping
  }
  problem_->RemoveParameterBlock(
      parameterBlockPtr(parameterBlockId)->parameters());  // remove parameter block
  id2ParameterBlock_Map_.erase(parameterBlockId);  // remove book-keeping
  return true;
}

// Remove a parameter block from the map.
bool Map::removeParameterBlock(
    std::shared_ptr<okvis::ceres::ParameterBlock> parameterBlock) {
  return removeParameterBlock(parameterBlock->id());
}

// Adds a residual block.
::ceres::ResidualBlockId Map::addResidualBlock(
    std::shared_ptr< ::ceres::CostFunction> cost_function,
    ::ceres::LossFunction* loss_function,
    std::vector<std::shared_ptr<okvis::ceres::ParameterBlock> >& parameterBlockPtrs) {

  ::ceres::ResidualBlockId return_id;
  std::vector<double*> parameter_blocks;
  ParameterBlockCollection parameterBlockCollection;
  for (size_t i = 0; i < parameterBlockPtrs.size(); ++i) {
    parameter_blocks.push_back(parameterBlockPtrs.at(i)->parameters());
    parameterBlockCollection.push_back(
        ParameterBlockSpec(parameterBlockPtrs.at(i)->id(),
                           parameterBlockPtrs.at(i)));
  }

  // add in ceres
  return_id = problem_->AddResidualBlock(cost_function.get(), loss_function,
                                         parameter_blocks);

  // add in book-keeping
  std::shared_ptr<ErrorInterface> errorInterfacePtr = std::dynamic_pointer_cast<
      ErrorInterface>(cost_function);
  OKVIS_ASSERT_TRUE_DBG(Exception,errorInterfacePtr!=0,"Supplied a cost function without okvis::ceres::ErrorInterface");
  residualBlockId2ResidualBlockSpec_Map_.insert(
      std::pair< ::ceres::ResidualBlockId, ResidualBlockSpec>(
          return_id,
          ResidualBlockSpec(return_id, loss_function, errorInterfacePtr)));

  // update book-keeping
  std::pair<ResidualBlockId2ParameterBlockCollection_Map::iterator, bool> insertion =
      residualBlockId2ParameterBlockCollection_Map_.insert(
          std::pair< ::ceres::ResidualBlockId, ParameterBlockCollection>(
              return_id, parameterBlockCollection));
  if (insertion.second == false)
    return ::ceres::ResidualBlockId(0);

  // update ResidualBlock pointers on involved ParameterBlocks
  for (uint64_t parameter_id = 0;
      parameter_id < parameterBlockCollection.size(); ++parameter_id) {
    id2ResidualBlock_Multimap_.insert(
        std::pair<uint64_t, ResidualBlockSpec>(
            parameterBlockCollection[parameter_id].first,
            ResidualBlockSpec(return_id, loss_function, errorInterfacePtr)));
  }

  return return_id;
}

// Add a residual block. See respective ceres docu. If more are needed, see other interface.
::ceres::ResidualBlockId Map::addResidualBlock(
    std::shared_ptr< ::ceres::CostFunction> cost_function,
    ::ceres::LossFunction* loss_function,
    std::shared_ptr<okvis::ceres::ParameterBlock> x0,
    std::shared_ptr<okvis::ceres::ParameterBlock> x1,
    std::shared_ptr<okvis::ceres::ParameterBlock> x2,
    std::shared_ptr<okvis::ceres::ParameterBlock> x3,
    std::shared_ptr<okvis::ceres::ParameterBlock> x4,
    std::shared_ptr<okvis::ceres::ParameterBlock> x5,
    std::shared_ptr<okvis::ceres::ParameterBlock> x6,
    std::shared_ptr<okvis::ceres::ParameterBlock> x7,
    std::shared_ptr<okvis::ceres::ParameterBlock> x8,
    std::shared_ptr<okvis::ceres::ParameterBlock> x9) {

  std::vector<std::shared_ptr<okvis::ceres::ParameterBlock> > parameterBlockPtrs;
  if (x0 != 0) {
    parameterBlockPtrs.push_back(x0);
  }
  if (x1 != 0) {
    parameterBlockPtrs.push_back(x1);
  }
  if (x2 != 0) {
    parameterBlockPtrs.push_back(x2);
  }
  if (x3 != 0) {
    parameterBlockPtrs.push_back(x3);
  }
  if (x4 != 0) {
    parameterBlockPtrs.push_back(x4);
  }
  if (x5 != 0) {
    parameterBlockPtrs.push_back(x5);
  }
  if (x6 != 0) {
    parameterBlockPtrs.push_back(x6);
  }
  if (x7 != 0) {
    parameterBlockPtrs.push_back(x7);
  }
  if (x8 != 0) {
    parameterBlockPtrs.push_back(x8);
  }
  if (x9 != 0) {
    parameterBlockPtrs.push_back(x9);
  }

  return Map::addResidualBlock(cost_function, loss_function, parameterBlockPtrs);

}

// Replace the parameters connected to a residual block ID.
void Map::resetResidualBlock(
    ::ceres::ResidualBlockId residualBlockId,
    std::vector<std::shared_ptr<okvis::ceres::ParameterBlock> >& parameterBlockPtrs) {

  // remember the residual block spec:
  ResidualBlockSpec spec =
      residualBlockId2ResidualBlockSpec_Map_[residualBlockId];
  // remove residual from old parameter set
  ResidualBlockId2ParameterBlockCollection_Map::iterator it =
      residualBlockId2ParameterBlockCollection_Map_.find(residualBlockId);
  OKVIS_ASSERT_TRUE_DBG(Exception,it!=residualBlockId2ParameterBlockCollection_Map_.end(),
      "residual block not in map.");
  for (ParameterBlockCollection::iterator parameter_it = it->second.begin();
      parameter_it != it->second.end(); ++parameter_it) {
    uint64_t parameterId = parameter_it->second->id();
    std::pair<Id2ResidualBlock_Multimap::iterator,
        Id2ResidualBlock_Multimap::iterator> range = id2ResidualBlock_Multimap_
        .equal_range(parameterId);
    OKVIS_ASSERT_FALSE_DBG(Exception,range.first==id2ResidualBlock_Multimap_.end(),"book-keeping is broken");
    for (Id2ResidualBlock_Multimap::iterator it2 = range.first;
        it2 != range.second;) {
      if (residualBlockId == it2->second.residualBlockId) {
        it2 = id2ResidualBlock_Multimap_.erase(it2);  // remove book-keeping
      } else {
        it2++;
      }
    }
  }

  ParameterBlockCollection parameterBlockCollection;
  for (size_t i = 0; i < parameterBlockPtrs.size(); ++i) {
    parameterBlockCollection.push_back(
        ParameterBlockSpec(parameterBlockPtrs.at(i)->id(),
                           parameterBlockPtrs.at(i)));
  }

  // update book-keeping
  it->second = parameterBlockCollection;

  // update ResidualBlock pointers on involved ParameterBlocks
  for (uint64_t parameter_id = 0;
      parameter_id < parameterBlockCollection.size(); ++parameter_id) {
    id2ResidualBlock_Multimap_.insert(
        std::pair<uint64_t, ResidualBlockSpec>(
            parameterBlockCollection[parameter_id].first, spec));
  }
}

// Remove a residual block.
bool Map::removeResidualBlock(::ceres::ResidualBlockId residualBlockId) {
  problem_->RemoveResidualBlock(residualBlockId);  // remove in ceres

  ResidualBlockId2ParameterBlockCollection_Map::iterator it =
      residualBlockId2ParameterBlockCollection_Map_.find(residualBlockId);
  if (it == residualBlockId2ParameterBlockCollection_Map_.end())
    return false;

  for (ParameterBlockCollection::iterator parameter_it = it->second.begin();
      parameter_it != it->second.end(); ++parameter_it) {
    uint64_t parameterId = parameter_it->second->id();
    std::pair<Id2ResidualBlock_Multimap::iterator,
        Id2ResidualBlock_Multimap::iterator> range = id2ResidualBlock_Multimap_
        .equal_range(parameterId);
    OKVIS_ASSERT_FALSE_DBG(Exception,range.first==id2ResidualBlock_Multimap_.end(),"book-keeping is broken");

    for (Id2ResidualBlock_Multimap::iterator it2 = range.first;
        it2 != range.second;) {
      if (residualBlockId == it2->second.residualBlockId) {
        it2 = id2ResidualBlock_Multimap_.erase(it2);  // remove book-keeping
      } else {
        it2++;
      }
    }
  }
  residualBlockId2ParameterBlockCollection_Map_.erase(it);  // remove book-keeping
  residualBlockId2ResidualBlockSpec_Map_.erase(residualBlockId);  // remove book-keeping
  return true;
}

// Do not optimise a certain parameter block.
bool Map::setParameterBlockConstant(uint64_t parameterBlockId) {
  if (!parameterBlockExists(parameterBlockId))
    return false;
  std::shared_ptr<ParameterBlock> parameterBlock = id2ParameterBlock_Map_.find(
      parameterBlockId)->second;
  parameterBlock->setFixed(true);
  problem_->SetParameterBlockConstant(parameterBlock->parameters());
  return true;
}

// Optimise a certain parameter block (this is the default).
bool Map::setParameterBlockVariable(uint64_t parameterBlockId) {
  if (!parameterBlockExists(parameterBlockId))
    return false;
  std::shared_ptr<ParameterBlock> parameterBlock = id2ParameterBlock_Map_.find(
      parameterBlockId)->second;
  parameterBlock->setFixed(false);
  problem_->SetParameterBlockVariable(parameterBlock->parameters());
  return true;
}

// Reset the (local) parameterisation of a parameter block.
bool Map::resetParameterization(uint64_t parameterBlockId,
                                int parameterization) {
  if (!parameterBlockExists(parameterBlockId))
    return false;
  // the ceres documentation states that a parameterization may never be changed on.
  // therefore, we have to remove the parameter block in question and re-add it.
  ResidualBlockCollection res = residuals(parameterBlockId);
  std::shared_ptr<ParameterBlock> parBlockPtr = parameterBlockPtr(
      parameterBlockId);

  // get parameter block pointers
  std::vector<std::vector<std::shared_ptr<okvis::ceres::ParameterBlock> > > parameterBlockPtrs(
      res.size());
  for (size_t r = 0; r < res.size(); ++r) {
    ParameterBlockCollection pspec = parameters(res[r].residualBlockId);
    for (size_t p = 0; p < pspec.size(); ++p) {
      parameterBlockPtrs[r].push_back(pspec[p].second);
    }
  }

  // remove
  //	int group = options.linear_solver_ordering->GroupId(parBlockPtr->parameters());
  removeParameterBlock(parameterBlockId);
  // add with new parameterization
  addParameterBlock(parBlockPtr, parameterization/*,group*/);

  // re-assemble
  for (size_t r = 0; r < res.size(); ++r) {
    addResidualBlock(
        std::dynamic_pointer_cast< ::ceres::CostFunction>(
            res[r].errorInterfacePtr),
        res[r].lossFunctionPtr, parameterBlockPtrs[r]);
  }

  return true;
}

// Set the (local) parameterisation of a parameter block.
bool Map::setParameterization(
    uint64_t parameterBlockId,
    ::ceres::LocalParameterization* local_parameterization) {
  if (!parameterBlockExists(parameterBlockId))
    return false;
  problem_->SetParameterization(
      id2ParameterBlock_Map_.find(parameterBlockId)->second->parameters(),
      local_parameterization);
  id2ParameterBlock_Map_.find(parameterBlockId)->second
      ->setLocalParameterizationPtr(local_parameterization);
  return true;
}

// getters
// Get a shared pointer to a parameter block.
std::shared_ptr<okvis::ceres::ParameterBlock> Map::parameterBlockPtr(
    uint64_t parameterBlockId) {
  // get a parameterBlock
  OKVIS_ASSERT_TRUE(
      Exception, parameterBlockExists(parameterBlockId),
      "parameterBlock with id "<<parameterBlockId<<" does not exist");
  if (parameterBlockExists(parameterBlockId)) {
    return id2ParameterBlock_Map_.find(parameterBlockId)->second;
  }
  return std::shared_ptr<okvis::ceres::ParameterBlock>();  // NULL
}

// Get a shared pointer to a parameter block.
std::shared_ptr<const okvis::ceres::ParameterBlock> Map::parameterBlockPtr(
    uint64_t parameterBlockId) const {
  // get a parameterBlock
  if (parameterBlockExists(parameterBlockId)) {
    return id2ParameterBlock_Map_.find(parameterBlockId)->second;
  }
  return std::shared_ptr<const okvis::ceres::ParameterBlock>();  // NULL
}

// Get the residual blocks of a parameter block.
Map::ResidualBlockCollection Map::residuals(uint64_t parameterBlockId) const {
  // get the residual blocks of a parameter block
  Id2ResidualBlock_Multimap::const_iterator it1 = id2ResidualBlock_Multimap_
      .find(parameterBlockId);
  if (it1 == id2ResidualBlock_Multimap_.end())
    return Map::ResidualBlockCollection();  // empty
  ResidualBlockCollection returnResiduals;
  std::pair<Id2ResidualBlock_Multimap::const_iterator,
      Id2ResidualBlock_Multimap::const_iterator> range =
      id2ResidualBlock_Multimap_.equal_range(parameterBlockId);
  for (Id2ResidualBlock_Multimap::const_iterator it = range.first;
      it != range.second; ++it) {
    returnResiduals.push_back(it->second);
  }
  return returnResiduals;
}

// Get a shared pointer to an error term.
std::shared_ptr<okvis::ceres::ErrorInterface> Map::errorInterfacePtr(
    ::ceres::ResidualBlockId residualBlockId) {  // get a vertex
  ResidualBlockId2ResidualBlockSpec_Map::iterator it =
      residualBlockId2ResidualBlockSpec_Map_.find(residualBlockId);
  if (it == residualBlockId2ResidualBlockSpec_Map_.end()) {
    return std::shared_ptr<okvis::ceres::ErrorInterface>();  // NULL
  }
  return it->second.errorInterfacePtr;
}

// Get a shared pointer to an error term.
std::shared_ptr<const okvis::ceres::ErrorInterface> Map::errorInterfacePtr(
    ::ceres::ResidualBlockId residualBlockId) const {  // get a vertex
  ResidualBlockId2ResidualBlockSpec_Map::const_iterator it =
      residualBlockId2ResidualBlockSpec_Map_.find(residualBlockId);
  if (it == residualBlockId2ResidualBlockSpec_Map_.end()) {
    return std::shared_ptr<okvis::ceres::ErrorInterface>();  // NULL
  }
  return it->second.errorInterfacePtr;
}

// Get the parameters of a residual block.
Map::ParameterBlockCollection Map::parameters(
    ::ceres::ResidualBlockId residualBlockId) const {  // get the parameter blocks connected
  ResidualBlockId2ParameterBlockCollection_Map::const_iterator it =
      residualBlockId2ParameterBlockCollection_Map_.find(residualBlockId);
  if (it == residualBlockId2ParameterBlockCollection_Map_.end()) {
    ParameterBlockCollection empty;
    return empty;  // empty vector
  }
  return it->second;
}

}  //namespace okvis
}  //namespace ceres

