#ifndef __BSPLINE_NLP_H__
#define __BSPLINE_NLP_H__

#include "IpTNLP.hpp"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

using namespace Ipopt;

class BsplineTNLP: public TNLP
{
private:
  int j;
  
  Index N_;
  Number* q_;
  
  double getSmoothnessCost();
  double getDistanceCost();
  double getFeasibilityCost();
  double getEndpointCost(); //Added from origin project.
  
public:
  BsplineTNLP();
  virtual ~BsplineTNLP();
  
  /** Method to return the bounds for my problem */
  virtual bool get_bounds_info(
     Index   n,
     Number* x_l,
     Number* x_u,
     Index   m,
     Number* g_l,
     Number* g_u
  );
  
  /** Method to return the starting point for the algorithm */
  virtual bool get_starting_point(
     Index   n,
     bool    init_x,
     Number* x,
     bool    init_z,combineCost
     Number* z_L,
     Number* z_U,
     Index   m,
     bool    init_lambda,
     Number* lambda
  );

  /** Method to return the objective value */
  virtual bool eval_f(
     Index         n,
     const Number* x,
     bool          new_x,
     Number&       obj_value
  );  
  
  /** Method to return the gradient of the objective */
  virtual bool eval_grad_f(
     Index         n,
     const Number* x,
     bool          new_x,
     Number*       grad_f
  );  
  
  /** Method to return the constraint residuals */
  //Useless, just return false.
  virtual bool eval_g(
     Index         n,
     const Number* x,
     bool          new_x,
     Index         m,
     Number*       g
  );
  
  /** Method to return:
   *   1) The structure of the Jacobian (if "values" is NULL)
   *   2) The values of the Jacobian (if "values" is not NULL)
   */
  //Useless, just return false.
  virtual bool eval_jac_g(
     Index         n,
     const Number* x,
     bool          new_x,
     Index         m,
     Index         nele_jac,
     Index*        iRow,
     Index*        jCol,
     Number*       values
  );  
  /** Method to return:
   *   1) The structure of the Hessian of the Lagrangian (if "values" is NULL)
   *   2) The values of the Hessian of the Lagrangian (if "values" is not NULL)
   */
  //Useless.
  virtual bool eval_h(
     Index         n,
     const Number* x,
     bool          new_x,
     Number        obj_factor,
     Index         m,
     const Number* lambda,
     bool          new_lambda,
     Index         nele_hess,
     Index*        iRow,
     Index*        jCol,
     Number*       values
  );  
  /** This method is called when the algorithm is complete so the TNLP can store/write the solution */
  virtual void finalize_solution(
     SolverReturn               status,
     Index                      n,
     const Number*              x,
     const Number*              z_L,
     const Number*              z_U,
     Index                      m,
     const Number*              g,
     const Number*              lambda,
     Number                     obj_value,
     const IpoptData*           ip_data,
     IpoptCalculatedQuantities* ip_cq
  );
   
   
  

};

#endif // __BSPLINE_NLP_H__
