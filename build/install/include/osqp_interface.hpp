// OSQP Interface for Path Optimizer
#ifndef PATH_OPTIMIZER__OSQP_INTERFACE_HPP_
#define PATH_OPTIMIZER__OSQP_INTERFACE_HPP_

#ifdef USE_OSQP
#include <osqp/osqp.h>
#endif

#include <Eigen/Core>
#include <memory>
#include <functional>
#include <vector>
#include <tuple>
#include <string>

namespace autoware::path_optimizer
{

// CSC (Compressed Sparse Column) Matrix structure
struct CSC_Matrix
{
  std::vector<double> m_vals;     // Non-zero values
  std::vector<long long> m_row_idxs;    // Row indices (OSQPInt)
  std::vector<long long> m_col_idxs;    // Column pointers (OSQPInt)
};

// Helper functions for CSC matrix conversion
CSC_Matrix calCSCMatrix(const Eigen::MatrixXd & mat);
CSC_Matrix calCSCMatrixTrapezoidal(const Eigen::MatrixXd & mat);

#ifdef USE_OSQP

/**
 * OSQPInterface: C++ wrapper for OSQP solver v1.x
 */
class OSQPInterface
{
public:
  explicit OSQPInterface(const double eps_abs);
  
  OSQPInterface(
    const CSC_Matrix & P,
    const CSC_Matrix & A,
    const std::vector<double> & q,
    const std::vector<double> & l,
    const std::vector<double> & u,
    const double eps_abs);
  
  ~OSQPInterface();
  
  // Main optimization methods
  std::tuple<std::vector<double>, std::vector<double>, int, int, int> optimize();
  std::tuple<std::vector<double>, std::vector<double>, int, int, int> solve();
  
  // Update methods (for warm start)
  void updateCscP(const CSC_Matrix & P_csc);
  void updateQ(const std::vector<double> & q);
  void updateCscA(const CSC_Matrix & A_csc);
  void updateBounds(
    const std::vector<double> & lower_bound,
    const std::vector<double> & upper_bound);
  
  // ⭐ Warm start: set initial guess for primal and dual variables
  void setWarmStart(
    const std::vector<double> & primal_vars,
    const std::vector<double> & dual_vars = {});
  
  void logUnsolvedStatus(const std::string & prefix) const;
  
private:
  int64_t param_n_;              // Number of variables
  bool work_initialized_;         // Initialization flag
  int exitflag_;                  // Exit status
  
  OSQPWorkspace* work_{nullptr};  // OSQP workspace (v0.6.3)
  OSQPSettings settings_;         // OSQP settings
  OSQPData data_;                 // OSQP data structure
  csc* P_csc_{nullptr};          // Objective matrix (owned by OSQPInterface)
  csc* A_csc_{nullptr};          // Constraint matrix (owned by OSQPInterface)
  
  std::vector<double> q_vec_;     // Linear objective (must persist)
  std::vector<double> l_vec_;     // Lower bounds (must persist)
  std::vector<double> u_vec_;     // Upper bounds (must persist)
};

#endif  // USE_OSQP

}  // namespace autoware::path_optimizer

#endif  // PATH_OPTIMIZER__OSQP_INTERFACE_HPP_
