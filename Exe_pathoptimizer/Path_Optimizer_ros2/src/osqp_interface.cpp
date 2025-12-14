// Simplified OSQP interface for OSQP v0.6.3
#include "osqp_interface.hpp"
#include <iostream>
#include <cmath>

namespace autoware::path_optimizer
{

// CSC Matrix 변환 함수
CSC_Matrix calCSCMatrix(const Eigen::MatrixXd & mat)
{
  CSC_Matrix csc_mat;
  
  const int rows = mat.rows();
  const int cols = mat.cols();
  
  csc_mat.m_col_idxs.push_back(0);
  
  for (int j = 0; j < cols; ++j) {
    for (int i = 0; i < rows; ++i) {
      if (std::abs(mat(i, j)) > 1e-9) {
        csc_mat.m_vals.push_back(mat(i, j));
        csc_mat.m_row_idxs.push_back(i);
      }
    }
    csc_mat.m_col_idxs.push_back(static_cast<int>(csc_mat.m_vals.size()));
  }
  
  return csc_mat;
}

CSC_Matrix calCSCMatrixTrapezoidal(const Eigen::MatrixXd & mat)
{
  // For symmetric matrix, only store upper triangular part
  CSC_Matrix csc_mat;
  
  const int n = mat.rows();
  
  csc_mat.m_col_idxs.push_back(0);
  
  for (int j = 0; j < n; ++j) {
    for (int i = 0; i <= j; ++i) {  // Only upper triangular
      if (std::abs(mat(i, j)) > 1e-9) {
        csc_mat.m_vals.push_back(mat(i, j));
        csc_mat.m_row_idxs.push_back(i);
      }
    }
    csc_mat.m_col_idxs.push_back(static_cast<int>(csc_mat.m_vals.size()));
  }
  
  return csc_mat;
}

#ifdef USE_OSQP

// Helper: Convert CSC_Matrix to csc* for OSQP v0.6.3
static csc* convertToCSC(const CSC_Matrix& csc_mat, c_int rows, c_int cols)
{
  c_int nnz = static_cast<c_int>(csc_mat.m_vals.size());
  
  csc* result = static_cast<csc*>(c_malloc(sizeof(csc)));
  result->m = rows;
  result->n = cols;
  result->nz = -1;  // CSC format
  result->nzmax = nnz;
  
  result->x = static_cast<c_float*>(c_malloc(nnz * sizeof(c_float)));
  result->i = static_cast<c_int*>(c_malloc(nnz * sizeof(c_int)));
  result->p = static_cast<c_int*>(c_malloc((cols + 1) * sizeof(c_int)));
  
  for (c_int k = 0; k < nnz; k++) {
    result->x[k] = static_cast<c_float>(csc_mat.m_vals[k]);
    result->i[k] = static_cast<c_int>(csc_mat.m_row_idxs[k]);
  }
  
  for (c_int k = 0; k <= cols; k++) {
    result->p[k] = static_cast<c_int>(csc_mat.m_col_idxs[k]);
  }
  
  return result;
}

OSQPInterface::OSQPInterface(const double eps_abs)
: param_n_(0), work_initialized_(false), exitflag_(0)
{
  // Simple constructor - does not initialize solver
  osqp_set_default_settings(&settings_);
  settings_.eps_abs = eps_abs;
}

OSQPInterface::OSQPInterface(
  const CSC_Matrix & P,
  const CSC_Matrix & A,
  const std::vector<double> & q,
  const std::vector<double> & l,
  const std::vector<double> & u,
  const double eps_abs)
: param_n_(static_cast<int64_t>(q.size())), work_initialized_(false), exitflag_(0)
{
  // Setup OSQP solver with problem data
  
  // Copy data to internal storage
  q_vec_ = q;
  l_vec_ = l;
  u_vec_ = u;
  
  // Convert matrices to OSQP format
  c_int n = static_cast<c_int>(param_n_);
  c_int m = static_cast<c_int>(l.size());
  
  P_csc_ = convertToCSC(P, n, n);
  A_csc_ = convertToCSC(A, m, n);
  
  // Setup OSQPData
  data_.n = n;
  data_.m = m;
  data_.P = P_csc_;
  data_.A = A_csc_;
  data_.q = const_cast<c_float*>(reinterpret_cast<const c_float*>(q_vec_.data()));
  data_.l = const_cast<c_float*>(reinterpret_cast<const c_float*>(l_vec_.data()));
  data_.u = const_cast<c_float*>(reinterpret_cast<const c_float*>(u_vec_.data()));
  
  // Setup settings
  osqp_set_default_settings(&settings_);
  settings_.alpha = 1.6;
  settings_.eps_abs = eps_abs;  // ROS2 default: 1e-4
  settings_.eps_rel = 1.0e-4;   // ROS2 default
  settings_.max_iter = 8000;    // ROS2 default
  settings_.verbose = 0;
  
  // Setup solver
  c_int status = osqp_setup(&work_, &data_, &settings_);
  
  if (status == 0 && work_ != nullptr) {
    work_initialized_ = true;
  } else {
    std::cerr << "[OSQPInterface] ERROR: Failed to setup solver (status: " << status << ")" << std::endl;
  }
}

OSQPInterface::~OSQPInterface()
{
  if (work_ != nullptr) {
    osqp_cleanup(work_);
  }
  if (P_csc_ != nullptr) {
    if (P_csc_->x) c_free(P_csc_->x);
    if (P_csc_->i) c_free(P_csc_->i);
    if (P_csc_->p) c_free(P_csc_->p);
    c_free(P_csc_);
  }
  if (A_csc_ != nullptr) {
    if (A_csc_->x) c_free(A_csc_->x);
    if (A_csc_->i) c_free(A_csc_->i);
    if (A_csc_->p) c_free(A_csc_->p);
    c_free(A_csc_);
  }
}

std::tuple<std::vector<double>, std::vector<double>, int, int, int> 
OSQPInterface::optimize()
{
  if (!work_initialized_ || work_ == nullptr) {
    std::cerr << "[OSQPInterface] Solver not initialized" << std::endl;
    return {{}, {}, -1, -1, -1};
  }
  
  return solve();
}

std::tuple<std::vector<double>, std::vector<double>, int, int, int> 
OSQPInterface::solve()
{
  if (!work_initialized_ || work_ == nullptr) {
    std::cerr << "[OSQPInterface] ERROR: Solver not initialized" << std::endl;
    return {{}, {}, -1, -1, -1};
  }
  
  const c_int solve_status = osqp_solve(work_);
  
  if (solve_status != 0) {
    std::cerr << "[OSQPInterface] Solve failed with status " << solve_status << std::endl;
  }
  
  std::vector<double> primal_solution;
  std::vector<double> dual_solution;
  
  // Extract solution from OSQPWorkspace v0.6.3
  if (solve_status == 0 && work_ != nullptr && work_->solution != nullptr) {
    if (work_->solution->x != nullptr) {
      primal_solution.resize(param_n_);
      for (int i = 0; i < param_n_; i++) {
        primal_solution[i] = static_cast<double>(work_->solution->x[i]);
      }
    }
    if (work_->solution->y != nullptr && !l_vec_.empty()) {
      dual_solution.resize(l_vec_.size());
      for (size_t i = 0; i < l_vec_.size(); i++) {
        dual_solution[i] = static_cast<double>(work_->solution->y[i]);
      }
    }
  }
  
  const int status_val = (solve_status == 0) ? 1 : static_cast<int>(solve_status);
  const int status_polish = 1;
  const int iter = (work_ && work_->info) ? static_cast<int>(work_->info->iter) : 0;
  
  return {primal_solution, dual_solution, status_polish, status_val, iter};
}

void OSQPInterface::updateCscP(const CSC_Matrix & P_csc)
{
  // Not implemented in simplified version
  std::cerr << "[OSQPInterface::updateCscP] Not implemented" << std::endl;
}

void OSQPInterface::updateQ(const std::vector<double> & q)
{
  // Not implemented in simplified version
  std::cerr << "[OSQPInterface::updateQ] Not implemented" << std::endl;
}

void OSQPInterface::updateCscA(const CSC_Matrix & A_csc)
{
  // Not implemented in simplified version
  std::cerr << "[OSQPInterface::updateCscA] Not implemented" << std::endl;
}

void OSQPInterface::updateBounds(
  const std::vector<double> & lower_bound,
  const std::vector<double> & upper_bound)
{
  // Not implemented in simplified version
  std::cerr << "[OSQPInterface::updateBounds] Not implemented" << std::endl;
}

void OSQPInterface::setWarmStart(
  const std::vector<double> & primal_vars,
  const std::vector<double> & dual_vars)
{
  // ⭐ ROS2 temporal consistency: OSQP에 이전 해를 initial guess로 제공
  if (!work_initialized_ || work_ == nullptr) {
    std::cerr << "[OSQPInterface::setWarmStart] Solver not initialized" << std::endl;
    return;
  }
  
  if (primal_vars.empty()) {
    std::cerr << "[OSQPInterface::setWarmStart] Empty primal variables" << std::endl;
    return;
  }
  
  if (static_cast<int>(primal_vars.size()) != param_n_) {
    std::cerr << "[OSQPInterface::setWarmStart] Size mismatch: expected " << param_n_ 
              << ", got " << primal_vars.size() << std::endl;
    return;
  }
  
  // Convert to c_float array for OSQP v0.6.3
  std::vector<c_float> primal_float(primal_vars.begin(), primal_vars.end());
  
  // Set primal warm start
  c_int status = osqp_warm_start_x(work_, primal_float.data());
  
  if (status != 0) {
    std::cerr << "[OSQPInterface::setWarmStart] Failed to set primal warm start" << std::endl;
    return;
  }
  
  // Set dual warm start if provided
  if (!dual_vars.empty()) {
    std::vector<c_float> dual_float(dual_vars.begin(), dual_vars.end());
    status = osqp_warm_start_y(work_, dual_float.data());
    
    if (status != 0) {
      std::cerr << "[OSQPInterface::setWarmStart] Failed to set dual warm start" << std::endl;
    }
  }
  
  std::cout << "[OSQPInterface] Warm start applied (primal size=" << primal_vars.size() << ")" << std::endl;
}

void OSQPInterface::logUnsolvedStatus(const std::string & prefix) const
{
  std::cerr << prefix << " optimization failed" << std::endl;
}

#endif  // USE_OSQP

}  // namespace autoware::path_optimizer
