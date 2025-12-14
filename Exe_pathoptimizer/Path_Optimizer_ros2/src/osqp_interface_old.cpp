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

OSQPInterface::OSQPInterface(const double eps_abs)
: param_n_(0), work_initialized_(false), exitflag_(0)
{
  settings_ = std::make_unique<OSQPSettings>();
  
  if (settings_) {
    osqp_set_default_settings(settings_.get());
    settings_->alpha = 1.6;  // ADMM relaxation parameter
    settings_->eps_abs = eps_abs;
    settings_->eps_rel = 1.0e-4;
    settings_->max_iter = 8000;
    settings_->polish = true;
    settings_->verbose = false;
  }
}

OSQPInterface::OSQPInterface(
  const CSC_Matrix & P,
  const CSC_Matrix & A,
  const std::vector<double> & q,
  const std::vector<double> & l,
  const std::vector<double> & u,
  const double eps_abs)
: OSQPInterface(eps_abs)
{
  param_n_ = static_cast<int64_t>(q.size());
  
  // OSQP data 설정
  data_ = std::make_unique<OSQPData>();
  
  data_->n = param_n_;
  data_->m = static_cast<int>(l.size());
  
  // P matrix (objective - quadratic term)
  data_->P = csc_matrix(
    data_->n, data_->n, P.m_vals.size(),
    const_cast<double *>(P.m_vals.data()),
    const_cast<int *>(P.m_row_idxs.data()),
    const_cast<int *>(P.m_col_idxs.data()));
  
  // q vector (objective - linear term)
  data_->q = const_cast<double *>(q.data());
  
  // A matrix (constraints)
  data_->A = csc_matrix(
    data_->m, data_->n, A.m_vals.size(),
    const_cast<double *>(A.m_vals.data()),
    const_cast<int *>(A.m_row_idxs.data()),
    const_cast<int *>(A.m_col_idxs.data()));
  
  // Bounds
  data_->l = const_cast<double *>(l.data());
  data_->u = const_cast<double *>(u.data());
  
  // Setup workspace
  OSQPWorkspace * workspace_ptr;
  const int setup_status = osqp_setup(&workspace_ptr, data_.get(), settings_.get());
  
  work_ = std::unique_ptr<OSQPWorkspace, std::function<void(OSQPWorkspace *)>>(
    workspace_ptr, OSQPWorkspaceDeleter);
  
  work_initialized_ = (setup_status == 0);
  
  if (!work_initialized_) {
    std::cerr << "[OSQPInterface] Failed to setup workspace" << std::endl;
  }
}

OSQPInterface::~OSQPInterface()
{
  // Cleanup is handled by unique_ptr with custom deleter
}

void OSQPInterface::OSQPWorkspaceDeleter(OSQPWorkspace * ptr) noexcept
{
  if (ptr != nullptr) {
    osqp_cleanup(ptr);
  }
}

std::tuple<std::vector<double>, std::vector<double>, int, int, int> 
OSQPInterface::optimize()
{
  if (!work_initialized_) {
    std::cerr << "[OSQPInterface] Workspace not initialized" << std::endl;
    return {{}, {}, -1, -1, -1};
  }
  
  return solve();
}

std::tuple<std::vector<double>, std::vector<double>, int, int, int> 
OSQPInterface::solve()
{
  const int solve_status = osqp_solve(work_.get());
  
  if (solve_status != 0) {
    std::cerr << "[OSQPInterface] Solve failed with status " << solve_status << std::endl;
  }
  
  latest_work_info_ = *(work_->info);
  
  std::vector<double> primal_solution;
  std::vector<double> dual_solution;
  
  if (work_->solution != nullptr) {
    if (work_->solution->x != nullptr) {
      primal_solution.assign(work_->solution->x, work_->solution->x + param_n_);
    }
    if (work_->solution->y != nullptr) {
      dual_solution.assign(work_->solution->y, work_->solution->y + data_->m);
    }
  }
  
  const int status_val = static_cast<int>(latest_work_info_.status_val);
  const int status_polish = static_cast<int>(latest_work_info_.status_polish);
  const int iter = static_cast<int>(latest_work_info_.iter);
  
  return {primal_solution, dual_solution, status_polish, status_val, iter};
}

void OSQPInterface::updateCscP(const CSC_Matrix & P_csc)
{
  if (!work_initialized_) {
    return;
  }
  
  // P_x만 업데이트 (구조는 동일하다고 가정)
  osqp_update_P(
    work_.get(),
    P_csc.m_vals.data(),
    OSQP_NULL,
    static_cast<int>(P_csc.m_vals.size()));
}

void OSQPInterface::updateQ(const std::vector<double> & q)
{
  if (!work_initialized_) {
    return;
  }
  
  osqp_update_lin_cost(work_.get(), q.data());
}

void OSQPInterface::updateCscA(const CSC_Matrix & A_csc)
{
  if (!work_initialized_) {
    return;
  }
  
  // A_x만 업데이트
  osqp_update_A(
    work_.get(),
    A_csc.m_vals.data(),
    OSQP_NULL,
    static_cast<int>(A_csc.m_vals.size()));
}

void OSQPInterface::updateBounds(
  const std::vector<double> & lower_bound,
  const std::vector<double> & upper_bound)
{
  if (!work_initialized_) {
    return;
  }
  
  osqp_update_bounds(work_.get(), lower_bound.data(), upper_bound.data());
}

void OSQPInterface::logUnsolvedStatus(const std::string & prefix) const
{
  std::cerr << prefix << " optimization failed. Status: " 
            << latest_work_info_.status_val << std::endl;
  std::cerr << prefix << " Status message: " 
            << (latest_work_info_.status != nullptr ? latest_work_info_.status : "N/A") 
            << std::endl;
}

#endif  // USE_OSQP

}  // namespace autoware::path_optimizer
