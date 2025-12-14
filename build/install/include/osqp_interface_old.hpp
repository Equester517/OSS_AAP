#ifndef PATH_OPTIMIZER_OSQP_INTERFACE_HPP_
#define PATH_OPTIMIZER_OSQP_INTERFACE_HPP_

#ifdef USE_OSQP
#include <osqp/osqp.h>
#endif

#include <Eigen/Core>
#include <Eigen/Sparse>

#include <memory>
#include <tuple>
#include <vector>
#include <functional>
#include <string>

namespace autoware::path_optimizer
{

constexpr double INF = 1e30;

// CSC (Compressed Sparse Column) Matrix 구조체
struct CSC_Matrix
{
  std::vector<double> m_vals;
  std::vector<int> m_row_idxs;
  std::vector<int> m_col_idxs;
};

// Eigen Matrix를 CSC 형식으로 변환
CSC_Matrix calCSCMatrix(const Eigen::MatrixXd & mat);
CSC_Matrix calCSCMatrixTrapezoidal(const Eigen::MatrixXd & mat);

#ifdef USE_OSQP

class OSQPInterface
{
public:
  explicit OSQPInterface(const double eps_abs = 1e-6);
  
  OSQPInterface(
    const CSC_Matrix & P,
    const CSC_Matrix & A,
    const std::vector<double> & q,
    const std::vector<double> & l,
    const std::vector<double> & u,
    const double eps_abs = 1e-6);
  
  ~OSQPInterface();

  // QP 문제 최적화
  std::tuple<std::vector<double>, std::vector<double>, int, int, int> optimize();
  
  // 문제 업데이트 (Warm start용)
  void updateCscP(const CSC_Matrix & P_csc);
  void updateQ(const std::vector<double> & q);
  void updateCscA(const CSC_Matrix & A_csc);
  void updateBounds(
    const std::vector<double> & lower_bound,
    const std::vector<double> & upper_bound);

  void logUnsolvedStatus(const std::string & prefix) const;

private:
  std::unique_ptr<OSQPWorkspace, std::function<void(OSQPWorkspace *)>> work_;
  std::unique_ptr<OSQPSettings> settings_;
  std::unique_ptr<OSQPData> data_;
  OSQPInfo latest_work_info_;
  
  int64_t param_n_;
  bool work_initialized_;
  int64_t exitflag_;

  std::tuple<std::vector<double>, std::vector<double>, int, int, int> solve();
  
  static void OSQPWorkspaceDeleter(OSQPWorkspace * ptr) noexcept;
};

#endif  // USE_OSQP

}  // namespace autoware::path_optimizer

#endif  // PATH_OPTIMIZER_OSQP_INTERFACE_HPP_
