/*
 * C++17 interface for the Levenberg-Marquardt non-linear least squares solver.
 *
 * Wraps levmar 2.6 (Copyright (C) 2004 Manolis Lourakis, GPL-2.0-or-later).
 */

#ifndef ASR_SDM_LEVMAR_SOLVER_LEVMAR_HPP_INCLUDED
#define ASR_SDM_LEVMAR_SOLVER_LEVMAR_HPP_INCLUDED

#if __cplusplus < 201703L
#  error "levmar.hpp requires a C++17 compiler"
#endif

#include <cstddef>
#include <functional>
#include <string_view>
#include <type_traits>
#include <utility>
#include <vector>

/**
 * \file levmar.hpp
 * \brief Modern C++17 interface for the Levenberg-Marquardt solver.
 *
 * Example:
 * \code
 *     #include <asr_sdm_levmar_solver/levmar.hpp>
 *
 *     std::vector<double> p{0.0, 0.0};
 *     const std::vector<double> target(2, 0.0);
 *     auto result = levmar::minimize(
 *         p, target,
 *         [](const double* p, double* hx, int m, int n) {
 *             for (int i = 0; i < n; ++i) {
 *                 hx[i] = (1.0 - p[0]) * (1.0 - p[0])
 *                       + 100.0 * (p[1] - p[0] * p[0]) * (p[1] - p[0] * p[0]);
 *             }
 *         },
 *         [](const double* p, double* jac, int m, int n) {
 *             for (int i = 0, j = 0; i < n; ++i) {
 *                 jac[j++] = -2.0 + 2.0 * p[0] - 400.0 * (p[1] - p[0] * p[0]) * p[0];
 *                 jac[j++] =  200.0 * (p[1] - p[0] * p[0]);
 *             }
 *         });
 *     if (result) {
 *         // result.status, result.final_error, result.iterations
 *     }
 * \endcode
 */
namespace levmar {

/// Termination reason reported in \ref Result::stop_reason (maps to levmar info[6]).
enum class StopReason : int {
    SmallGradient   = 1,  ///< ||J^T e||_inf below epsilon1.
    SmallStep       = 2,  ///< Relative change in parameters below epsilon2.
    MaxIterations   = 3,  ///< Reached \ref Parameters::max_iterations.
    Singular        = 4,  ///< Step norm indicates an almost-singular system.
    MuOverflow      = 5,  ///< Damping factor overflowed.
    SmallError      = 6,  ///< ||e||_2 below epsilon3.
    NonFinite       = 7,  ///< Non-finite residual encountered.
    Unknown         = 0,
};

/// Return status of \ref minimize(). Non-negative values denote success.
enum class Status : int {
    Success = 0,
    Error   = -1,
};

/// Solver options (maps to levmar opts[]).
template <typename Float>
struct Parameters {
    static_assert(std::is_floating_point_v<Float>,
                  "levmar::Parameters requires a floating-point type");

    Float mu            = static_cast<Float>(1e-3);   ///< Initial damping scale (opts[0]).
    Float epsilon1      = static_cast<Float>(1e-17);  ///< Gradient tolerance (opts[1]).
    Float epsilon2      = static_cast<Float>(1e-17);  ///< Relative step tolerance (opts[2]).
    Float epsilon3      = static_cast<Float>(1e-17);  ///< Residual tolerance (opts[3]).
    Float diff_delta    = static_cast<Float>(1e-6);   ///< Finite-difference step (opts[4], dif only).
    int   max_iterations = 100;
};

/// Result returned from \ref minimize().
template <typename Float>
struct Result {
    Status     status           = Status::Error;
    StopReason stop_reason      = StopReason::Unknown;
    Float      initial_error    = static_cast<Float>(0);  ///< ||e||_2 at start (info[0]).
    Float      final_error      = static_cast<Float>(0);  ///< ||e||_2 at end   (info[1]).
    Float      gradient_inf_norm = static_cast<Float>(0); ///< ||J^T e||_inf    (info[2]).
    Float      step_norm        = static_cast<Float>(0);  ///< ||Dp||_2         (info[3]).
    Float      mu_over_max_diag = static_cast<Float>(0); ///< mu/max(diag J^T J) (info[4]).
    int        iterations       = 0;                      ///< info[5]
    int        function_evals   = 0;                      ///< info[7]
    int        jacobian_evals   = 0;                      ///< info[8]
    int        linear_solves    = 0;                      ///< info[9]

    [[nodiscard]] constexpr bool ok() const noexcept {
        return status == Status::Success;
    }
    explicit constexpr operator bool() const noexcept { return ok(); }
};

/// Returns a human-readable description of a \ref StopReason code.
[[nodiscard]] std::string_view strerror(StopReason reason) noexcept;

namespace detail {

template <typename Float>
using ResidualFn = std::function<void(const Float* p, Float* hx, int m, int n)>;

template <typename Float>
using JacobianFn = std::function<void(const Float* p, Float* jac, int m, int n)>;

template <typename Float>
Result<Float> minimize_der_impl(int m,
                                int n,
                                Float* p,
                                const Float* measurements,
                                const ResidualFn<Float>& residual,
                                const JacobianFn<Float>& jacobian,
                                const Parameters<Float>& params);

template <typename Float>
Result<Float> minimize_dif_impl(int m,
                                int n,
                                Float* p,
                                const Float* measurements,
                                const ResidualFn<Float>& residual,
                                const Parameters<Float>& params);

template <typename T>
inline constexpr bool is_nullptr_v = std::is_same_v<std::decay_t<T>, std::nullptr_t>;

} // namespace detail

/// Minimize with an analytic Jacobian (\c dlevmar_der / \c slevmar_der).
template <typename Float, typename Residual, typename Jacobian>
[[nodiscard]] Result<Float> minimize(int m,
                                     int n,
                                     Float* p,
                                     const Float* measurements,
                                     Residual&& residual,
                                     Jacobian&& jacobian,
                                     const Parameters<Float>& params = {}) {
    static_assert(std::is_floating_point_v<Float>,
                  "levmar::minimize requires a floating-point type");
    static_assert(std::is_invocable_v<Residual, const Float*, Float*, int, int>,
                  "residual must be callable as: void(const Float*, Float*, int m, int n)");
    static_assert(std::is_invocable_v<Jacobian, const Float*, Float*, int, int>,
                  "jacobian must be callable as: void(const Float*, Float*, int m, int n)");

    detail::ResidualFn<Float> res = std::forward<Residual>(residual);
    detail::JacobianFn<Float> jac = std::forward<Jacobian>(jacobian);
    return detail::minimize_der_impl(m, n, p, measurements, res, jac, params);
}

/// Minimize with a finite-difference Jacobian (\c dlevmar_dif / \c slevmar_dif).
template <typename Float, typename Residual>
[[nodiscard]] Result<Float> minimize_dif(int m,
                                         int n,
                                         Float* p,
                                         const Float* measurements,
                                         Residual&& residual,
                                         const Parameters<Float>& params = {}) {
    static_assert(std::is_floating_point_v<Float>,
                  "levmar::minimize_dif requires a floating-point type");
    static_assert(std::is_invocable_v<Residual, const Float*, Float*, int, int>,
                  "residual must be callable as: void(const Float*, Float*, int m, int n)");

    detail::ResidualFn<Float> res = std::forward<Residual>(residual);
    return detail::minimize_dif_impl(m, n, p, measurements, res, params);
}

/// \c std::vector overload with analytic Jacobian.
template <typename Float, typename Residual, typename Jacobian>
[[nodiscard]] Result<Float> minimize(std::vector<Float>& p,
                                     const std::vector<Float>& measurements,
                                     Residual&& residual,
                                     Jacobian&& jacobian,
                                     const Parameters<Float>& params = {}) {
    return minimize<Float>(static_cast<int>(p.size()),
                           static_cast<int>(measurements.size()),
                           p.data(),
                           measurements.data(),
                           std::forward<Residual>(residual),
                           std::forward<Jacobian>(jacobian),
                           params);
}

/// \c std::vector overload with finite-difference Jacobian.
template <typename Float, typename Residual>
[[nodiscard]] Result<Float> minimize_dif(std::vector<Float>& p,
                                         const std::vector<Float>& measurements,
                                         Residual&& residual,
                                         const Parameters<Float>& params = {}) {
    return minimize_dif<Float>(static_cast<int>(p.size()),
                               static_cast<int>(measurements.size()),
                               p.data(),
                               measurements.data(),
                               std::forward<Residual>(residual),
                               params);
}

/// Convenience base class for least-squares problems with an analytic Jacobian.
template <typename Float>
class Problem {
public:
    static_assert(std::is_floating_point_v<Float>,
                  "levmar::Problem requires a floating-point type");

    Problem() = default;
    virtual ~Problem() = default;

    Problem(const Problem&) = delete;
    Problem& operator=(const Problem&) = delete;
    Problem(Problem&&) = delete;
    Problem& operator=(Problem&&) = delete;

    [[nodiscard]] Result<Float> run(int m,
                                    int n,
                                    Float* p,
                                    const Float* measurements,
                                    const Parameters<Float>& params = {}) {
        return minimize<Float>(
            m, n, p, measurements,
            [this](const Float* pv, Float* hx, int mv, int nv) {
                this->residual(pv, hx, mv, nv);
            },
            [this](const Float* pv, Float* jac, int mv, int nv) {
                this->jacobian(pv, jac, mv, nv);
            },
            params);
    }

    [[nodiscard]] Result<Float> run(std::vector<Float>& p,
                                    const std::vector<Float>& measurements,
                                    const Parameters<Float>& params = {}) {
        return run(static_cast<int>(p.size()),
                   static_cast<int>(measurements.size()),
                   p.data(),
                   measurements.data(),
                   params);
    }

protected:
    virtual void residual(const Float* p, Float* hx, int m, int n) = 0;
    virtual void jacobian(const Float* p, Float* jac, int m, int n) = 0;
};

extern template Result<double> detail::minimize_der_impl<double>(
    int, int, double*, const double*,
    const detail::ResidualFn<double>&,
    const detail::JacobianFn<double>&,
    const Parameters<double>&);
extern template Result<double> detail::minimize_dif_impl<double>(
    int, int, double*, const double*,
    const detail::ResidualFn<double>&,
    const Parameters<double>&);
extern template Result<float> detail::minimize_dif_impl<float>(
    int, int, float*, const float*,
    const detail::ResidualFn<float>&,
    const Parameters<float>&);

} // namespace levmar

#endif // ASR_SDM_LEVMAR_SOLVER_LEVMAR_HPP_INCLUDED
