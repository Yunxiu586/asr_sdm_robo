/*
 *      C++17 library of Limited memory BFGS (L-BFGS).
 *
 * Copyright (c) 1990, Jorge Nocedal
 * Copyright (c) 2007-2010 Naoaki Okazaki
 * Copyright (c) 2024      C++17 port
 * All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 */

#ifndef LBFGS_HPP_INCLUDED
#define LBFGS_HPP_INCLUDED

#if __cplusplus < 201703L
#  error "lbfgs.hpp requires a C++17 compiler"
#endif

#include <cstddef>
#include <functional>
#include <string_view>
#include <type_traits>
#include <utility>
#include <vector>

/**
 * \file lbfgs.hpp
 * \brief Modern C++17 interface for the L-BFGS / OWL-QN solver.
 *
 * Example:
 * \code
 *     #include <asr_sdm_lbfgs_solver/lbfgs.hpp>
 *
 *     std::vector<double> x(100, 1.0);
 *     auto result = lbfgs::minimize(x, [](const double* x, double* g, int n, double) {
 *         double fx = 0;
 *         for (int i = 0; i < n; ++i) { g[i] = 2 * x[i]; fx += x[i] * x[i]; }
 *         return fx;
 *     });
 *     if (result) {
 *         // result.status, result.fx, result.iterations
 *     }
 * \endcode
 */
namespace lbfgs {

/// Return status of \ref minimize().
///
/// Non-negative values denote success-ish termination (the solver returned a
/// usable answer). Negative values denote errors.
enum class Status : int {
    Success           = 0,  ///< Reached convergence (gradient tolerance).
    Convergence       = 0,  ///< Alias of \c Success.
    Stop              = 1,  ///< Reached convergence (delta tolerance).
    AlreadyMinimized  = 2,  ///< The initial variables already minimize the objective.

    UnknownError              = -1024,
    LogicError                = -1023,
    OutOfMemory               = -1022,
    Canceled                  = -1021,
    InvalidN                  = -1020,
    InvalidNSse               = -1019,
    InvalidXSse               = -1018,
    InvalidEpsilon            = -1017,
    InvalidTestPeriod         = -1016,
    InvalidDelta              = -1015,
    InvalidLineSearch         = -1014,
    InvalidMinStep            = -1013,
    InvalidMaxStep            = -1012,
    InvalidFtol               = -1011,
    InvalidWolfe              = -1010,
    InvalidGtol               = -1009,
    InvalidXtol               = -1008,
    InvalidMaxLineSearch      = -1007,
    InvalidOrthantwise        = -1006,
    InvalidOrthantwiseStart   = -1005,
    InvalidOrthantwiseEnd     = -1004,
    OutOfInterval             = -1003,
    IncorrectTminMax          = -1002,
    RoundingError             = -1001,
    MinimumStep               = -1000,
    MaximumStep               = -999,
    MaximumLineSearch         = -998,
    MaximumIteration          = -997,
    WidthTooSmall             = -996,
    InvalidParameters         = -995,
    IncreaseGradient          = -994,
};

/// Line search algorithm selection.
enum class LineSearch : int {
    Default                 = 0,
    MoreThuente             = 0,
    BacktrackingArmijo      = 1,
    Backtracking            = 2,
    BacktrackingWolfe       = 2,
    BacktrackingStrongWolfe = 3,
};

/// Optimization parameters. Default-constructed instances hold the library
/// defaults; modify any field as needed before calling \ref minimize().
template <typename Float>
struct Parameters {
    static_assert(std::is_floating_point_v<Float>,
                  "lbfgs::Parameters requires a floating-point type");

    int        m                 = 6;
    Float      epsilon           = static_cast<Float>(1e-5);
    int        past              = 0;
    Float      delta             = static_cast<Float>(1e-5);
    int        max_iterations    = 0;
    LineSearch linesearch        = LineSearch::Default;
    int        max_linesearch    = 40;
    Float      min_step          = static_cast<Float>(1e-20);
    Float      max_step          = static_cast<Float>(1e20);
    Float      ftol              = static_cast<Float>(1e-4);
    Float      wolfe             = static_cast<Float>(0.9);
    Float      gtol              = static_cast<Float>(0.9);
    Float      xtol              = static_cast<Float>(1.0e-16);
    Float      orthantwise_c     = static_cast<Float>(0.0);
    int        orthantwise_start = 0;
    /// Use \c -1 to mean the full vector (n).
    int        orthantwise_end   = -1;
};

/// Per-iteration progress information passed to the optional callback.
template <typename Float>
struct Progress {
    const Float* x;     ///< Current variables.
    const Float* g;     ///< Current gradient (pseudo-gradient under OWL-QN).
    Float        fx;    ///< Current objective value (including L1 term under OWL-QN).
    Float        xnorm; ///< Euclidean norm of \c x.
    Float        gnorm; ///< Euclidean norm of \c g.
    Float        step;  ///< Line-search step used.
    int          n;     ///< Number of variables.
    int          k;     ///< Iteration count (starts at 1).
    int          ls;    ///< Number of function evaluations used this iteration.
};

/// Result returned from \ref minimize().
template <typename Float>
struct Result {
    Status status      = Status::UnknownError;
    Float  fx          = static_cast<Float>(0);
    int    iterations  = 0;
    int    evaluations = 0;

    [[nodiscard]] constexpr bool ok() const noexcept {
        return static_cast<int>(status) >= 0;
    }
    explicit constexpr operator bool() const noexcept { return ok(); }
};

/// Returns a human-readable description of a \ref Status code.
/// Returned view is backed by a static string with infinite lifetime.
[[nodiscard]] std::string_view strerror(Status status) noexcept;

namespace detail {

/// Internal type-erased signature for the evaluate callback.
template <typename Float>
using EvaluateFn = std::function<Float(const Float* x, Float* g, int n, Float step)>;

/// Internal type-erased signature for the progress callback.
template <typename Float>
using ProgressFn = std::function<int(const Progress<Float>&)>;

/// Underlying solver. Defined for \c float and \c double via explicit
/// instantiation in lbfgs.cpp. Users should use the templated \ref minimize
/// overloads below instead of calling this directly.
template <typename Float>
Result<Float> minimize_impl(int n,
                            Float* x,
                            const EvaluateFn<Float>& evaluate,
                            const ProgressFn<Float>& progress,
                            const Parameters<Float>& params);

template <typename T>
inline constexpr bool is_nullptr_v = std::is_same_v<std::decay_t<T>, std::nullptr_t>;

} // namespace detail

// ---------------------------------------------------------------------------
// Public templated API.
//
// These overloads accept any callable shaped like:
//
//     Float evaluate(const Float* x, Float* g, int n, Float step);
//     int   progress(const Progress<Float>& p);  // optional, return non-zero to cancel
//
// The Float type is deduced from the variable buffer (Float* or
// std::vector<Float>&); you do not need to spell it out explicitly.
// ---------------------------------------------------------------------------

/// Raw-pointer overload with optional progress callback.
template <typename Float, typename Evaluate, typename Progress = std::nullptr_t>
[[nodiscard]] Result<Float> minimize(
    int n,
    Float* x,
    Evaluate&& evaluate,
    Progress&& progress = nullptr,
    const Parameters<Float>& params = {}) {
    static_assert(std::is_floating_point_v<Float>,
                  "lbfgs::minimize requires a floating-point type");
    static_assert(std::is_invocable_r_v<Float, Evaluate&, const Float*, Float*, int, Float>,
                  "evaluate must be callable as: Float(const Float*, Float*, int, Float)");

    detail::EvaluateFn<Float> ev = std::forward<Evaluate>(evaluate);
    detail::ProgressFn<Float> pr;
    if constexpr (!detail::is_nullptr_v<Progress>) {
        static_assert(
            std::is_invocable_r_v<int, Progress&, const lbfgs::Progress<Float>&>,
            "progress must be callable as: int(const lbfgs::Progress<Float>&)");
        pr = std::forward<Progress>(progress);
    }
    return detail::minimize_impl<Float>(n, x, ev, pr, params);
}

/// \c std::vector overload (deduces \c Float, sizes from \c x.size()).
template <typename Float, typename Evaluate, typename Progress = std::nullptr_t>
[[nodiscard]] Result<Float> minimize(
    std::vector<Float>& x,
    Evaluate&& evaluate,
    Progress&& progress = nullptr,
    const Parameters<Float>& params = {}) {
    return minimize<Float>(static_cast<int>(x.size()),
                           x.data(),
                           std::forward<Evaluate>(evaluate),
                           std::forward<Progress>(progress),
                           params);
}

/// Convenience base class for objective-function objects.
/// Override \ref evaluate (and optionally \ref progress).
template <typename Float>
class Optimizer {
public:
    static_assert(std::is_floating_point_v<Float>,
                  "lbfgs::Optimizer requires a floating-point type");

    Optimizer() = default;
    virtual ~Optimizer() = default;

    Optimizer(const Optimizer&) = delete;
    Optimizer& operator=(const Optimizer&) = delete;
    Optimizer(Optimizer&&) = delete;
    Optimizer& operator=(Optimizer&&) = delete;

    [[nodiscard]] Result<Float> run(int n,
                                    Float* x,
                                    const Parameters<Float>& params = {}) {
        return minimize<Float>(
            n, x,
            [this](const Float* xv, Float* g, int nv, Float step) {
                return this->evaluate(xv, g, nv, step);
            },
            [this](const Progress<Float>& p) { return this->progress(p); },
            params);
    }

    [[nodiscard]] Result<Float> run(std::vector<Float>& x,
                                    const Parameters<Float>& params = {}) {
        return run(static_cast<int>(x.size()), x.data(), params);
    }

protected:
    /// Compute objective value and gradient at \c x. Must be overridden.
    virtual Float evaluate(const Float* x, Float* g, int n, Float step) = 0;

    /// Optional progress reporter. Return non-zero to cancel.
    virtual int progress(const Progress<Float>& /*p*/) { return 0; }
};

// Explicit instantiations for the underlying solver are provided in lbfgs.cpp.
extern template Result<float>  detail::minimize_impl<float>(
    int, float*, const detail::EvaluateFn<float>&,
    const detail::ProgressFn<float>&, const Parameters<float>&);
extern template Result<double> detail::minimize_impl<double>(
    int, double*, const detail::EvaluateFn<double>&,
    const detail::ProgressFn<double>&, const Parameters<double>&);

} // namespace lbfgs

#endif // LBFGS_HPP_INCLUDED
