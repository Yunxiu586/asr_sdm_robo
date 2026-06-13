/*
 * C++17 wrapper for levmar 2.6.
 *
 * Copyright (C) 2004 Manolis Lourakis (levmar library, GPL-2.0-or-later).
 */

#include <asr_sdm_levmar_solver/levmar.hpp>

extern "C" {
#include <levmar.h>
}

#include <cstring>
#include <type_traits>
#include <vector>

namespace levmar {

std::string_view strerror(const StopReason reason) noexcept {
    switch (reason) {
        case StopReason::SmallGradient: return "stopped by small gradient ||J^T e||_inf";
        case StopReason::SmallStep:     return "stopped by small relative parameter step";
        case StopReason::MaxIterations: return "reached maximum iterations";
        case StopReason::Singular:      return "almost singular linear system";
        case StopReason::MuOverflow:    return "damping factor overflow";
        case StopReason::SmallError:    return "stopped by small residual ||e||_2";
        case StopReason::NonFinite:     return "non-finite residual encountered";
        default:                        return "unknown stop reason";
    }
}

namespace detail {

namespace {

struct CallbackData {
    const void* residual;
    const void* jacobian;
};

void c_residual_double(double* p, double* hx, int m, int n, void* adata) {
    const auto& fn = *static_cast<const ResidualFn<double>*>(
        static_cast<CallbackData*>(adata)->residual);
    fn(p, hx, m, n);
}

void c_jacobian_double(double* p, double* jac, int m, int n, void* adata) {
    const auto& fn = *static_cast<const JacobianFn<double>*>(
        static_cast<CallbackData*>(adata)->jacobian);
    fn(p, jac, m, n);
}

void c_residual_float(float* p, float* hx, int m, int n, void* adata) {
    const auto& fn = *static_cast<const ResidualFn<float>*>(
        static_cast<CallbackData*>(adata)->residual);
    fn(p, hx, m, n);
}

template <typename Float>
Result<Float> make_result(const int ret, const Float info[LM_INFO_SZ]) {
    Result<Float> result;
    result.status = (ret >= 0) ? Status::Success : Status::Error;
    result.stop_reason = static_cast<StopReason>(static_cast<int>(info[6]));
    result.initial_error = info[0];
    result.final_error = info[1];
    result.gradient_inf_norm = info[2];
    result.step_norm = info[3];
    result.mu_over_max_diag = info[4];
    result.iterations = static_cast<int>(info[5]);
    result.function_evals = static_cast<int>(info[7]);
    result.jacobian_evals = static_cast<int>(info[8]);
    result.linear_solves = static_cast<int>(info[9]);
    return result;
}

} // namespace

template <typename Float>
Result<Float> minimize_der_impl(const int m,
                                const int n,
                                Float* const p,
                                const Float* const measurements,
                                const ResidualFn<Float>& residual,
                                const JacobianFn<Float>& jacobian,
                                const Parameters<Float>& params) {
    static_assert(std::is_same_v<Float, double>,
                  "analytic Jacobian solver is only available for double");

    double opts[LM_OPTS_SZ] = {
        params.mu,
        params.epsilon1,
        params.epsilon2,
        params.epsilon3,
        0.0,
    };

    double info[LM_INFO_SZ] = {};
    const int worksz = LM_DER_WORKSZ(m, n);
    std::vector<double> work(static_cast<std::size_t>(worksz));
    std::vector<double> x(n);
    std::memcpy(x.data(), measurements, static_cast<std::size_t>(n) * sizeof(double));

    CallbackData adata{&residual, &jacobian};
    const int ret = dlevmar_der(
        c_residual_double,
        c_jacobian_double,
        p,
        x.data(),
        m,
        n,
        params.max_iterations,
        opts,
        info,
        work.data(),
        nullptr,
        &adata);

    return make_result(ret, info);
}

template <typename Float>
Result<Float> minimize_dif_impl(const int m,
                                const int n,
                                Float* const p,
                                const Float* const measurements,
                                const ResidualFn<Float>& residual,
                                const Parameters<Float>& params) {
    if constexpr (std::is_same_v<Float, double>) {
        double opts[LM_OPTS_SZ] = {
            params.mu,
            params.epsilon1,
            params.epsilon2,
            params.epsilon3,
            params.diff_delta,
        };

        double info[LM_INFO_SZ] = {};
        const int worksz = LM_DIF_WORKSZ(m, n);
        std::vector<double> work(static_cast<std::size_t>(worksz));
        std::vector<double> x(n);
        std::memcpy(x.data(), measurements, static_cast<std::size_t>(n) * sizeof(double));

        CallbackData adata{&residual, nullptr};
        const int ret = dlevmar_dif(
            c_residual_double,
            p,
            x.data(),
            m,
            n,
            params.max_iterations,
            opts,
            info,
            work.data(),
            nullptr,
            &adata);

        return make_result(ret, info);
    } else if constexpr (std::is_same_v<Float, float>) {
        float opts[LM_OPTS_SZ] = {
            params.mu,
            params.epsilon1,
            params.epsilon2,
            params.epsilon3,
            params.diff_delta,
        };

        float info[LM_INFO_SZ] = {};
        const int worksz = LM_DIF_WORKSZ(m, n);
        std::vector<float> work(static_cast<std::size_t>(worksz));
        std::vector<float> x(n);
        std::memcpy(x.data(), measurements, static_cast<std::size_t>(n) * sizeof(float));

        CallbackData adata{&residual, nullptr};
        const int ret = slevmar_dif(
            c_residual_float,
            p,
            x.data(),
            m,
            n,
            params.max_iterations,
            opts,
            info,
            work.data(),
            nullptr,
            &adata);

        return make_result(ret, info);
    } else {
        static_assert(sizeof(Float) == 0, "unsupported floating-point type");
    }
}

template Result<double> minimize_der_impl<double>(
    int, int, double*, const double*,
    const ResidualFn<double>&,
    const JacobianFn<double>&,
    const Parameters<double>&);

template Result<double> minimize_dif_impl<double>(
    int, int, double*, const double*,
    const ResidualFn<double>&,
    const Parameters<double>&);

template Result<float> minimize_dif_impl<float>(
    int, int, float*, const float*,
    const ResidualFn<float>&,
    const Parameters<float>&);

} // namespace detail
} // namespace levmar
