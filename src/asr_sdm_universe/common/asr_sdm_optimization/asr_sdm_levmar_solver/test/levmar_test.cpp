// Rosenbrock least-squares demo for the Levenberg-Marquardt solver.
//
// Minimizes sum_i ((1 - p0)^2 + 100 (p1 - p0^2)^2) replicated n times.

#include <asr_sdm_levmar_solver/levmar.hpp>

#include <cstdio>
#include <vector>

namespace {

constexpr double ROSD = 105.0;

void rosenbrock_residual(const double* p, double* hx, int /*m*/, int n) {
    for (int i = 0; i < n; ++i) {
        hx[i] = (1.0 - p[0]) * (1.0 - p[0])
              + ROSD * (p[1] - p[0] * p[0]) * (p[1] - p[0] * p[0]);
    }
}

void rosenbrock_jacobian(const double* p, double* jac, int /*m*/, int n) {
    for (int i = 0, j = 0; i < n; ++i) {
        jac[j++] = -2.0 + 2.0 * p[0] - 4.0 * ROSD * (p[1] - p[0] * p[0]) * p[0];
        jac[j++] =  2.0 * ROSD * (p[1] - p[0] * p[0]);
    }
}

class RosenbrockProblem final : public levmar::Problem<double> {
protected:
    void residual(const double* p, double* hx, int m, int n) override {
        rosenbrock_residual(p, hx, m, n);
    }
    void jacobian(const double* p, double* jac, int m, int n) override {
        rosenbrock_jacobian(p, jac, m, n);
    }
};

void print_result(const levmar::Result<double>& r, const std::vector<double>& p) {
    const auto msg = levmar::strerror(r.stop_reason);
    std::printf("status=%d stop=%d (%.*s)\n",
                static_cast<int>(r.status),
                static_cast<int>(r.stop_reason),
                static_cast<int>(msg.size()), msg.data());
    std::printf("error: %.6e -> %.6e  iters=%d  fevals=%d  jevals=%d\n",
                r.initial_error, r.final_error,
                r.iterations, r.function_evals, r.jacobian_evals);
    std::printf("solution: p0=%.6f p1=%.6f\n\n", p[0], p[1]);
}

} // namespace

int main() {
    constexpr int n = 2;
    const std::vector<double> target(n, 0.0);

    levmar::Parameters<double> params;
    params.max_iterations = 1000;

    std::printf("=== Free-function / lambda style ===\n");
    {
        std::vector<double> p{-1.2, 1.0};
        const auto result = levmar::minimize(
            p, target, rosenbrock_residual, rosenbrock_jacobian, params);
        print_result(result, p);
        if (!result) return 1;
    }

    std::printf("=== Problem subclass style ===\n");
    {
        std::vector<double> p{-1.2, 1.0};
        RosenbrockProblem problem;
        const auto result = problem.run(p, target, params);
        print_result(result, p);
        return result ? 0 : 1;
    }
}
