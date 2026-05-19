// Modern C++17 sample for the L-BFGS solver.
//
// Solves the extended Rosenbrock function f(x) = sum_{i even} ((1 - x_i)^2 +
// 100 (x_{i+1} - x_i^2)^2). This demonstrates two styles:
//   * Free-function style with a lambda (preferred / most concise).
//   * Class-based style via lbfgs::Optimizer<double>.

#include <asr_sdm_lbfgs_solver/lbfgs.hpp>

#include <cstdio>
#include <vector>

namespace {

double rosenbrock(const double* x, double* g, int n, double /*step*/) {
    double fx = 0.0;
    for (int i = 0; i < n; i += 2) {
        const double t1 = 1.0 - x[i];
        const double t2 = 10.0 * (x[i + 1] - x[i] * x[i]);
        g[i + 1] = 20.0 * t2;
        g[i]     = -2.0 * (x[i] * g[i + 1] + t1);
        fx += t1 * t1 + t2 * t2;
    }
    return fx;
}

int report(const lbfgs::Progress<double>& p) {
    std::printf("Iter %2d: fx=%.6e xnorm=%.4f gnorm=%.4e step=%.4f ls=%d\n",
                p.k, p.fx, p.xnorm, p.gnorm, p.step, p.ls);
    return 0;
}

class RosenbrockOptimizer final : public lbfgs::Optimizer<double> {
protected:
    double evaluate(const double* x, double* g, int n, double step) override {
        return rosenbrock(x, g, n, step);
    }
    int progress(const lbfgs::Progress<double>& p) override {
        return report(p);
    }
};

} // namespace

int main() {
    constexpr int N = 100;
    std::vector<double> x(N);
    for (int i = 0; i < N; i += 2) {
        x[i]     = -1.2;
        x[i + 1] =  1.0;
    }

    auto print_result = [&](const lbfgs::Result<double>& r) {
        const auto msg = lbfgs::strerror(r.status);
        std::printf("status=%d (%.*s)\n",
                    static_cast<int>(r.status),
                    static_cast<int>(msg.size()), msg.data());
        std::printf("fx=%.6e iters=%d evals=%d x[0]=%.6f x[1]=%.6f\n\n",
                    r.fx, r.iterations, r.evaluations, x[0], x[1]);
    };

    std::printf("=== Free-function / lambda style ===\n");
    {
        // Free-function API. Float is deduced from the vector.
        const auto result = lbfgs::minimize(x, rosenbrock, report);
        print_result(result);
        if (!result) return 1;
    }

    std::printf("=== Optimizer subclass style ===\n");
    {
        for (int i = 0; i < N; i += 2) { x[i] = -1.2; x[i + 1] = 1.0; }

        RosenbrockOptimizer obj;
        lbfgs::Parameters<double> params;  // tweak fields here if desired
        const auto result = obj.run(x, params);
        print_result(result);
        return result ? 0 : 1;
    }
}
