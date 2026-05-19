/*
 *      C++17 port of Limited memory BFGS (L-BFGS).
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

#include <asr_sdm_lbfgs_solver/lbfgs.hpp>

#include <algorithm>
#include <cmath>
#include <vector>

namespace lbfgs::detail {

// ---------------------------------------------------------------------------
// Math helpers
// ---------------------------------------------------------------------------

template <typename Float>
[[nodiscard]] inline bool fsigndiff(Float a, Float b) noexcept {
    return std::signbit(a) != std::signbit(b);
}

// Vector helpers (mirroring the original arithmetic_ansi.h so numerical
// behaviour matches the reference C implementation exactly).

template <typename Float>
inline void veccpy(Float* y, const Float* x, int n) noexcept {
    for (int i = 0; i < n; ++i) y[i] = x[i];
}

template <typename Float>
inline void vecncpy(Float* y, const Float* x, int n) noexcept {
    for (int i = 0; i < n; ++i) y[i] = -x[i];
}

template <typename Float>
inline void vecadd(Float* y, const Float* x, Float c, int n) noexcept {
    for (int i = 0; i < n; ++i) y[i] += c * x[i];
}

template <typename Float>
inline void vecdiff(Float* z, const Float* x, const Float* y, int n) noexcept {
    for (int i = 0; i < n; ++i) z[i] = x[i] - y[i];
}

template <typename Float>
inline void vecscale(Float* y, Float c, int n) noexcept {
    for (int i = 0; i < n; ++i) y[i] *= c;
}

template <typename Float>
[[nodiscard]] inline Float vecdot(const Float* x, const Float* y, int n) noexcept {
    Float s = Float(0);
    for (int i = 0; i < n; ++i) s += x[i] * y[i];
    return s;
}

template <typename Float>
[[nodiscard]] inline Float vec2norm(const Float* x, int n) noexcept {
    return std::sqrt(vecdot(x, x, n));
}

// ---------------------------------------------------------------------------
// OWL-QN helpers
// ---------------------------------------------------------------------------

template <typename Float>
[[nodiscard]] Float owlqn_x1norm(const Float* x, int start, int end) noexcept {
    Float norm = Float(0);
    for (int i = start; i < end; ++i) norm += std::fabs(x[i]);
    return norm;
}

template <typename Float>
void owlqn_pseudo_gradient(Float* pg,
                           const Float* x,
                           const Float* g,
                           int n,
                           Float c,
                           int start,
                           int end) noexcept {
    for (int i = 0; i < start; ++i) pg[i] = g[i];
    for (int i = start; i < end; ++i) {
        if (x[i] < Float(0)) {
            pg[i] = g[i] - c;
        } else if (x[i] > Float(0)) {
            pg[i] = g[i] + c;
        } else if (g[i] < -c) {
            pg[i] = g[i] + c;
        } else if (g[i] >  c) {
            pg[i] = g[i] - c;
        } else {
            pg[i] = Float(0);
        }
    }
    for (int i = end; i < n; ++i) pg[i] = g[i];
}

template <typename Float>
void owlqn_project(Float* d, const Float* sign, int start, int end) noexcept {
    for (int i = start; i < end; ++i) {
        if (d[i] * sign[i] <= Float(0)) d[i] = Float(0);
    }
}

// ---------------------------------------------------------------------------
// Internal state passed around line-search routines
// ---------------------------------------------------------------------------

template <typename Float>
struct CallbackData {
    int n = 0;
    const EvaluateFn<Float>* evaluate = nullptr;
    int evaluations = 0; ///< Running total of objective evaluations.

    Float invoke(const Float* x, Float* g, Float step) {
        ++evaluations;
        return (*evaluate)(x, g, n, step);
    }
};

template <typename Float>
struct IterationData {
    Float alpha = Float(0);
    std::vector<Float> s;
    std::vector<Float> y;
    Float ys = Float(0);
};

// ---------------------------------------------------------------------------
// Cubic / quadratic minimizers used by the More-Thuente line search.
// These mirror the CUBIC_MINIMIZER / QUAD_MINIMIZER macros in the C version.
// ---------------------------------------------------------------------------

template <typename Float>
[[nodiscard]] inline Float cubic_minimizer(Float u, Float fu, Float du,
                                           Float v, Float fv, Float dv) noexcept {
    const Float d = v - u;
    Float theta   = (fu - fv) * Float(3) / d + du + dv;
    const Float s = std::max({std::fabs(theta), std::fabs(du), std::fabs(dv)});
    const Float a = theta / s;
    Float gamma   = s * std::sqrt(a * a - (du / s) * (dv / s));
    if (v < u) gamma = -gamma;
    const Float p = gamma - du + theta;
    const Float q = gamma - du + gamma + dv;
    const Float r = p / q;
    return u + r * d;
}

template <typename Float>
[[nodiscard]] inline Float cubic_minimizer2(Float u, Float fu, Float du,
                                            Float v, Float fv, Float dv,
                                            Float xmin, Float xmax) noexcept {
    const Float d = v - u;
    Float theta   = (fu - fv) * Float(3) / d + du + dv;
    const Float s = std::max({std::fabs(theta), std::fabs(du), std::fabs(dv)});
    const Float a = theta / s;
    Float gamma   = s * std::sqrt(std::max(Float(0), a * a - (du / s) * (dv / s)));
    if (u < v) gamma = -gamma;
    const Float p = gamma - dv + theta;
    const Float q = gamma - dv + gamma + du;
    const Float r = p / q;
    if (r < Float(0) && gamma != Float(0)) return v - r * d;
    return (d > Float(0)) ? xmax : xmin;
}

template <typename Float>
[[nodiscard]] inline Float quad_minimizer(Float u, Float fu, Float du,
                                          Float v, Float fv) noexcept {
    const Float a = v - u;
    return u + du / ((fu - fv) / a + du) / Float(2) * a;
}

template <typename Float>
[[nodiscard]] inline Float quad_minimizer2(Float u, Float du,
                                           Float v, Float dv) noexcept {
    const Float a = u - v;
    return v + dv / (dv - du) * a;
}

// ---------------------------------------------------------------------------
// Safeguarded trial-value update for More-Thuente line search.
// See More & Thuente, ACM TOMS 20(3):286-307, 1994.
// ---------------------------------------------------------------------------

template <typename Float>
Status update_trial_interval(Float& x, Float& fx, Float& dx,
                             Float& y, Float& fy, Float& dy,
                             Float& t, Float& ft, Float& dt,
                             Float tmin, Float tmax,
                             bool& brackt) noexcept {
    const bool dsign = fsigndiff(dt, dx);
    Float newt;
    bool bound = false;

    if (brackt) {
        if (t <= std::min(x, y) || std::max(x, y) <= t)
            return Status::OutOfInterval;
        if (Float(0) <= dx * (t - x))
            return Status::IncreaseGradient;
        if (tmax < tmin)
            return Status::IncorrectTminMax;
    }

    if (fx < ft) {
        // Case 1: higher function value.
        brackt = true;
        bound  = true;
        const Float mc = cubic_minimizer(x, fx, dx, t, ft, dt);
        const Float mq = quad_minimizer(x, fx, dx, t, ft);
        newt = (std::fabs(mc - x) < std::fabs(mq - x))
                   ? mc
                   : mc + Float(0.5) * (mq - mc);
    } else if (dsign) {
        // Case 2: lower function value, derivatives of opposite sign.
        brackt = true;
        bound  = false;
        const Float mc = cubic_minimizer(x, fx, dx, t, ft, dt);
        const Float mq = quad_minimizer2(x, dx, t, dt);
        newt = (std::fabs(mc - t) > std::fabs(mq - t)) ? mc : mq;
    } else if (std::fabs(dt) < std::fabs(dx)) {
        // Case 3: lower function value, same-sign derivatives, magnitude decreases.
        bound = true;
        const Float mc = cubic_minimizer2(x, fx, dx, t, ft, dt, tmin, tmax);
        const Float mq = quad_minimizer2(x, dx, t, dt);
        if (brackt)
            newt = (std::fabs(t - mc) < std::fabs(t - mq)) ? mc : mq;
        else
            newt = (std::fabs(t - mc) > std::fabs(t - mq)) ? mc : mq;
    } else {
        // Case 4: lower function value, same-sign derivatives, magnitude does not decrease.
        bound = false;
        if (brackt)        newt = cubic_minimizer(t, ft, dt, y, fy, dy);
        else if (x < t)    newt = tmax;
        else               newt = tmin;
    }

    if (fx < ft) {
        // Case a
        y = t; fy = ft; dy = dt;
    } else {
        // Cases b, c
        if (dsign) { y = x; fy = fx; dy = dx; }
        x = t; fx = ft; dx = dt;
    }

    newt = std::clamp(newt, tmin, tmax);

    if (brackt && bound) {
        const Float mq2 = x + Float(0.66) * (y - x);
        if (x < y) { if (mq2 < newt) newt = mq2; }
        else       { if (newt < mq2) newt = mq2; }
    }

    t = newt;
    return Status::Success;
}

// ---------------------------------------------------------------------------
// Line search variants. Each returns >0 (number of evaluations) on success,
// or a negative Status code converted to int on failure.
// ---------------------------------------------------------------------------

template <typename Float>
int line_search_backtracking(int n,
                             Float* x,
                             Float& f,
                             Float* g,
                             const Float* s,
                             Float& stp,
                             const Float* xp,
                             const Float* /*gp*/,
                             Float* /*wp*/,
                             CallbackData<Float>& cd,
                             const Parameters<Float>& param) {
    constexpr Float dec = Float(0.5);
    constexpr Float inc = Float(2.1);

    if (stp <= Float(0)) return static_cast<int>(Status::InvalidParameters);

    const Float dginit = vecdot(g, s, n);
    if (dginit > Float(0)) return static_cast<int>(Status::IncreaseGradient);

    const Float finit  = f;
    const Float dgtest = param.ftol * dginit;
    int count = 0;
    Float width;

    for (;;) {
        veccpy(x, xp, n);
        vecadd(x, s, stp, n);

        f = cd.invoke(x, g, stp);
        ++count;

        if (f > finit + stp * dgtest) {
            width = dec;
        } else {
            if (param.linesearch == LineSearch::BacktrackingArmijo)
                return count;

            const Float dg = vecdot(g, s, n);
            if (dg < param.wolfe * dginit) {
                width = inc;
            } else {
                if (param.linesearch == LineSearch::BacktrackingWolfe)
                    return count;
                if (dg > -param.wolfe * dginit)
                    width = dec;
                else
                    return count;  // strong Wolfe
            }
        }

        if (stp < param.min_step)             return static_cast<int>(Status::MinimumStep);
        if (stp > param.max_step)             return static_cast<int>(Status::MaximumStep);
        if (param.max_linesearch <= count)    return static_cast<int>(Status::MaximumLineSearch);

        stp *= width;
    }
}

template <typename Float>
int line_search_backtracking_owlqn(int n,
                                   Float* x,
                                   Float& f,
                                   Float* g,
                                   const Float* s,
                                   Float& stp,
                                   const Float* xp,
                                   const Float* gp,
                                   Float* wp,
                                   CallbackData<Float>& cd,
                                   const Parameters<Float>& param) {
    constexpr Float width = Float(0.5);

    if (stp <= Float(0)) return static_cast<int>(Status::InvalidParameters);

    const Float finit = f;
    int count = 0;

    // Choose the orthant for the new point.
    for (int i = 0; i < n; ++i)
        wp[i] = (xp[i] == Float(0)) ? -gp[i] : xp[i];

    for (;;) {
        veccpy(x, xp, n);
        vecadd(x, s, stp, n);
        owlqn_project(x, wp, param.orthantwise_start, param.orthantwise_end);

        f = cd.invoke(x, g, stp);

        const Float norm = owlqn_x1norm(x, param.orthantwise_start, param.orthantwise_end);
        f += norm * param.orthantwise_c;

        ++count;

        Float dgtest = Float(0);
        for (int i = 0; i < n; ++i) dgtest += (x[i] - xp[i]) * gp[i];

        if (f <= finit + param.ftol * dgtest) return count;

        if (stp < param.min_step)             return static_cast<int>(Status::MinimumStep);
        if (stp > param.max_step)             return static_cast<int>(Status::MaximumStep);
        if (param.max_linesearch <= count)    return static_cast<int>(Status::MaximumLineSearch);

        stp *= width;
    }
}

template <typename Float>
int line_search_morethuente(int n,
                            Float* x,
                            Float& f,
                            Float* g,
                            const Float* s,
                            Float& stp,
                            const Float* xp,
                            const Float* /*gp*/,
                            Float* /*wa*/,
                            CallbackData<Float>& cd,
                            const Parameters<Float>& param) {
    if (stp <= Float(0)) return static_cast<int>(Status::InvalidParameters);

    const Float dginit = vecdot(g, s, n);
    if (dginit > Float(0)) return static_cast<int>(Status::IncreaseGradient);

    int count = 0;
    bool brackt = false;
    bool stage1 = true;
    Status uinfo = Status::Success;

    const Float finit  = f;
    const Float dgtest = param.ftol * dginit;
    Float width        = param.max_step - param.min_step;
    Float prev_width   = Float(2) * width;

    Float stx = Float(0), sty = Float(0);
    Float fx  = finit,    fy  = finit;
    Float dgx = dginit,   dgy = dginit;

    for (;;) {
        Float stmin, stmax;
        if (brackt) {
            stmin = std::min(stx, sty);
            stmax = std::max(stx, sty);
        } else {
            stmin = stx;
            stmax = stp + Float(4) * (stp - stx);
        }

        stp = std::clamp(stp, param.min_step, param.max_step);

        if ((brackt &&
             ((stp <= stmin || stmax <= stp) ||
              param.max_linesearch <= count + 1 ||
              uinfo != Status::Success)) ||
            (brackt && (stmax - stmin <= param.xtol * stmax))) {
            stp = stx;
        }

        veccpy(x, xp, n);
        vecadd(x, s, stp, n);

        f             = cd.invoke(x, g, stp);
        const Float dg     = vecdot(g, s, n);
        const Float ftest1 = finit + stp * dgtest;
        ++count;

        if (brackt && ((stp <= stmin || stmax <= stp) || uinfo != Status::Success))
            return static_cast<int>(Status::RoundingError);
        if (stp == param.max_step && f <= ftest1 && dg <= dgtest)
            return static_cast<int>(Status::MaximumStep);
        if (stp == param.min_step && (ftest1 < f || dgtest <= dg))
            return static_cast<int>(Status::MinimumStep);
        if (brackt && (stmax - stmin) <= param.xtol * stmax)
            return static_cast<int>(Status::WidthTooSmall);
        if (param.max_linesearch <= count)
            return static_cast<int>(Status::MaximumLineSearch);
        if (f <= ftest1 && std::fabs(dg) <= param.gtol * (-dginit))
            return count;

        if (stage1 && f <= ftest1 &&
            std::min(param.ftol, param.gtol) * dginit <= dg) {
            stage1 = false;
        }

        if (stage1 && ftest1 < f && f <= fx) {
            // Modified-function update.
            Float fm   = f   - stp * dgtest;
            Float fxm  = fx  - stx * dgtest;
            Float fym  = fy  - sty * dgtest;
            Float dgm  = dg  - dgtest;
            Float dgxm = dgx - dgtest;
            Float dgym = dgy - dgtest;

            uinfo = update_trial_interval(stx, fxm, dgxm,
                                          sty, fym, dgym,
                                          stp, fm,  dgm,
                                          stmin, stmax, brackt);

            fx  = fxm + stx * dgtest;
            fy  = fym + sty * dgtest;
            dgx = dgxm + dgtest;
            dgy = dgym + dgtest;
        } else {
            Float dg_mut = dg;
            Float f_mut  = f;
            uinfo = update_trial_interval(stx, fx, dgx,
                                          sty, fy, dgy,
                                          stp, f_mut, dg_mut,
                                          stmin, stmax, brackt);
            f = f_mut;
        }

        if (brackt) {
            if (Float(0.66) * prev_width <= std::fabs(sty - stx))
                stp = stx + Float(0.5) * (sty - stx);
            prev_width = width;
            width = std::fabs(sty - stx);
        }
    }
}

// ---------------------------------------------------------------------------
// Main driver
// ---------------------------------------------------------------------------

template <typename Float>
Result<Float> minimize_impl(int n,
                            Float* x,
                            const EvaluateFn<Float>& evaluate,
                            const ProgressFn<Float>& progress,
                            const Parameters<Float>& params_in) {
    using LineSearchProc = int (*)(int, Float*, Float&, Float*, const Float*,
                                   Float&, const Float*, const Float*, Float*,
                                   CallbackData<Float>&,
                                   const Parameters<Float>&);

    Result<Float> result{};
    result.status = Status::UnknownError;

    if (!x)        { result.status = Status::InvalidN;    return result; }
    if (!evaluate) { result.status = Status::LogicError;  return result; }

    Parameters<Float> param = params_in;
    const int m = param.m;

    // Input validation
    if (n <= 0)                                  { result.status = Status::InvalidN;           return result; }
    if (m <= 0)                                  { result.status = Status::InvalidN;           return result; }
    if (param.epsilon < Float(0))                { result.status = Status::InvalidEpsilon;     return result; }
    if (param.past < 0)                          { result.status = Status::InvalidTestPeriod;  return result; }
    if (param.delta < Float(0))                  { result.status = Status::InvalidDelta;       return result; }
    if (param.min_step < Float(0))               { result.status = Status::InvalidMinStep;     return result; }
    if (param.max_step < param.min_step)         { result.status = Status::InvalidMaxStep;     return result; }
    if (param.ftol < Float(0))                   { result.status = Status::InvalidFtol;        return result; }
    if (param.linesearch == LineSearch::BacktrackingWolfe ||
        param.linesearch == LineSearch::BacktrackingStrongWolfe) {
        if (param.wolfe <= param.ftol || Float(1) <= param.wolfe) {
            result.status = Status::InvalidWolfe;         return result;
        }
    }
    if (param.gtol < Float(0))                   { result.status = Status::InvalidGtol;        return result; }
    if (param.xtol < Float(0))                   { result.status = Status::InvalidXtol;        return result; }
    if (param.max_linesearch <= 0)               { result.status = Status::InvalidMaxLineSearch;  return result; }
    if (param.orthantwise_c < Float(0))          { result.status = Status::InvalidOrthantwise; return result; }
    if (param.orthantwise_start < 0 ||
        n < param.orthantwise_start)             { result.status = Status::InvalidOrthantwiseStart; return result; }
    if (param.orthantwise_end < 0) param.orthantwise_end = n;
    if (n < param.orthantwise_end)               { result.status = Status::InvalidOrthantwiseEnd; return result; }

    LineSearchProc linesearch = nullptr;
    if (param.orthantwise_c != Float(0)) {
        if (param.linesearch == LineSearch::Backtracking) {
            linesearch = &line_search_backtracking_owlqn<Float>;
        } else {
            result.status = Status::InvalidLineSearch;
            return result;
        }
    } else {
        switch (param.linesearch) {
            case LineSearch::MoreThuente:
                linesearch = &line_search_morethuente<Float>;
                break;
            case LineSearch::BacktrackingArmijo:
            case LineSearch::BacktrackingWolfe:
            case LineSearch::BacktrackingStrongWolfe:
                linesearch = &line_search_backtracking<Float>;
                break;
            default:
                result.status = Status::InvalidLineSearch;
                return result;
        }
    }

    // Working storage (RAII; std::vector zero-initializes).
    std::vector<Float> xp(n), g(n), gp(n), d(n), w(n);
    std::vector<Float> pg;
    std::vector<Float> pf;
    std::vector<IterationData<Float>> lm(m);

    if (param.orthantwise_c != Float(0)) pg.assign(n, Float(0));
    for (auto& it : lm) { it.s.assign(n, Float(0)); it.y.assign(n, Float(0)); }
    if (param.past > 0)                   pf.assign(param.past, Float(0));

    CallbackData<Float> cd{n, &evaluate, 0};

    Float fx = evaluate(x, g.data(), n, Float(0));
    cd.evaluations = 1;

    if (param.orthantwise_c != Float(0)) {
        const Float xn = owlqn_x1norm(x, param.orthantwise_start, param.orthantwise_end);
        fx += xn * param.orthantwise_c;
        owlqn_pseudo_gradient(pg.data(), x, g.data(), n,
                              param.orthantwise_c,
                              param.orthantwise_start,
                              param.orthantwise_end);
    }

    if (!pf.empty()) pf[0] = fx;

    // Initial direction
    vecncpy(d.data(),
            (param.orthantwise_c == Float(0)) ? g.data() : pg.data(),
            n);

    Float xnorm = vec2norm(x, n);
    Float gnorm = vec2norm(
        (param.orthantwise_c == Float(0)) ? g.data() : pg.data(), n);

    if (xnorm < Float(1)) xnorm = Float(1);
    if (gnorm / xnorm <= param.epsilon) {
        result.status      = Status::AlreadyMinimized;
        result.fx          = fx;
        result.iterations  = 0;
        result.evaluations = cd.evaluations;
        return result;
    }

    Float step = Float(1) / vec2norm(d.data(), n);

    int k   = 1;
    int end = 0;

    Status ret = Status::UnknownError;

    for (;;) {
        veccpy(xp.data(), x, n);
        veccpy(gp.data(), g.data(), n);

        int ls;
        if (param.orthantwise_c == Float(0)) {
            ls = linesearch(n, x, fx, g.data(), d.data(), step,
                            xp.data(), gp.data(), w.data(), cd, param);
        } else {
            ls = linesearch(n, x, fx, g.data(), d.data(), step,
                            xp.data(), pg.data(), w.data(), cd, param);
            owlqn_pseudo_gradient(pg.data(), x, g.data(), n,
                                  param.orthantwise_c,
                                  param.orthantwise_start,
                                  param.orthantwise_end);
        }
        if (ls < 0) {
            veccpy(x, xp.data(), n);
            veccpy(g.data(), gp.data(), n);
            result.status      = static_cast<Status>(ls);
            result.fx          = fx;
            result.iterations  = k - 1;
            result.evaluations = cd.evaluations;
            return result;
        }

        xnorm = vec2norm(x, n);
        gnorm = vec2norm(
            (param.orthantwise_c == Float(0)) ? g.data() : pg.data(), n);

        if (progress) {
            Progress<Float> p{
                x, g.data(), fx, xnorm, gnorm, step, n, k, ls
            };
            if (progress(p)) {
                result.status      = Status::Canceled;
                result.fx          = fx;
                result.iterations  = k;
                result.evaluations = cd.evaluations;
                return result;
            }
        }

        if (xnorm < Float(1)) xnorm = Float(1);
        if (gnorm / xnorm <= param.epsilon) {
            ret = Status::Success;
            break;
        }

        if (!pf.empty()) {
            if (param.past <= k) {
                const Float rate = (pf[k % param.past] - fx) / fx;
                if (std::fabs(rate) < param.delta) {
                    ret = Status::Stop;
                    break;
                }
            }
            pf[k % param.past] = fx;
        }

        if (param.max_iterations != 0 && param.max_iterations < k + 1) {
            ret = Status::MaximumIteration;
            break;
        }

        // Update s_{k+1} = x_{k+1} - x_k, y_{k+1} = g_{k+1} - g_k
        auto& it = lm[end];
        vecdiff(it.s.data(), x, xp.data(), n);
        vecdiff(it.y.data(), g.data(), gp.data(), n);

        const Float ys = vecdot(it.y.data(), it.s.data(), n);
        const Float yy = vecdot(it.y.data(), it.y.data(), n);
        it.ys = ys;

        const int bound = (m <= k) ? m : k;
        ++k;
        end = (end + 1) % m;

        // Steepest direction
        vecncpy(d.data(),
                (param.orthantwise_c == Float(0)) ? g.data() : pg.data(),
                n);

        int j = end;
        for (int i = 0; i < bound; ++i) {
            j = (j + m - 1) % m;
            auto& itj = lm[j];
            itj.alpha = vecdot(itj.s.data(), d.data(), n) / itj.ys;
            vecadd(d.data(), itj.y.data(), -itj.alpha, n);
        }

        vecscale(d.data(), ys / yy, n);

        for (int i = 0; i < bound; ++i) {
            auto& itj = lm[j];
            const Float beta = vecdot(itj.y.data(), d.data(), n) / itj.ys;
            vecadd(d.data(), itj.s.data(), itj.alpha - beta, n);
            j = (j + 1) % m;
        }

        if (param.orthantwise_c != Float(0)) {
            for (int i = param.orthantwise_start; i < param.orthantwise_end; ++i)
                if (d[i] * pg[i] >= Float(0)) d[i] = Float(0);
        }

        step = Float(1);
    }

    result.status      = ret;
    result.fx          = fx;
    result.iterations  = k;
    result.evaluations = cd.evaluations;
    return result;
}

// Explicit instantiations -----------------------------------------------------

template Result<float>  minimize_impl<float>(int, float*,
                                             const EvaluateFn<float>&,
                                             const ProgressFn<float>&,
                                             const Parameters<float>&);
template Result<double> minimize_impl<double>(int, double*,
                                              const EvaluateFn<double>&,
                                              const ProgressFn<double>&,
                                              const Parameters<double>&);

} // namespace lbfgs::detail

// ---------------------------------------------------------------------------
// strerror
// ---------------------------------------------------------------------------

namespace lbfgs {

std::string_view strerror(Status status) noexcept {
    using namespace std::string_view_literals;
    switch (status) {
        case Status::Success:                 return "Success: reached convergence (gtol)."sv;
        case Status::Stop:                    return "Success: met stopping criteria (ftol)."sv;
        case Status::AlreadyMinimized:        return "The initial variables already minimize the objective function."sv;
        case Status::UnknownError:            return "Unknown error."sv;
        case Status::LogicError:              return "Logic error."sv;
        case Status::OutOfMemory:             return "Insufficient memory."sv;
        case Status::Canceled:                return "The minimization process has been canceled."sv;
        case Status::InvalidN:                return "Invalid number of variables specified."sv;
        case Status::InvalidNSse:             return "Invalid number of variables (for SSE) specified."sv;
        case Status::InvalidXSse:             return "The array x must be aligned to 16 (for SSE)."sv;
        case Status::InvalidEpsilon:          return "Invalid parameter Parameters::epsilon specified."sv;
        case Status::InvalidTestPeriod:       return "Invalid parameter Parameters::past specified."sv;
        case Status::InvalidDelta:            return "Invalid parameter Parameters::delta specified."sv;
        case Status::InvalidLineSearch:       return "Invalid parameter Parameters::linesearch specified."sv;
        case Status::InvalidMinStep:          return "Invalid parameter Parameters::min_step specified."sv;
        case Status::InvalidMaxStep:          return "Invalid parameter Parameters::max_step specified."sv;
        case Status::InvalidFtol:             return "Invalid parameter Parameters::ftol specified."sv;
        case Status::InvalidWolfe:            return "Invalid parameter Parameters::wolfe specified."sv;
        case Status::InvalidGtol:             return "Invalid parameter Parameters::gtol specified."sv;
        case Status::InvalidXtol:             return "Invalid parameter Parameters::xtol specified."sv;
        case Status::InvalidMaxLineSearch:    return "Invalid parameter Parameters::max_linesearch specified."sv;
        case Status::InvalidOrthantwise:      return "Invalid parameter Parameters::orthantwise_c specified."sv;
        case Status::InvalidOrthantwiseStart: return "Invalid parameter Parameters::orthantwise_start specified."sv;
        case Status::InvalidOrthantwiseEnd:   return "Invalid parameter Parameters::orthantwise_end specified."sv;
        case Status::OutOfInterval:           return "The line-search step went out of the interval of uncertainty."sv;
        case Status::IncorrectTminMax:        return "A logic error occurred; alternatively, the interval of uncertainty became too small."sv;
        case Status::RoundingError:           return "A rounding error occurred; alternatively, no line-search step satisfies the sufficient decrease and curvature conditions."sv;
        case Status::MinimumStep:             return "The line-search step became smaller than Parameters::min_step."sv;
        case Status::MaximumStep:             return "The line-search step became larger than Parameters::max_step."sv;
        case Status::MaximumLineSearch:       return "The line-search routine reaches the maximum number of evaluations."sv;
        case Status::MaximumIteration:        return "The algorithm routine reaches the maximum number of iterations."sv;
        case Status::WidthTooSmall:           return "Relative width of the interval of uncertainty is at most Parameters::xtol."sv;
        case Status::InvalidParameters:       return "A logic error (negative line-search step) occurred."sv;
        case Status::IncreaseGradient:        return "The current search direction increases the objective function value."sv;
    }
    return "(unknown)"sv;
}

} // namespace lbfgs
