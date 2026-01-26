import numpy as np


def wn_zeta(arr):
    wn = np.abs(arr)
    zeta = -np.real(arr) / wn
    return wn, zeta


def OptimalOutputFeedback(sys_cmd2latf, Q, R, K_0, max_iters=100, tol=1e-6, stab_tol=1e-8):
    """Iterative optimal output-feedback update (pure NumPy).

    Notes:
    - Solves continuous Lyapunov A'P + P A + Q = 0 with a custom solver.
    - Ensures Q_eff is positive definite, recomputes cost for each candidate
      during line search, and uses safer inverses/pinv where needed.
    """
    A = sys_cmd2latf.A
    B = sys_cmd2latf.B
    C = sys_cmd2latf.C

    K_k = K_0.copy()
    nstates = sys_cmd2latf.nstates

    def is_pos_def(M):
        vals = np.linalg.eigvalsh(M)
        return np.all(vals >= 0)

    # Initial evaluation
    A_k = A - B @ K_k @ C
    # require initial closed-loop to be asymptotically stable (margin stab_tol)
    eigs_init = np.linalg.eigvals(A_k)
    if np.max(np.real(eigs_init)) >= -stab_tol:
        raise ValueError("Initial closed-loop (A - B K_0 C) is not asymptotically stable with required margin")
    Q_eff = C.T @ K_k.T @ R @ K_k @ C + Q
    if not is_pos_def(Q_eff):
        raise ValueError("Q_eff is not positive definite for initial K_0")

    P = lyapunov(A_k, Q_eff)
    S = lyapunov(A_k, np.eye(nstates))
    J_current = 0.5 * np.trace(P)

    iter_count = 0
    error = np.inf

    while iter_count < max_iters and error > tol:
        A_k = A - B @ K_k @ C
        # require current closed-loop to remain asymptotically stable
        eigs_curr = np.linalg.eigvals(A_k)
        if np.max(np.real(eigs_curr)) >= -stab_tol:
            raise ValueError("Current closed-loop (A - B K C) lost asymptotic stability during iteration")

        Q_eff = C.T @ K_k.T @ R @ K_k @ C + Q
        if not is_pos_def(Q_eff):
            raise ValueError("Q_eff is not positive definite during iteration")

        P = lyapunov(A_k, Q_eff)
        S = lyapunov(A_k.T, np.eye(nstates))

        # compute dK using safer inverses
        M = C @ S @ C.T
        cond_M = np.inf
        try:
            cond_M = np.linalg.cond(M)
        except Exception:
            pass
        if cond_M < 1 / np.finfo(float).eps:
            Minv = np.linalg.inv(M)
        else:
            Minv = np.linalg.pinv(M)

        RHS = B.T @ P @ S @ C.T @ Minv
        dK = np.linalg.solve(R, RHS) - K_k

        accepted = False
        # line search over decreasing alphas
        for alpha in np.linspace(1, 1e-4, 20):
            K_cand = K_k + alpha * dK
            A_cand = A - B @ K_cand @ C
            # Require closed-loop asymptotic stability with margin stab_tol
            eigs_A = np.linalg.eigvals(A_cand)
            if np.max(np.real(eigs_A)) >= -stab_tol:
                # unstable or marginal candidate; skip
                continue

            Q_eff_cand = C.T @ K_cand.T @ R @ K_cand @ C + Q
            if not is_pos_def(Q_eff_cand):
                continue

            # attempt Lyapunov solve; skip candidate on numerical issues
            try:
                P_cand = lyapunov(A_cand, Q_eff_cand)
            except np.linalg.LinAlgError:
                continue
            if not np.all(np.isfinite(P_cand)):
                continue

            J_cand = 0.5 * np.trace(P_cand)
            # require a relative improvement (avoid accepting tiny absolute drops
            # when absolute cost is huge)
            rel_impr = abs(J_current - J_cand) / max(1.0, abs(J_current))
            if J_cand < J_current and rel_impr > tol:
                error = rel_impr
                K_k = K_cand
                J_current = J_cand
                accepted = True
                break

        if not accepted:
            # no improvement found
            break

        iter_count += 1

    return K_k


# Solve the continuous time Lyapunov equation: A'P + PA + Q = 0
def lyapunov(A, Q):
    n = A.shape[0]
    # vec(A'P + P A) = (I kron A' + A.T kron I) vec(P)
    A_kron = np.kron(np.eye(n), A.T) + np.kron(A.T, np.eye(n))
    Q_vec = -Q.flatten(order='F')
    P_vec = np.linalg.solve(A_kron, Q_vec)
    P = P_vec.reshape(n, n, order='F')
    P = 0.5 * (P + P.T)
    return P