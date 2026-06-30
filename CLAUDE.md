# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Overview

This is a MATLAB codebase of modern control theory examples and homework assignments, based on Ogata Katsuhiko's *Modern Control Engineering*. All files are `.m` scripts or function files intended to run in MATLAB with the Control System Toolbox.

## Running Code

Scripts are run directly in MATLAB or Octave:

```matlab
% From the MATLAB command window, navigate to the folder and run:
run('HW6/HW6.m')

% Or open and run sections with Ctrl+Enter for %% cell blocks
```

There is no build system, test framework, or linter. Validation is visual — scripts produce plots that are inspected manually.

## Repository Structure and Architecture

The codebase is organized into topic-based directories and homework assignment directories:

**Topic directories** (intended for instructional examples):
- `Mathematical Models/` — transfer functions, Laplace-domain modeling of mechanical/electrical/fluid/thermal systems
- `Transient and Steady-State/` — first/second/higher-order step response, Routh-Hurwitz stability
- `Root-Locus/` — root locus plots, lead/lag compensation design
- `Frequency-Response/` — Bode diagrams, Nyquist stability, polar plots
- `PID Controllers/` — PID tuning, Ziegler-Nichols, frequency-domain tuning
- `State-Space/` — state-space representation, controllability, observability, pole placement, LQR, state observers, robust control
- `Intro/` — introductory examples

**Topic-directory content**: Each `.m` file in the topic directories is a self-contained, runnable tutorial for its named subject, written in the "code + full derivation" style — `%%` cell blocks with LaTeX-formatted derivations interleaved with Control System Toolbox code, based on the corresponding chapters of Ogata. (These files previously held byte-identical copies of an unrelated discrete observer Markov-parameter script; they have since been replaced with distinct, topic-correct content.) When adding new examples, follow the same style and keep each file's content matched to its name and directory.

**Homework directories** (`HW2/`, `HW3/`, `HW5/`, `HW6/`, `Final/`) contain the actual worked examples with unique content:
- Root-level `e145hw1*.m` files: HW1 problems (symbolic Laplace, `lsim`, `ss`, `c2d`)
- `HW2/`: `hw2_1.m` through `hw2_4.m` — state-space simulation problems
- `HW3/`: `hw4p1d.m`, `hw4p2d.m`, `hw4p3.m` — discrete-time pole placement
- `HW5/`: `hw5p1.m` — continuous-to-discrete conversion (`c2d`)
- `HW6/`: `HW6.m` — Iterative Learning Control (ILC) with gradient and optimal update laws; `pmatrix.m` — helper function building the Toeplitz-structured impulse response matrix P for ILC
- `Final/`: `e145p1.m` through `e145p11.m` — final exam problems covering controllability SVD analysis, observer Markov parameter recovery, system identification

**Root-level helper functions**:
- `disc_m.m` — computes a discretization sum `sum_{i=1}^{k} A^{n_i} * B * sin(0.2 * m_i)`
- `disc_nm.m` — computes a single term `A^n * B * sin(0.2 * m)`
- `YV_Form_nonzero.m` — builds the OKID regression data matrices `[Y_bar, V_bar]` from an input/output record, allowing a nonzero initial condition
- `pinv2.m` — tolerance-truncated Moore-Penrose pseudoinverse via the SVD (keeps singular values above a fraction of the largest)
- `recover_SYSMP.m` — recovers system Markov parameters from the observer Markov parameters produced by the OKID least-squares step

## Key Patterns and Conventions

**State-space system construction**: Systems are defined with continuous matrices `(A_c, B_c, C, D)`, converted to discrete-time with `c2d`, then simulated with `dlsim` or `lsim`.

```matlab
mat_ss = ss(A_c, B_c, C, D);
sys_dsct = c2d(mat_ss, dt);
A_d = sys_dsct.A;  B_d = sys_dsct.B;
```

**Observer gain design**: Uses `acker` (Ackermann's formula) or `place` on the transposed system:
```matlab
G = acker(A_d', -C_d', desired_poles)';
```

**Plot formatting**: Axes labels use LaTeX interpreter with FontSize 20–30; `ylabel` rotation is set to 0 with right horizontal alignment. This is consistent across all files and should be preserved.

**Section structure**: Scripts use `%%` cell blocks for MATLAB's cell-mode execution. Each `%% Part N` block is a self-contained analysis step. Block-level comments explain the mathematical derivation inline using `%` and LaTeX (`$$...$$`).

**Markov parameter / OKID workflow** (used in the `Final/` system-identification problems):
1. Build observer Markov parameters (OMP) analytically from `(A, B, C, D, G)`
2. Form `[Y_bar, V_bar]` data matrices via `YV_Form_nonzero`
3. Recover OMP via pseudoinverse: `cap_y_hat = Y_bar' * pinv2(V_bar, tol)` where `pinv2` is a tolerance-based SVD pseudoinverse
4. Recover system Markov parameters from OMP via `recover_SYSMP`

The functions `YV_Form_nonzero`, `pinv2`, and `recover_SYSMP` are now provided as root-level helper functions in this repository (see above), so this workflow runs without any external dependencies.
