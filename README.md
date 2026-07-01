# Modern Control Theory — Techniques and Examples

[![language](https://img.shields.io/badge/language-MATLAB-brightgreen.svg)](#)

A MATLAB tutorial collection covering the core of classical and modern
control, based on Ogata Katsuhiko's *Modern Control Engineering*. Each
`.m` file is a **self-contained, runnable tutorial**: derivations and
explanations live in the comments, interleaved with Control System
Toolbox code and the figures it produces.

Every instructional file is written in MATLAB's **Publishing Markup**, so
you can turn any of them into a formatted HTML/PDF report (with the code
evaluated and the figures embedded) using the `publish` command.

## Requirements

- MATLAB with the **Control System Toolbox** (some files also use the
  **Symbolic Math Toolbox**, e.g. for Laplace transforms and Routh
  ranges). Most files also run in GNU Octave.

## How to use these tutorials

**Step through interactively.** Open a file and run it cell-by-cell with
`Ctrl+Enter` (each `%%` starts a new cell). The comments explain what each
block does before you run it.

**Run a whole file.**

```matlab
run('Root-Locus/LeadCompensation.m')
```

**Publish to a formatted report.** Generate an HTML report — code,
prose, equations, and figures together:

```matlab
publish('Root-Locus/LeadCompensation.m')        % one file
publish_all                                       % every tutorial -> html/
publish_all('dirs',{'Digital-Control'})           % just one directory
publish_all('format','pdf')                        % PDF instead of HTML
```

`publish_all` (at the repo root) walks the instructional directories,
publishes each tutorial, and drops the reports in an `html/` subfolder of
each directory. Helper functions and the homework folders are skipped.

## Directory guide

Suggested reading order:

| Directory | Topic |
|---|---|
| `Intro/` | What a control system is; open- vs closed-loop |
| `Mathematical Models/` | Transfer functions; modeling mechanical / electrical / fluid / thermal systems; block-diagram algebra |
| `Transient and Steady-State/` | Test signals, first/second/higher-order response, Routh–Hurwitz stability |
| `Root-Locus/` | Root-locus analysis and lead/lag compensator design |
| `Frequency-Response/` | Bode, polar/Nyquist plots, gain/phase margins, frequency-domain design |
| `PID Controllers/` | P/PI/PD/PID, Ziegler–Nichols and frequency-domain tuning |
| `State-Space/` | State models, controllability/observability, pole placement, observers, LQR, robustness |
| `Servo-and-Tracking/` | Integral control, disturbance rejection, and two-degree-of-freedom design |
| `Digital-Control/` | Sampling, ZOH, discretization, digital PID, deadbeat control |
| `Nonlinear-Systems/` | Phase-plane analysis and Lyapunov stability |
| `System-Identification/` | Markov parameters from data, OKID, and the ERA realization |
| `Kalman-Filtering/` | The Kalman filter, its steady-state form, and LQG control |
| `Case-Studies/` | End-to-end designs that tie the toolbox together (inverted pendulum, DC motor) |

The `HW*/` and `Final/` folders hold the original homework and exam
scripts the collection grew out of, plus a few root-level helper
functions (`pinv2.m`, `YV_Form_nonzero.m`, `recover_SYSMP.m`, `disc_m.m`,
…) used by the system-identification problems.

## A note on the visuals

Each tutorial tells its story with figures: a *before* shot, the
intervention ("here's what we did"), and an *after* / "what changed"
comparison — so the effect of every design choice is something you can
see, not just read.

## Source

Ogata Katsuhiko, *Modern Control Engineering*.
