# Admittance Control — MATLAB / Simulink

This project is the **Control Subteam's 500 Hz hand-guiding laboratory**.
MATLAB prepares test inputs and analyzes outputs; the Simulink models YOU
built execute synthetic F/T processing and six-axis Tustin admittance.

## Daily workflow (two steps)

1. Run `setup_handguiding_lab.m` **once per MATLAB session**. This runs
   `main.m` (adds project paths), optimizes the illustrative M/D controller,
   and prepares sensor parameters and the 500 Hz wrench input.
2. Open a script in `experiments/` and click **Run**. Choose the experiment
   that answers the question you're studying. Results stay in MATLAB tables,
   variables and interactive figures; **no automatic CSV/MAT/PNG exports**.

### Pick an experiment

| Script in `experiments/` | Engineering question |
| --- | --- |
| `analyze_handguiding_lab.m` | How do EMA and Butterworth differ at 10 Hz? |
| `sweep_filter_cutoff.m` | What is the noise/delay/control-error tradeoff across cutoffs? |
| `analyze_handguiding_sensitivity.m` | Does sensor noise cause unwanted commands at zero force? |
| `analyze_dynamic_handguiding.m` | How do push, reverse and release affect X velocity? |
| `analyze_sensor_drift.m` | What happens when bias changes after tare calibration? |
| `analyze_sensor_coupling.m` | Can a true X force cause a false Y measurement/motion? |

Don't run all experiments every time. Run **only** the one you need.

## Where things live

```text
AdmittanceControl_GP/
├── setup_handguiding_lab.m       # Daily initialization
├── main.m                        # Adds paths; standalone compatibility
├── experiments/                  # The six active MATLAB experiments
├── Admittance_Controller_6D.slx  # Referenced controller component
├── FT_Sensor_6D.slx              # Referenced F/T sensor component
├── 6D/
│   ├── Simulink/                # Standalone & integrated testbenches
│   ├── FT_Sensor/               # Sensor config, generator and phase-1 studies
│   │   └── simulink/            # Standalone sensor testbench
│   ├── models/                  # Six-axis mathematical functions
│   ├── analysis/                # Earlier design studies
│   ├── tests/                   # Existing research tests
│   └── design_6axis_admittance.m
└── 1D/                          # Earlier one-axis research (retained)
```

**Why not move the .slx files?** Subsystem references and MATLAB model lookup
can depend on filenames and search paths. Their paths are preserved until
they can be validated interactively in MATLAB. The five six-axis .slx models
are *not* automatically duplicates: components and testbenches have distinct
purposes.

The generated `*.slxc`, `slprj/` and previous results under
`6D/FT_Sensor/results/` are ignored by Git but may still exist locally.
They are not source models. Review any old outputs before deleting them.

## Scope and limitations

All sensor parameters are **illustrative**, not vendor specifications.
Results describe simulated Cartesian *commands*, not measured robot motion
or physical safety. Robot joint dynamics / Simscape plant integration remains
a separate Simulation Subteam activity. The optimizer's pass checks don't
mean every requested speed and T90 target was achieved.

Dependencies include MATLAB, Simulink, Optimization Toolbox and
Signal Processing Toolbox. Some earlier research scripts use additional
toolboxes. `main.m` can be used to access older 1D and 6D studies.

No .slx models have been changed by this organization pass.
