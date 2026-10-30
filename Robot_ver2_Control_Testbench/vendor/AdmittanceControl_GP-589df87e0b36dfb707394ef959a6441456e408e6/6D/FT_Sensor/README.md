# F/T Sensor — MATLAB research components

There are two uses for this folder:

- `ft_lab_config.m` and `ft_generate_wrench.m` provide test inputs and
  synthetic sensor characteristics to the active Simulink hand-guiding lab.
- `run_ft_laboratory.m` is the earlier MATLAB-only synthetic sensor study.
  It includes additional illustrative quantization/saturation modeling not
  yet included in the referenced Simulink sensor component.

## Run

From the repository root in MATLAB:

```matlab
main
test_ft_lab_phase1                % Original numerical checks
results = run_ft_laboratory;     % Optional, standalone MATLAB-only study
```

This runs without automatic file exports. Tables and figures appear in MATLAB.

For the **integrated controller**, instead run `setup_handguiding_lab.m` and
an appropriate script from `experiments/` (see the repository README).

All sensor parameters are synthetic, not measured vendor specifications.
The Simulink sensor reference is `FT_Sensor_6D.slx` at repository root;
`FT_Sensor_Lab.slx` in this folder is a standalone testbench.
