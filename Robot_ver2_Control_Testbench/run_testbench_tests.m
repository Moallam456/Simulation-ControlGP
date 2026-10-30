function results=run_testbench_tests()
% RUN_TESTBENCH_TESTS No-argument MATLAB validation entry point.
root=fileparts(mfilename('fullpath'));addpath(root,fullfile(root,'src'),fullfile(root,'control'),fullfile(root,'tests'));
if exist('rb2_native','file')~=3 || exist('rb2_controller_native','file')~=3,build_robot_testbench();end
results.mechanicsAndKinematics=validate_robot_testbench();
results.references=validate_references();
results.safety=validate_safety_and_faults();
results.pass=results.mechanicsAndKinematics.pass && results.references.pass && results.safety.pass;
end
