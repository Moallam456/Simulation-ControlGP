function test_ft_lab_phase1()
% No sensor or robot hardware is accessed.
cfg=ft_lab_config(); [t,W]=ft_generate_wrench(cfg);
assert(numel(t)==2001 && isequal(size(W),[2001,6]));
assert(max(abs(diff(t)-cfg.Ts)) < 1e-12);
ideal=cfg;
ideal.sensor.bias(:)=0;
ideal.sensor.noise_std(:)=0;
ideal.sensor.drift_per_s(:)=0;
ideal.sensor.quantization_step(:)=0;
ideal.sensor.saturation(:)=Inf;
ideal.sensor.coupling=eye(6);
assert(isequal(ft_sensor_model(t,W,ideal),W));
A=ft_sensor_model(t,W,cfg);
assert(isequal(A,ft_sensor_model(t,W,cfg)));
rng(123); r1=rand(); ft_sensor_model(t,W,cfg); r2=rand();
rng(123); assert(r1==rand() && r2==rand());
c=ideal; c.sensor.coupling(2,1)=0.1;
mixed=ft_sensor_model(t,W,c);
i=find(t>=0.5,1); assert(abs(mixed(i,2)-0.5)<1e-12);
F=ft_compare_filters(A,cfg);
assert(isequal(size(F.ema),size(A)));
A2=A; A2(end,:)=A2(end,:)+100;
F2=ft_compare_filters(A2,cfg);
assert(isequal(F.ema(1:end-1,:),F2.ema(1:end-1,:)));
assert(isequal(F.butterworth(1:end-1,:),F2.butterworth(1:end-1,:)));
metrics=ft_lab_metrics(t,W,F,cfg);
assert(height(metrics)==3);
assert(all(isfinite(metrics.FxNoiseStd_N)));
assert(all(isfinite(metrics.ForceRMSE_N)));
assert(all(isfinite(metrics.TorqueRMSE_Nm)));
disp('PASS: phase 1 deterministic model/filter tests');
end
