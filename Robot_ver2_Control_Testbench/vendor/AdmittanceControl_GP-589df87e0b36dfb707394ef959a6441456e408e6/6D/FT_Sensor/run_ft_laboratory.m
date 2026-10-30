function results = run_ft_laboratory()
% Original synthetic-only F/T experiment. Results stay in MATLAB.
% This is independent from the newer referenced Simulink sensor model.
cfg = ft_lab_config();
[t,Wtrue] = ft_generate_wrench(cfg);
Wmeas = ft_sensor_model(t,Wtrue,cfg);
filtered = ft_compare_filters(Wmeas,cfg);
metrics = ft_lab_metrics(t,Wtrue,filtered,cfg);
f = figure('Name','Synthetic F/T X-axis filtering');
subplot(2,1,1);
plot(t,Wtrue(:,1),'k',t,Wmeas(:,1),'Color',[0.7 0.7 0.7]); hold on;
plot(t,filtered.ema(:,1),'b',t,filtered.butterworth(:,1),'r');
ylabel('F_x (N)'); legend('True','Measured','EMA','Butterworth'); grid on;
subplot(2,1,2);
plot(t,filtered.ema(:,1)-Wtrue(:,1),'b', ...
    t,filtered.butterworth(:,1)-Wtrue(:,1),'r');
xlabel('Time (s)'); ylabel('F_x error (N)'); grid on;
disp(metrics);
results = struct('config',cfg,'time_s',t,'true',Wtrue, ...
    'measured',Wmeas,'filtered',filtered,'metrics',metrics);
end
