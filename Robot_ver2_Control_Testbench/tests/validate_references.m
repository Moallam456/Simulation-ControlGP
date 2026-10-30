function report=validate_references()
root=fileparts(fileparts(mfilename('fullpath')));addpath(root,fullfile(root,'src'),fullfile(root,'control'));
cfg=testbench_config();M=cfg.demo.virtualMass(:);D=cfg.demo.virtualDamping(:);dt=.002;
t=(0:dt:.5).';W=zeros(numel(t),6);W(t>=.1 & t<.3,1)=1;W(:,5)=.01*sin(2*pi*5*t);
A=(2*M-D*dt)./(2*M+D*dt);B=dt./(2*M+D*dt);expected=zeros(size(W));prevW=zeros(1,6);v=zeros(1,6);
for k=1:numel(t),v=A.'.*v+B.'.*(W(k,:)+prevW);expected(k,:)=v;prevW=W(k,:);end
actual=rb2_admittance_native(W,M,D,dt,100*ones(6,1),1000*ones(6,1),.004);
report.cEquationError=max(abs(actual-expected),[],'all');assert(report.cEquationError<1e-12);
model='rb2_reference_test';if bdIsLoaded(model),close_system(model,0);end
new_system(model);cleanup=onCleanup(@() close_system(model,0));
load_system(fullfile(root,'control','rb2_admittance_reference.slx'));
add_block('simulink/Sources/From Workspace',[model '/W'],'VariableName','wrench');
add_block('built-in/Subsystem',[model '/Controller'],'ReferencedSubsystem','rb2_admittance_reference');
add_block('simulink/Sinks/To Workspace',[model '/Output'],'VariableName','v','SaveFormat','Timeseries');
add_line(model,'W/1','Controller/1');add_line(model,'Controller/1','Output/1');
in=Simulink.SimulationInput(model);in=in.setVariable('A6',A).setVariable('B6',B).setVariable('wrench',[t W]);
in=in.setModelParameter('Solver','FixedStepDiscrete','FixedStep','.002','StopTime','.5','ReturnWorkspaceOutputs','on');
out=sim(in);got=squeeze(out.v.Data);report.simulinkEquationError=max(abs(got-expected),[],'all');
assert(report.simulinkEquationError<1e-12);
limited=rb2_admittance_native(100*ones(numel(t),6),M,D,dt,.1*ones(6,1),.2*ones(6,1),.004);
assert(all(abs(limited)<=.1+1e-12,'all'));assert(all(abs(diff([zeros(1,6);limited]))<=.2*dt+1e-12,'all'));
report.cLimiterChecks=true;report.pass=true;disp(report);
end
