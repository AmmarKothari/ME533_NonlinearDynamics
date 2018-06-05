%% Exact linearization of the Simulink model bh_io_model_for_CONT_design

%% Load model parameters into BASE workspace

bh_parameters_for_4dof_manipulator_CONTROL

%% Specify the model name
model = 'bh_io_model_for_CONT_design';
open_system(model)
%% Specify the analysis I/Os
% Use root level inports and outports in bh_io_model_for_CONT_design
% by passing no input argument to the linearize command for analysis I/Os

%% Specify the operating point
% Use the model initial condition
op = operpoint(model);

%% Linearize the model
sys = linearize(model,op);

% NOTE the following names for the OUTPUTS and INPUTS
% [ sys.OutputName,    sys.InputName]
% 
%     'TH1_DOT'         'Tau_m1'
%     'TH2_DOT'         'Tau_m2'
%     'TH3_DOT'         'Tau_m3'
%     'TH4_DOT'         'Tau_m4'

%% Convert SS to Transfer Functions
tf_sys = tf(sys)

%% Do the Control system design
% NOTE: even though we have a MIMO system, we'll start our Control design
% by considering 4 independent SISO systems

% Focus on a selection of TF's
TF_J1 = tf_sys('TH1_DOT','Tau_m1');
TF_J2 = tf_sys('TH2_DOT','Tau_m2');
TF_J3 = tf_sys('TH3_DOT','Tau_m3');
TF_J4 = tf_sys('TH4_DOT','Tau_m4');

% use the PID tuning blocks
open_system('bh_control_design_sandpit')

%% Test with the NON-Linear model

% *AFTER* you've designed the 4 individual joint controllers, then test 
% the designs on the NON-LINEAR robot model
open_system('bh_4dof_LAGR_manipulator_CONTROL_TQ_FFWD_design_2')

