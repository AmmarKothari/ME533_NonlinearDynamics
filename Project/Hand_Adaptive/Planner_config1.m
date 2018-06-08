Q_INPUTS = 5;
TRAJ_NOISE = 1e-2; % noise scaled to joint range in trajectory in STOMP
NOISE_COOLING = 0.99;
TRAJ_STEPS = 100;
OBJ_NOISE = 0.05;
BLUR_FILTER_SIZE = 5;
NOISEY_TRAJS = 5;
TRAJ_DT = 0.01;
STOMP_STOP_COND = 0.01;

% % % % % % % % % % % % % % % % % % 
% Rectangle Object %
% % % % % % % % % % % % % % % % % % 
obj = grasp_object('rectangle');
obj = obj.setDims([0.5,0.5]);
obj = obj.setLocation([5,2]);
obj = obj.setBoundary();

% % % % % % % % % % % % % % % % % % 
% Contact Points on Top and Bottom %
% % % % % % % % % % % % % % % % % % 
contact_points = [obj.center(1), obj.center(2)+obj.dims(2);
                obj.center(1), obj.center(2)-obj.dims(2)];