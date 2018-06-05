density = 1000; 

L1x     = 0.25; %1.50; % m
L1y     = 0.050; % m
L1z     = 0.005; % m

mass1   = density * L1x*L1y*L1z; % kg

Hz      = 0.20;  % m
Rturn   = 0.05;  % m  
mass4   = density * Hz*pi*(Rturn^2); % kg

g = 9.80665;  % m/sec^2

b1_damp = 0.005; %0.1 (N.m/(rad/sec));
b4_damp = 0.005; %0.1 (N.m/(rad/sec));

theta1_0     = 0;                    % rad
theta1_dot_0 = 0;                    % rad/sec

theta4_0     = 0;                    % rad
theta4_dot_0 = 0;                    % rad/sec 