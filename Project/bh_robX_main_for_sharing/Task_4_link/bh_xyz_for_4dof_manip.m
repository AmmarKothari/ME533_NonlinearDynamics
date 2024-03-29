function out1 = bh_xyz_for_4dof_manip(L1Y_s,L2Y_s,L3Y_s,theta1,theta2,theta3,theta4)
%BH_XYZ_FOR_4DOF_MANIP
%    OUT1 = BH_XYZ_FOR_4DOF_MANIP(L1Y_S,L2Y_S,L3Y_S,THETA1,THETA2,THETA3,THETA4)

%    This function was generated by the Symbolic Math Toolbox version 7.2.
%    20-Jul-2017 14:24:04

t2 = theta1+theta2;
t3 = cos(t2);
t4 = L2Y_s.*t3;
t5 = cos(theta1);
t6 = L1Y_s.*t5;
t7 = theta1+theta2+theta3;
t8 = cos(t7);
t9 = L3Y_s.*t8;
t10 = t4+t6+t9;
out1 = [-t10.*sin(theta4),t10.*cos(theta4),L2Y_s.*sin(t2)+L3Y_s.*sin(t7)+L1Y_s.*sin(theta1)];
