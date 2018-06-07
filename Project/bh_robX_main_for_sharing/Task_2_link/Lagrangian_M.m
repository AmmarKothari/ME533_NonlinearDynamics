function out1 = Lagrangian_M(I1,I2,l1,l2,m1,m2,th2)
%LAGRANGIAN_M
%    OUT1 = LAGRANGIAN_M(I1,I2,L1,L2,M1,M2,TH2)

%    This function was generated by the Symbolic Math Toolbox version 8.1.
%    06-Jun-2018 16:09:54

t2 = l1.^2;
t3 = l2.^2;
t4 = m2.*t3.*(1.0./4.0);
t5 = cos(th2);
t6 = l1.*l2.*m2.*t5.*(1.0./2.0);
t7 = I2+t4+t6;
out1 = reshape([I1+I2+t4+m1.*t2.*(1.0./4.0)+m2.*t2+l1.*l2.*m2.*t5,t7,t7,I2+t4],[2,2]);