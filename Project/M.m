function out1 = M(l1,l2,m1,m2,th2)
%M
%    OUT1 = M(L1,L2,M1,M2,TH2)

%    This function was generated by the Symbolic Math Toolbox version 8.1.
%    05-Jun-2018 14:59:16

t2 = l1.^2;
t3 = cos(th2);
t4 = l1.*t3;
t5 = l2+t4;
t6 = l2.*m2.*t5.*(1.0./4.0);
t7 = l2.^2;
out1 = reshape([m1.*t2.*(1.0./3.0)+m2.*t2.*(1.0./4.0)+m2.*t7.*(1.0./4.0)+l1.*l2.*m2.*t3.*(1.0./2.0),t6,t6,m2.*t7.*(1.0./3.0)],[2,2]);
