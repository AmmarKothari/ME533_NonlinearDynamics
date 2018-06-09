function M = Lagrangian_M_manual(I1,m1,l1, I2,m2,l2, theta1,theta2)
   % manually figured out the lagrangian for a 2 link manipulator
   z1 = l1^2*m1/4+I1+I2;
   z2 = l2^2*m2/4+I2;
   z3 = l1^2*m2;
   z4 = l1*l2*m2;
   
   M11 = z1+z3+z3+z4*cos(theta2);
   M12 = z2 + z4/2*cos(theta2);
   M21 = z2 + z4/2*cos(theta2);
   M22 = z2;
   
   M = [M11, M12;
       M21, M22];

end