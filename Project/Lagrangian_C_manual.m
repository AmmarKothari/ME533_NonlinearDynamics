function C = Lagrangian_C_manual(I1,m1,l1, I2,m2,l2,  theta1,theta2, theta1_d,theta2_d, b1, b2)
    % manually defining C matrix
    
   z4 = l1*l2*m2;
   C11 = -z4*theta2_d*sin(theta2) + b1;
   C12 = z4/2*theta2_d*sin(theta2);
   C21 = z4/2*theta1_d*sin(theta2);
   C22 = b2;
   
   C = [C11, C12;
       C21, C22];
   
   
   

end