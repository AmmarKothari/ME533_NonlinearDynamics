function out1 = Y_func(qr1_dd, qr2_dd, qr1_d, qr2_d, q1_d, q2_d, q2)
% Generate Y matrix / vector to show linear dependency between H, C, G,
% and a

Y_11 = qr1_dd;
Y_12 = qr2_dd;
Y_21 = 0;
Y_22 = qr1_dd + qr2_dd;
Y_13 = (2*qr1_dd + qr2_dd)*cos(q2) - (q2_d*qr1_d + q1_d*qr2_d + q2_d*qr2_d)*sin(q2);
Y_14 = (2*qr1_dd + qr2_dd)*sin(q2) + (q2_d*qr1_d + q1_d*qr2_d + q2_d*qr2_d)*cos(q2);
Y_23 = qr1_dd*cos(q2) + q1_d*qr1_d*sin(q2);
Y_24 = qr1_dd*sin(q2) - q1_d*qr1_d*cos(q2);
out1 = [Y_11 Y_12 Y_13 Y_14; Y_21 Y_22 Y_23 Y_24];
end