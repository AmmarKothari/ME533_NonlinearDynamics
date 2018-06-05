syms    b1_s      b2_s
syms taum1_s   taum2_s

syms t theta1(t)  TH1_s  
syms   theta2(t)  TH2_s 

actual_list_SYM_pos = formula( [     theta1,          theta2]     );
holder_list_SYM_pos = [                 TH1_s,           TH2_s  ];

th1dot = formula( diff(theta1, t)  );
th2dot = formula( diff(theta2, t)  );

% I'm going to define a matrix, whos columns represent the vectors of the 
% NON conservative torques for LINK 1, and another matrix whose columns 
% represent the angular velocities associated with these torques:

the_tau_mat_LINK_1 = ...
   [       0,                0,            0,                  0;          
           0,                0,            0,                  0;          
   (taum1_s),   (-b1_s*th1dot),   (-taum2_s),     (b2_s*th2dot)   ];

the_w_mat_LINK_1 = ...
   [      0,                0,            0,                  0;
          0,                0,            0,                  0;
     th1dot,           th1dot,       th1dot,             th1dot ];
 
% And similarly for LINK 2: 
the_tau_mat_LINK_2 = ...
   [         0,                  0;          
             0,                  0;          
     (taum2_s),     (-b2_s*th2dot)   ];

the_w_mat_LINK_2 = ...
   [         0,                  0;
             0,                  0;
     (th1dot + th2dot),  (th1dot + th2dot) ];
 
% So concatenate these into single matrices:
the_tau_mat_actual = [the_tau_mat_LINK_1, the_tau_mat_LINK_2];
the_w_mat_actual   = [  the_w_mat_LINK_1,   the_w_mat_LINK_2];

% Create a generalised force object using the class <bh_genF4manips_CLS>
genF_OBJ = bh_genF4manips_CLS( the_tau_mat_actual, ...
                               the_w_mat_actual, ...
                               actual_list_SYM_pos, ...
                               holder_list_SYM_pos);
                           
% And now calculate our system's generalised forces:                          
genF_OBJ = genF_OBJ.calc_genF();

% What do the these generalised forces look like?
genF_OBJ.show_genF_holder() 

% And we can retrive the individual generalised forces as well :
the_Q1    = genF_OBJ.get_Qk(    1, 'holder')

% Or we can retrive all of the generalised forces   :
the_Qk_vec = genF_OBJ.get_Qk('all', 'holder')