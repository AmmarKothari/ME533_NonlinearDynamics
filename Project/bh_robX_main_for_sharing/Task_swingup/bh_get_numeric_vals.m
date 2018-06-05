function  res_T = bh_get_numeric_vals()
       % run script
      %bh_parameters_for_swing_explore
       bh_parameters_for_4dof_manipulator_swingup 
        tmp_T = whos;
        for kk=1:length(tmp_T)
           eval(['res_T.',tmp_T(kk).name,'=',tmp_T(kk).name,';']);
        end
          
%             whos
%               Name              Size            Bytes  Class        
% 
%               IG3               3x3                72  double                       
%               IG3_xx            1x1                 8  double                       
%               I_brick           1x1                32  function_handle              
%               Io3_xx            1x1                 8  double                       
%               L3x               1x1                 8  double                       
%               L3y               1x1                 8  double                       
%               L3z               1x1                 8  double                       
%               b3_damp           1x1                 8  double                       
%               density           1x1                 8  double                       
%               g                 1x1                 8  double                       
%               mass3             1x1                 8  double                       
%               theta3_0          1x1                 8  double                       
%               theta3_dot_0      1x1                 8  double       
    end

