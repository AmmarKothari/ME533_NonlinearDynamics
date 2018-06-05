function  bh_pend_solve_ODE_sys( q_dot_col_s, q0_vec_num )

validateattributes(q0_vec_num, {'numeric'}, {'vector', 'numel', 2});
q0_vec_num = q0_vec_num(:);

% ATTENTION:
% we expect our symbolic expressions in q_dot_col_s to contain the following 
% symbols:
%
% syms    m3_s    % masses
% syms    b3_s    % damping
% syms     g_s    % gravity 
% syms   I_G_s    % Inertia about CoM
% 
% syms L3X_s   L3Y_s   L3Z_s  % lengths for LINK #3
% syms theta_s  theta_D_s  theta_DD_s
% syms    yo_s     yo_D_s     yo_DD_s

mass3   = [];
g       = [];
L3y     = [];
b3_damp = [];
IG3_xx  = [];
Io3_xx  = [];

% now modify the values of the model parameters
res_T   = LOC_import_numeric_params();

mass3   = res_T.mass3;
g       = res_T.g;
L3y     = res_T.L3y;
b3_damp = res_T.b3_damp;
IG3_xx  = res_T.IG3_xx;
Io3_xx  = res_T.Io3_xx;


new_q_dot_col_s = subs(q_dot_col_s, {'m3_s',  'b3_s', 'g_s','I_G_s', 'L3Y_s'}, ...
                                     {mass3, b3_damp,     g, IG3_xx,   L3y});
   
tmp_M_fh = matlabFunction(new_q_dot_col_s, 'vars', {'t','theta_s', 'theta_D_s'});
M_fh     = @(t,q) tmp_M_fh(t, q(1), q(2) );

[t,q]  = ode45(M_fh, [0, 5], q0_vec_num); 
%[t,q]  = ode23t(M_fh, [0, 10], q0_vec_num); 

figure
subplot(2,2,[1 3]);
plot(q(:,1), q(:,2), '-r');
grid('on'); xlabel('\theta'); ylabel('\theta_{DOT}');
hold('on'); plot(q0_vec_num(1), q0_vec_num(2), '.b', 'MarkerSize', 25); 

subplot(2,2,2);
               plot(t, rad2deg( q(:,1) ), '-b');
               grid('on'); xlabel('time (secs)'); ylabel('\theta (degrees)');
               axis('tight')
               hL = refline(0,90);
               hL.Color = 'r';
               hL.LineWidth = 2;
subplot(2,2,4);
               plot(t, rad2deg( q(:,2) ), '-k');
               grid('on'); xlabel('time (secs)'); ylabel('\theta_{DOT} (degrees/sec)');
               axis('tight')

end %  end of MAIN function
%--------------------------------------------------------------------------
% END of MAIN function
%--------------------------------------------------------------------------
function res_T = LOC_import_numeric_params()
        % run script to load parameters into local workspace
        bh_parameters_for_4dof_manipulator_swingup
        
        % now absorb these local variables into a STRUCT
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

