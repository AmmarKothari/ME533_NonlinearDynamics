function [t, q, the_yo_DD_num_col] =  bh_pend_solve_ODE_sys_custom(thetaD_and_thetaDD_col_s, q0_vec_num, k_term )

validateattributes(q0_vec_num, {'numeric'}, {'vector', 'numel', 4});

%          STATE vector:   q     =   {theta,    theta_D,   yo,    yo_D  }
% deriv of STATE vector:   q_dot =   {theta_D,  theta_DD,  yo_D,  yo_DD }
q0_vec_num = q0_vec_num(:);

% Load the values of the model parameters
res_T   = LOC_import_numeric_params();

% insert the k_term
res_T.k_term = k_term;

% convert the thetaDOT and thetaDOTDOT symbolic vector into a 
% MATLAB function handle
syms I_G_s L3Y_s b3_s g_s m3_s theta_s theta_D_s yo_DD_s

TD_and_TDD_fh = matlabFunction(thetaD_and_thetaDD_col_s,...
                               'Vars',{I_G_s,L3Y_s,b3_s,g_s,m3_s,theta_s,theta_D_s,yo_DD_s});                          
m   = res_T.mass3;
g   = res_T.g;
L   = res_T.L3y;
b   = res_T.b3_damp;
Ig  = res_T.IG3_xx;
  
theta_dot_and_dotdot_fh = @(theta, theta_dot, yo_DD) ...
                           TD_and_TDD_fh(Ig,L,b,g,m,theta,theta_dot,yo_DD);    
    
                           
% define the state derivative vector
M_fh        = @(t,q) LOC_calc_q_dot(t,q, res_T, theta_dot_and_dotdot_fh);

[t, q] = bh_ode4(M_fh,[0 6],q0_vec_num,[],0.001);

%--------------------------------------------------------------------------
% ATTENTION: kept seeing warning message about NOT able to meet integration
% tolerances.  Experimented with some of teh STIFF solvers ... but also got
% the same warning.
%
% opts        = odeset;
% opts.AbsTol = 1e-2;
% opts.RelTol = 1e-4;
% [t,q]      = ode15s(M_fh, [0, 5], q0_vec_num, opts); 
% [t,q]       = ode45(M_fh, [0, 5], q0_vec_num); 
% [t,q]       = ode15s(M_fh, [0, 5], q0_vec_num); 
%--------------------------------------------------------------------------

% plot the results
figure;
subplot(2,2,[1 3])
plot(q(:,1), q(:,2), '-r');
grid('on'); xlabel('\theta (rad)'); ylabel('\theta_{DOT} (rad/sec)');
hold('on'); plot(q0_vec_num(1), q0_vec_num(2), '.b', 'MarkerSize', 25); 

subplot(2,2,2);
               plot(t, rad2deg( q(:,1) ), '-b');
               grid('on'); xlabel('t'); ylabel('\theta (degrees)'); axis('tight');
               hL = refline(0,90);
               hL.Color = 'r';
               hL.LineWidth = 2;
subplot(2,2,4);
               plot(t, rad2deg( q(:,2) ), '-k');
               grid('on'); xlabel('t'); ylabel('\theta_{DOT} (degrees/sec)'); axis('tight');
     
% what does yo_DD look like ?
the_yo_DD_num_col = LOC_calc_yo_DD_num_col( [], q, res_T);

end %  end of MAIN function
%##########################################################################
% END of MAIN function
%##########################################################################

function q_dot = LOC_calc_q_dot(t,q, res_T, theta_dot_and_dotdot_fh)

    the_yo_DD      = LOC_calc_yo_DD( t, q, res_T);
    theta          = q(1);   
    theta_dot      = q(2);

    td_and_tdd_col = theta_dot_and_dotdot_fh(theta,theta_dot,the_yo_DD);
    
    q_dot(1,1)     = td_and_tdd_col(1);
    q_dot(2,1)     = td_and_tdd_col(2);
    q_dot(3,1)     = q(4);
    q_dot(4,1)     = the_yo_DD;

    %fprintf('\n ... [%12.6f]',t);
end % function
%--------------------------------------------------------------------------
% end LOCAL function
%--------------------------------------------------------------------------
function the_yo_DD = LOC_calc_yo_DD( t, q, res_T)

    validateattributes(q, {'numeric'}, {'vector', 'numel', 4});

    mass3     = res_T.mass3;
    L3y       = res_T.L3y;
    b3        = res_T.b3_damp;

    theta     = q(1);   
    theta_dot = q(2);

    if( abs(theta) < 1e-3 )
          THE_SIGN = sign(theta);
          if(0==THE_SIGN)
              THE_SIGN = 1;
          end
          special_term = theta_dot/(THE_SIGN*1e-3);
    else
          special_term = theta_dot/sin(theta);
    end
    
    k          = res_T.k_term;
    the_yo_DD  =     k * b3 * 2 * special_term /(mass3 * L3y);

end
%--------------------------------------------------------------------------
% end LOCAL function
%--------------------------------------------------------------------------
function the_yo_DD_num_col = LOC_calc_yo_DD_num_col(t, q_mat, res_T)

    N                 = size(q_mat,1);
    the_yo_DD_num_col = zeros( N , 1);

    for kk=1:N
            theta     =  q_mat(kk,1);
            theta_dot =  q_mat(kk,2);
            yo        =  q_mat(kk,3);
            yo_dot    =  q_mat(kk,4);
            q         = [theta, theta_dot,  yo, yo_dot]';
            yo_DD_num = LOC_calc_yo_DD( [], q, res_T );

            the_yo_DD_num_col(kk) = yo_DD_num; 
    end

end % function
%--------------------------------------------------------------------------
% end LOCAL function
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
%--------------------------------------------------------------------------
% end LOCAL function
%--------------------------------------------------------------------------
function [t_col, y_mat] = bh_ode4(f,tspan,y0,options,h)

    t_end      = tspan(2);
    N_steps    = round(t_end/h);
    N          = 1 + N_steps;
    
    t_col      = h * [0:(N-1)]'       ;
    y_mat      = zeros(N, length(y0) );

    y_mat(1,:) = [y0(:)]' ;
    t_col(1)   = 0;
    
    for kk=1:N_steps
        t_kk           =  t_col(kk);
        y_kk           =  y_mat(kk,:)';
        y_next         =  LOC_rk4(f, t_kk, y_kk, h);
        
        y_mat(kk+1, :) = y_next';
    end

end % MAIN function

function y_next = LOC_rk4(f,t,y,h)
    s1 = f(t      ,  y);
    s2 = f(t + h/2,  y + h*s1/2 );
    s3 = f(t + h/2,  y + h*s2/2 );
    s4 = f(t + h  ,  y + h*s3   );

    y_next = y +  (h/6)*( s1 + 2*s2 + 2*s3 + s4);
end
