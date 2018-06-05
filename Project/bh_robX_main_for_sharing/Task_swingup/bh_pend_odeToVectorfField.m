function [q_dot_s, q_col_s] = bh_pend_odeToVectorfField( EQ_theta_DD, EQ_yo_DD, desired_list_CES  )
% EXAMPLE:
% desired_list = {'theta3','Dtheta3','yo','Dyo'};
 
% Convert into a system of 1st order ODEs:
[V,S]           = odeToVectorField(EQ_theta_DD, EQ_yo_DD);
assert(4==length(S),'ERRR .... you should have 4 state variables');

% Reorder the equations to suit our desired state vector order:
new_Y        = {};
for kk=1:length(desired_list_CES)
    tmp_str   = desired_list_CES{kk};
    ind(kk)   = find(  logical(formula(S) == sym(tmp_str))   );
    new_Y{kk} = sprintf('Y[%d]', ind(kk) );
end

S = S(ind);
V = V(ind);
warning('off','symbolic:sym:sym:DeprecateExpressions');

V = subs(V,{'Y[1]','Y[2]','Y[3]','Y[4]'}, new_Y);

% take care of outputs

q_dot_s = V;
q_col_s = S;
end

