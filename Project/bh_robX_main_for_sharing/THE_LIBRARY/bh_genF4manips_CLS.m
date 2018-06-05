classdef bh_genF4manips_CLS < bh_qman4manips_CLS
%BH_GENF4MANIPS_CLS calculating GENERALISED FORCES
%   A class for computing the GENERALISED forces for an open chain robotic
%   manipulator.
%==========================================================================
% EXAMPLE FILES:
%  a.) bh_LAGRANGE_double_PEND.mlx
%  b.) bh_LAGRANGE_triple_PEND.mlx
%==========================================================================
% EXAMPLE_USAGE: - PART 1, creation
%
% OBJ_genF = bh_genF4manips_CLS( the_tau_mat_actual, ...
%                                the_w_mat_actual, ...
%                                actual_list_SYM_pos, ...
%                                holder_list_SYM_pos);
%==========================================================================
% EXAMPLE_USAGE: - PART 2, calculate the Generalised forces
%
%  OBJ_genF = OBJ_genF.calc_genF()
%==========================================================================
% EXAMPLE_USAGE: - PART 3, echo the Generalised forces
%
%   OBJ_genF.show_genF_holder()
%   OBJ_genF.show_genF_actual()
%   OBJ_genF.show_genF()
%==========================================================================
% EXAMPLE_USAGE: - PART 4, retrieve the Generalised forces
%
%     Q    = OBJ.get_Qk(    1, 'actual')
%     Q    = OBJ.get_Qk(    1, 'holder')
% 
%    Q_vec = OBJ.get_Qk('all', 'actual')
%    Q_vec = OBJ.get_Qk(  1:5, 'actual')
% 
%    Q_vec = OBJ.get_Qk('all', 'holder')
%    Q_vec = OBJ.get_Qk(  1:5, 'holder')
%==========================================================================

    properties (SetAccess = protected)
        holder_eom_Q;
        actual_eom_Q;
        THE_CORE;
    end
    
    methods
        function OBJ = bh_genF4manips_CLS(the_tau_mat_actual, ...
                                          the_w_mat_actual, ...
                                          actual_list_SYM_pos, ...
                                          holder_list_SYM_pos)
            % initialise the SUPERCLASS that we'll inherit from
            OBJ@bh_qman4manips_CLS(actual_list_SYM_pos, holder_list_SYM_pos);
            
            % populate the class
            THE_CORE.the_tau_mat_actual = formula( the_tau_mat_actual );
            THE_CORE.the_w_mat_actual   = formula( the_w_mat_actual   );
            
            OLD_list = [ OBJ.actual_list_SYM_pos;
                         OBJ.actual_list_SYM_vel;
                         OBJ.actual_list_SYM_acc ];
                     
            new_list = [ OBJ.holder_list_SYM_pos;
                         OBJ.holder_list_SYM_vel;
                         OBJ.holder_list_SYM_acc ];
            
           THE_CORE.the_tau_mat_holder = subs(THE_CORE.the_tau_mat_actual, ...
                                               OLD_list, new_list);
                                           
           THE_CORE.the_w_mat_holder   = subs(THE_CORE.the_w_mat_actual, ...
                                               OLD_list, new_list);
                                           
           OBJ.THE_CORE = THE_CORE;                                
        end
        %==================================================================
        function OBJ = calc_genF(OBJ)
            
            N_dof       = OBJ.N_dof;
            the_tau_mat = OBJ.THE_CORE.the_tau_mat_holder;
            the_w_mat   = OBJ.THE_CORE.the_w_mat_holder;
            
            for kk =1:N_dof

                the_qdot_sym = OBJ.holder_list_SYM_vel(kk);

                % initialise the Qk
                the_Q        = sym(0);

                for jj=1:size(the_tau_mat,2) 
                    the_tau_col  = the_tau_mat(:,jj);
                    the_w_col    =   the_w_mat(:,jj);

                    the_dwdq_col = diff(the_w_col, the_qdot_sym);

                    % now do the DOT product
                    this_Q       = sum( the_tau_col.* the_dwdq_col   );

                    % accumulate 
                    the_Q = the_Q + this_Q;
                end % jj

                % assign the final holder result
                the_holder_eom_Q(kk,1) = the_Q;
                
                % create and assign the ACTUAL symbol result
                act_list = [ OBJ.actual_list_SYM_pos;
                             OBJ.actual_list_SYM_vel;
                             OBJ.actual_list_SYM_acc ];

                hol_list = [ OBJ.holder_list_SYM_pos;
                             OBJ.holder_list_SYM_vel;
                             OBJ.holder_list_SYM_acc ];

               the_actual_eom_Q(kk,1) = subs( the_holder_eom_Q(kk),  ...
                                              hol_list, act_list);
            end % kk
            
            % assign into our object
            OBJ.holder_eom_Q = the_holder_eom_Q;
            OBJ.actual_eom_Q = the_actual_eom_Q;
            
        end % calc_genF()
        %==================================================================
        function Q = get_Qk(OBJ, kk, type_str)
           % ALLOWED_USAGE:
           %  Q    = OBJ.get_Qk(    1, 'actual')
           %  Q    = OBJ.get_Qk(    1, 'holder')
           %
           % Q_vec = OBJ.get_Qk('all', 'actual')
           % Q_vec = OBJ.get_Qk(  1:5, 'actual')
           %
           % Q_vec = OBJ.get_Qk('all', 'holder')
           % Q_vec = OBJ.get_Qk(  1:5, 'holder')
           
           validateattributes(kk, {'numeric', 'char'}, {'2d'});           
           
           if(isa(kk,'numeric'))
               % do nothing
           elseif(isa(kk,'char'))
                   assert(strcmp(lower(kk),'all'), 'ERR: unknown str input');
                   % make kk a numeric list
                   kk=1:OBJ.N_dof;
           end           
           assert( max(kk) <= OBJ.N_dof, 'ERR: your kk was greater than N_dof');
           switch(upper(type_str))
               case {'ACTUAL'}
                   Q = OBJ.actual_eom_Q(kk);
               case {'HOLDER'}
                   Q = OBJ.holder_eom_Q(kk);
               otherwise
               error('ERR: unknown type str');     
           end
           
           % just make Q a column
           Q = Q(:);
        end
        %==================================================================
        function show_genF(OBJ)
            show_genF_actual(OBJ);
            show_genF_holder(OBJ);
        end
        %==================================================================
        function show_genF_actual(OBJ)
            % So what do the generalised FORCES actually look like ?
            for kk=1:OBJ.N_dof
               fprintf('\n%s',repmat('#',1,55));
               fprintf('\n### q = %s', char(OBJ.actual_list_SYM_pos(kk)) );
               fprintf('\n### ');
               fprintf('\n### Qk (ACTUALS) is: ');
               fprintf('\n### ');
               
               tmp_str = char(OBJ.actual_eom_Q(kk) );
               
               fprintf('\n      %s', tmp_str)
            end    
        end
        %==================================================================
        function show_genF_holder(OBJ)
            % So what do the generalised FORCES actually look like ?
            for kk=1:OBJ.N_dof
               fprintf('\n%s',repmat('#',1,55));
               fprintf('\n### q = %s', char(OBJ.holder_list_SYM_pos(kk)) );
               fprintf('\n### ');
               fprintf('\n### Qk (HOLDER) is: ');
               fprintf('\n### ');
               
               tmp_str = char(OBJ.holder_eom_Q(kk) );
               
               fprintf('\n      %s', tmp_str)
            end    
        end
        %==================================================================
        
    end % METHODS
    
end
%_#########################################################################
%_ END of CLASSDEF
%_#########################################################################


