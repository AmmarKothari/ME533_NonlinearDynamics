classdef bh_lagr4manips_CLS < bh_qman4manips_CLS
%BH_LAGR4MANIPS_CLS Lagrange's equation for dynamic systems
%   A class for computing Lagrange's equation for an open chain robotic
%   manipulator.
%==========================================================================
% EXAMPLE FILES:
%  a.) bh_LAGRANGE_double_PEND.mlx
%  b.) bh_LAGRANGE_triple_PEND.mlx
%==========================================================================

    properties (SetAccess = protected)
       T_KE    = [];  % system kinetic energy
       V_PE    = [];  % system potential energy
       L       = [];  % system Lagrangian(== KE - PE)
       Qk_list = [];  % list of Generalised Forces
    end
    
    properties (SetAccess = protected)
       EOM     = bh_eom_CLS();  % an internal data container used for 
                                 % storing the equations.
    end    
    
    properties(Access = protected)
       res_MCKGQ_T = bh_MCKGQ_CLS();  % an internal data container used for 
                                      % storing the equations.
    end
    
    methods
        function OBJ = bh_lagr4manips_CLS(T,V,actual_list_SYM_pos, holder_list_SYM_pos)
            % initialise the SUPERCLASS that we'll inherit from
            OBJ@bh_qman4manips_CLS(actual_list_SYM_pos, holder_list_SYM_pos);
            
            % populate the object
            OBJ.T_KE = formula( T );
            OBJ.V_PE = formula( V );
            OBJ.L    = formula( T - V );
            
            % construct placeholder Generalised FORCE symbols
            for kk=1:OBJ.N_dof
                tmp_str = sprintf('Q%d_s',kk);
                Qk_list(kk,1) = sym(tmp_str);
                
                % check that we have NO clashes with our POS,VEL and ACC
                % symbols
                OBJ.assert_no_symbol_clash( Qk_list(kk) );
                
                % and check our Lagrangian also
                
                the_intersect_list = intersect(symvar(OBJ.L), Qk_list(kk));
                tf_val = isempty(the_intersect_list);
                assert(tf_val,'ERR: you have a clash with our existing symbols');
            end
            OBJ.Qk_list = Qk_list;
            
        end
        %==================================================================
        function OBJ = calc_eom(OBJ, genF_OBJ)
            % ALLOWED_USAGE:
            %  OBJ = OBJ.calc_eom();
            %  OBJ = OBJ.calc_eom(genF_OBJ);
           
            % did we call this function with 2 inputs
            if(2==nargin)
               assert( isa(genF_OBJ,'bh_genF4manips_CLS') );
               
               % so populate the QK list
               Q_vec       = genF_OBJ.get_Qk('all', 'actual');
               OBJ.Qk_list = Q_vec;
            end
           
           
            % define the ACTUAL symbol list
            states_actual_list = [OBJ.actual_list_SYM_pos;
                                  OBJ.actual_list_SYM_vel  ];
                              
            % define the HOLDER symbol list
            states_holder_list = [OBJ.holder_list_SYM_pos;
                                  OBJ.holder_list_SYM_vel  ];
                                                            
            % define the time variable
            syms t
            OBJ.assert_no_symbol_clash( t );
            
            % define our EOM storage container
            EOM = bh_eom_CLS();
            
            for kk=1:OBJ.N_dof
                
                L_ORIGINAL = OBJ.L;
                                      % OLD               NEW      
                L = subs(L_ORIGINAL,  states_actual_list, states_holder_list );

                THE_q  = OBJ.holder_list_SYM_pos(kk);
                THE_qp = OBJ.holder_list_SYM_vel(kk);

                dLdqp = diff(L, THE_qp);
                                               % OLD               NEW         
                dLdqp           = subs(dLdqp,  states_holder_list, states_actual_list);  
                der_dt_of_dLdqp = diff(dLdqp, t);

                dLdq = diff(L, THE_q);
                dLdq = subs(dLdq,  states_holder_list, states_actual_list);

                eom_LHS = der_dt_of_dLdqp - dLdq;
                eom_LHS = simplify( eom_LHS );

                THE_Q   = OBJ.Qk_list(kk); % actual
                eom_RHS = simplify( THE_Q   );

                eom_LHS = formula( eom_LHS );
                eom_RHS = formula( eom_RHS );

                % now store into a struct array
                EOM(kk).actual_eom_LHS = eom_LHS;
                EOM(kk).actual_eom_RHS = eom_RHS;
                EOM(kk).actual_eom_EQ  = eom_LHS == eom_RHS;

                % store a few other useful things
                EOM(kk).actual_SYM_pos = OBJ.actual_list_SYM_pos(kk);
                EOM(kk).actual_SYM_vel = OBJ.actual_list_SYM_vel(kk);
                EOM(kk).actual_SYM_acc = OBJ.actual_list_SYM_acc(kk);
            end % for kk=1:OBJ.N_dof 
            
            % To make further expression manipulation a little easier, I'm 
            % going to use NEW symbols for our generalised co-ordinates.            
            
            OLD_list = [ OBJ.actual_list_SYM_pos;
                         OBJ.actual_list_SYM_vel;
                         OBJ.actual_list_SYM_acc ];
                     
            new_list = [ OBJ.holder_list_SYM_pos;
                         OBJ.holder_list_SYM_vel;
                         OBJ.holder_list_SYM_acc ];

            for kk =1:length(EOM)
               EOM(kk).holder_eom_LHS = subs(EOM(kk).actual_eom_LHS,  OLD_list, new_list);
               EOM(kk).holder_eom_RHS = subs(EOM(kk).actual_eom_RHS,  OLD_list, new_list);
               EOM(kk).holder_eom_EQ  = subs(EOM(kk).actual_eom_EQ,   OLD_list, new_list);
            end            
            
            % And finally assign to our object
            OBJ.EOM = EOM;
        end % calc_eom()
        %==================================================================
        function show_eom(OBJ)
            % USAGE:
            %  >> OBJ.show_eom()
            
            % So what do the equations of motion actually look like ?
            for kk=1:OBJ.N_dof
               fprintf('\n%s',repmat('#',1,55));
               fprintf('\n### q = %s', char(OBJ.actual_list_SYM_pos(kk)) );
               fprintf('\n### ');
               fprintf('\n### LHS of EOM is: ');
               fprintf('\n### ');
               
               tmp_str = char(OBJ.EOM(kk).actual_eom_LHS );
               fprintf('\n        %s', tmp_str)
               
               fprintf('\n### ');
               fprintf('\n### RHS of EOM is: '); 

               tmp_str = char(OBJ.EOM(kk).actual_eom_RHS );
               fprintf('\n        %s', tmp_str)
            end    
        end
        %==================================================================
        function show_eom_in_txt_file(OBJ)
            % USAGE:
            %  >> OBJ.show_eom_in_txt_file()
            
            % Open or create new file for reading and writing. 
            % Discard existing contents, if any.
            THE_FILENAME = 'bh_tmp_EOM_file_WILL_BE_DELETED.txt';
            fid          = fopen(THE_FILENAME, 'w+');
            if(-1==fid)
                error('###_ERROR:  could NOT open tmp file <%s>',THE_FILENAME);
            end
            
            fprintf(fid, '%s',repmat('+',1,75)  );
            fprintf(fid, '\n ATTENTION:');
            fprintf(fid, '\n This text file was generated by the <%s>',mfilename);
            fprintf(fid, '\n %s', datestr(now) );
            fprintf(fid, '\n%s',repmat('+',1,75)  );
            fprintf(fid, '\n');
            
            % So what do the equations of motion actually look like ?
            for kk=1:OBJ.N_dof
                
               % do the LHS
               fprintf(fid,'\n%s',repmat('#',1,75));
               fprintf(fid,'\n### q = %s', char(OBJ.holder_list_SYM_pos(kk)) );
               fprintf(fid,'\n### ');
               fprintf(fid,'\n### LHS of EOM is: ');
               fprintf(fid,'\n### ');
               A = children( OBJ.EOM(kk).holder_eom_LHS ).' ;
               for dd=1:length(A)
                   if(1==dd)
                        fprintf(fid,'\n %3d       %s',dd,A(dd));
                   else
                        fprintf(fid,'\n %3d   +   %s',dd,A(dd));
                   end
               end
               
               % do the RHS
               fprintf(fid,'\n### ');
               fprintf(fid,'\n### RHS of EOM is: '); 
               B = children( OBJ.EOM(kk).holder_eom_RHS ).' ;
               for dd=1:length(B)
                   if(1==dd)
                        fprintf(fid,'\n %3d       %s',dd,B(dd));
                   else
                        fprintf(fid,'\n %3d   +   %s',dd,B(dd));
                   end
               end
            end   % for kk=1:OBJ.N_dof
            
            % close the file handle
            fclose(fid);
        end
        %==================================================================
        function the_eom = get_eom(OBJ, kk, type_str, side_str)
           % ALLOWED_USAGE:
           %  the_eom  = OBJ.get_eom(    1, 'actual', 'LHS')
           %  the_eom  = OBJ.get_eom(    1, 'actual', 'RHS')
           %  the_eom  = OBJ.get_eom(    1, 'actual', 'EQ')
           %
           %  the_eom  = OBJ.get_eom(    1, 'holder')
           %
           % the_eom = OBJ.get_eom('all', 'actual')
           % the_eom = OBJ.get_eom(  1:5, 'actual')
           %
           % the_eom = OBJ.get_eom('all', 'holder')
           % the_eom = OBJ.get_eom(  1:5, 'holder')
           
           validateattributes(kk, {'numeric', 'char'}, {'2d'});           
           
           if(isa(kk,'numeric'))
               % do nothing
           elseif(isa(kk,'char'))
                   assert(strcmp(lower(kk),'all'), 'ERR: unknown str input');
                   % make kk a numeric list
                   kk=1:OBJ.N_dof;
           end
           
           assert( max(kk) <= OBJ.N_dof, 'ERR: your kk was greater than N_dof');
           
           action_str = upper( [type_str, '_', side_str] );
           
           switch(action_str)
               case {'ACTUAL_LHS'}
                   the_eom = [ OBJ.EOM(kk).actual_eom_LHS ];
               case {'ACTUAL_RHS'}
                   the_eom = [ OBJ.EOM(kk).actual_eom_RHS ];
               case {'ACTUAL_EQ'}
                   the_eom = [ OBJ.EOM(kk).actual_eom_EQ ];
               case {'HOLDER_LHS'}
                   the_eom = [ OBJ.EOM(kk).holder_eom_LHS ];
               case {'HOLDER_RHS'}
                   the_eom = [ OBJ.EOM(kk).holder_eom_RHS ];
               case {'HOLDER_EQ'}
                   the_eom = [ OBJ.EOM(kk).holder_eom_EQ ];
               otherwise
                  error('ERR: unknown type str');     
           end
           
           % retun as a column
           the_eom = the_eom(:);
        end
        %==================================================================
        function OBJ = create_MCKGQ(OBJ)
            ACC_col = OBJ.holder_list_SYM_acc;
            VEL_col = OBJ.holder_list_SYM_vel;
            POS_col = OBJ.holder_list_SYM_pos;

            NUM_DOF = OBJ.N_dof;
            M = zeros(NUM_DOF, NUM_DOF, 'sym');
            C = zeros(NUM_DOF, NUM_DOF, 'sym');
            K = zeros(NUM_DOF, NUM_DOF, 'sym');
            G = zeros(NUM_DOF, 1,       'sym');
            Q = zeros(NUM_DOF, 1,       'sym');

            for kk=1:NUM_DOF

                EOM_ele = OBJ.EOM(kk);
                
                [M_row, C_row, K_row, G_row, Q_row] = ...
                     bh_create_M_C_K_G_rows( EOM_ele, ...
                                             POS_col, ...
                                             VEL_col, ...
                                             ACC_col);

                M(kk,:) = M_row;
                C(kk,:) = C_row;
                K(kk,:) = K_row;
                G(kk,:) = G_row;
                Q(kk,:) = Q_row;
            end

            res_T   = bh_MCKGQ_CLS();
            
            res_T.M = simplify(M);
            res_T.C = simplify(C);
            res_T.K = simplify(K);
            res_T.G = simplify(G);
            res_T.Q = simplify(Q);


            res_T.ACC_col = ACC_col;
            res_T.VEL_col = VEL_col;
            res_T.POS_col = POS_col;

            % From this:
            %                   M*ACC + C*VEL + K*POS + G = Q
            % We can get this:
            %                   ACC = inv(M) * ( Q - G - K*POS - C*VEL )

            res_T.acc_eoms = inv(M) * (Q - G - K*POS_col - C*VEL_col);
            %res_T.acc_eoms = simplify( res_T.acc_eoms ); 
            
            % and assign to our object
            OBJ.res_MCKGQ_T = res_T;
        end
        %==================================================================
        function res_T = get_MCKGQ(OBJ)
                 res_T = OBJ.res_MCKGQ_T;
        end
        %==================================================================
        function create_MLF_blocks(OBJ)
            %Now create the MATLAB function blocks for Simulink:
            MODEL_NAME = 'bh_tmp_model_for_lagr_WILL_BE_DELETED';
            if(4==exist(MODEL_NAME))
                close_system(MODEL_NAME, 0);
                delete(MODEL_NAME);
            end
            new_system(MODEL_NAME)
            open_system(MODEL_NAME)      

            % the Theta DOT DOT equations
            matlabFunctionBlock( [MODEL_NAME,'/THE_TH_DD'], ...
                                 OBJ.res_MCKGQ_T.acc_eoms, ...
                                 'Optimize', false, ...
                                 'Outputs', {'theta_DD'}   );

            % the M, C, K, G, Q                     
            matlabFunctionBlock( [MODEL_NAME,'/MCKGQ'], ... 
                                 OBJ.res_MCKGQ_T.M, ...
                                 OBJ.res_MCKGQ_T.C, ...
                                 OBJ.res_MCKGQ_T.K, ...
                                 OBJ.res_MCKGQ_T.G, ...
                                 OBJ.res_MCKGQ_T.Q, ...                         
                                 'Optimize', false, ...
                                 'Outputs', {'M', 'C', 'K', 'G', 'Q'}   );
            set_param([MODEL_NAME,'/MCKGQ'],'BackgroundColor', '[0.996078, 1.000000, 0.705882]');                 
            % set the background color of the MCKGQ block
            %set_param([MODEL_NAME,'/MCKGQ'],'BackgroundColor',[1, 0.650980, 0.890196])
        end        
        %==================================================================
    end % METHODS
    
end
%_#########################################################################
%_ END of CLASSDEF
%_#########################################################################
function [M_row, C_row, K_row, G_row, Q_row] = ...
         bh_create_M_C_K_G_rows( EOM_ele, POS_list, VEL_list, ACC_list )
%    EOM(kk).holder_eom_LHS = subs(EOM(kk).actual_eom_LHS,  OLD_list, new_list);
%    EOM(kk).holder_eom_RHS = subs(EOM(kk).actual_eom_RHS,  OLD_list, new_list);
%    EOM(kk).holder_eom_EQ  = subs(EOM(kk).actual_eom_EQ,   OLD_list, new_list);
%     

eom_LHS = EOM_ele.holder_eom_LHS;
eom_RHS = EOM_ele.holder_eom_RHS;
NUM_DOF = length(POS_list);

M_row = zeros(1,NUM_DOF,'sym');
C_row = zeros(1,NUM_DOF,'sym');
K_row = zeros(1,NUM_DOF,'sym'); 
G_row = zeros(1,1,'sym');
Q_row = zeros(1,1,'sym');

%Q_row = EOM_ele.holder_eom_RHS;
Q_row = eom_RHS;

% create a list of ALL of the individual terms in this expression
%express_list = children( eom_LHS );
express_list = children( expand(eom_LHS) );

% process each term in the list
for kk=1:length(express_list)
    the_term = express_list(kk);
    
    % does the term contain ACCELERATION symbols
    [tf_has_acc_term, this_M_row] = LOC_create_M_row(the_term, ACC_list);
    if(true==tf_has_acc_term)
        M_row = M_row + this_M_row;
        continue
    end
    
    % does the term contain VELOCITY symbols
    [tf_has_vel_term, this_C_row] = LOC_create_C_row(the_term, VEL_list);
    if(true==tf_has_vel_term)
        C_row = C_row + this_C_row;
        continue
    end
    
    % does the term contain POSITION symbols
    [tf_has_pos_term, this_K_row] = LOC_create_K_row(the_term, POS_list);
    if(true==tf_has_pos_term)
        K_row = K_row + this_K_row;
        continue
    end
    
    % and if we get to here .... you're in the G_row
    G_row = G_row + the_term;
    
end % for kk=1:length(express_list)

% just make sure that nothing silly happened:
my_tmp = M_row*ACC_list(:) + ...
         C_row*VEL_list(:) + ...
         K_row*POS_list(:) + ...
         G_row;
     
my_diff = my_tmp - eom_LHS;
my_diff = simplify(my_diff); %should be ZERO
assert(isequal(my_diff,sym(0)), 'ERR: WHY did we NOT get expression equivalence!');

end
%_=========================================================================
%  END of SUB function
%_=========================================================================
function [tf_has_acc_term, M_row] = LOC_create_M_row(the_term, ACC_list)
    N_DOF = length(ACC_list);
    M_row = zeros(1, N_DOF, 'sym');

    tf_has_acc_term = false;
    
    for aa=1:length(ACC_list)
        the_acc_sym = ACC_list(aa);
        [C, T]      = coeffs(the_term, the_acc_sym, 'All');
        
        if( length(C)>1 )
            assert( 2==length(C) ); 
            M_row(1,aa) = C(1); 
            tf_has_acc_term = true;
            break;
        end
    end % aa
end
%_=========================================================================
%  END of SUB function
%_=========================================================================
function [tf_has_vel_term, C_row] = LOC_create_C_row(the_term, VEL_list)
    N_DOF = length(VEL_list);
    C_row = zeros(1, N_DOF, 'sym');
    
    tf_has_vel_term = false;

    for vv=1:length(VEL_list)
        the_vel_sym = VEL_list(vv);
        [C, T]      = coeffs(the_term, the_vel_sym, 'All');
        % example output:
        %     syms x
        %     [C,T] = coeffs(10*x^3, x, 'All')
        %
        %      C = 
        %          [ 10,   0,   0,  0]
        %      T =
        %          [ x^3, x^2,  x,  1]
        %-------------------------------------------
        %  Np = 3 = numel(C) - 1
        %
        % So what I want to do is:  C(1)*x^2:
        %
        %  C_row = 10*x^2 
        %-------------------------------------------
        
        if( length(C)>1 )
            %assert( 2==length(C) ); 
            n           = length(C); 
            pow_term    = the_vel_sym^(n-2);
            C_row(1,vv) = C(1) * pow_term; 
            tf_has_vel_term = true;
            break;
        end
    end % vv
end
%_=========================================================================
%  END of SUB function
%_=========================================================================
function [tf_has_pos_term, K_row] = LOC_create_K_row(the_term, POS_list)
    N_DOF = length(POS_list);
    K_row = zeros(1, N_DOF, 'sym');
    
    tf_has_pos_term = false;

    for pp=1:length(POS_list)
        the_pos_sym = POS_list(pp);
        [C, T]      = coeffs(the_term, the_pos_sym, 'All');
        
        % look for linear terms ONLY. Anything NOT linear
        % we'll let it drop through to the G vector
        if( length(C)==2 )
            K_row(1,pp) = C(1); 
            tf_has_pos_term = true;
            break;
        end
        
        %--------------------------------------------------
        %ORIG:
        %if( length(C)>1 )
        %    %assert( 2==length(C) ); % ###
        %    K_row(1,pp) = C(1); 
        %    tf_has_pos_term = true;
        %    break;
        %end
        %---------------------------------------------------
        
    end % pp
end
%_=========================================================================
%  END of SUB function
%_=========================================================================


