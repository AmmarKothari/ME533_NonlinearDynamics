classdef bh_qman4manips_CLS
%BH_QMAN4MANIPS_CLS A dof access suport class
%   A class for accessing dof symbols.
%   It is expected that this class will be inherited by:
%        a.) BH_LAGR4MANIPS_CLS
%        b.) BH_GENF4MANIPS_CLS
%==========================================================================
% EXAMPLE FILES:
%  see bh_test_qman4manips.m
%==========================================================================
% EXAMPLE_USAGE: - PART 1, creation
%
% syms t theta1(t)  TH1_s  
% syms   theta2(t)  TH2_s  
% 
% actual_list_SYM_pos = formula( [     theta1,          theta2    ]   );
% holder_list_SYM_pos =          [        TH1_s,           TH2_s  ];
% 
% OBJ = bh_qman4manips_CLS(actual_list_SYM_pos, holder_list_SYM_pos)
%==========================================================================
% EXAMPLE_USAGE: - PART 2, get q
%
% q = OBJ.get_q(1,     'actual')
% q = OBJ.get_q(1:2,   'actual')
% q = OBJ.get_q('all', 'actual')
% 
% q = OBJ.get_q(1,     'holder')
% q = OBJ.get_q(1:2,   'holder')
% q = OBJ.get_q('all', 'holder')
%==========================================================================
% EXAMPLE_USAGE: - PART 3, get q DOT
%
% q = OBJ.get_qD(1,     'actual')
% q = OBJ.get_qD(1:2,   'actual')
% q = OBJ.get_qD('all', 'actual')
% 
% q = OBJ.get_qD(1,     'holder')
% q = OBJ.get_qD(1:2,   'holder')
% q = OBJ.get_qD('all', 'holder')
%==========================================================================
% EXAMPLE_USAGE: - PART 4, get q DOT DOT
%
% q = OBJ.get_qDD(1,     'actual')
% q = OBJ.get_qDD(1:2,   'actual')
% q = OBJ.get_qDD('all', 'actual')
% 
% q = OBJ.get_qDD(1,     'holder')
% q = OBJ.get_qDD(1:2,   'holder')
% q = OBJ.get_qDD('all', 'holder')
%==========================================================================

    properties (SetAccess = protected)
        N_dof               = 0;
        
        actual_list_SYM_pos = [];
        actual_list_SYM_vel = [];
        actual_list_SYM_acc = [];

        holder_list_SYM_pos = [];
        holder_list_SYM_vel = [];
        holder_list_SYM_acc = [];
    end
    
    methods
        function OBJ = bh_qman4manips_CLS(actual_list_SYM_pos, holder_list_SYM_pos)
            
           % validate the inputs
           validateattributes(actual_list_SYM_pos, {'sym', 'symfun'}, {'vector'});           
           validateattributes(holder_list_SYM_pos, {'sym', 'symfun'}, {'vector'});           

           % assert that boths lists are the same length
           N_act = numel(actual_list_SYM_pos);
           N_hol = numel(holder_list_SYM_pos);
           assert(N_act==N_hol, 'ERR: why diferent length lists ?')
            
           % If the inputs are SYMFUN, then convert to SYM
           actual_list_SYM_pos = formula(actual_list_SYM_pos);
           holder_list_SYM_pos = formula(holder_list_SYM_pos);
           
           % assert that our actuals are dependent on t
           LOC_assert_actuals_depend_on_t(actual_list_SYM_pos);
           
           % just make things COLUMNS
           actual_list_SYM_pos = actual_list_SYM_pos(:);
           holder_list_SYM_pos = holder_list_SYM_pos(:);
                  
           % create VELOCITIES and ACCELERATIONS
           [act_vel, act_acc, ...
            hol_vel, hol_acc] = LOC_create_vel_and_acc( actual_list_SYM_pos, ...
                                                        holder_list_SYM_pos);
           
           % now populate our class properties
           OBJ.N_dof = N_act;
           
           % NOTE: note how we are creating COLUMNS via (:)
           OBJ.actual_list_SYM_pos = actual_list_SYM_pos;
           OBJ.actual_list_SYM_vel = act_vel;
           OBJ.actual_list_SYM_acc = act_acc;
           
           OBJ.holder_list_SYM_pos = holder_list_SYM_pos;
           OBJ.holder_list_SYM_vel = hol_vel;
           OBJ.holder_list_SYM_acc = hol_acc;
        end
        %==================================================================
        function q = get_q(OBJ, kk, type_str)
           % ALLOWED_USAGE:
           %  q    = OBJ.get_q(    1, 'actual')
           %  q    = OBJ.get_q(    1, 'holder')
           %
           % q_vec = OBJ.get_q('all', 'actual')
           % q_vec = OBJ.get_q(  1:5, 'actual')
           %
           % q_vec = OBJ.get_q('all', 'holder')
           % q_vec = OBJ.get_q(  1:5, 'holder')
           
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
                   q = OBJ.actual_list_SYM_pos(kk);
               case {'HOLDER'}
                   q = OBJ.holder_list_SYM_pos(kk);
               otherwise
               error('ERR: unknown type str');     
           end
        end
        %==================================================================
        function q = get_qD(OBJ, kk, type_str)
           % ALLOWED_USAGE:
           %  q    = OBJ.get_qD(    1, 'actual')
           %  q    = OBJ.get_qD(    1, 'holder')
           %
           % q_vec = OBJ.get_qD('all', 'actual')
           % q_vec = OBJ.get_qD(  1:5, 'actual')
           %
           % q_vec = OBJ.get_qD('all', 'holder')
           % q_vec = OBJ.get_qD(  1:5, 'holder')
           
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
                   q = OBJ.actual_list_SYM_vel(kk);
               case {'HOLDER'}
                   q = OBJ.holder_list_SYM_vel(kk);
               otherwise
               error('ERR: unknown type str');     
           end
        end
        %==================================================================
        function q = get_qDD(OBJ, kk, type_str)
           % ALLOWED_USAGE:
           %  q    = OBJ.get_qDD(    1, 'actual')
           %  q    = OBJ.get_qDD(    1, 'holder')
           %
           % q_vec = OBJ.get_qDD('all', 'actual')
           % q_vec = OBJ.get_qDD(  1:5, 'actual')
           %
           % q_vec = OBJ.get_qDD('all', 'holder')
           % q_vec = OBJ.get_qDD(  1:5, 'holder')
           
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
                   q = OBJ.actual_list_SYM_acc(kk);
               case {'HOLDER'}
                   q = OBJ.holder_list_SYM_acc(kk);
               otherwise
               error('ERR: unknown type str');     
           end
        end
        %==================================================================
        function show_q(OBJ)
            show_q_actual(OBJ);
            show_q_holder(OBJ);
        end
        %==================================================================
        function show_q_actual(OBJ)
            
            fprintf('\n%s', repmat('-',1,50));
            fprintf('\n ACTUALS:');
            fprintf('\n');
            fprintf('\n COL #1 : POSITION');
            fprintf('\n COL #2 : VELOCITY');
            fprintf('\n COL #3 : ACCELERATION');
            fprintf('\n ');
            
            [OBJ. get_q(  'all','actual'), ...
             OBJ. get_qD( 'all','actual'), ...
             OBJ. get_qDD('all','actual')]
        end
        %==================================================================
        function show_q_holder(OBJ)
            
            fprintf('\n%s', repmat('-',1,50));
            fprintf('\n HOLDER:');
            fprintf('\n');
            fprintf('\n COL #1 : POSITION');
            fprintf('\n COL #2 : VELOCITY');
            fprintf('\n COL #3 : ACCELERATION');
            fprintf('\n ');
            
            [OBJ. get_q(  'all','holder'), ...
             OBJ. get_qD( 'all','holder'), ...
             OBJ. get_qDD('all','holder')]
        end
        %==================================================================
        function assert_no_symbol_clash(OBJ, some_sym)
        
           all_of_my_symbols = [ 
                OBJ.actual_list_SYM_pos;
                OBJ.actual_list_SYM_vel;
                OBJ.actual_list_SYM_acc;

                OBJ.holder_list_SYM_pos;
                OBJ.holder_list_SYM_vel;
                OBJ.holder_list_SYM_acc;
                ];
            
            the_intersect_list = intersect(all_of_my_symbols, some_sym);
            tf_val = isempty(the_intersect_list);
            
            assert(tf_val,'ERR: you have a clash with our existing symbols');
            
        end
        %==================================================================
    end % METHODS
    
end
%_#########################################################################
%_ END of CLASSDEF
%_#########################################################################
function LOC_assert_actuals_depend_on_t(actual_list_SYM_pos)

    for kk=1:numel(actual_list_SYM_pos)
       list_CE = symvar(char(actual_list_SYM_pos(kk)));

       assert(1==numel(list_CE), 'ERR: why not exactly one variable');
       assert(strcmp('t', list_CE{1}), 'ERR: NO a function of "t" ');
    end

end
%==========================================================================
function [act_vel, act_acc, ...
          hol_vel, hol_acc] = LOC_create_vel_and_acc( actual_list_SYM_pos, ...
                                                      holder_list_SYM_pos)
    N = length(actual_list_SYM_pos);
    
    % create the actual vels and acc
    for kk=1:N
        q             = actual_list_SYM_pos(kk);
        act_vel(kk,1) = diff(q,1);
        act_acc(kk,1) = diff(q,2);
    end
    
    % create the holder vels and acc
    for kk=1:N
        q             = holder_list_SYM_pos(kk);
        hol_vel(kk,1) = sym([char(q),'_D']);
        hol_acc(kk,1) = sym([char(q),'_DD']);
    end
    
end
%==========================================================================
                                                    
