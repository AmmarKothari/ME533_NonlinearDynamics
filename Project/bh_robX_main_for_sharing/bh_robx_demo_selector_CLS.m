classdef bh_robx_demo_selector_CLS
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        ROOT_FOLDER = '';
        REF_FOLDER  = '';
    end
    
    methods
        function OBJ = bh_robx_demo_selector_CLS()
            p = mfilename('fullpath');
            [pathstr,~,~] = fileparts(p);
            OBJ.ROOT_FOLDER = pathstr;
            OBJ.REF_FOLDER  = pathstr;
        end
        %------------------------------------------------------------------
        function clear_base(OBJ)
            LOC_clear_base();
        end
        %------------------------------------------------------------------
        function close_editor(OBJ)
            LOC_close_M_editor();
        end
        %------------------------------------------------------------------
        function open_pres(OBJ)
            PPT_FILE = [OBJ.ROOT_FOLDER, filesep, 'bh_lag_dyn_robx.pptx'];
            PDF_FILE = [OBJ.ROOT_FOLDER, filesep, 'bh_lag_dyn_robx.pdf'];
            
            if exist(PPT_FILE)
                open(PPT_FILE);
                return
            end
            if exist(PDF_FILE)
                open(PDF_FILE);
                return
            end
                       
        end % open_pres
        %------------------------------------------------------------------
        function do_pc_warmup(OBJ)
                 LOC_clear_base();   LOC_close_M_editor
                 cd([OBJ.ROOT_FOLDER,filesep,'PC_warmup']);
                                 
                 LOC_clear_base();
        end
        %------------------------------------------------------------------ 
        function task_patterns_short_intro(OBJ)
                 LOC_clear_base();   LOC_close_M_editor
                 cd([OBJ.ROOT_FOLDER,filesep,'Task_patterns']);
                 
                 edit('bh_intro_num_vs_sym_comp.mlx', ...
                      'bh_short_intro_fundamental_symbolic_patterns.mlx');
                  
                 edit('bh_intro_num_vs_sym_comp.mlx') 
        end
        %------------------------------------------------------------------ 
        function task_patterns_explore_deeper(OBJ)
                 LOC_clear_base();   LOC_close_M_editor
                 cd([OBJ.ROOT_FOLDER,filesep,'Task_patterns']);
                 
                 edit(  'bh_explore_SYMBOLIC_computing_part_1.mlx', ...
                        'bh_explore_SYMBOLIC_computing_part_2.mlx' );
                 
                 edit('bh_explore_SYMBOLIC_computing_part_1.mlx');
        end        
        %------------------------------------------------------------------ 
        function position_all_MD_on_screen(OBJ)
            h_MD_list = find_system('type','block_diagram');
            % don't preocess library blockdiagrams
            isLib     = bdIsLibrary(h_MD_list);
            h_MD_list = h_MD_list(~isLib);
            %OK, so we should only have models (NOT libraries) left.
            for kk=1:length(h_MD_list)
                hMD = h_MD_list(kk);
                if(iscell(hMD))
                    hMD = hMD{1};
                end
                LOC_place_SLMODEL_onscreen(hMD)
            end
        end
        %------------------------------------------------------------------
        %_#################################################################
        %    C T     T A B
        %_#################################################################
        function task_ct_open_pres(OBJ)
            PPT_FILE = [OBJ.ROOT_FOLDER, filesep, 'bh_comp_thinking_robx.pptx'];
            PDF_FILE = [OBJ.ROOT_FOLDER, filesep, 'bh_comp_thinking_robx.pdf'];
            
            if exist(PPT_FILE)
                open(PPT_FILE);
                return
            end
            if exist(PDF_FILE)
                open(PDF_FILE);
                return
            end
                       
        end % open_pres
        %------------------------------------------------------------------
        function task_ct_open_rbd_pres(OBJ)
            PPT_FILE = [OBJ.ROOT_FOLDER, filesep, 'bh_rigid_dyn_robx.pptx'];
            PDF_FILE = [OBJ.ROOT_FOLDER, filesep, 'bh_rigid_dyn_robx.pdf'];
            
            if exist(PPT_FILE)
                open(PPT_FILE);
                return
            end
            if exist(PDF_FILE)
                open(PDF_FILE);
                return
            end
                       
        end % task_ct_open_rbd_pres
        %------------------------------------------------------------------
        function task_ct_smd_derivation(OBJ)
                 LOC_clear_base();   LOC_close_M_editor
                 cd([OBJ.ROOT_FOLDER,filesep,'Task_patterns']);
                 %edit('bh_LAGRANGE_4dof_manipulator.mlx')
                 edit('bh_smd_model_derivation.mlx');
        end
        %------------------------------------------------------------------
        function task_ct_automate(OBJ)
                 LOC_clear_base();   LOC_close_M_editor
                 edit('bh_lagr4manips_CLS.m', ...
                      'bh_genF4manips_CLS.m');
                  
                 edit('bh_lagr4manips_CLS.m');
        end
        %------------------------------------------------------------------        
        function task_ct_4_link_derivation(OBJ)
                 LOC_clear_base();   LOC_close_M_editor
                 cd([OBJ.ROOT_FOLDER,filesep,'Task_4_link']);
                 %edit('bh_LAGRANGE_4dof_manipulator.mlx')
                 edit('bh_LAGRANGE_4dof_manipulator_CT_ver.mlx');
        end
        %------------------------------------------------------------------
        function task_ct_hello_robot(OBJ)
                 LOC_clear_base();   LOC_close_M_editor
                 cd([OBJ.ROOT_FOLDER,filesep,'Task_4_link']);
                 
                 % load some parameters into the workspace
                 evalin('base', 'bh_parameters_for_4dof_manipulator_CONTROL')
                 
                 % open the model
                 MODEL = 'bh_4dof_LAGR_manipulator_CONTROL_TQ_FFWD';
                 open_system(MODEL);
                 h_MD = bdroot;
                 
                 % position model on screen
                 LOC_place_SLMODEL_onscreen(h_MD)               
        end
        %_#################################################################
        %    T R I C K S     T A B
        %_#################################################################
        function task_tricks_swingup_explore(OBJ)
                 LOC_clear_base();   LOC_close_M_editor
                 cd([OBJ.ROOT_FOLDER,filesep,'Task_swingup']);
                 edit('bh_swing_ODE_explore_SWINGUP.mlx');
        end
        %------------------------------------------------------------------
        function task_tricks_balance_explore(OBJ)
                 LOC_clear_base();   LOC_close_M_editor
                 cd([OBJ.ROOT_FOLDER,filesep,'Task_swingup']);
                 edit('bh_swing_ODE_explore_BALANCE.mlx');
        end
        %------------------------------------------------------------------
        function task_tricks_swingup_model(OBJ)
                 cd([OBJ.ROOT_FOLDER,filesep,'Task_swingup']);
                 
                 % load some parameters into the workspace
                 evalin('base', 'bh_parameters_for_4dof_manipulator_swingup')
                 
                 % open the model
                 MODEL = 'bh_proto_swingup_OUT_OF_PHASE_CONTROL';
                 open_system(MODEL);
                 h_MD = bdroot;
                 
                 % position model on screen
                 LOC_place_SLMODEL_onscreen(h_MD)               
        end
        %------------------------------------------------------------------
        function task_tricks_balance_model(OBJ)
                 cd([OBJ.ROOT_FOLDER,filesep,'Task_swingup']);
                 
                 % load some parameters into the workspace
                 evalin('base', 'bh_parameters_for_4dof_manipulator_swingup')
                 
                 % open the model
                 MODEL = 'bh_proto_balance';
                 open_system(MODEL);
                 h_MD = bdroot;
                 
                 % position model on screen
                 LOC_place_SLMODEL_onscreen(h_MD)               
        end
        %------------------------------------------------------------------
        function task_tricks_robot_balance(OBJ)
                 cd([OBJ.ROOT_FOLDER,filesep,'Task_swingup']);
                 
                 % load some parameters into the workspace
                 evalin('base', 'bh_parameters_for_4dof_manipulator_swingup')
                 
                 % open the model
                 MODEL = 'bh_CONTROL_4dof_manip_swing_balance';
                 open_system(MODEL);
                 h_MD = bdroot;
                 
                 % position model on screen
                 LOC_place_SLMODEL_onscreen(h_MD)               
        end
        %_#################################################################
        %    4 - L I N K     T A B
        %_#################################################################
        function task_4_link_lagrange_derivation(OBJ)
                 LOC_clear_base();   LOC_close_M_editor
                 cd([OBJ.ROOT_FOLDER,filesep,'Task_4_link']);
                 %edit('bh_LAGRANGE_4dof_manipulator.mlx')
                 edit('bh_LAGRANGE_4dof_manipulator.mlx','bh_intro_num_vs_sym_comp.mlx');
                 edit('bh_intro_num_vs_sym_comp.mlx');
        end
        %------------------------------------------------------------------
        function task_4_link_lagrange_model(OBJ)
                 LOC_clear_base();   LOC_close_M_editor
                 cd([OBJ.ROOT_FOLDER,filesep,'Task_4_link']);
                 
                 % load some parameters into the workspace
                 evalin('base', 'bh_parameters_for_4dof_manipulator')
                 
                 % open the model
                 MODEL = 'bh_4dof_LAGR_manipulator';
                 open_system(MODEL);
                 h_MD = bdroot;
                 
                 % position model on screen
                 LOC_place_SLMODEL_onscreen(h_MD)
                 
                 % if the model had any opened TABS ... then close them
                 LOC_close_all_but_root_level_of_model(h_MD)
                 
                 sys_1  = sprintf('%s/SIMULINK_MODEL',MODEL);
                 sys_2  = sprintf('%s/SIMSCAPE_MODEL',MODEL);
                 sys_3  = sprintf('%s/Compare_models',MODEL);
                 
                 open_system(MODEL, 'tab');
                 open_system(sys_1, 'tab');
                 open_system(sys_2, 'tab');
                 open_system(sys_3, 'tab');

                 LOC_place_scope_on_screen( 'COMP_angles' );
                 
                 open_system(MODEL, 'tab'); % bring Model tab into focus                              
        end       
        %------------------------------------------------------------------
        function task_4_link_rate_jacob_derivation(OBJ)
                 LOC_clear_base();   LOC_close_M_editor
                 cd([OBJ.ROOT_FOLDER,filesep,'Task_4_link']);
                 edit('bh_rate_jacob_4dof_manipulator_derivation.mlx') 
        end
        %------------------------------------------------------------------
        function task_4_link_rate_jacob_model(OBJ)
                 LOC_clear_base();   LOC_close_M_editor
                 cd([OBJ.ROOT_FOLDER,filesep,'Task_4_link']);
                 
                 % load some parameters into the workspace
                 evalin('base', 'bh_parameters_for_J_test_4dof_manipulator')
                 
                 % open the model
                 MODEL = 'bh_test_J_4dof_manip';
                 open_system(MODEL);
                 h_MD = bdroot;
                 
                 % position model on screen
                 LOC_place_SLMODEL_onscreen(h_MD)
                 
                 % if the model had any opened TABS ... then close them
                 LOC_close_all_but_root_level_of_model(h_MD)
                 
                 sys_1  = sprintf('%s/SIMSCAPE_MODEL',MODEL);
                 
                 open_system(MODEL, 'tab');
                 open_system(sys_1, 'tab');
                 open_system(MODEL, 'tab'); % bring Model tab into focus                               
        end              
        %------------------------------------------------------------------
        function task_4_link_invkin_via_optim(OBJ)
                 LOC_clear_base();   LOC_close_M_editor
                 cd([OBJ.ROOT_FOLDER,filesep,'Task_4_link']);
                 edit('bh_invKIN_4dof_manipulator_NUMERICAL_OPTIM.mlx') 
        end
        %------------------------------------------------------------------
        function task_4_link_CONTROL_DL(OBJ)
                 LOC_clear_base();   LOC_close_M_editor
                 cd([OBJ.ROOT_FOLDER,filesep,'Task_4_link']);
                 
                 % load some parameters into the workspace
                 evalin('base', 'bh_parameters_for_4dof_manipulator_CONTROL')
                 
                 % open the model
                 MODEL = 'bh_4dof_LAGR_manipulator_CONTROL_DL_discrete';
                 open_system(MODEL);
                 h_MD = bdroot;
                 
                 % position model on screen
                 LOC_place_SLMODEL_onscreen(h_MD)               
        end              
        %------------------------------------------------------------------
        function task_4_link_CONTROL_TQ_FFWD(OBJ)
                 LOC_clear_base();   LOC_close_M_editor
                 cd([OBJ.ROOT_FOLDER,filesep,'Task_4_link']);
                 
                 % load some parameters into the workspace
                 evalin('base', 'bh_parameters_for_4dof_manipulator_CONTROL')
                 
                 % open the model
                 MODEL = 'bh_4dof_LAGR_manipulator_CONTROL_TQ_FFWD';
                 open_system(MODEL);
                 h_MD = bdroot;
                 
                 % position model on screen
                 LOC_place_SLMODEL_onscreen(h_MD)               
        end
        %------------------------------------------------------------------
        function task_4_link_LINEARISE_the_model(OBJ)
                 LOC_clear_base();   LOC_close_M_editor
                 cd([OBJ.ROOT_FOLDER,filesep,'Task_4_link']);
                 
                 % load some parameters into the workspace
                 evalin('base', 'bh_parameters_for_4dof_manipulator_CONTROL');
                 
                 % open the script
                 edit('bh_linearise_4LINK_model.m');
        end
        %------------------------------------------------------------------
        function task_4_link_invkin_derivation(OBJ)
                 LOC_clear_base();   LOC_close_M_editor
                 cd([OBJ.ROOT_FOLDER,filesep,'Task_4_link']);
                 edit('bh_invKIN_4dof_manipulator_NUMERICAL_OPTIM.mlx')
        end
        %_#################################################################
        %    3 - L I N K     T A B
        %_#################################################################
        function task_3_link_lagrange_derivation(OBJ)
                 LOC_clear_base();   LOC_close_M_editor
                 cd([OBJ.ROOT_FOLDER,filesep,'Task_3_link']);
                 edit('bh_LAGRANGE_triple_PEND.mlx')
        end
        %------------------------------------------------------------------ 
        function task_3_link_rate_jacob_derivation(OBJ)
                 LOC_clear_base();   LOC_close_M_editor
                 cd([OBJ.ROOT_FOLDER,filesep,'Task_3_link']);
                 edit('bh_rate_jacob_triple_PEND_derivation.mlx') 
        end    
        %------------------------------------------------------------------ 
        function task_3_link_lagrange_model(OBJ)
                 LOC_clear_base();   LOC_close_M_editor
                 cd([OBJ.ROOT_FOLDER,filesep,'Task_3_link']);
                 
                 % load some parameters into the workspace
                 evalin('base', 'bh_parameters_for_TRIPLE_compound_pendulum')
                 
                 % open the model
                 MODEL = 'bh_TRI_LAGR_compound_pendulum';
                 open_system(MODEL);
                 h_MD = bdroot;
                 
                 % position model on screen
                 LOC_place_SLMODEL_onscreen(h_MD)
                 
                 % if the model had any opened TABS ... then close them
                 LOC_close_all_but_root_level_of_model(h_MD)
                 
                 sys_1  = sprintf('%s/SIMULINK_MODEL',MODEL);
                 sys_2  = sprintf('%s/SIMSCAPE_MODEL',MODEL);
                 sys_3  = sprintf('%s/Compare_models',MODEL);
                 
                 open_system(MODEL, 'tab');
                 open_system(sys_1, 'tab');
                 open_system(sys_2, 'tab');
                 open_system(sys_3, 'tab');

                 LOC_place_scope_on_screen( 'COMP_angles' );
                 
                 open_system(MODEL, 'tab'); % bring Model tab into focus                              
        end      
        %------------------------------------------------------------------ 
        function task_3_link_rate_jacob_model(OBJ)
                 LOC_clear_base();   LOC_close_M_editor
                 cd([OBJ.ROOT_FOLDER,filesep,'Task_3_link']);
                 
                 % load some parameters into the workspace
                 evalin('base', 'bh_parameters_for_J_test_TRIPLE_compound_pendulum')
                 
                 % open the model
                 MODEL = 'bh_test_J_TRI_pend';
                 open_system(MODEL);
                 h_MD = bdroot;
                 
                 % position model on screen
                 LOC_place_SLMODEL_onscreen(h_MD)
                 
                 % if the model had any opened TABS ... then close them
                 LOC_close_all_but_root_level_of_model(h_MD)
                 
                 sys_1  = sprintf('%s/SIMSCAPE_MODEL',MODEL);
                 
                 open_system(MODEL, 'tab');
                 open_system(sys_1, 'tab');
                 open_system(MODEL, 'tab'); % bring Model tab into focus                               
        end          
        %_#################################################################
        %    2 - L I N K     T A B
        %_#################################################################
        function task_2_link_newton_derivation(OBJ)
                 LOC_clear_base();   LOC_close_M_editor
                 cd([OBJ.ROOT_FOLDER,filesep,'Task_2_link']);
                 edit('bh_NEWTON_double_PEND_derivation.mlx')
        end
        %------------------------------------------------------------------         
        function task_2_link_lagrange_derivation_manual(OBJ)
                 LOC_clear_base();   LOC_close_M_editor
                 cd([OBJ.ROOT_FOLDER,filesep,'Task_2_link']);
                 edit('bh_LAGRANGE_double_PEND_MANUAL_derivation.mlx')
        end
        %------------------------------------------------------------------ 
        function task_2_link_lagrange_derivation(OBJ)
                 LOC_clear_base();   LOC_close_M_editor
                 cd([OBJ.ROOT_FOLDER,filesep,'Task_2_link']);
                 edit('bh_LAGRANGE_double_PEND.mlx')
        end
        %------------------------------------------------------------------ 
        function task_2_link_rate_jacob_derivation(OBJ)
                 LOC_clear_base();   LOC_close_M_editor
                 cd([OBJ.ROOT_FOLDER,filesep,'Task_2_link']);
                 edit('bh_invKIN_double_PEND_derivation.mlx') 
        end    
        %------------------------------------------------------------------ 
        function task_2_link_newton_model(OBJ)
                 LOC_clear_base();   LOC_close_M_editor
                 cd([OBJ.ROOT_FOLDER,filesep,'Task_2_link']);
                 
                 % load some parameters into the workspace
                 evalin('base', 'bh_parameters_for_DOUBLE_compound_pendulum')
                 
                 % open the model
                 MODEL = 'bh_DBL_NEWT_compound_pendulum';
                 open_system(MODEL);
                 h_MD = bdroot;
                 
                 % position model on screen
                 LOC_place_SLMODEL_onscreen(h_MD)
                 
                 % if the model had any opened TABS ... then close them
                 LOC_close_all_but_root_level_of_model(h_MD)
                 
                 sys_1  = sprintf('%s/SIMULINK_MODEL',MODEL);
                 sys_2  = sprintf('%s/SIMSCAPE_MODEL',MODEL);
                 sys_3  = sprintf('%s/Compare_models',MODEL);
                 
                 open_system(MODEL, 'tab');
                 open_system(sys_1, 'tab');
                 open_system(sys_2, 'tab');
                 open_system(sys_3, 'tab');

                 LOC_place_scope_on_screen( 'COMP_angles' );
                 
                 open_system(MODEL, 'tab'); % bring Model tab into focus                              
        end      
        %------------------------------------------------------------------ 
        function task_2_link_lagrange_model(OBJ)
                 LOC_clear_base();   LOC_close_M_editor
                 cd([OBJ.ROOT_FOLDER,filesep,'Task_2_link']);
                 
                 % load some parameters into the workspace
                 evalin('base', 'bh_parameters_for_DOUBLE_compound_pendulum')
                 
                 % open the model
                 MODEL = 'bh_DBL_LAGR_alt_MCKGQ_compound_pendulum';
                 open_system(MODEL);
                 h_MD = bdroot;
                 
                 % position model on screen
                 LOC_place_SLMODEL_onscreen(h_MD)
                 
                 % if the model had any opened TABS ... then close them
                 LOC_close_all_but_root_level_of_model(h_MD)
                 
                 sys_1  = sprintf('%s/SIMULINK_MODEL',MODEL);
                 sys_2  = sprintf('%s/SIMSCAPE_MODEL',MODEL);
                 sys_3  = sprintf('%s/Compare_models',MODEL);
                 
                 open_system(MODEL, 'tab');
                 open_system(sys_1, 'tab');
                 open_system(sys_2, 'tab');
                 open_system(sys_3, 'tab');

                 LOC_place_scope_on_screen( 'COMP_angles' );
                 
                 open_system(MODEL, 'tab'); % bring Model tab into focus                              
        end                              
        %------------------------------------------------------------------ 
        function task_2_link_lagrange_manual_model(OBJ)
                 LOC_clear_base();   LOC_close_M_editor
                 cd([OBJ.ROOT_FOLDER,filesep,'Task_2_link']);
                 
                 % load some parameters into the workspace
                 evalin('base', 'bh_parameters_for_DOUBLE_compound_pendulum')
                 
                 % open the model
                 MODEL = 'bh_DBL_LAGR_compound_pendulum';
                 open_system(MODEL);
                 h_MD = bdroot;
                 
                 % position model on screen
                 LOC_place_SLMODEL_onscreen(h_MD)
                 
                 % if the model had any opened TABS ... then close them
                 LOC_close_all_but_root_level_of_model(h_MD)
                 
                 sys_1  = sprintf('%s/SIMULINK_MODEL',MODEL);
                 sys_2  = sprintf('%s/SIMSCAPE_MODEL',MODEL);
                 sys_3  = sprintf('%s/Compare_models',MODEL);
                 
                 open_system(MODEL, 'tab');
                 open_system(sys_1, 'tab');
                 open_system(sys_2, 'tab');
                 open_system(sys_3, 'tab');

                 LOC_place_scope_on_screen( 'COMP_angles' );
                 
                 open_system(MODEL, 'tab'); % bring Model tab into focus                              
        end
        %------------------------------------------------------------------ 
        function task_2_link_rate_jacob_model(OBJ)
                 LOC_clear_base();   LOC_close_M_editor
                 cd([OBJ.ROOT_FOLDER,filesep,'Task_2_link']);
                 
                 % load some parameters into the workspace
                 evalin('base', 'bh_parameters_for_J_test_DOUBLE_compound_pendulum')
                 
                 % open the model
                 MODEL = 'bh_test_J_DBL_pend';
                 open_system(MODEL);
                 h_MD = bdroot;
                 
                 % position model on screen
                 LOC_place_SLMODEL_onscreen(h_MD)
                 
                 % if the model had any opened TABS ... then close them
                 LOC_close_all_but_root_level_of_model(h_MD)
                 
                 sys_1  = sprintf('%s/SIMSCAPE_MODEL',MODEL);
                 
                 open_system(MODEL, 'tab');
                 open_system(sys_1, 'tab');
                 open_system(MODEL, 'tab'); % bring Model tab into focus                               
        end        
        %_#################################################################
        %    C L A S S E S     T A B
        %_#################################################################
        function show_classes(OBJ)
                 LOC_clear_base();   LOC_close_M_editor
                 cd([OBJ.ROOT_FOLDER,filesep,'THE_LIBRARY']);
                 edit('bh_qman4manips_CLS.m', ...
                      'bh_lagr4manips_CLS.m', ...
                      'bh_genF4manips_CLS.m');
                  
                 edit('bh_qman4manips_CLS.m');
        end    
        %------------------------------------------------------------------
        function plot_hello(OBJ, hax)
            dat_TB = readtable('bh_hello_complete.xlsx', 'Sheet','COMP_SPLINE');
            plot(hax, dat_TB.XE, dat_TB.YE, '-r')
            hax.YLim = [0.4 1.2];
            hax.XLim = [0.2 1.1];
            hax.Box  = 'off';
            %hax = gca;
            %hax.Visible = 'off'; 
            hax.XTickLabel = [];
            hax.YTickLabel = [];
            hax.Title.String = '';
            hax.XLabel.String = '';
            hax.YLabel.String = '';
            grid(hax,'on');                       
        end
        %------------------------------------------------------------------ 
        %_#################################################################
        %    2 - L I N K     N O N  P L A N A R     T A B
        %_#################################################################
        function task_2dof_np_lagrange_derivation(OBJ)
                 LOC_clear_base();   LOC_close_M_editor
                 cd([OBJ.ROOT_FOLDER,filesep,'Task_2dof_non_planar']);
                 edit('bh_LAGRANGE_derivation_2dof_NP.mlx') 
        end
        %------------------------------------------------------------------ 
        function task_2dof_np_lagrange_model(OBJ)
                 LOC_clear_base();   LOC_close_M_editor
                 cd([OBJ.ROOT_FOLDER,filesep,'Task_2dof_non_planar']);
                 
                 % load some parameters into the workspace
                 evalin('base', 'bh_SIMPARAMS_for_2dof_NP')
                 
                 % open the model
                 MODEL = 'bh_2dof_NP_compare_lagr_vs_sm';
                 open_system(MODEL);
        end
        %------------------------------------------------------------------ 
        function task_2dof_np_electromech_sys_model(OBJ)
                 LOC_clear_base();   LOC_close_M_editor
                 cd([OBJ.ROOT_FOLDER,filesep,'Task_2dof_non_planar']);
                 
                 % load some parameters into the workspace
                 evalin('base', 'bh_SIMPARAMS_for_2dof_NP_Control_sim')
                 
                 % open the model
                 MODEL = 'bh_motor_and_full_mech_system';
                 open_system(MODEL);
        end
        %------------------------------------------------------------------ 
        function task_2dof_np_controlled_model(OBJ)
                 LOC_clear_base();   LOC_close_M_editor
                 cd([OBJ.ROOT_FOLDER,filesep,'Task_2dof_non_planar']);
                 
                 % load some parameters into the workspace
                 evalin('base', 'bh_SIMPARAMS_for_2dof_NP_Control_sim')
                 
                 % open the model
                 %MODEL = 'bh_2dof_NP_CONTROLLED';
                 MODEL = 'bh_2dof_NP_CONTROLLED_compare_lagr_sm';
                 open_system(MODEL);
        end
        %------------------------------------------------------------------ 
%        function open_report(OBJ, ID_str)
%            
%            % my database
%            LUP_CE = ...
%             {'PRINCIPAL_INERTIA', 'bh_task_principal_I.pdf', [OBJ.ROOT_FOLDER, filesep, 'Task_PRINCIPAL_moments_of_inertia'];
%              'EULER_RATES',       'bh_task_explore_euler_rates_CONCEPT.pdf', [OBJ.ROOT_FOLDER, filesep, 'Task_EULER_rates'];
%              'DCM_DERIVATIVE',    'bh_task_DCM_derivative.pdf', [OBJ.ROOT_FOLDER, filesep, 'Task_DCM_derivative'];
%              'CALIBRATE_TQ_F',    'bh_task_do_F_and_TQ_calibration.pdf', [OBJ.ROOT_FOLDER, filesep,'Task_CALLIBRATION_FORCE_TORQUE'];
%              'CALIBRATE_MOTOR',   'bh_task_do_motor_calibration.pdf', [OBJ.ROOT_FOLDER, filesep,'Task_CALIBRATION_electric_motor'];
%              'SOLVING_ODES',      'bh_task_integrate_6dof_MATLAB.pdf',[OBJ.ROOT_FOLDER, filesep,'Task_INTEGRATE'];
%              'CONTROL_LINEARIZE', 'bh_task_find_trim_and_linearise.pdf',[OBJ.ROOT_FOLDER, filesep,'Task_LINEARISE'];
%              'CONTROL_DESIGN',    'bh_task_siso_control_design.pdf', [OBJ.ROOT_FOLDER, filesep,'Task_LINEARISE'];
%              'PASSIVE_ROTATIONS', 'bhLIVE_TUT_rot_passive_G2B_example_1_CONCEPT.pdf', [OBJ.ROOT_FOLDER, filesep,'THE_UTILITIES',filesep,'bh_patch_rots',filesep,'TEST_SCRIPTS'];
%              'ACTIVE_ROTATIONS',  'bhLIVE_TUT_rot_ACTIVE_B2G_example_1_CONCEPT.pdf', [OBJ.ROOT_FOLDER, filesep,'THE_UTILITIES',filesep,'bh_patch_rots',filesep,'TEST_SCRIPTS'];
%              'PARALLEL_AXIS',     'bh_TUT_parallel_I.pdf', [OBJ.ROOT_FOLDER, filesep,'THE_UTILITIES',filesep,'bh_inertia',filesep,'TEST_SCRIPTS'];
%              '3_BLADED_PROPELLER','bh_TUT_3blade_propeller.pdf', [OBJ.ROOT_FOLDER, filesep,'THE_UTILITIES',filesep,'bh_inertia',filesep,'TEST_SCRIPTS'];
%             };
%             % does ID_str exist in the database
%             ID_tf_list = strcmp(LUP_CE(:,1), ID_str);
%             
%             if( ~any(ID_tf_list) )
%                 tmp_str = fprintf('Could NOT find report ID --->\n %s',ID_str);
%                 warndlg(tmp_str);
%                 return
%             end
%             
%             % get file and folder
%             kidx         = find(ID_tf_list);
%             file_str     = LUP_CE{kidx, 2};
%             folder_str   = LUP_CE{kidx, 3};
%             fullfile_str = [folder_str, filesep, file_str];
%             
%             % try and open the file
%             try
%                   open(fullfile_str);
%             catch
%                   tmp_str = fprintf('Could NOT open FILE --->\n %s',file_str);
%                   warndlg(tmp_str);
%                   return
%             end
%         end % open_report
%        %------------------------------------------------------------------ 
    end % METHODS
    
end  % CLASSDEF
%_#########################################################################
function LOC_clear_base()
   evalin('base', 'clear all; clc')
end
%**************************************************************************
function LOC_close_M_editor
%close the M-file Editor
  evalin('base',['com.mathworks.mlservices.MatlabDesktopServices.getDesktop().closeGroup(''Editor'')']);
end
%**************************************************************************
function LOC_place_SLMODEL_onscreen(hMD)

    % what is the maxscreensize of the COMPUTER
    scr_max_pos    = get(0,'ScreenSize');
    scr_max_width  = scr_max_pos(3);
    scr_max_height = scr_max_pos(4);

    tgt_md_width  = 0.8*scr_max_width;
    tgt_md_height = 0.8*scr_max_height;

    % what is current MODEL window location
    md_Loc = get_param(hMD,'Location');

    % set the new MODEl location
    set_param(hMD,'Location',[20 20 tgt_md_width tgt_md_height]);
end
%**************************************************************************
function  LOC_place_scope_on_screen( scope_name )
    h = findall(0,'Type', 'figure', 'Name', scope_name);
    if(isempty(h))
        return
    end
    
    h(1).Position = [2 41 647 634];   
end
%**************************************************************************
function  LOC_close_all_but_root_level_of_model(THE_MODEL)
    Blocks_List = find_system(THE_MODEL);
    Blocks_To_Close = Blocks_List;

    for III=length(Blocks_List):-1:1
        if isempty(strfind(Blocks_List{III},'/'))
            Blocks_To_Close(III)=[];
        end
    end

    close_system(Blocks_To_Close)
end
%**************************************************************************

