classdef bh_eom_CLS
%BH_EOM_CLS a suuport clssed used by bh_lagr4manips_CLS 
%   This class is intended for use by *ONLY* the bh_lagr4manips_CLS
    
    properties
        actual_eom_LHS = sym(0);
        actual_eom_RHS = sym(0);
         actual_eom_EQ = sym(0);
         
        holder_eom_LHS = sym(0);
        holder_eom_RHS = sym(0);
         holder_eom_EQ = sym(0);
         
        actual_SYM_pos = sym(0);
        actual_SYM_vel = sym(0);
        actual_SYM_acc = sym(0);
    end
end
%_#########################################################################
%_ END of CLASSDEF
%_#########################################################################


