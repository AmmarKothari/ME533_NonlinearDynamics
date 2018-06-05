classdef bh_MCKGQ_CLS
%BH_MCKGQ_CLS a suuport clssed used by bh_lagr4manips_CLS 
%   This class is intended for use by *ONLY* the bh_lagr4manips_CLS
    
    properties
               M = sym(0);
               C = sym(0);
               K = sym(0);
               G = sym(0);
               Q = sym(0);
         ACC_col = sym(0);
         VEL_col = sym(0);
         POS_col = sym(0);
        acc_eoms = sym(0);
    end
end
%_#########################################################################
%_ END of CLASSDEF
%_#########################################################################

