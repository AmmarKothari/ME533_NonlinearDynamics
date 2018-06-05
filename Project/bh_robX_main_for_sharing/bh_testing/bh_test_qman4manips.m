
syms t theta1(t)  TH1_s
syms   theta2(t)  TH2_s

actual_list_SYM_pos = formula( [     theta1,          theta2]     );
holder_list_SYM_pos = [        TH1_s,           TH2_s  ];

OBJ = bh_qman4manips_CLS(actual_list_SYM_pos, holder_list_SYM_pos)

% get q
q = OBJ.get_q(1,     'actual')
q = OBJ.get_q(1:2,   'actual')
q = OBJ.get_q('all', 'actual')

q = OBJ.get_q(1,     'holder')
q = OBJ.get_q(1:2,   'holder')
q = OBJ.get_q('all', 'holder')

% get q DOT
q = OBJ.get_qD(1,     'actual')
q = OBJ.get_qD(1:2,   'actual')
q = OBJ.get_qD('all', 'actual')

q = OBJ.get_qD(1,     'holder')
q = OBJ.get_qD(1:2,   'holder')
q = OBJ.get_qD('all', 'holder')

% get q DOT DOT
q = OBJ.get_qDD(1,     'actual')
q = OBJ.get_qDD(1:2,   'actual')
q = OBJ.get_qDD('all', 'actual')

q = OBJ.get_qDD(1,     'holder')
q = OBJ.get_qDD(1:2,   'holder')
q = OBJ.get_qDD('all', 'holder')

% show stuff
OBJ.show_q_actual();

OBJ.show_q_holder();




