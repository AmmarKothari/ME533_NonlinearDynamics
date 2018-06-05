function bh_LINK2_TRAJ_BUS_OBJ() 
% BH_LINK2_TRAJ_BUS_OBJ initializes a set of bus objects in the MATLAB base workspace 

% Bus object: slBus1 
clear elems;
elems(1) = Simulink.BusElement;
elems(1).Name = 'LINK2_EFF_Y_D';
elems(1).Dimensions = 1;
elems(1).DimensionsMode = 'Fixed';
elems(1).DataType = 'double';
elems(1).SampleTime = -1;
elems(1).Complexity = 'real';
elems(1).Min = [];
elems(1).Max = [];
elems(1).DocUnits = '';
elems(1).Description = '';

elems(2) = Simulink.BusElement;
elems(2).Name = 'LINK2_EFF_Y';
elems(2).Dimensions = 1;
elems(2).DimensionsMode = 'Fixed';
elems(2).DataType = 'double';
elems(2).SampleTime = -1;
elems(2).Complexity = 'real';
elems(2).Min = [];
elems(2).Max = [];
elems(2).DocUnits = '';
elems(2).Description = '';

elems(3) = Simulink.BusElement;
elems(3).Name = 'LINK2_EFF_Z_D';
elems(3).Dimensions = 1;
elems(3).DimensionsMode = 'Fixed';
elems(3).DataType = 'double';
elems(3).SampleTime = -1;
elems(3).Complexity = 'real';
elems(3).Min = [];
elems(3).Max = [];
elems(3).DocUnits = '';
elems(3).Description = '';

elems(4) = Simulink.BusElement;
elems(4).Name = 'LINK2_EFF_Z';
elems(4).Dimensions = 1;
elems(4).DimensionsMode = 'Fixed';
elems(4).DataType = 'double';
elems(4).SampleTime = -1;
elems(4).Complexity = 'real';
elems(4).Min = [];
elems(4).Max = [];
elems(4).DocUnits = '';
elems(4).Description = '';

slBus1 = Simulink.Bus;
slBus1.HeaderFile = '';
slBus1.Description = '';
slBus1.DataScope = 'Auto';
slBus1.Alignment = -1;
slBus1.Elements = elems;
clear elems;
assignin('base','BUS_DEF_2LINK_TRAJ', slBus1);

