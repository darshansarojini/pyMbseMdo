function [e] = Aircraft_Sizing(input)
cmdout = DL0_1();
% load the .mat file created in Python
data = load('variables.mat');

% % access the variables by their names
Landing_constraint = data.a;
Cruise_constraint = data.b;
Fuel_Tank_Volume_constraint = data.c;
Takeoff_thrust = data.d;
[e] = [Landing_constraint;Cruise_constraint;Fuel_Tank_Volume_constraint;Takeoff_thrust];
end