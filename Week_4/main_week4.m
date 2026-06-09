clear all ; close all ; clc;

addpath('C:\Users\BAPS\Documents\MATLAB\Robotics_Optimization\Week_1\');                     % Week 1 functions
addpath('C:\Users\BAPS\Documents\MATLAB\Robotics_Optimization\Week_2\');          % Week 2 functions
addpath('C:\Users\BAPS\Documents\MATLAB\Robotics_Optimization\Week_3\');                     % Week 3 functions
p = params();


%config_name = 'simple';
%config_name = 'moderate';
config_name = 'hard';


%ipt = input("Press Enter to contiue with FMINCON"); 
[res_M2, ~] = run_method_M2_gcs_final(config_name, p);
%ipt = input("Press Enter to contiue with IPOPT"); 
[res_M4, ~] = run_method_M4_gcs_final(config_name, p);

%ipt = input("Press Enter to contiue with result comparision"); 
compare_methods(config_name, res_M2, res_M4);