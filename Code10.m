function cmdout = Code10()
% First follow these steps to ensure your windows command line recognizes
% conda commands
% https://www.partitionwizard.com/partitionmanager/conda-is-not-recognized.html

environment_name = "mbsemdo";
python_file_path = fullfile("C:\Users\horty\Desktop\Matlab_work\pyMbseMdo-main");
python_file_name = "aircraft_level_sizing.py";

% command_to_activate_anaconda = '%windir%\System32\cmd.exe "/K" C:\Users\darsh\anaconda3\Scripts\activate.bat C:\Users\darsh\anaconda3';
% set your conda path here
initialize_conda = 'set PATH=%PATH%;C:\Users\horty\anaconda3;C:\Users\horty\anaconda3\Scripts';
command_to_activate_anaconda = "activate base";
%activate your conda environment
command_to_activate_environment = strcat("conda activate ", environment_name);
command_to_cd_to_python_script = strcat("cd ", python_file_path);
m=0;
[Aircraft] = Code();
[Aircraft1] = Code1(); 
[Aircraft2] = Code2(); 
[Aircraft3] = Code3(); 
[Aircraft4] = Code4(); 
Aeff=Aircraft3(1);
BPR=Aircraft4(1);
M=Aircraft(1);
Phi25=Aircraft3(2);
R=Aircraft(2);
dFo = Aircraft2(1);
lambdaeff = Aircraft3(3);
nE = Aircraft4(2);
tbyc = Aircraft3(4);
CLmax_L_unswept = Aircraft1(1);
CLmax_TO_unswept = Aircraft1(2);
dTl = Aircraft1(3);
dTto = Aircraft1(4);
slfl = Aircraft1(5);
stofl = Aircraft1(6);
command = strcat(initialize_conda, "&&", command_to_activate_anaconda, " && ", ...
    command_to_activate_environment, " && ",...
    command_to_cd_to_python_script, " && ",...
    "python ", python_file_name," ", num2str(Phi25)," ",num2str(Aeff)," ", ...
    num2str(lambdaeff)," ",num2str(dFo)," ",num2str(BPR)," ",num2str(tbyc), ...
    " ",num2str(CLmax_L_unswept)," ", num2str(CLmax_TO_unswept)," ", ...
    num2str(M)," ", num2str(R)," ",num2str(nE)," ",num2str(slfl)," ", ...
    num2str(dTl)," ",num2str(stofl)," ", num2str(dTto));

[status, cmdout] = system(command);

%assert(status==0, "Anaconda did not run properly")
y = 1;
end
