function y = run_anaconda_from_matlab(a)


% First follow these steps to ensure your windows command line recognizes
% conda commands
% https://www.partitionwizard.com/partitionmanager/conda-is-not-recognized.html

environment_name = "DaeSystemsSolver";
python_file_path = fullfile("C:\Users\darsh\Documents\GitHub\Lsdo\gebt");
python_file_name = "implicit_op.py";

% command_to_activate_anaconda = '%windir%\System32\cmd.exe "/K" C:\Users\darsh\anaconda3\Scripts\activate.bat C:\Users\darsh\anaconda3';
command_to_activate_anaconda = "activate base";
command_to_activate_environment = strcat("conda activate ", environment_name);
command_to_cd_to_python_script = strcat("cd ", python_file_path);



command = strcat(command_to_activate_anaconda, " && ", ...
    command_to_activate_environment, " && ",...
    command_to_cd_to_python_script, " && ",...
    "python ", python_file_name)
[status, cmdout] = system(command);

assert(status==0, "Anaconda did not run properly")

y = 1;

end