# pyMbseMdo
A package for Model-Based Systems Engineering (MBSE) and Multi-Disciplinary Design Optimization (MDO)

# PreRequirements: 
Install Matlab 2023a and additional apps in Matlab -- namely Simulink, System Composer, Requirements Manager, and Simulink Test locally. 

# Instructions to run the file: 
1. Download the zip file from github, and update the file path in Matlab, and in the terminal.
2. Create an environment from an environment.yml file
Use the terminal for the following steps:

Create the environment from the environment.yml file:
```sh
conda env create -f environment_mbsemdo.yml
```
The first line of the yml file sets the new environment's name. Activate the new environment: 
```sh
conda activate mbsemdo
```
Verify that the new environment was installed correctly:
```sh
conda env list
```
3. Update **Code 10.m**
```sh
environment_name = "mbsemdo";
%Give the python file path here: 
python_file_path = fullfile("C:\Users\horty\Desktop\Matlab_work\pyMbseMdo-main");
python_file_name = "aircraft_level_sizing.py";

% command_to_activate_anaconda = '%windir%\System32\cmd.exe "/K" C:\Users\darsh\anaconda3\Scripts\activate.bat C:\Users\darsh\anaconda3';
% set your conda path here
initialize_conda = 'set PATH=%PATH%;C:\Users\horty\anaconda3;C:\Users\horty\anaconda3\Scripts';
command_to_activate_anaconda = "activate base";
%activate your conda environment
command_to_activate_environment = strcat("conda activate ", environment_name);
command_to_cd_to_python_script = strcat("cd ", python_file_path);
```
4. Open **Final_AIAA.mldatx** and run all the tests. To see how the requirements are linked - open the different requirements in the requirements editor. 
