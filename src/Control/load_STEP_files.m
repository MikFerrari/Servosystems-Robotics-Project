% Open the Simulink file, assign its name to the variable 'sim_filename'
% and then launch this script to load STEP files in all SOLID blocks

sim_filename = 'robot_assembly_control_joints_3';

% Ricky
directory  = 'C:\Users\monti\OneDrive - unibs.it\Servosystems & Robotics Project\Robot Model - Solidworks\Manipulator\simscape_linked_assembly';

% Micky
% directory  = 'C:\Users\miche\OneDrive - unibs.it\University\Magistrale - UNIBS\1_Primo Anno\Servosystems & Robotics\Servosystems & Robotics Project\src\Control\Micky';


blocks = find_system;

assembly = blocks(contains(blocks,sim_filename));
assembly = sort(assembly);
solids = assembly(contains(assembly,'Solid'));

handles = getSimulinkBlockHandle(solids);

STEP_files = {'base_Default_sldprt.STEP', 'link_1a_Default_sldprt.STEP', ...
              'link_1a_cap_Default_sldprt.STEP', 'link_1b_Default_sldprt.STEP', ...
              'link_1c_base_Default_sldprt.STEP', 'link_1c_cylinder_Default_sldprt.STEP', ...
              'link_2_piston_Default_sldprt.STEP', 'link_2_ending_Default_sldprt.STEP', ...
              'link_3_Default_sldprt.STEP'};

for i = 1:numel(handles)
    set_param(handles(i),'ExtGeomFileName',char(strcat(directory,'\',STEP_files(i))));
end

