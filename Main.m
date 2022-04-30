[filename, pathname] = uigetfile( ...
    {
     '*.*',  'All Files (*.*)'}, 'Pick a File to Import');
full_filename = fullfile(pathname, filename);
S_Joints = importdata(full_filename);
final = TB(S_Joints);