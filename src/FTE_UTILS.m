classdef FTE_UTILS
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here

    %

    methods
        function out = func_step(start, x)
        out = 1/(1+exp(-1*(x-start)));
        end
        
        function out = func_piece(start, end_var, x)
        out = func_step(start, x) - func_step(end_var, x);
        end
        
        function output_cost = redescending_loss(err, a, b, c)
        e = abs(err);
        cost = 0;
        cost = cost + (1-func_step(a,e))/(2*exp(2));
        cost = cost + func_piece(a,b,e)*(a*e- (a^2)/2);
        cost = cost + func_piece(b,c,e)*(a*b - (a^2)/2 + (a*(c-b)/2)*(1-((c-e)/(c-b))^2));
        cost = cost + func_step(c,e)*(a*b-(a^2)/2 + (a*(c-b)/2));
        output_cost = cost;
        end
        
        function [vidHeight,vidWidth, vidFPS, vidFrameNo] = getVideoInfo(projectdir)
        %left out codec
        dinfo = dir(fullfile(projectdir, '*.mp4'));
        filenames = fullfile({dinfo.folder}, {dinfo.name});
        numfiles = length(filenames);
        vidRes = zeros(numfiles,2);
        for K = 1 : numfiles
            thisfile = filenames{K};
            video = VideoReader(thisfile);
        
            vidWidth(K) = video.Width;
            vidHeight(K) = video.Height;
            vidFPS(K) = video.FrameRate;
            vidFrameNo(K) = video.NumFrames;
        end
        end
        
        function params = get_pose_params()
            % Define symbolic variables
            syms x_0 y_0 z_0 phi_0 theta_0 psi_0 phi_1 theta_1 psi_1 theta_2 phi_3 theta_3 psi_3 ...
                 theta_4 psi_4 theta_5 psi_5 theta_6 theta_7 theta_8 theta_9 theta_10 theta_11 ...
                 theta_12 theta_13
        
            % Define the states cell array
            states = {'x_0', 'y_0', 'z_0', ...
                      'phi_0', 'theta_0', 'psi_0', ...
                      'phi_1', 'theta_1', 'psi_1', ...
                      'theta_2', ...
                      'phi_3', 'theta_3', 'psi_3', ...
                      'theta_4', 'psi_4', ...
                      'theta_5', 'psi_5', ...
                      'theta_6', 'theta_7', ...
                      'theta_8', 'theta_9', ...
                      'theta_10', 'theta_11', ...
                      'theta_12', 'theta_13'};
        
            % Create a dictionary mapping symbolic variables to indices
            params = containers.Map(states, 1:numel(states)); %where states is the 'key' and index is the value
        end
        
        
        function symsMatrix = get_3d_marker_coords(x)
        %In python, Returns either a numpy array or a sympy Matrix of the 3D marker 
        % coordinates (shape Nx3) for a given state vector x.
        %In MATLAB: retuns a symbolic matrix of 3D Marker coordinates (Nx3) for a
        % given state vector x.
        idx = get_pose_params();
        % Check if the first element of x is a symbolic expression
        if isa(x(1), 'sym')
            func = @(x) sym(x); % Return the input as a symbolic variable
        else
            func = @(x) double(x); % Convert the input to double precision
        end
        
            %rotations
            RI_0  = rotz(x(idx('psi_0'))) * rotx(x(idx('phi_0'))) * roty(x(idx('theta_0')));         % head
            R0_I  = transpose(RI_0);
            RI_1  = rotz(x(idx('psi_1'))) * rotx(x(idx('phi_1'))) * roty(x(idx('theta_1'))) * RI_0 ; % neck
            R1_I  = transpose(RI_1);
            RI_2  = roty(x(idx('theta_2'))) * RI_1;                                                    % front torso
            R2_I  = transpose(RI_2);
            RI_3  = rotz(x(idx('psi_3'))) * rotx(x(idx('phi_3'))) * roty(x(idx('theta_3'))) * RI_2 ; % back torso
            R3_I  = transpose(RI_3);
            RI_4  = rotz(x(idx('psi_4'))) * roty(x(idx('theta_4'))) * RI_3;                           % tail base
            R4_I  = transpose(RI_4);
            RI_5  = rotz(x(idx('psi_5'))) * roty(x(idx('theta_5'))) * RI_4;                           % tail mid
            R5_I  = transpose(RI_5);
            RI_6  = roty(x(idx('theta_6'))) * RI_2;                                                    % l_shoulder
            R6_I  = transpose(RI_6);
            RI_7  = roty(x(idx('theta_7'))) * RI_6;                                                    % l_front_knee
            R7_I  = transpose(RI_7);
            RI_8  = roty(x(idx('theta_8'))) * RI_2;                                                    % r_shoulder
            R8_I  = transpose(RI_8);
            RI_9  = roty(x(idx('theta_9'))) * RI_8;                                                    % r_front_knee
            R9_I  = transpose(RI_9);
            RI_10 = roty(x(idx('theta_10'))) * RI_3;                                                   % l_hip
            R10_I = transpose(RI_10);
            RI_11 = roty(x(idx('theta_11'))) * RI_10;                                                  % l_back_knee
            R11_I = transpose(RI_11);
            RI_12 = roty(x(idx('theta_12'))) * RI_3;                                                   % r_hip
            R12_I = transpose(RI_12);
            RI_13 = roty(x(idx('theta_13'))) * RI_12;                                                  % r_back_knee
            R13_I = transpose(RI_13);
        
            %positions
            p_head          = [x(idx('x_0')), x(idx('y_0')), x(idx('z_0'))];
        
            p_l_eye         = p_head         + R0_I  * func(transpose([0, 0.03, 0]));
            p_r_eye         = p_head         + R0_I  * func(transpose([0, -0.03, 0]));
            p_nose          = p_head         + R0_I  * func(transpose([0.055, 0, -0.055]));
        
            p_neck_base     = p_head         + R1_I  * func(transpose([-0.28, 0, 0]));
            p_spine         = p_neck_base    + R2_I  * func(transpose([-0.37, 0, 0]));
        
            p_tail_base     = p_spine        + R3_I  * func(transpose([-0.37, 0, 0]));
            p_tail_mid      = p_tail_base    + R4_I  * func(transpose([-0.28, 0, 0]));
            p_tail_tip      = p_tail_mid     + R5_I  * func(transpose([-0.36, 0, 0]));
        
            p_l_shoulder    = p_neck_base    + R2_I  * func(transpose([-0.04, 0.08, -0.10]));
            p_l_front_knee  = p_l_shoulder   + R6_I  * func(transpose([0, 0, -0.24]));
            p_l_front_ankle = p_l_front_knee + R7_I  * func(transpose([0, 0, -0.28]));
        
            p_r_shoulder    = p_neck_base    + R2_I  * func(transpose([-0.04, -0.08, -0.10]));
            p_r_front_knee  = p_r_shoulder   + R8_I  * func(transpose([0, 0, -0.24]));
            p_r_front_ankle = p_r_front_knee + R9_I  * func(transpose([0, 0, -0.28]));
        
            p_l_hip         = p_tail_base    + R3_I  * func(transpose([0.12, 0.08, -0.06]));
            p_l_back_knee   = p_l_hip        + R10_I * func(transpose([0, 0, -0.32]));
            p_l_back_ankle  = p_l_back_knee  + R11_I * func(transpose([0, 0, -0.25]));
        
            p_r_hip         = p_tail_base    + R3_I  * func(transpose([0.12, -0.08, -0.06]));
            p_r_back_knee   = p_r_hip        + R12_I * func(transpose([0, 0, -0.32]));
            p_r_back_ankle  = p_r_back_knee  + R13_I * func(transpose([0, 0, -0.25]));
        
         symsMatrix = func([transpose(p_nose), transpose(p_r_eye), transpose(p_l_eye),...
                         transpose(p_neck_base), transpose(p_spine),...
                         transpose(p_tail_base), transpose(p_tail_mid), transpose(p_tail_tip),...
                         transpose(p_r_shoulder), transpose(p_r_front_knee), transpose(p_r_front_ankle),...
                         transpose(p_l_shoulder), transpose(p_l_front_knee), transpose(p_l_front_ankle),...
                         transpose(p_r_hip), transpose(p_r_back_knee), transpose(p_r_back_ankle),...
                         transpose(p_l_hip), transpose(p_l_back_knee), transpose(p_l_back_ankle)
                        ]);
        end
        
        %%Projection functions
        function [u, v] = pt3d_to_2d(x, y, z, K, D, R, t)
            x_2d = x*R(1,1) + y*R(1,2) + z*R(1,3) + t(1);
            y_2d = x*R(2,1) + y*R(2,2) + z*R(2,3) + t(2);
            z_2d = x*R(3,1) + y*R(3,2) + z*R(3,3) + t(3);
            % project onto camera plane
            a    = x_2d/z_2d;
            b    = y_2d/z_2d;
            % fisheye params
            r    = (a^2 + b^2 + 1e-12)^0.5;
            th   = pyo.atan(r);
           % distortion
            th_D = th * (1 + D(0)*th^2 + D(1)*th^4 + D(2)*th^6 + D(3)*th^8);
            x_P  = a*th_D/r;
            y_P  = b*th_D/r;
            u    = K(0,0)*x_P + K(0,2);
            v    = K(1,1)*y_P + K(1,2);
        end
        
        function u = pt3d_to_x2d(x, y, z, K, D, R, t)
        [u,~] = pt3d_to_2d(x, y, z, K, D, R, t);
        
        end
        
        function v = pt3d_to_y2d(x, y, z, K, D, R, t)
        [~,v] = pt3d_to_2d(x, y, z, K, D, R, t);
        
        end
        
        function [k_arr, d_arr, r_arr, t_arr, cam_res, n_cams, scene_files_end] = find_scene_file(dir_path,dir_extcalib,verbose, scene_fname) %scene_fname taken out of params
            if nargin < 2
                verbose = 1; % Default verbose value
            end
        if isempty(scene_fname)
            file_list = ls(fullfile(dir_path,'cam*.mp4'));
            n_cams = size(file_list);
            n_cams = n_cams(1);
            if n_cams>0
                scene_fname = sprintf('%d_cam_scene_sba.json', n_cams);
            else 
                scene_fname = sprintf('%d_cam_scene*.json', (1:9));
            end
        end
        if ~isempty(dir_path) && ~strcmp(dir_path, fullfile('..','data'))
            %scene_fpath = fullfile(dir_extcalib, 'extrinsic_calib', scene_fname); 
            all_files = dir(fullfile(dir_extcalib, 'extrinsic_calib'));
        
            % Filter the files based on the condition ('before_corrections' not in scene_file)
            filtered_files = all_files(~contains({all_files.name}, 'before_corrections'));
        
            % Filter the files further based on the condition (scene_file == scene_fpath)
            filtered_files = filtered_files(strcmp({filtered_files.name}, scene_fname));
        
            % Sort the filtered files
            [~, idx] = sort({filtered_files.name});
            scene_files = filtered_files(idx);
            scene_files_end = scene_files(size(scene_files)).name;
            if ~isempty(scene_files)
                 [k_arr, d_arr, r_arr, t_arr, cam_res] = load_scene(scene_files_end,0);
                 [~,scene_fname,~] = fileparts(scene_files_end);
                 n_cams = int8(scene_fname(1)); %assuming scene_fname is of the form '[1-9]_cam_scene*'
            else 
                [k_arr, d_arr, r_arr, t_arr, cam_res, n_cams, scene_files_end]= find_scene_file(dir_path, scene_fname, verbose);
            end
        else ENOENT = 2; % Error code for file not found (you may need to change this according to your error code)
            error('FileNotFoundError:ENOENT', ...
              'No such file or directory: %s', fullfile('extrinsic_calib', scene_fname));
        end
        end
        
        function [k_arr, d_arr, r_arr, t_arr, cam_res] = load_scene(fpath, verbose)
            
            fid = fopen(fpath, 'r');
            data = jsondecode(fread(fid, inf, '*char')');
            fclose(fid);
            cam_res = data.camera_resolution;
            k_arr = [];
            d_arr = [];
            r_arr = [];
            t_arr = [];
            for idx = 1:numel(data.cameras)
                k_arr = [k_arr; data.cameras(idx).k];
                d_arr = [d_arr; data.cameras(idx).d];
                r_arr = [r_arr; data.cameras(idx).r];
                t_arr = [t_arr; data.cameras(idx).t];
            end
            k_arr = double(k_arr);
            d_arr = double(d_arr);
            r_arr = double(r_arr);
            t_arr = double(t_arr);
            
            if verbose
                fprintf('Loaded extrinsics from %s\n\n', fpath);
            end
        
        end
        
        function markers = get_markers()
        markers = ['nose', 'r_eye', 'l_eye', 'neck_base',...
                'spine', 'tail_base', 'tail1', 'tail2',...
                'r_shoulder', 'r_front_knee', 'r_front_ankle',...
                'l_shoulder', 'l_front_knee', 'l_front_ankle',...
                'r_hip', 'r_back_knee', 'r_back_ankle',...
                'l_hip', 'l_back_knee', 'l_back_ankle'];
        end
        
        
        function dlc_df = load_dlc_points_as_df(dlc_h5_file_paths, verbose)
            % Initialize an empty cell array to store DataFrames
            dfs = cell(1, numel(dlc_h5_file_paths));
            
            % Read each HDF5 file
            for i = 1:numel(dlc_h5_file_paths)
                % Read HDF5 file
                info = h5info(dlc_h5_file_paths{i});
                
                % Extract data from the HDF5 file
                dlc_df_single = table();
                for j = 1:length(info.Groups)
                    group_info = info.Groups(j);
                    group_name = group_info.Name;
                    % Check if group contains a sub-group named '_i_table'
                    if any(strcmp({group_info.Datasets.Name}, '*table*'))
                        subgroup_info = group_info.Groups(strcmp({group_info.Groups.Name}, '*table*'))
                        % Initialize a struct to hold the data for this group
                        group_data = struct();
                        % Iterate over datasets in the subgroup
                        for k = 1:length(subgroup_info.Datasets)
                            dataset_name = subgroup_info.Datasets(k).Name;
                            % Read dataset
                            dataset_data = h5read(dlc_h5_file_paths{i}, [group_name, dataset_name]);
                            % Store dataset data in the group data struct
                            group_data.(dataset_name) = dataset_data
                        end
                        % Assume the data structure is similar to the Python code
                        % Combine datasets into a single table (adjust as needed)
                        df_single = table(group_data.indices, group_data.ranges, group_data.bounds, group_data.mranges, ...
                            group_data.sorted, group_data.zbounds, group_data.mbounds, group_data.indicesLR, ...
                            group_data.sortedLR, group_data.abounds, 'VariableNames', ...
                            {'frame', 'camera', 'marker', 'x', 'y', 'likelihood'});
                        % Append to DataFrame
                        dlc_df_single = [dlc_df_single; df_single]
                    end
                end
                
                % Append DataFrame to cell array
                dfs{i} = dlc_df_single;
            end
            
            % Concatenate DataFrames
            dlc_df = vertcat(dfs{:});
            
            % Display DataFrame if verbose is true
            if verbose
                disp('DLC points dataframe:');
                disp(dlc_df);
            end
            
            % Save DataFrame to CSV
            writetable(dlc_df, 'dlc_data.csv', 'Delimiter', ',');
        end

    end
end