function cad_meshes = extract_robot_meshes()
% EXTRACT_ROBOT_MESHES Extracts and prepares 3D CAD meshes for myPalletizer 260 Pi
% Reads official Collada .dae files, aligns them with URDF visual transforms,
% and saves an optimized robot_cad_meshes.mat for real-time 3D GUI rendering.

    meshDir = 'd:\Lidar\myPalletizer_BTL\models\meshes';
    outMat = 'd:\Lidar\myPalletizer_BTL\models\robot_cad_meshes.mat';
    links = {'base', 'link1', 'link2', 'link3', 'link4', 'link5'};
    
    % URDF visual origin offsets: xyz (meters -> mm) and rpy (radians)
    % From mypalletizer_260_pi.urdf:
    % base:  xyz="0 0 0" rpy="0 0 3.1415926"
    % link1: xyz="0 -0.282 -0.115" rpy="1.5707963 0 -1.5707963"
    % link2: xyz="0 0.105 -0.414" rpy="0 1.5707963 3.1415926"
    % link3: xyz="0 0.235 -0.325" rpy="0 1.5707963 3.1419626"
    % link4: xyz="-0.12 -0.235 0.252" rpy="0 -1.5707963 0"
    % link5: cylinder/mesh
    
    urdf_visual = struct();
    urdf_visual.base.xyz  = [0, 0, 0] * 1000;
    urdf_visual.base.rpy  = [0, 0, pi];
    
    urdf_visual.link1.xyz = [0, -0.282, -0.115] * 1000;
    urdf_visual.link1.rpy = [pi/2, 0, -pi/2];
    
    urdf_visual.link2.xyz = [0, 0.105, -0.414] * 1000;
    urdf_visual.link2.rpy = [0, pi/2, pi];
    
    urdf_visual.link3.xyz = [0, 0.235, -0.325] * 1000;
    urdf_visual.link3.rpy = [0, pi/2, pi];
    
    urdf_visual.link4.xyz = [-0.12, -0.235, 0.252] * 1000;
    urdf_visual.link4.rpy = [0, -pi/2, 0];
    
    urdf_visual.link5.xyz = [0.044, 0, 0] * 1000;
    urdf_visual.link5.rpy = [0, 0, 0];

    cad_meshes = struct();

    for i = 1:length(links)
        name = links{i};
        filePath = fullfile(meshDir, [name, '.dae']);
        fprintf('Parsing %s...\n', name);
        
        txt = fileread(filePath);
        
        % Extract all geometry elements
        geom_blocks = regexp(txt, '<geometry[^>]*>.*?</geometry>', 'match');
        
        all_V = [];
        all_F = [];
        v_offset = 0;
        
        for g = 1:length(geom_blocks)
            g_txt = geom_blocks{g};
            
            % Positions array
            pos_tok = regexp(g_txt, '<float_array[^>]+id="[^"]*position[^"]*"[^>]*>(.*?)</float_array>', 'tokens');
            if isempty(pos_tok), continue; end
            v_raw = sscanf(pos_tok{1}{1}, '%f');
            if isempty(v_raw), continue; end
            V_sub = reshape(v_raw, 3, [])';
            
            % Check scale: in DAE Rhinoceros, units are in meters (scale 0.001) or mm
            % If max abs > 10, it's already in mm. If max abs < 1.0, convert to mm
            if max(abs(V_sub(:))) < 2.0
                V_sub = V_sub * 1000.0;
            end
            
            % Triangles
            tri_matches = regexp(g_txt, '<triangles[^>]*>(.*?)</triangles>', 'match');
            for tm = 1:length(tri_matches)
                t_str = tri_matches{tm};
                p_tok = regexp(t_str, '<p>(.*?)</p>', 'tokens');
                if isempty(p_tok), continue; end
                
                % Determine stride from number of <input tags
                inputs = regexp(t_str, '<input[^>]*/>', 'match');
                stride = max(1, length(inputs));
                
                p_arr = sscanf(p_tok{1}{1}, '%d');
                v_idx = p_arr(1:stride:end) + 1; % 1-based vertex index
                
                F_sub = reshape(v_idx, 3, [])' + v_offset;
                all_F = [all_F; F_sub];
            end
            
            all_V = [all_V; V_sub];
            v_offset = size(all_V, 1);
        end
        
        if isempty(all_V) || isempty(all_F)
            fprintf('  Warning: empty geometry in %s, generating surrogate CAD model.\n', name);
            [all_F, all_V] = generate_surrogate_link(name);
        else
            % Apply visual origin transform: R = Rz * Ry * Rx, then + xyz
            rpy = urdf_visual.(name).rpy;
            xyz = urdf_visual.(name).xyz;
            R = rpy2rot(rpy(1), rpy(2), rpy(3));
            all_V = (R * all_V')' + xyz;
            
            % Reduce patch complexity for smooth 60 FPS real-time rendering
            orig_faces = size(all_F, 1);
            if orig_faces > 3000
                red_p = reducepatch(struct('faces', all_F, 'vertices', all_V), 2500);
                all_F = red_p.faces;
                all_V = red_p.vertices;
                fprintf('  Reduced %s from %d to %d faces\n', name, orig_faces, size(all_F, 1));
            end
        end
        
        cad_meshes.(name).faces = all_F;
        cad_meshes.(name).vertices = all_V;
        fprintf('  %s: %d vertices, %d faces\n', name, size(all_V, 1), size(all_F, 1));
    end
    
    save(outMat, 'cad_meshes');
    fprintf('Saved all CAD meshes to: %s\n', outMat);
end

function R = rpy2rot(r, p, y)
    Rx = [1, 0, 0; 0, cos(r), -sin(r); 0, sin(r), cos(r)];
    Ry = [cos(p), 0, sin(p); 0, 1, 0; -sin(p), 0, cos(p)];
    Rz = [cos(y), -sin(y), 0; sin(y), cos(y), 0; 0, 0, 1];
    R = Rz * Ry * Rx;
end

function [F, V] = generate_surrogate_link(name)
    switch name
        case 'base'
            [X, Y, Z] = cylinder([50, 45], 20);
            Z = Z * 60;
        case 'link5'
            [X, Y, Z] = cylinder([15, 15], 16);
            Z = Z * 25;
        otherwise
            [X, Y, Z] = cylinder([20, 15], 16);
            Z = Z * 100;
    end
    [F, V] = surf2patch(X, Y, Z, 'triangles');
end
