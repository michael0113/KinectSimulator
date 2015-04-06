clear all
close all

% Add opcodemesh folders
folder_path = pwd;
path_opcodemesh = [folder_path '\opcodemesh'];
if isempty(strfind(path,path_opcodemesh))
    disp('Adding opcodemesh path...\n')
    addpath(genpath(path_opcodemesh))
end 

%% DEMO FLAT WALLS BETWEEN DEFAULT MIN AND MAX DEPTHS =====================
testMinDepth = 1000; % mm
testMaxDepth = 3000; % mm

% Set parameters of wall CAD model
vertex_wall = [2e4  2e4 -2e4 -2e4;...
               7e3 -7e3  7e3 -7e3;...
               0    0    0    0];
face_wall   = [1 1;...
               2 3;...
               4 4];
norm_wall   = [0  0;...
               0  0;...
              -1 -1];

vertex_wall(3,:) = testMinDepth*ones(1,4);
[~,~,~] = KinectSimulator_IR(vertex_wall,face_wall,norm_wall);  
DpthImg = KinectSimulator_Depth(vertex_wall,face_wall,norm_wall);
figure, imshow(DpthImg,[min(DpthImg(:)) max(DpthImg(:))])
title('Noisy depth image of flat wall at 1000 mm')

vertex_wall(3,:) = testMaxDepth*ones(1,4);
[~,~,~] = KinectSimulator_IR(vertex_wall,face_wall,norm_wall); 
DpthImg = KinectSimulator_Depth(vertex_wall,face_wall,norm_wall);
figure, imshow(DpthImg,[min(DpthImg(:)) max(DpthImg(:))])
title('Noisy depth image of flat wall at 3000 mm')

%% DEMO TILTED WALL =======================================================
tilt_Angle = 60;   % deg
tilt_Center = 1000; % mm
tilt_X = -tilt_Center/tand(tilt_Angle);
tilt_Z =  tilt_Center + 1e5*tand(tilt_Angle);
vertex_tilt = [1e5     1e5     tilt_X  tilt_X;...
               1e5    -1e5     1e5    -1e5;...
               tilt_Z  tilt_Z  0       0];
face_tilt   = [1 1;...
               2 3;...
               4 4];
norm_tilt   = [sind(tilt_Angle)  sind(tilt_Angle);...
               0                 0;...
              -cosd(tilt_Angle) -cosd(tilt_Angle)];

DpthImg = KinectSimulator_Depth(vertex_tilt,face_tilt,norm_tilt,...
    'default','default','max','imgrng',[400 4000],'displayIR','on');
figure, imshow(DpthImg,[min(DpthImg(:)) max(DpthImg(:))])
title('Noisy depth image of 60 degree tilted wall, center depth at 1000 mm')

%% DEMO BOX GRID ==========================================================
load('example_CAD\BoxGrid.mat')

box_width = 96;  % mm
box_depth = 54;  % mm
wallDist  = 854; % mm

% Scale size of box and shift grid to further depth
vertex_grid = vertex;
vertex_grid(1:2,:) = vertex_grid(1:2,:).*box_width;
vertex_grid(3,:) = vertex_grid(3,:).*box_depth + wallDist-box_depth;
face_grid = face;
norm_grid = normalf;
GridRng = [min(vertex_grid(3,:)) max(vertex_grid(3,:))];
    
DpthImg = KinectSimulator_Depth(vertex_grid,face_grid,norm_grid,...
    'simple','default',wallDist,'imgrng',GridRng,'quant11','off',...
    'quant10','off','displayIR','on');
figure, imshow(DpthImg,[min(DpthImg(:)) max(DpthImg(:))])
title('Noisy depth image of box grid in front of flat wall')

%% DEMO TEAPOT ============================================================
load('example_CAD\Teapot.mat')

% Shift CAD model to further depth
vertex_teapot = vertex;
vertex_teapot(3,:) = vertex_teapot(3,:) + 600;
face_teapot = face;
norm_teapot = normalf;

DpthImg = KinectSimulator_Depth(vertex_teapot,face_teapot,norm_teapot,...
    'default','default','max','imgrng',[400 4000],...
    'window',[7 9],'displayIR','on');
figure, imshow(DpthImg,[0 1500])
title('Noisy depth image of teapot in front of flat wall')

%% DEMO XBOX CONTROLLER ===================================================
load('example_CAD\XboxController.mat')

% Shift CAD model to further depth
vertex_xbox = vertex;
vertex_xbox(3,:) = vertex_xbox(3,:) + 300;
face_xbox = face;
norm_xbox = normalf;

DpthImg = KinectSimulator_Depth(vertex_xbox,face_xbox,norm_xbox,...
    'simple','default',[],'imgrng',[204 4000],'subray',[5 9]);
figure, imshow(DpthImg,[170 320])
title('Noisy depth image of xbox controller in front of nothing')