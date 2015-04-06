%KINECTSIMULATOR_DEPTH Simulate Kinect depth images. 
%   KINECTSIMULATOR_DEPTH is a program developed to simulate high fidelity
%   Kinect depth images by closely following the Microsoft Kinect's 
%   mechanics. For a detailed description of how this simulator was 
%   developed, please refer to [1]. If this simulator is used for 
%   publication, please cite [1] in your references.
%
%   DEPTHimg = KinectSimulator_Depth(vertex,face) returns the simulated
%   depth image 'DEPTHimg'. The parameters 'vertex' and 'face' define the
%   CAD model of the 3D scene used to generate an image.
%
%       vertex  - 3xn, n vertices of each 3D coordinate that defines the  
%                 CAD model.
%       face    - 3xm, m facets, each represented by 3 vertices that  
%                 defines the CAD model.
%       normalf - 3xm, m facets, representing the normal direction of each
%                 facet.
%
%   The depth image simulator calls the function 'KinectSimulator_IR' to
%   generate a series of non-noisy reference IR images and the output noisy 
%   IR image of the 3D scene to estimate the noisy depth image. Note, The 
%   IR image simulator program utilizes a Matlab wrapper written by Vipin 
%   Vijayan [2] of the ray casting program OPCODE. The Original OPCODE
%   program was written by Pierre Terdiman [3].
%   
%   Depth image estimation undergoes two steps. The first step estimates an
%   integer disparity value by correlating a pixel window of the binary  
%   measured image to a binary reference image of a flat wall at either the   
%   min or max operational depth. This step finds a best match between a  
%   window centered around a pixel of the projected IR pattern from the 
%   measurement and reference images. The epipolar lines of the transmitter 
%   and receiver are assumed coincident, which means the pattern can only
%   shift by a varying number of column pixels. The program then performs a 
%   sub-pixel refinement step to estimate a fractional disparity value. 
%   This is done by finding the minimum sum of absolute differences between  
%   the noisy measured IR image and a non-noisy reference image of a flat 
%   wall at a distance computed by the initial integer disparity 
%   estimation. This step estimates where the location where IR dots in the  
%   window template split pixels. The depth 'z' is computed from a given  
%   disparity 'd' by
%
%       z = b*f / d,
%
%   where 'f' is the horizontal focal length of the Kinect sensor, and 'b' 
%   is the baseline distance between the transmitter and receiver. The 
%   default baseline distance is fixed to 75 mm, and the focal length is 
%   computed from the inputted FOV, where the default is 571.4 pixels.
%   
%   DEPTHimg = KinectSimulator_Depth(vertex,face,IR_intensity,IR_noise) 
%   allows the user to specify a different IR intensity and noise model for
%   the IR image simulator.
%
%       IR_intensity Options
%           'default' - IR intensity model determined empirically from data
%                       collected from IR images recorded by the Kinect
%                       positioned in front of a flat wall at various
%                       depths. The model follows an alpha*dot(-n,l)/r^2 
%                       intensity model, where r is the range between the
%                       sensor and the surface point, n is the surface
%                       normal, and l is the IR lighting direction. Alpha 
%                       accounts for a constant illumination, surface  
%                       properties, etc. The actual intensity model is
%                       
%                           I = Iu.*5.90x10^8*dot(-n,l)/r^2
%
%                       Iu is the fractional intensity contributed by a
%                       sub-ray of a transmitted dot. r is the distance
%                       between the center of the transmitter coordinate 
%                       system, and the 3D location of where the sub-ray 
%                       intersected the CAD model (range).
%
%           'simple'  - Additional IR intensity model determined 
%                       empirically from data collected from Kinect IR  
%                       images. 
%
%                           I = Iu.*5.96x10^8/r^2
%
%                       This model is a simplified version of the 'default'
%                       model, which excluded the surface normal and IR 
%                       lighting direction to fit the model.
%
%           'none'    - No model is used to compute an IR intensity. This 
%                       is modeled by 
%
%                           I = Iu
%
%                       This option is used when generating reference 
%                       images for depth estimation.
%
%           @(Iu,r)fn - User defined IR intensity model given the
%                       fractional intensity and the range of the
%                       transmitted sub-ray.
%
%       IR_noise Options
%           'default' - IR noise model determined empirically from data
%                       collected from IR images recorded by the Kinect
%                       positioned in front of a flat wall at various
%                       depths. The model has multiplicative speckle noise 
%                       that fits a gamma distribution with shape value 
%                       4.54 and scale value 0.196. The model also has 
%                       additive detector noise that fits a normal 
%                       distribution with mean -0.126 and standard 
%                       deviation 10.4, with units of 10-bit intensity. The
%                       actual noise model is 
%
%                           Z = I + n_I + n
%                             = I*Gamma + n
%                             = I*gamrnd(4.54,0.196) - 0.126+10.4*randn()
%
%           'none'    - No model is used to compute an IR noise. This is
%                       modeled by 
%
%                           Z = I = Iu
%
%                       This option is used when generating reference 
%                       images for depth estimation. Note, this option must
%                       be set if the IR_intensity model is set to 'none'.
%
%           @(I)fn    - User defined IR noise model given the intensity of
%                       the sub-ray representing a part of the transmitted
%                       IR dot.
%
%   DEPTHimg = KinectSimulator_Depth(vertex,face,IR_intensity,IR_noise,wallDist) 
%   allows the user the option to add a wall to the CAD model of the
%   simulated 3D scene.
%
%       wallDist Options
%           []        - No wall is added to the scene. This is the default
%                       setting.
%
%           'max'     - A flat wall is added to the 3D scene at the maximum
%                       operational depth.
%
%           wallDist  - The user can input an single value in millimeters
%                       between the min and max operational depths.
%
%   DEPTHimg = KinectSimulator_Depth(vertex,face,IR_intensity,IR_noise,wallDist,options) 
%   gives the user the option to change default Kinect input parameters and
%   the default IR and depth simulator parameters. The options are listed 
%   as follows:
%
%       Depth Simulator Parameters
%           'refine'  - The interpolation factor to perform sub-pixel
%                       refinement. Since the Kinect performs interpolation
%                       by a factor of 1/8th of a pixel, the default
%                       for this option is set to 8.
%
%           'quant11' - The option to quantize depth values to real
%                       allowable 11-bit depth values outputted by the 
%                       Kinect sensor. The 'default' option loads an array
%                       of depth values collected from real data outputted 
%                       by a Kinect for Windows sensor. The user may also
%                       set this option to 'off', or input a new array of
%                       quantized depth values, all of which must be
%                       greater than zero.
%
%         'displayIR' - The user may set this option to 'on' to display
%                       the noisy IR image of the 3D scene. The default is
%                       set to 'off'.
%
%       IR Simulator Parameters
%           'window'  - Size of correlation window used to process IR image
%                       images for depth estimation. The default is set to
%                       9x9 rows and columns, i.e. [9 9] pixels. Note, 
%                       these values must be greater than zero, and must be 
%                       odd to allow the pixel being processed to be at the 
%                       center of the window. Also, given the limited size 
%                       of the idealized dot pattern used to simulate IR 
%                       images, the number of rows in the window cannot 
%                       exceed 15 pixels.
%
%           'subray'  - Size of sub-ray grid used to simulate the physical
%                       cross-sectional area of a transmitted IR dot. The
%                       default is set to 7x17 rows and cols, i.e. [7 17]. 
%                       Note, it is preferable to set each value to an odd
%                       number as to allow the center of the pixel to be 
%                       represented by a sub-ray. Also, since Kinect 
%                       performs an interpolation to achieve a sub-pixel 
%                       disparity accuracy of 1/8th of a pixel, there 
%                       should be at least 8 columns of sub-rays.
%
%           'pattern' - The dot pattern used to simulate the IR image. The
%                       default is adapted from the work done by Andreas 
%                       Reichinger, in his blog post entitled 'Kinect 
%                       Pattern Uncovered' [4]. Note, this idealized binary
%                       representation of Kinect's dot pattern does not
%                       include the pincushion distortion observed in real
%                       Kinect IR images.
%
%           'quant10' - The option to quantize IR intensity into a 10-bit
%                       value. The default is set to 'on', but can be 
%                       turned to 'off'. 
%
%       Kinect Parameters
%           'imgfov'  - The field of view of Kinect's transmitter/receiver.
%                       The default is set to 45.6 x 58.5 degrees for the 
%                       verticle and horizontal FOVs, respectively, i.e. 
%                       [45.6 58.5].
%
%           'imgrng'  - The minimum and maximum operational depths of the
%                       Kinect sensor. Dots with depths that fall outside 
%                       of this range are filtered out from the IRimg and 
%                       IRimg_disp images. This is important for the depth
%                       image simulator because reference images generated
%                       at the min and max depths with only be able to find
%                       matches in the simulated measured IR image between
%                       the set operational depth range. The default is set
%                       to 800 mm for the minimum depth, and 4000 mm for
%                       the maximum depth, i.e. [800 4000].
%
%   Notes about the options:
%       By limiting disparity estimation to an 1/8th of a pixel, this
%       simulator in essence quantizes depth similar to the way Kinect 
%       quantizes depth images into 11-bit values. However, the estimated
%       horizontal FOV and simulated disparity values most likely differ 
%       from the exact Kinect parameters, and therefore setting 'quant11' 
%       to 'off' will result in output depths different from real Kinect 
%       depth values.
%
%       Keeping the IR image quantization option 'quant10' to 'on' will
%       result in introducing more noise to the output IR values on the
%       order of 10*log10(2^10) = 30.1 dB, which impacts depth estimation
%       in the depth image simulator. Depending on the inputted IR  
%       intensity and noise models, this may introduce erroneous depth 
%       error, so the user can choose to set this option to 'off' to avoid
%       this.        
%
%       If 'imgrng' is set to a larger range, processing will be slower
%       because pixel templates of the measured IR image need to be 
%       compared to more columns of pixel templates in the min or max 
%       reference images. Also, if the range is smaller, the error in depth 
%       image estimates will be smaller.
%
% References: 
%   [1] M. J. Landau, B. Y. Choo, P. A. Beling, “Simulating Kinect Infrared  
%       and Depth Images,” 2015 (under submission).
%
%   [2] V. Vijayan, “Ray casting for deformable triangular 3d meshes -
%       file exchange - MATLAB central,” Apr. 2013.
%       http://www.mathworks.com/matlabcentral/fileexchange/41504-ray-casting-for-deformable-triangular-3d-meshes/content/opcodemesh/matlab/opcodemesh.m
%
%	[3] P. Terdiman, “OPCODE,” Aug. 2002. 
%       http://www.codercorner.com/Opcode.htm
%
%	[4] A. Reichinger, “Kinect pattern uncovered | azt.tm’s blog,” Mar. 2011.
%       https://azttm.wordpress.com/2011/04/03/kinect-pattern-uncovered/

function DEPTHimg = KinectSimulator_Depth(vertex,face,normalf,varargin)

% DEFAULT PARAMETERS ======================================================
% Depth simulator parameters ----------------------------------------------
% Number of levels to perform interpolation for sub-pixel accuracy
nlev = 8;
% Option to quantize depth image into 11-bit value
isQuant11 = 1;
% Option to use depth quantization model
isQuantLoad = 1;
% Option to plot IR image from depth image simulator
isPlotIR = 0;

% IR simulator parameters -------------------------------------------------
% IR intensity and noise models 
model_Intensity = @(i,r,n,l) i.*5.90e+08.*dot(-n,l)'./r.^2;
model_Noise     = @(i) i.*gamrnd(4.54,0.196,size(i))-0.126+10.4.*randn(size(i));
% Size of correlation window used for depth estimation 
corrWind = [9 9];
% Option to add a wall at depth
isWall = 0;
% Option to load idealized binary replication of the Kinect dot pattern
isLoad = 1;
% If IR intensity model is set to 'none', turn off IR image quantizing
isQuantOK = 1;

% Kinect parameters -------------------------------------------------------
% Resolution of real outputted Kinect IR image (rows x cols)
ImgRes = [480 640];   % pix
% Field of view of transmitter/receiver (vertFOV x horzFOV)
ImgFOV = [45.6 58.5]; % deg
% Minimum and maximum operational depths of the Kinect sensor (min x max)
ImgRng = [800 4000];  % mm
% Distance between IR transmitter and receiver
baseRT = 75; % mm

% ERROR MESSAGES ==========================================================
narginchk(3, inf)
nargoutchk(0, 1)
if size(vertex,1) ~= 3 || size(vertex,2) < 4 || length(size(vertex)) > 2
    error('Input ''vertex'' must have the form 3xn, and must have at least 3 values.')
end
if size(face,1) ~= 3 || length(size(face)) > 2
    error('Input ''face'' must have the form 3xm, and must have at least 3 values.')
end
if size(normalf,1) ~= 3 || length(size(normalf)) ~= length(size(face))
    error('Input ''normalf'' must have the form 3xm, and must equal the number of face values.')
end
if min(vertex(3,:)) < 0
    error('CAD model must have only positive depth (i.e. must be in front of the camera).')
end

% SET INPUT PARAMETERS ====================================================
if nargin == 4
    error('The IR intensity and noise models need to be specified.')
end
if nargin > 4
    if strcmp(varargin{1},'none')
        isQuantOK = 0;
    end
    if strcmp(varargin{1},'none') && ~strcmp(varargin{2},'none')
        error('Noise model must be set to ''none'' if intensity model is set to ''none''.');
    else
        % IR intensity model
        if strcmp(varargin{1},'default')
            model_Intensity = @(i,r,n,l) i.*5.90e+08.*dot(-n,l)'./r.^2;
        elseif strcmp(varargin{1},'simple')
            model_Intensity = @(i,r,n,l) i.*5.96e+08./r.^2;
        elseif strcmp(varargin{1},'none')
            model_Intensity = @(i,r,n,l) i.*ones(size(r));
        elseif ischar(varargin{1})
            error('The argument ''%s'' is not a valid IR intensity model option.', varargin{1})
        elseif nargin(varargin{1}) ~= 2
            error('IR intensity model must have the form ''@(Iu,r) fnc''.')
        else % User inputted model
            model_Intensity = varargin{1};
        end
        % IR noise model
        if strcmp(varargin{2},'default')
            model_Noise = @(i) i.*gamrnd(4.54,0.196,size(i))-0.126+10.4.*randn(size(i));
        elseif strcmp(varargin{2},'none')
            model_Noise = @(i) i;
        elseif ischar(varargin{2})
            error('The argument ''%s'' is not a valid IR noise model option.', varargin{2})
        elseif nargin(varargin{2}) ~= 1
            error('IR noise model must have the form ''@(I) fnc''.')
        else % User inputted model
            model_Noise = varargin{2};
        end
    end
end
if nargin > 5
    % Option for adding wall
    wallDist = varargin{3};
    if isempty(wallDist)
        isWall = 0;
    elseif strcmp(wallDist,'max')
        isWall = 1;
    elseif wallDist <= 0 || length(wallDist) ~= 1 || ~isnumeric(wallDist)
        error('Wall depth must be a single value greater than zero.')
    else
        isWall = 1;
    end
end
if nargin > 6
    k = 4;
    while k < length(varargin)+1
        switch varargin{k}
            % Depth Simulator Parameters
            case 'refine'
                nlev = varargin{k+1};
                if length(nlev) ~= 1 || rem(nlev,1)~=0 || nlev<=0 || ~isnumeric(nlev)
                    error('Sub-pixel refinement value must be integer greater than zero.')
                end
                k = k+2;
            case 'quant11'
                if strcmp(varargin{k+1},'off')
                    isQuant11 = 0;
                elseif strcmp(varargin{k+1},'default')
                    isQuant11 = 1;
                    isQuantLoad = 1;
                elseif ischar(varargin{k+1})
                    error('The argument ''%s'' is not a valid depth quantization model option.', varargin{k+1})
                elseif sum(varargin{k+1}<=0) > 0 || ~isnumeric(varargin{k+1})
                    error('Quantized depth list must have values greater than 0 mm.')
                else
                    isQuant11 = 1;
                    isQuantLoad = 0;
                    distReal = varargin{k+1};
                end
                k = k+2;
            case 'displayIR'
                if strcmp(varargin{k+1},'off')
                    isPlotIR = 0;
                elseif strcmp(varargin{k+1},'on')
                    isPlotIR = 1;
                else
                    error('Display option must be set to ''off'' or ''on''.')
                end
                k = k+2;
            % IR Simulator Parameters
            case 'window'
                corrWind = varargin{k+1};
                if length(corrWind) ~= 2 || sum(mod(corrWind,2)) ~= 2 || sum(corrWind>0) ~= 2 || ~isnumeric(corrWind)
                    error('Correlation window must have two odd integer values greater than zero.')
                elseif corrWind(1) > 15
                    error('Number of rows in correlation window cannot exceed 15 pixels.')
                end
                k = k+2;
            case 'subray'
                nsub = varargin{k+1};
                if length(nsub) ~= 2 || sum(rem(nsub,1)==0) ~=2 || sum(nsub>0) ~= 2 || ~isnumeric(nsub)
                    error('Sub-ray grid must have two integer values greater than zero.')
                end
                k = k+2;
            case 'pattern'
                dotPattern = varargin{k+1};
                isLoad = 0;
                if size(find(dotPattern==0),2)+size(find(dotPattern==1),2) ~= numel(dotPattern)
                    error('Dot pattern image must be binary.')
                end
                k = k+2;
            case 'quant10'
                if strcmp(varargin{k+1},'off')
                elseif strcmp(varargin{k+1},'on')
                else
                    error('Quantize IR image option must be set to ''off'' or ''on''.')
                end
                k = k+2;
            % Kinect Parameters
            case 'imgfov'
                ImgFOV = varargin{k+1};
                if length(ImgFOV) ~= 2 || sum(ImgFOV>0) ~= 2 || ~isnumeric(ImgFOV)
                    error('Image field of view must have two values greater than zero.')
                end
                k = k+2;
            case 'imgrng'
                ImgRng = varargin{k+1};
                if length(ImgRng) ~= 2 || sum(ImgRng>0) ~= 2 || ~isnumeric(ImgRng)
                    error('Operational range of depths must have two values greater than zero.')
                elseif ImgRng(1) > ImgRng(2)
                    error('Min depth must be less than max depth.')
                end
                k = k+2;
            otherwise
                if isnumeric(varargin{k})
                    error('The argument ''%d'' is not a valid input parameter.', varargin{k})
                elseif ischar(varargin{k})
                    error('The argument ''%s'' is not a valid input parameter.', varargin{k})
                else
                    error('Not a valid input parameter')
                end
        end
    end
end

% =========================================================================
% PROCESS DEPTH IMAGE =====================================================
% =========================================================================

% PREPROCESS PARAMETERS ===================================================
% Determine horizontal and vertical focal lengths
ImgFOV = ImgFOV*(pi/180); % rad
FocalLength = [ImgRes(2)/(2*tan(ImgFOV(2)/2)); ImgRes(1)/(2*tan(ImgFOV(1)/2))]; % pix

% Number of rows and columns to pad IR image for cross correlation
corrRow = (corrWind(1)-1)/2;
corrCol = (corrWind(2)-1)/2;

% Find min and max depths for ref image so dots intersect one pixel -------
if baseRT*FocalLength(1)/ImgRng(2) < 1
    error('Maximum depth is too large to compute good max reference image.')
end

% Set new depth and find offset disparity for minimum reference image
dOff_min   = ceil(baseRT*FocalLength(1)/ImgRng(1));
minRefDpth = baseRT*FocalLength(1)/dOff_min;

% Set new depth and find offset disparity for maximum reference image
dOff_max   = floor(baseRT*FocalLength(1)/ImgRng(2));
maxRefDpth = baseRT*FocalLength(1)/dOff_max;

% Load idealized binary replication of the Kinect dot pattern -------------
if isLoad == 1 
    % Check if depth range provides enough coverage for reference images
    if dOff_min-dOff_max > 211
        error('Depth range too large for default dot pattern (hint: try a minimum depth of at least 204 mm).')
    else
        load('default_load_files\kinect-pattern_3x3.mat')
    end
else
    if (baseRT*FocalLength(1)/ImgRng(1))-(baseRT*FocalLength(1)/ImgRng(2)) > size(dotPattern,2)
        error('Depth range too large for size of dot pattern.')
    end
end
ImgSizeDot = size(dotPattern);

% Change input parameter options for ref and measured IR images -----------
if nargin < 6
    varargin_ref = {};
    varargin_now = {};
else
    varargin_ref = varargin(4:end);
    varargin_now = varargin(4:end);
end

% Set new depth range for reference images
ImgRngIN = ImgRng;
ImgRng = [minRefDpth maxRefDpth];
varargin_ref{end+1} = 'imgrng';
varargin_ref{end+1} = ImgRng;

% Change sub-ray grid size for reference images
varargin_ref{end+1} = 'subray';
varargin_ref{end+1} = [1 1];

% Set IR image display option to off
varargin_ref{end+1} = 'display';
varargin_ref{end+1} = 'off';

varargin_now{end+1} = 'display';
varargin_now{end+1} = 'off';

% Setup wall CAD model ----------------------------------------------------
% Make corners of the wall span FOV at max distance
minDisparity = ceil((baseRT*FocalLength(1))/ImgRng(1));
maxDisparity = floor((baseRT*FocalLength(1))/ImgRng(2));

if isLoad == 1
    pixShftLeft_T = min([211,max([0, minDisparity+corrCol])]);
    pixShftRght_T = min([211,max([0, ImgRes(2)-ImgSizeDot(2)-maxDisparity+corrCol+1])]);
else
    pixShftLeft_T = min([size(dotPattern,2),max([0, minDisparity+corrCol])]);
    pixShftRght_T = min([size(dotPattern,2),max([0, ImgRes(2)-ImgSizeDot(2)-maxDisparity+corrCol+1])]);
end

maxPixRow = max([ceil(ImgSizeDot(1)/2) ceil(ImgRes(1)/2)]); 
maxPixCol = max([ceil(ImgSizeDot(2)/2) ceil(ImgRes(2)/2)]); 

wallX1 =  ImgRng(2)*tan((maxPixCol+pixShftLeft_T)*ImgFOV(2)/ImgRes(2));
wallX2 = -ImgRng(2)*tan((maxPixCol+pixShftRght_T)*ImgFOV(2)/ImgRes(2)) - baseRT;
wallY1 =  ImgRng(2)*tan((maxPixRow+corrRow)*ImgFOV(1)/ImgRes(1));
wallY2 = -ImgRng(2)*tan((maxPixRow+corrRow)*ImgFOV(1)/ImgRes(1));

% Set the depth of the wall to temporary value
wallZ  = 0; 

% Set parameters of wall CAD model
vertex_wall  = [wallX1 wallX1 wallX2 wallX2;...
                wallY1 wallY2 wallY1 wallY2;...
                wallZ  wallZ  wallZ  wallZ];
face_wall    = [size(vertex_wall,2)-3 size(vertex_wall,2)-3;...
                size(vertex_wall,2)-2 size(vertex_wall,2)-1;...
                size(vertex_wall,2)   size(vertex_wall,2)];
norm_wall    = [0  0;...
                0  0;...
               -1 -1];

% Parameters for disparity estimation by horizontal covariance ------------
% Generate reference image of flat wall at min range 
vertex_wall(3,:) = ImgRng(1)*ones(1,4);
IR_ref_min = KinectSimulator_IR(vertex_wall,face_wall,norm_wall,'none','none',varargin_ref);

% Generate reference image of flat wall at max range 
vertex_wall(3,:) = ImgRng(2)*ones(1,4);
IR_ref_max = KinectSimulator_IR(vertex_wall,face_wall,norm_wall,'none','none',varargin_ref);

% Find max shift for a pattern projected between min and max depth 
numHorzPix = dOff_min - dOff_max + 1;

% Preprocess norm of correlation window for each pixel in reference images
windSize     = corrWind(1)*corrWind(2);
IR_min_snorm = cell(ImgRes);
IR_max_snorm = cell(ImgRes);
for ipix_row = 1:ImgRes(1)
    for ipix_col = 1:ImgRes(2)
        row_now = ipix_row:ipix_row+corrWind(1)-1;
        col_now = ipix_col:ipix_col+corrWind(2)-1;
        
        window_ref = IR_ref_min(row_now,col_now);
        IR_min_snorm{ipix_row,ipix_col} = window_ref - sum(window_ref(:)) / windSize;
        
        window_ref = IR_ref_max(row_now,col_now);
        IR_max_snorm{ipix_row,ipix_col} = window_ref - sum(window_ref(:)) / windSize;
    end
end

% Parameters for sub-pixel refinement -------------------------------------
% Preprocess reference images with intensities for lookup table
IR_ref_refine = zeros([ImgRes+corrWind-1 numHorzPix]);
for ilev = 1:numHorzPix
    idepth = baseRT*FocalLength(1)/(dOff_max+ilev-1);
    vertex_wall(3,:) = idepth*ones(1,4);
    IR_ref_refine(:,:,ilev) = KinectSimulator_IR(vertex_wall,face_wall,norm_wall,model_Intensity,'none',varargin_ref);
end

% Find fractional intensities 
ilev_main = 1-1/nlev:-1/nlev:0+1/nlev;
ilev_splt = 1 - ilev_main;

% Preprocess depths for all simulated disparities
disp_all  = dOff_max:1/nlev:dOff_min;
depth_all = baseRT*FocalLength(1)./disp_all;

% Quantize simulated depths -----------------------------------------------
if isQuant11
    % Load default list of quantized depths
    if isQuantLoad == 1
        load('default_load_files/distReal.mat')
        distReal = [1:distReal(1) distReal(2:end)];
    end
    
    % Find closest quantized depth value
    distQuant = zeros(size(depth_all));
    for idist = 1:length(depth_all)
        [~,indx] = min(abs(depth_all(idist)-distReal));
        distQuant(idist) = distReal(indx);
    end
    depth_all = distQuant;
end

% GENERATE OUTPUT DEPTH IMAGE =============================================
% Set up scene by adding wall CAD to object CAD ---------------------------
if isWall == 1
    if strcmp(wallDist,'max') || nargin < 5
        wallDist = ImgRngIN(2);
    end
    if wallDist < ImgRngIN(1) || wallDist > ImgRngIN(2)
        error('The wall depth must be between the min and max operational depths.')
    end

    vertex_wall(3,:) = wallDist*ones(1,4);
    vertex           = [vertex vertex_wall];
    
    face_wallAdd     = [size(vertex,2)-3 size(vertex,2)-3;...
                        size(vertex,2)-2 size(vertex,2)-1;...
                        size(vertex,2)   size(vertex,2)];
    face             = [face face_wallAdd];
    
    normalf          = [normalf norm_wall];
end

% Generate IR image of object ---------------------------------------------
% Binary IR image
IR_bin = KinectSimulator_IR(vertex,face,normalf,'none','none',varargin_now);

% Noisy IR image
if isPlotIR == 1
    varargin_now{end-1} = 'display';
    varargin_now{end}   = 'on';
end
if isQuantOK == 0
    IR_now = KinectSimulator_IR(vertex,face,normalf,'none','none',varargin_now);
else
    IR_now = KinectSimulator_IR(vertex,face,normalf,model_Intensity,model_Noise,varargin_now);
end

% Estimate depth image of object with correlation window ------------------
DEPTHimg = zeros(ImgRes);

% (Vectorizing is slower due to lines 651 and 652 being called many times)
% for ipix = 1:prod(ImgRes) 
        % Convert index to row and colum (based on ind2sub)
%         ipix_row = rem(ipix-1, ImgRes(1)) + 1;
%         ipix_col = (ipix - ipix_row)/ImgRes(1) + 1;
for ipix_row = 1:ImgRes(1)
    for ipix_col = 1:ImgRes(2)       
        row_now = ipix_row:ipix_row+corrWind(1)-1;
        col_now = ipix_col:ipix_col+corrWind(2)-1;
        
        % Binary window
        window_bin = IR_bin(row_now,col_now);
        snorm_now  = window_bin - sum(window_bin(:)) / windSize;
        
        % Noisy window
        window_now = IR_now(row_now,col_now);
        
        % If there is no pattern within the window, it is shadowed
        if sum(sum(window_now)) ~= 0
            horzCov_ref = zeros(1,numHorzPix);
            
            % Set parameters for left and right half of meas img ----------
            % For pixels on left half, compare to min ref img 
            if ipix_col <= round(ImgRes(2)/2)
                IR_snorm_now = IR_min_snorm;
                IR_ref_now = IR_ref_min;
                dOff_now   = dOff_min;
                sign_now   = 1;
            % For pixels on right half, compare to max ref img 
            else
                IR_snorm_now = IR_max_snorm;
                IR_ref_now = IR_ref_max;
                dOff_now   = dOff_max;
                sign_now   = -1;
            end
            
            % Estimate integer disparity with binary IR image -------------
            col_ref = ipix_col-sign_now;
            for iHorzPix = 1:numHorzPix
                % Maximize horizontal covariance
                horzCov_ref(iHorzPix) = sum(sum(snorm_now.*IR_snorm_now{ipix_row,col_ref+sign_now*iHorzPix}));
            end
            [~,dispInd] = max(horzCov_ref);
            dispNow = dOff_now - sign_now*(dispInd-1);
            
            % Sub-pixel refinement with noisy IR image --------------------
            % Determine indices where IR dots intersect pix
            window_ref = IR_ref_now(row_now,col_now+sign_now*(dispInd-1));
            [row_main, col_main] = find(window_ref == 1);
            ind_main = row_main + (col_main-1).*corrWind(1);
            
            % Find window from reference image lookup table
            window_sub = IR_ref_refine(row_now,col_now,dispNow-dOff_max+1);
            ilev_subNow = window_sub(ind_main);
            
            % Indices where dots split pixels on right
            col_rght = col_main + 1;
            ind_rght = row_main + (col_rght-1).*corrWind(1);
            
            % Indices where dots split pixels on left
            col_left = col_main - 1;
            ind_left = row_main + (col_left-1).*corrWind(1);
            
            % If disparity is same as min ref img
            if dispNow == dOff_min
                % Indices to process
                ind_nowFilt = find(col_left>0);
                ind_now = ind_left(ind_nowFilt);

                % Fractional disparity values
                dispRef = 0:-1/nlev:1/nlev-1;
                
                % Fractional intensities for split pixels
                ilev_mainNow = [1 ilev_main];
                ilev_spltNow = [0 ilev_splt];
                
            % If disparity is same as max ref img
            elseif dispNow == dOff_max
                % Indices to process
                ind_nowFilt = find(col_rght<=corrWind(2));
                ind_now = ind_rght(ind_nowFilt);

                % Fractional disparity values
                dispRef = 0:1/nlev:1-1/nlev;
                
                % Fractional intensities for split pixels
                ilev_mainNow = [1 ilev_main];
                ilev_spltNow = [0 ilev_splt];

            % Dot could split main pixel with left or right pixel
            else
                % Indices to process
                ind_nowFilt = find(col_rght<=corrWind(2));
                ind_addFilt = find(col_left>0);
                ind_now = ind_rght(ind_nowFilt);
                ind_add = ind_left(ind_addFilt);
                
                % Fractional disparity values
                dispRef = [0:1/nlev:1-1/nlev -1/nlev:-1/nlev:1/nlev-1];
                
                % Fractional intensities for split pixels
                ilev_mainNow = [1 ilev_main ilev_main];
                ilev_spltNow = [0 ilev_splt ilev_splt];
            end
            
            % Perform sub-pixel refinement loop
            totlev = size(dispRef,2);
            horzCov_sub = zeros(1,totlev);
            for isub = 1:totlev
                window_sub = zeros(corrWind);
                window_sub(ind_main) = ilev_mainNow(isub)*ilev_subNow;
                if isub <= nlev
                    window_sub(ind_now) = ilev_spltNow(isub)*ilev_subNow(ind_nowFilt);
                else
                    window_sub(ind_add) = ilev_spltNow(isub)*ilev_subNow(ind_addFilt);
                end

                % Minimize sum of absolute differences
                horzCov_sub(isub) = sum(sum(abs(window_now-window_sub)));
            end
            [~,dispInd] = min(horzCov_sub);
            dispNow = dispNow + dispRef(dispInd);
                
            % Convert disparity to depth from lookup table ----------------
%             DEPTHimg(ipix) = depth_all(disp_all==dispNow);
            DEPTHimg(ipix_row,ipix_col) = depth_all(disp_all==dispNow);
        end
    end
end
% end