%% Continuum Manipulator Constant Curvature Kinematics Finalised Theory Single Section
% Script description:
% Program is an extension of CC Kinematics_SingleSection with the addition
% of the finalised theory.

%% Clear workspace, Command Window and Close all Figures
clear; clc; close all; pause(1e-1);

%% Define Helix Dimensions

Wire_Radius = 0.75; %mm, the radius of the actual SMA
Helix_Radius  = 7.325/2; %mm the outer diameter + inner diameter of the SMA averaged - divide by two for the radius 
Helix_Turns = 48.5;
Helix_Length = 894; %mm 
Helix_Pitch = Helix_Length / Helix_Turns * [1, 1, 1, 1, 1, 1, 1, 1, 1, 1]; %mm/turn
Min_Pitch = (Wire_Radius)*2 ;

% Define the number of bending sections.
N_Modules = 10;
N_Segments = 3; 
N_ATurns = [5, 5, 5, 5, 5, 5, 5, 5, 5, 3.5]; % number of active turns
N_IATurns = 0 + [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]; % number of inactive turns, we have none in this case


%% Pre-allocate Arrays
% Where:
% Active_Time is array with 3 individual Active Time for each section
% section.
% Delta Lead Ange is the change in helix angle.
% Delta_Pitch is the differential of the pitch wrt to the helix angle.
% Int_L is the integral of the pitch wrt to the polar angle.

[Active_Time, Delta_Lead_Angle, Delta_Pitch, Int_L] = deal(zeros(N_Modules, N_Segments));

%% Determine Input Parameters.
% For multiple bending segments, each row would represent a single segment.
% Each column represent the desired temperatures for polar angle 0 - 120,
% 120 - 240 and 240 - 360 respectively.

% exp figs are y-z plane views, -y: right
% 1st column: bends toward -y, fit with 1st seg in exp
% 2nd column: bends toward +x+y, not wired in experiments
% 3rd column: bends toward -x+y, fit with 2nd seg in exp

% t0 represents the thermal lag, found from the tests carried out in labs
t0 = ones(N_Modules, N_Segments).* 1.7;

% Create dummy variables that are the correct size (i.e. pad C with the
% correct number of ones to create a matrix of size N_Modules x
% N_Segments)

% dummy C of correct size:

% coeffecients set to equal one
C = (0 + ...
    ones(N_Modules, N_Segments));




% for loop to generate set number of random points on the workspace
points = 5500;
for i = 1:points
    %Create a random active-time matrix on each iteration. Sections are
    %randomly activated between 0 and 30 seconds
    Active_Time = ones(N_Modules,N_Segments) + ...
        [randi([0,30],N_Modules,N_Segments)];
    
    Exp_Angle = '3D' ; % '2D' or '3D': view angles in the paper
    if( numel(Active_Time) ~= (N_Modules * N_Segments))
        disp('Invalid number of elements in Active_Time array.')
    end
    
    % Determines the helix lead angle for each Active_Time & determines the
    % differential of the pitch wrt to the lead angle.
    Ts = eye(4); phis = 0; is = 0 ;
    N_steps = 3 ;
    for i = 1:N_Modules
        for j = 1:N_Segments
            Delta_Pitch(i,j) = Pitch_Time(Active_Time(i,j),C(i,j),t0(i,j),Min_Pitch); % change in lead angle
        end
        
        %%Robot Independent Mapping (Configuration Space to Task Space) %%
        % Average values of 3 lengths were used for L0.
        L0(i) = N_ATurns(i) * Helix_Pitch(i); %m, curve (active) length
        LIN(i) = N_IATurns(i) * Helix_Pitch(i) ; % inactive length
        
        % Calculate the 3 Lengths
        L1(i) = L0(i) + N_ATurns(i)/N_Segments * Delta_Pitch(i,1);
        L2(i) = L0(i) + N_ATurns(i)/N_Segments * Delta_Pitch(i,2);
        L3(i) = L0(i) + N_ATurns(i)/N_Segments * Delta_Pitch(i,3);
        
        % Determine Configuration space values (Length, Phase angle and Curvature)
        S_Length(i) = Length(L1(i), L2(i), L3(i)); %S_Length --> Section length
        S_Phase_Angle(i) = Phase_Angle(L1(i), L2(i), L3(i)); %S_Phase_Angle --> Section phase angle
        S_Curvature(i) = Kappa(L1(i), L2(i), L3(i), Helix_Radius); %S_Curvature --> Section Curvature
        
        % Equations obtained from Transformation matrix from base to end effector.
        dL = S_Length(i) / N_steps ;
        for ii = 1 : N_steps % curve (active) part
            is = is + 1 ;
            Ts = Ts * Transformation_Matrix(S_Phase_Angle(i), S_Curvature(i), dL, 0);
            X_pos(is) = Ts(1,4);
            Y_pos(is) = Ts(2,4);
            Z_pos(is) = Ts(3,4);
            phis = phis + N_ATurns(i) * 2*pi * dL/S_Length(i) ;
            Temp_Helix = Ts * [Helix_Radius*[cos(phis), sin(phis), 0], 1]';
            X_helix(is) = Temp_Helix(1);
            Y_helix(is) = Temp_Helix(2);
            Z_helix(is) = Temp_Helix(3);
        end
        if LIN(i) > 0 % if inactive section presents
            dL = LIN(i) / N_steps ;
            for ii = 1 : N_steps % straight (inactive) part
                is = is + 1 ;
                Ts = Ts * Transformation_Matrix(0, 0, dL, 0);
                X_pos(is) = Ts(1,4);
                Y_pos(is) = Ts(2,4);
                Z_pos(is) = Ts(3,4);
                phis = phis + N_IATurns(i) * 2*pi * dL/LIN(i) ;
                Temp_Helix = Ts * [Helix_Radius*[cos(phis), sin(phis), 0], 1]';
                X_helix(is) = Temp_Helix(1);
                Y_helix(is) = Temp_Helix(2);
                Z_helix(is) = Temp_Helix(3);
            end
        end
    end
    S_Theta_deg = S_Length .* S_Curvature * 180 / pi; % for identification of correction coefficient
    
    % Plots continuum backbone using X, Y and Z coordinates.
    % defines figure configuration
    
    % plots the end of the manipulator using the XYZ coordinates.
    plot3(X_pos(end), Y_pos(end), Z_pos(end), 'k.'); hold on
    xlabel('x [mm]')
    ylabel('y [mm]')
    zlabel('z [mm]')
    axis equal
    if strcmp(Exp_Angle, '2D')
        view([1, 0 ,0]);
    end
    set(gca, 'FontSize', 12);
end

%% Function Definitions
% Temperature - Helix lead angle relation
function y = Pitch_Time(x,C,t0E,Min_Pitch) % Gamma vs. activation time
% Pitch_Time(Active_Time(i,j),C(i,j),t0(i,j))

%coefficients in equation
A = - 0.3573 ;
y0 = 6.1078 ;
%t0 = - y0 / A ;
%t0E = 2.13;

if x < t0E
    y = 0 ;
% need to ensure that the robot doesn't bend in on itself, therefore we
% need to add a condition such that it cannot bend beyond the minimum pitch (Min_Pitch)
elseif x < Min_Pitch
     y = Min_Pitch;
else
    y = C * A * ( x - t0E ) ; % Averaged R^2: 0.95085
end


end



% Configuration Space - Length
function L = Length(x1, x2, x3)
L = (x1 + x2 + x3) / 3;
end

% Configuration Space - Phase angle
function P_A = Phase_Angle(x1, x2, x3)
%P_A is the phase angle
P_A = atan2(  sqrt(3)*(x2 + x3 - (2*x1)) , (3*(x2 - x3))  ); %Answer is given in radians
if isnan(P_A); P_A = 0; end % bypass singularity
end

% Configuration Space - Curvature
function K = Kappa(x1, x2, x3, Helix_Radius)
K = 2*sqrt(x1*x1 + x2*x2 + x3*x3 - x1*x2 - x1*x3 - x2*x3) / (Helix_Radius*(x1 + x2 + x3) );
end

function [TM_Base_Tip] = Transformation_Matrix(PA, K, S, L)
Kt = K; if K == 0; Kt=1; end % bypass singularity
TM_Base_Tip = [             cos(K*S)*cos(PA)^2 + sin(PA)^2,   cos(K*S)*cos(PA)*sin(PA) - cos(PA)*sin(PA),   sin(K*S)*cos(PA),     -(cos(PA)*(cos(K*S) - 1))/Kt;
    cos(K*S)*cos(PA)*sin(PA) - cos(PA)*sin(PA),               cos(PA)^2 + cos(K*S)*sin(PA)^2,   sin(K*S)*sin(PA),     -(sin(PA)*(cos(K*S) - 1))/Kt;
    -sin(K*S)*cos(PA),                            -sin(K*S)*sin(PA),           cos(K*S),                      sin(K*S)/Kt;
    0,                                            0,                  0,                                1] * ...
    [1 0 0 0; 0 1 0 0; 0 0 1 L; 0 0 0 1];
% Note that the results of the function is given in radians
end

