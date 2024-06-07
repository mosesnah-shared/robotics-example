% [Title]     Conflation with Task-description and Task-execution
% [Author]    Moses Chong-ook Nah
% [Email]     mosesnah@mit.edu
% [Update]    At 2024.06.07

%% (--) Initialization of the 2-DOF Robot Simulator
%% Cleaning up + Environment Setup
clear; close all; clc;

dt    = 1e-3;               % Time-step 
T     = 6;                  % Total run-time 
t_arr = 0:dt:T;             % Total time array of the simulation
Nt    = length( t_arr );    % Total number of time step

%% (1A) Joint-command for Oscillatory Movement

c     = [ 0.0; 1.0 ];                   % Center location          
r     = 0.3;                            % Radius
p_osc = c + r*[ cos( 2*pi*t_arr ); ...
                sin( 2*pi*t_arr ) ];    % Circular Trajectory

% Initialization of the corresponding joint movements. 
q_osc = zeros( 2, Nt );

% Calculate the input q via Inverse Kinematics.
% We assume that the length of each limb is 1m. 
% 2-DOF Inverse Kinematics, lower-limb configuration:
% [REF] https://robotacademy.net.au/lesson/inverse-kinematics-for-a-2-joint-robot-arm-using-geometry/
for i = 1 : Nt
    q_osc( 2, i ) = acos( ( p_osc( 1, i )^2 + p_osc( 2, i )^2 - 2 )/2 );
    q_osc( 1, i ) = atan2( p_osc( 2, i ), p_osc( 1, i ) ) - atan2( sin( q_osc( 2, i ) ), 1 + cos( q_osc( 2, i ) ) );
end

% Add angle offset for visualization
q_osc( 1, : ) = q_osc( 1, : ) + pi/2;

%% (1B) Joint-command for Discrete Movement

% Consider a minimum-jerk trajectory 
p_init = [  0.8, 1.4 ];     % Initial position
pf     = [ -0.8, 1.4 ];     %   Final position
ti = 2.0;                   % Start time
D  = 2.0;                   % Movement duration

% Create the minimum-jerk trajectory array
p_mjt = min_jerk_traj( p_init, pf, D, t_arr, ti );

% The corresponding joint angles
q_mjt = zeros( 2, Nt );

% Calculate the input q via Inverse Kinematics.
% We assume that the length of each limb is 1m. 
% 2-DOF Inverse Kinematics, lower-limb configuration:
% [REF] https://robotacademy.net.au/lesson/inverse-kinematics-for-a-2-joint-robot-arm-using-geometry/for i = 1 : Nt
for i = 1 : Nt
    q_mjt( 2, i ) = acos( ( p_mjt( 1, i )^2 + p_mjt( 2, i )^2 - 2 )/2 );
    q_mjt( 1, i ) = atan2( p_mjt( 2, i ), p_mjt( 1, i ) ) - atan2( sin( q_mjt( 2, i ) ), 1 + cos( q_mjt( 2, i ) ) );
end

% Add angle offset for visualization
q_mjt( 1, : ) = q_mjt( 1, : ) + pi/2;

%% (1C) Simulation

% Initialization of time
t = 0;

% Set the q_command and its result for visualization
% Choose one of the following choices
% q_command = q_mjt;    
% q_command = q_osc;
q_command = q_osc + q_mjt;

% the resulting p
p_result = [  sin( q_command( 1, : ) ) + sin( q_command( 1, : ) + q_command( 2, : ) );  ...
             -cos( q_command( 1, : ) ) - cos( q_command( 1, : ) + q_command( 2, : ) ) ];


% Robot Initialization
robot = DoublePendulum( 1, 1, 1, 1 );
robot.init( );

% Animation Initialization and attach Robot
anim = Animation( 'Dimension', 2, 'xLim', [ -2.1, 2.1 ], 'yLim', [ -1.1, 3.1 ], 'isSaveVideo', true );
anim.init( );
anim.attachRobot( robot ) 

% Attach the plot to the right-side for torus visualization
subplot( 1, 2, 2, anim.hAxes );
set( anim.hAxes, 'xticklabel', {}, 'yticklabel', {} )
anim.SubTitle = '';
xlabel( '' ); ylabel( '' )
% Plot the results
plot( anim.hAxes, p_result( 1, : ), p_result( 2, : ), 'linewidth', 2, 'color', [0.6350 0.0780 0.1840] )
% scatter( anim.hAxes, p_result( 1,   1 ), p_result( 2,   1 ), 150, 'o', 'linewidth', 2, 'markeredgecolor', 'k', 'markerfacecolor', [0.6350 0.0780 0.1840] )
% scatter( anim.hAxes, p_result( 1, end ), p_result( 2, end ), 150, 'd', 'linewidth', 2, 'markeredgecolor', 'k', 'markerfacecolor', [0.6350 0.0780 0.1840] )

% Torus Visualization
a1 = subplot( 1, 2, 1 );
hold on
% Parameters for the torus
R = 3.0; r = 1.0;

% Define the angles
theta = linspace( 0, 2*pi, 100 ); 
phi   = linspace( 0, 2*pi, 100 ); 

% Create the meshgrid for the angles
[ Theta, Phi ] = meshgrid( theta, phi );

% Parametric equations for the T2 Torus
X = ( R + r*cos( Theta ) ) .* cos( Phi );
Y = ( R + r*cos( Theta ) ) .* sin( Phi );
Z =       r*sin( Theta );

% Plotting Torus
surf( a1, X, Y, Z, 'EdgeColor', 'k', 'linewidth', 1, 'edgealpha', 0.3, 'FaceColor', 'none' );

% Plot the q trajectory on Torus
Xq = ( R + r*cos( q_command( 1, : ) ) ) .* cos( q_command( 2, : ) );
Yq = ( R + r*cos( q_command( 1, : ) ) ) .* sin( q_command( 2, : ) );
Zq =       r*sin( q_command( 1, : ) );

% Plot the line and marker
plot3( a1, Xq, Yq, Zq, 'linewidth', 3, 'color', [0.6350 0.0780 0.1840] )
% scatter3( a1, Xq(   1 ), Yq(   1 ), Zq(   1 ), 150, 'o', 'linewidth', 2, 'markeredgecolor', 'k', 'markerfacecolor', [0.6350 0.0780 0.1840] )
% scatter3( a1, Xq( end ), Yq( end ), Zq( end ), 150, 'd', 'linewidth', 2, 'markeredgecolor', 'k', 'markerfacecolor', [0.6350 0.0780 0.1840] )

mkr = scatter3( a1, Xq( 1 ), Yq( 1 ), Zq( 1 ), 500, 'filled', 'markeredgecolor', [0.6350 0.0780 0.1840], 'markerfacecolor', 'w', 'linewidth', 4 );
axis equal;
set( a1, 'visible', 'off', 'view', [8.2212, 36.7950] )

% Run Animation! 
q_init = q_command( :, 1 );
robot.updateKinematics( q_init );
anim.update( 0 );

ns = 1;

for i = 1 : Nt
    
    if round( t / anim.FrameUpdateTime ) >= ns

        % Update the linkage plot
        robot.updateKinematics( q_command( :, i ) );
        anim.update( t );    
        ns = ns + 1;
        
        set( mkr, 'XData', Xq( i ), 'YData', Yq( i ), 'ZData', Zq( i ) )

    end

    % Get the forward kinematics of the EE
    t = t + dt;                                                                
    
end

anim.close( )
