% [Title]     Joint-space Impedance Control, 2DOF
% [Author]    Moses Chong-ook Nah
% [Email]     mosesnah@mit.edu
% [Update]    At 2023.11.29

%% (--) Initialization of the Robot Simulator
%% Cleaning up + Environment Setup
clear; close all; clc;

%% =======================================================
%% (1-) Task-space Impedance Control, Position and Orientation
%%  -- (1A) Parameters for Simulation

% Simulation settings
T     = 6.0;             % Total simulation time
t     = 0.0;             % The current time of simulation   
t0i   = 1.0;             % Initial Time
dt    = 1e-3;            % Time-step of simulation 
t_arr = 0:dt:T;          % Time array for the simulation
Nt    = length( t_arr ); % Total time steps

% Set figure size and attach robot to simulation
robot       = DoublePendulum( 1, 1, 0.3, 0.3 );
robot_ghost = DoublePendulum( 1, 1, 0.3, 0.3 );

robot.init( );
robot_ghost.init( );

% Create animation and attach robots
offy = 0.3;
anim = Animation( 'Dimension', 2, 'xLim', [-0.9,0.9], 'yLim', [-0.9+offy,0.9+offy], 'isSaveVideo', true );
anim.init( );

anim.attachRobot( robot_ghost ) 
anim.attachRobot( robot       ) 

% Set the transparency of the ghost
anim.gLinks{ 1 }{ 2 }.Children( 1 ).MarkerFaceAlpha = 0.4;
anim.gLinks{ 1 }{ 2 }.Children( 1 ).MarkerEdgeAlpha = 0.4;
anim.gLinks{ 1 }{ 2 }.Children( 2 ).Color(4)=0.1;

anim.gLinks{ 1 }{ 3 }.Children( 1 ).MarkerFaceAlpha = 0.4;
anim.gLinks{ 1 }{ 3 }.Children( 1 ).MarkerEdgeAlpha = 0.4;
anim.gLinks{ 1 }{ 3 }.Children( 2 ).MarkerFaceAlpha = 0.4;
anim.gLinks{ 1 }{ 3 }.Children( 2 ).MarkerEdgeAlpha = 0.4;
anim.gLinks{ 1 }{ 3 }.Children( 3 ).Color(4)=0.1;

% Update kinematics
q_init = [ pi/2+0.1; 0.4];
delq   = [ 1.0; 1.0 ];
robot.updateKinematics( q_init );
robot_ghost.updateKinematics( q_init );
anim.update( 0 );

% Title: simulation time
titleString = sprintf( 'Time: %2.1f sec', 0 );
mytitle = title( titleString );
set( mytitle, 'FontSize' , 15);

%%  -- (1B) Parameters for Minimum-jerk Trajectory

% Initial joint posture and velocity
q  = q_init;
dq = zeros( robot.nq, 1 );


% Joint-space impedances
Kq = 5 * eye( 2 );
Bq = 0.1 * Kq;
D  = 1.6;
[ q0_arr, dq0_arr, ~ ] = min_jerk_traj( q_init, q_init + delq, D, t_arr, t0i );

ns = 1;

for i = 1 : Nt
    
    % Get the mass matrix of the Acrobot
    M = robot.getMassMatrix( q );
    C = robot.getCoriolisMatrix( q, dq );
    
    % The joint-space impedance controller
    tau = Kq * ( q0_arr( :, i ) - q ) + Bq * ( dq0_arr( :, i ) - dq );
    rhs = M\( -C * dq + tau ); 
    
    [ q1, dq1 ] = func_symplecticEuler( q, dq, rhs, dt );
    q  =  q1;
    dq = dq1;
    
    if round( t / anim.FrameUpdateTime ) >= ns
        % Update the linkage plot
        robot.updateKinematics( q );
        robot_ghost.updateKinematics( q0_arr( :, i ) );
        anim.update( t );    
        ns = ns + 1;
        
        % Set animation title
        set( mytitle, 'String', sprintf( 'Time: %2.1f sec', t ) );
        
    end

    % Get the forward kinematics of the EE
    t = t + dt;                                                                
    
end

anim.close( )
