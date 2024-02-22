% [Title]     Task-space Impedance Control, 2DOF, No redundancy
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
robot.init( );

% Create animation and attach robots
offy = 0.3;
lw   = 0.7;
anim = Animation( 'Dimension', 2, 'xLim', [ -lw, lw ], 'yLim', [-lw+offy, lw+offy], 'isSaveVideo', true );
anim.init( );
anim.attachRobot( robot );

% Update kinematics
q1 = 0.3;
q2 = pi-2*q1;
q_init = [ pi/2+q1; q2];
delq   = [ 1.0; 1.0 ];
robot.updateKinematics( q_init );
anim.update( 0 );

% Title: simulation time
titleString = sprintf( 'Time: %2.1f sec', 0 );
mytitle = title( titleString );
set( mytitle, 'FontSize' , 15);

%%  -- (1B) Parameters for Minimum-jerk Trajectory

% Initial joint posture and velocity
q  = q_init;
dq = zeros( robot.nq, 1 );

% Get the initial position 
Hi = robot.getForwardKinematics( q );
pi = Hi( 1:3, 4 );
delp = [ 0.0; 0.5; 0.0 ];

% Joint-space impedances
Kp = 400 * eye( 3 );
Bp = 0.1 * Kp;
D  = 1.6;
[ p0_arr, dp0_arr, ~ ] = min_jerk_traj( pi, pi +delp, D, t_arr, t0i );

ns = 1;

for i = 1 : Nt
    
    % Get the mass matrix of the Acrobot
    M = robot.getMassMatrix( q );
    C = robot.getCoriolisMatrix( q, dq );
    
    % Get the position and velocity 
    H = robot.getForwardKinematics( q );
    p = H( 1:3, 4 );

    JH  = robot.getHybridJacobian( q );
    JHp = JH( 1:3, : );
    dp  = JHp * dq;

    % The joint-space impedance controller
    tau = JHp' * ( Kp * ( p0_arr( :, i ) - p ) + Bp * ( dp0_arr( :, i ) - dp ) );
    rhs = M\( -C * dq + tau ); 
    
    [ q1, dq1 ] = func_symplecticEuler( q, dq, rhs, dt );
    q  =  q1;
    dq = dq1;
    
    if round( t / anim.FrameUpdateTime ) >= ns
        % Update the linkage plot
        robot.updateKinematics( q );
        anim.update( t );    
        ns = ns + 1;
        
        % Set animation title
        set( mytitle, 'String', sprintf( 'Time: %2.1f sec', t ) );
        
    end

    % Get the forward kinematics of the EE
    t = t + dt;                                                                
    
end

anim.close( )
