% [Title]     Joint-space Impedance Control
% [Author]    Moses Chong-ook Nah
% [Email]     mosesnah@mit.edu
% [Update]    At 2023.11.27

%% (--) Initialization of the Robot Simulator
%% Cleaning up + Environment Setup
clear; close all; clc;

%% =======================================================
%% (1-) Joint-space Impedance Control
%%  -- (1A) Parameters for Simulation

% Simulation settings
T  =  6;              % Total simulation time
t  =  0;              % The current time of simulation   
dt = 1e-4;            % Time-step of simulation 
t_arr = 0:dt:T;       % Time array
Nt = length( t_arr ); % The number of elements for time array

% Define robot, initialization
robot = iiwa14( 'high' );
robot.init( );

% Create animation and Attach robot
anim = Animation( 'Dimension', 3, 'xLim', [-0.9,0.9], 'yLim', [-0.9,0.9], 'zLim', [0,1.8], 'isSaveVideo', true );
anim.init( );
anim.attachRobot( robot ) 

% Set initial, final joint positions and update kinematics
q_init  = [0, 28.56, 0, -87.36, -7.82, 75.56, -9.01]' * pi/180;
q_del   = zeros( 7, 1 ); q_del( 1 ) = 0.3; q_del( 3 ) = -0.3;
q_final = q_init + q_del;
D       = 2.0;
t0i     = 0.3;

robot.updateKinematics( q_init );
anim.update( 0 );

% Generating the min-jerk trajectory in joint space
[ q0, dq0, ~ ] = min_jerk_traj( q_init, q_final, D, t_arr, t0i );


%%  -- (1B) Main Simulator

% Title: simulation time
titleString = sprintf( 'Time: %2.1f sec', 0 );
mytitle = title( titleString );
set( mytitle, 'FontSize' , 15);

% Set initial condition for joint-space position and velocity
q  = q_init;
dq = zeros( robot.nq, 1 );

% Time step for the simulation
n  = 1;
ns = 1;

% Joint-space impedances
Kq = 2.0 * eye( robot.nq );
Bq = 0.5 * eye( robot.nq );

for i = 1 : Nt

    t = t_arr( i );

    % Get the mass matrix of the Acrobot
    M = robot.getMassMatrix( q );
    C = robot.getCoriolisMatrix( q, dq );

    % Get the Gravity term of the robot
    G = robot.getGravityVector( q );

    % Get the Hybrid Jacobian 
    JH = robot.getHybridJacobian( q );
  
    tau = Kq * ( q0( :, i ) - q ) + Bq * ( dq0( :, i ) - dq );
    rhs = M\( -C * dq + tau ); 
    
    [ q1, dq1 ] = func_symplecticEuler( q, dq, rhs, dt );
    q  =  q1;
    dq = dq1;
    
    % Update the simulator with smaller rate
    if round( t / anim.FrameUpdateTime ) >= ns

        % Update the linkage plot
        robot.updateKinematics( q );
        anim.update( t );    

        ns = ns + 1;

        % Set animation title
        set( mytitle, 'String', sprintf( 'Time: %2.1f sec', t ) );
        
    end

end

anim.close( )