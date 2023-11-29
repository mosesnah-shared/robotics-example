% [Title]     Task-space Impedance Control, Position and Orientation Type1
% [Author]    Moses Chong-ook Nah
% [Email]     mosesnah@mit.edu
% [Update]    At 2023.11.27

%% (--) Initialization of the Robot Simulator
%% Cleaning up + Environment Setup
clear; close all; clc;

%% =======================================================
%% (1-) Task-space Impedance Control, Position and Orientation
%%  -- (1A) Parameters for Simulation

% Simulation settings
T   =   20;            % Total simulation time
t   =    0;            % The current time of simulation   
t0i =  0.1;            % Initial time start of movement
dt  = 1e-4;            % Time-step of simulation 
t_arr  = 0:dt:T;       % Time array
Nt  = length( t_arr ); % Number of time-array elements

% Define robot, initialization
robot = iiwa14( 'high' );
robot.init( );

% Create animation and Attach robot
anim = Animation( 'Dimension', 3, 'xLim', [-0.9,0.9], 'yLim', [-0.9,0.9], 'zLim', [0,1.8], 'isSaveVideo', true );
anim.init( );
anim.attachRobot( robot ) 

% Set the initial joint position and update
q_init = [0, 28.56, 0, -87.36, 0, 75.56, -9.01]' * pi/180;
robot.updateKinematics( q_init );
anim.update( 0 );

% Title: simulation time
titleString = sprintf( 'Time: %2.1f sec', 0 );
mytitle = title( titleString );
set( mytitle, 'FontSize' , 15);

%% -- (1B) Motion Planning of end-effector 

% delta movements
delx = [0.2; 0.0; 0];
dely = [0.0; 0.2; 0];

% Time-offset between, duration of movements 
toff = 0.5;
D    = 1.5;

% Get the initial end-effector position and orientation 
Hi = robot.getForwardKinematics( q_init );
Ri = Hi( 1:3, 1:3 );
pi = Hi( 1:3,   4 );

% Generate the min-jerk movements 
[ p_tmp1, dp_tmp1, ~ ] = min_jerk_traj(            pi, pi + delx, D, t_arr, t0i                    );
[ p_tmp2, dp_tmp2, ~ ] = min_jerk_traj( zeros( 3, 1 ),    - delx, D, t_arr, t0i + 1 * ( D + toff ) );
[ p_tmp3, dp_tmp3, ~ ] = min_jerk_traj( zeros( 3, 1 ),    + dely, D, t_arr, t0i + 2 * ( D + toff ) );
[ p_tmp4, dp_tmp4, ~ ] = min_jerk_traj( zeros( 3, 1 ),    - dely, D, t_arr, t0i + 3 * ( D + toff ) );
[ p_tmp5, dp_tmp5, ~ ] = min_jerk_traj( zeros( 3, 1 ),    - delx, D, t_arr, t0i + 4 * ( D + toff ) );
[ p_tmp6, dp_tmp6, ~ ] = min_jerk_traj( zeros( 3, 1 ),    + delx, D, t_arr, t0i + 5 * ( D + toff ) );
[ p_tmp7, dp_tmp7, ~ ] = min_jerk_traj( zeros( 3, 1 ),    - dely, D, t_arr, t0i + 6 * ( D + toff ) );
[ p_tmp8, dp_tmp8, ~ ] = min_jerk_traj( zeros( 3, 1 ),    + dely, D, t_arr, t0i + 7 * ( D + toff ) );

% The movements for pos., vel. and acc.
  p0_arr =   p_tmp1 +   p_tmp2 +   p_tmp3 +   p_tmp4 +   p_tmp5 +   p_tmp6 +   p_tmp7 +   p_tmp8;
 dp0_arr =  dp_tmp1 +  dp_tmp2 +  dp_tmp3 +  dp_tmp4 +  dp_tmp5 +  dp_tmp6 +  dp_tmp7 +  dp_tmp8;

% Plotting this to anim
p_tmp1 = p_tmp1 + pi; p_tmp2 = p_tmp2 + pi; p_tmp3 = p_tmp3 + pi; p_tmp4 = p_tmp4 + pi;
p_tmp5 = p_tmp5 + pi; p_tmp6 = p_tmp6 + pi; p_tmp7 = p_tmp7 + pi; p_tmp8 = p_tmp8 + pi;

plot3( anim.hAxes, p_tmp1( 1, : ), p_tmp1( 2, : ), p_tmp1( 3, : ), 'linewidth', 3, 'color', 'k' )
plot3( anim.hAxes, p_tmp2( 1, : ), p_tmp2( 2, : ), p_tmp2( 3, : ), 'linewidth', 3, 'color', 'k' )
plot3( anim.hAxes, p_tmp3( 1, : ), p_tmp3( 2, : ), p_tmp3( 3, : ), 'linewidth', 3, 'color', 'k' )
plot3( anim.hAxes, p_tmp4( 1, : ), p_tmp4( 2, : ), p_tmp4( 3, : ), 'linewidth', 3, 'color', 'k' )
plot3( anim.hAxes, p_tmp5( 1, : ), p_tmp5( 2, : ), p_tmp5( 3, : ), 'linewidth', 3, 'color', 'k' )
plot3( anim.hAxes, p_tmp6( 1, : ), p_tmp6( 2, : ), p_tmp6( 3, : ), 'linewidth', 3, 'color', 'k' )
plot3( anim.hAxes, p_tmp7( 1, : ), p_tmp7( 2, : ), p_tmp7( 3, : ), 'linewidth', 3, 'color', 'k' )
plot3( anim.hAxes, p_tmp8( 1, : ), p_tmp8( 2, : ), p_tmp8( 3, : ), 'linewidth', 3, 'color', 'k' )

% Initial joint posture and velocity
q  = q_init;
dq = zeros( robot.nq, 1 );

%% -- (1C) Main Simulation

% Time step for the simulation
ns = 1;

% Task-space impedances for Position
Kp = 400 * eye( 3 );
Bp = 0.1 * Kp;

% Joint-space impedance, damping
Bq = .8 * eye( robot.nq );

% Task-space impedances for Orientation
Kr = 6.0 * eye( 3 );
Br = 0.6 * eye( 3 );

% The whole movement's frequency
t_freq = t0i + 8 * ( D + toff ) - toff;
N_freq = round( t_freq/dt ) + 1;

for i = 1 : Nt

    t = t_arr( i );
    
    % Get the mass matrix of the Acrobot
    M = robot.getMassMatrix( q );
    C = robot.getCoriolisMatrix( q, dq );

    % Get the Hybrid Jacobian 
    JH = robot.getHybridJacobian( q );
    
    % Get the end-effector position and velocity 
    dp = JH( 1:3, : ) * dq;
    
    % The initial end-effector position 
    H = robot.getForwardKinematics( q );
    R = H( 1:3, 1:3 );
    p = H( 1:3,   4 );

    ii = rem( i , N_freq )+1;
    tau1 = JH( 1:3, : )' * ( Kp * ( p0_arr( :, ii ) - p ) + Bp * ( dp0_arr( :, ii ) - dp ) );
    tau2 = JH( 4:6, : )' * Kr * R * so3_to_R3( LogSO3( R' * Ri ) );
    tau3 = - Bq * dq;
    tau  = tau1 + tau2 + tau2;

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

                                                                 
end

anim.close( )
