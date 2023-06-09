% [Project]        Robot Simulator - iiwa14, task space impedance
% Authors                       Email
%   [1] Johannes Lachner        jlachner@mit.edu
%   [2] Moses C. Nah            mosesnah@mit.edu
%
%
% The code is heavily commented. A famous quote says:
% "Code is read more often than it is written"
%           - Guido Van Rossum, the Creator of Python

%% Cleaning up + Environment Setup
clear; close all; clc;

% Simulation settings
simTime = 40;        % Total simulation time
t  = 0;             % The current time of simulation   
dt = 0.001;         % Time-step of simulation 

% Set figure size and attach robot to simulation
robot = iiwa14( 'high' );
robot.init( );

% Create animation
anim = Animation( 'Dimension', 3, 'xLim', [-0.9,0.9], 'yLim', [-0.9,0.9], 'zLim', [0,1.8], 'isSaveVideo', true );
anim.init( );

view( 90, 90 )

anim.attachRobot( robot ) 

% Update kinematics
q_init = [0, 28.56, 0, -87.36, -7.82, 75.56, -9.01]' * pi/180;
robot.updateKinematics( q_init );
anim.update( 0 );

% Title: simulation time
titleString = sprintf( 'Time: %2.1f sec', 0 );
mytitle = title( titleString );
set( mytitle, 'FontSize' , 15);

% Point-to-point Joint Space Impedance Controller
% Using Minimum-jerk trajectory as the virtual trajectory 

% Initial joint posture and velocity
q  = q_init;
dq = zeros( robot.nq, 1 );

% The initial end-effector position 
Hi = robot.getForwardKinematics( q );
pi = Hi( 1:3, 4 );

% Desired final posture 
delx = [0.2; 0.0; 0];
dely = [0.0; 0.2; 0];

toff = 0.5;

% Time step for the simulation
ns = 0;

% End-effector task-space impedances
Kp = 400 * eye( 3 );
Bp = 0.1 * Kp;

Bq = .8 * eye( robot.nq );

t0 = 0.1;
D  = 1.6;

t_freq = 8 * ( D + toff );

while t <= simTime
    
    % Get the mass matrix of the Acrobot
    M = robot.getMassMatrix( q );
    C = robot.getCoriolisMatrix( q, dq );

    % Get the Gravity term of the robot
    G = robot.getGravityVector( q );

    % Get the Hybrid Jacobian 
    JH = robot.getHybridJacobian( q );
    
    % The joint-space impedance controller
    [ p_ref1, dp_ref1, ~ ] = min_jerk_traj( t0                 , rem( t, t_freq ), D, pi, pi + dely );
    [ p_ref2, dp_ref2, ~ ] = min_jerk_traj( t0 +      D + toff , rem( t, t_freq ), D, zeros( 3, 1 ), -dely );
    [ p_ref3, dp_ref3, ~ ] = min_jerk_traj( t0 + 2 * (D + toff), rem( t, t_freq ), D, zeros( 3, 1 ),  delx );    
    [ p_ref4, dp_ref4, ~ ] = min_jerk_traj( t0 + 3 * (D + toff), rem( t, t_freq ), D, zeros( 3, 1 ), -delx );  
    [ p_ref5, dp_ref5, ~ ] = min_jerk_traj( t0 + 4 * (D + toff), rem( t, t_freq ), D, zeros( 3, 1 ), -dely );
    [ p_ref6, dp_ref6, ~ ] = min_jerk_traj( t0 + 5 * (D + toff), rem( t, t_freq ), D, zeros( 3, 1 ),  dely );
    [ p_ref7, dp_ref7, ~ ] = min_jerk_traj( t0 + 6 * (D + toff), rem( t, t_freq ), D, zeros( 3, 1 ), -delx );
    [ p_ref8, dp_ref8, ~ ] = min_jerk_traj( t0 + 7 * (D + toff), rem( t, t_freq ), D, zeros( 3, 1 ),  delx );


    p_ref  =  p_ref1 +  p_ref2 +  p_ref3 +  p_ref4 +  p_ref5 +  p_ref6 +  p_ref7 +  p_ref8;
    dp_ref = dp_ref1 + dp_ref2 + dp_ref3 + dp_ref4 + dp_ref5 + dp_ref6 + dp_ref7 + dp_ref8;

    % Get the end-effector position and velocity 
    dp = JH( 1:3, : ) * dq;
    
    % The initial end-effector position 
    H = robot.getForwardKinematics( q );
    p = H( 1:3, 4 );

    % Summing up the minimum-jerk trajectories

    tau = JH( 1:3, : )' * ( Kp * ( p_ref - p ) + Bp * ( dp_ref - dp ) ) - Bq * dq;
    rhs = M\( -C * dq + tau ); 

%     rhs = zeros( robot.nq, 1 );
    
    % We compare the C matrices with the matrices provided from Eq.9
    % The default values of our Acrobot geometrical/inertial parameters are as follows:
    % m1 = 1, m2 = 1, lc1 = 0.5, lc2 = 0.5, l1 = 1.0, l2 = 1.0    
    % The following 
    % [REF] https://underactuated.csail.mit.edu/acrobot.html#section1
    
    [ q1, dq1 ] = func_symplecticEuler( q, dq, rhs, dt );
    q  =  q1;
    dq = dq1;
    
    if round( t / anim.FrameUpdateTime ) >= ns
        % Update the linkage plot
        robot.updateKinematics( q );
        anim.update( t );    
        ns = ns + 1;
        
        p_arr( ns, : ) = p;
        p_ref_arr( ns, : ) = p_ref;
        
        % Set animation title
        set( mytitle, 'String', sprintf( 'Time: %2.1f sec', t ) );
        
    end

    % Get the forward kinematics of the EE
    t = t + dt;                                                                
    
end

anim.close( )

%% Minimum-jerk trajectory
function [p, dp, ddp] = min_jerk_traj( t0, t, D, qi, qf )

% Assertion
assert( t0 > 0 && ( length( qi ) == length( qf ) ) )

% The normalized time 
tau = (t - t0)/D; 

% The length of qi and qf
N = length( qi );

if t <= t0
    p   = qi;
    dp  = zeros( N, 1 );
    ddp = zeros( N, 1 );
    
elseif t0 <= t && t <= t0 + D
    p   =        qi + ( qf - qi ) * ( 10 * tau^3 -  15 * tau^4 +   6 * tau^5 );
    dp  =      1./D * ( qf - qi ) * ( 30 * tau^2 -  60 * tau^3 +  30 * tau^4 );
    ddp =  1./(D^2) * ( qf - qi ) * ( 60 * tau^1 - 180 * tau^2 + 120 * tau^3 );
    
else
    p   = qf;
    dp  = zeros( N, 1 );
    ddp = zeros( N, 1 );
end


end