% [Title]     Task-space Impedance Control, Imitation Learning
% [Author]    Moses Chong-ook Nah
% [Email]     mosesnah@mit.edu
% [Update]    At 2024.02.21

%% (--) Initialization of the Robot Simulator
%% Cleaning up + Environment Setup
clear; close all; clc;

%% =======================================================
%% (1-) Drawing M, Scaling Invariance
%%  -- (1A) Parameters for Simulation

% Simulation settings
T   =  5.0;            % Total simulation time
t0i =  0.3;            % Initial time start of movement
dt  = 1e-3;            % Time-step of simulation 
t_arr  = 0:dt:T;       % Time array
Nt  = length( t_arr ); % Number of time-array elements

% Create animation and Attach robot
anim = Animation( 'Dimension', 3, 'xLim', [-1.5,1.5], 'yLim', [-2.0,2.0], 'zLim', [0, 1.0], 'isSaveVideo', true, 'Title', 'sim1' );
anim.init( );

% Define robot, initialization
func_Hbase = @( x, y ) [ rotz( x ), rotz( x ) * [ 0; y; 0 ]; zeros( 1, 3 ), 1];

c = 0.3;
robot1 = iiwa14( 'high' ); robot1.init( ); robot1.H_base = func_Hbase( 0, -5*c );
robot2 = iiwa14( 'high' ); robot2.init( ); robot2.H_base = func_Hbase( 0, -3*c );
robot3 = iiwa14( 'high' ); robot3.init( ); robot3.H_base = func_Hbase( 0, -1*c );
robot4 = iiwa14( 'high' ); robot4.init( ); robot4.H_base = func_Hbase( 0,  1*c );
robot5 = iiwa14( 'high' ); robot5.init( ); robot5.H_base = func_Hbase( 0,  3*c );
robot6 = iiwa14( 'high' ); robot6.init( ); robot6.H_base = func_Hbase( 0,  5*c );

anim.attachRobot( robot1 ) 
anim.attachRobot( robot2 ) 
anim.attachRobot( robot3 ) 
anim.attachRobot( robot4 ) 
anim.attachRobot( robot5 ) 
anim.attachRobot( robot6 ) 

% Set the initial joint position and update
q_init = [-0.80167,  0.97555,  0.70568, -1.60438, -0.46002, -0.51510, -0.25311]';
robot1.updateKinematics( q_init );
robot2.updateKinematics( q_init );
robot3.updateKinematics( q_init );
robot4.updateKinematics( q_init );
robot5.updateKinematics( q_init );
robot6.updateKinematics( q_init );

robot_arr = { robot1, robot2, robot3, robot4, robot5, robot6};

% Get one of the initial positoin
p_init_arr = zeros( 3, 6 );
for i = 1: 6
    H_init = robot_arr{ i }.H_base*robot_arr{ i }.getForwardKinematics( q_init );
    p_init_arr( :, i ) = H_init( 1:3, 4 );
end

R_init = H_init( 1:3, 1:3 );
anim.update( 0 );

% Title: simulation time
titleString = sprintf( 'Time: %2.1f sec', 0 );
mytitle = title( titleString );
set( mytitle, 'FontSize' , 15);
set( anim.hAxes, 'view', [90 0], 'xticklabel', {}, 'yticklabel', {}, 'zticklabel', {} )

%% -- (1B) Motion Planning of end-effector Position

load('C:\Users\moses\OneDrive\Documents\GitHub\DMP-MATLAB\learned_parameters\discrete\drawM_pos.mat')

% Simply load the MAT file
scl_arr = [0.5, 0.75, 0.9, 1.0, 1.1, 1.2];

 p0_arr = zeros( 3, Nt, 6 );

data.tau = 4.0;
tau = data.tau;

for i = 1 : 6
    scl = scl_arr( i );

    % Getting the number of basis functions from the nonlinear forcing term
    [ ~, N ] = size( data.weight );
    
    % The Three elements of DMP
    cs        = CanonicalSystem( 'discrete', data.tau, data.alpha_s );
    fs        = NonlinearForcingTerm( cs, N );
    trans_sys = TransformationSystem( data.alpha_z, data.beta_z, cs );
    
    % Calculate the nonlinear forcing term 
    % This can be diminishing
    input_arr = fs.calc_forcing_term( t_arr( 1:end-1 ), data.weight, t0i, eye( 3 ), 'trimmed' );
    
    % Rollout, note that initial condition is set to be zeros. 
    [ y_arr, z_arr, dy_arr ] = trans_sys.rollout( zeros( 3, 1 ), zeros( 3, 1 ), scl*data.goal, scl * input_arr, t0i, t_arr  );    
    p0_arr( 1, :, i ) = y_arr( 1,  : );
    p0_arr( 2, :, i ) = y_arr( 2,  : );
    p0_arr( 3, :, i ) = y_arr( 3,  : );

end

marker_handle = cell( 1, 6 );
off = 0.3;
for i = 1 : 6
    plot3( anim.hAxes, off+p_init_arr( 1, i ) + squeeze( p0_arr( 1, :, i ) ), ...
                       p_init_arr( 2, i ) + squeeze( p0_arr( 2, :, i ) ), ...
                       p_init_arr( 3, i ) + squeeze( p0_arr( 3, :, i ) ), 'linewidth', 10, 'linestyle', '-', 'color', [0.6350 0.0780 0.1840] )
    scatter3( anim.hAxes, 0.01+off+p_init_arr( 1, i ), p_init_arr( 2, i ), p_init_arr( 3, i ), 400, 'filled', 'o', 'markerfacecolor', 'w', 'markeredgecolor', [0.6350 0.0780 0.1840], 'linewidth', 5);
    scatter3( anim.hAxes, 0.01+ off+p0_arr( 1, end, i )+p_init_arr( 1, i ), p0_arr( 2, end, i )+p_init_arr( 2, i ), p0_arr( 3, end, i )+p_init_arr( 3, i ), 400, 'filled', 'd', 'markerfacecolor', 'w', 'markeredgecolor', [0.6350 0.0780 0.1840], 'linewidth', 5);

    marker_handle{ i } =  scatter3( anim.hAxes, 0.01+off+p_init_arr( 1, i ), p_init_arr( 2, i ), p_init_arr( 3, i ), 200, 'filled', 'o', 'markerfacecolor', 'w', 'markeredgecolor', [0.6350 0.0780 0.1840], 'linewidth', 3);
end


%% -- (1C) Main Simulation

% Time step for the simulation
ns = 1;

% Task-space impedances for Position
Kp = 500 * eye( 3 );
Bp = 0.1 * Kp;

% Joint-space impedance, damping
Bq = .8 * eye( robot1.nq );

% Task-space impedances for Orientation
Kr = 10.0 * eye( 3 );
Br =  0.1*Kr;

D = tau;

tmp = zeros( robot1.nq, 1 );

 q_arr = { q_init, q_init, q_init, q_init, q_init, q_init };
dq_arr = {    tmp,    tmp,    tmp,    tmp,    tmp,    tmp };
 p_arr_tmp = cell( 1, 7 );

for i = 1 : Nt

    t = t_arr( i );
    
    % The initial end-effector position 
    for j = 1 : 6 

        robot = robot_arr{ j };

        q  =  q_arr{ j };
        dq = dq_arr{ j };
        % Get the mass matrix of the Acrobot
        M = robot.getMassMatrix( q );
        M( 7, 7 ) = 10 * M( 7, 7 );
        C = robot.getCoriolisMatrix( q, dq );   
    
        % Get the Hybrid Jacobian 
        JH = robot.getHybridJacobian( q );
        
        % Get the end-effector position and velocity 
        dp = JH( 1:3, : ) * dq;
                
        H = robot.getForwardKinematics( q );
        H = robot.H_base * H;
        R = H( 1:3, 1:3 );
        p = H( 1:3,   4 );

        tau1 = JH( 1:3, : )' * ( Kp * ( p_init_arr( :, j ) +  p0_arr( :, i, j ) - p ) + Bp * ( - dp ) );
        tau2 = JH( 4:6, : )' * Kr * R * so3_to_R3( logm( R' * R_init ) ) - JH( 4:6, : )' * Br * JH( 4:6, : ) * dq ;
        tau3 = - Bq * dq;
        tau  = tau1 + tau2 + tau3;
    
        rhs = M\( -C*dq + tau ); 

    
        [ q1, dq1 ] = func_symplecticEuler( q, dq, rhs, dt );
        q_arr{ j }  =  q1;
        dq_arr{ j } = dq1;

        p_arr_tmp{ j } = p;
    end

    if round( t / anim.FrameUpdateTime ) >= ns
        % Update the linkage plot
        for j = 1 : 6
            robot_arr{ j }.updateKinematics( q_arr{ j } );
        end
        anim.update( t );    
        ns = ns + 1;

        % Set animation title
        set( mytitle, 'String', sprintf( 'Time: %2.1f sec', t ) );  
        for j = 1 : 6
            set( marker_handle{ j }, 'XData', 0.01+off+p_arr_tmp{ j }( 1 ), 'YData', p_arr_tmp{ j }( 2 ), 'ZData', p_arr_tmp{ j }( 3 ))
        end
    end

                                                                 
end

anim.close( )

%% (2-) Drawing M, Rotational Invariance
%%  -- (2A) Parameters for Simulation

clear; close all;clc;
% Simulation settings
T   =  5.0;            % Total simulation time
t   =    0;            % The current time of simulation   
t0i =  0.3;            % Initial time start of movement
dt  = 1e-3;            % Time-step of simulation 
t_arr  = 0:dt:T;       % Time array
Nt  = length( t_arr ); % Number of time-array elements

% Create animation and Attach robot
anim = Animation( 'Dimension', 3, 'xLim', [-0.2,1.6], 'yLim', [-1.0,0.8], 'zLim', [0, 1.0], 'isSaveVideo', true, 'Title', 'sim2' );
anim.init( );

% Define robot, initialization
func_Hbase = @( x, y ) [ rotz( x ), rotz( x ) * [ 0; y; 0 ]; zeros( 1, 3 ), 1];

c = 0.3;
robot1 = iiwa14( 'high' ); robot1.init( ); robot1.H_base = func_Hbase( 0, 0 );
robot2 = iiwa14( 'high' ); robot2.init( ); robot2.H_base = func_Hbase( 0, 0 );
robot3 = iiwa14( 'high' ); robot3.init( ); robot3.H_base = func_Hbase( 0, 0 );
robot4 = iiwa14( 'high' ); robot4.init( ); robot4.H_base = func_Hbase( 0, 0 );
robot5 = iiwa14( 'high' ); robot5.init( ); robot5.H_base = func_Hbase( 0, 0 );
robot6 = iiwa14( 'high' ); robot6.init( ); robot6.H_base = func_Hbase( 0, 0 );

anim.attachRobot( robot1 ) 
anim.attachRobot( robot2 ) 
anim.attachRobot( robot3 ) 
anim.attachRobot( robot4 ) 
anim.attachRobot( robot5 ) 
anim.attachRobot( robot6 ) 

% Set the initial joint position and update
q_init = [-0.80167,  0.77555,  0.70568, -1.60438, -0.46002, -0.51510, -0.25311]';
robot1.updateKinematics( q_init );
robot2.updateKinematics( q_init );
robot3.updateKinematics( q_init );
robot4.updateKinematics( q_init );
robot5.updateKinematics( q_init );
robot6.updateKinematics( q_init );

robot_arr = { robot1, robot2, robot3, robot4, robot5, robot6};

% Get one of the initial positoin
p_init_arr = zeros( 3, 6 );
for i = 1: 6
    H_init = robot_arr{ i }.H_base*robot_arr{ i }.getForwardKinematics( q_init );
    p_init_arr( :, i ) = H_init( 1:3, 4 );
end

R_init = H_init( 1:3, 1:3 );
anim.update( 0 );

% Title: simulation time
titleString = sprintf( 'Time: %2.1f sec', 0 );
mytitle = title( titleString );
set( mytitle, 'FontSize' , 15);
set( anim.hAxes, 'view', [90 0], 'xticklabel', {}, 'yticklabel', {}, 'zticklabel', {} )

%% -- (2B) Motion Planning of end-effector Position

load('C:\Users\moses\OneDrive\Documents\GitHub\DMP-MATLAB\learned_parameters\discrete\drawM_pos.mat')

% Simply load the MAT file
p0_arr = zeros( 3, Nt, 6 );

data.tau = 4.0;
tau = data.tau;

for i = 1 : 6
    scl = 0.7;

    % Getting the number of basis functions from the nonlinear forcing term
    [ ~, N ] = size( data.weight );
 
    tmpR = rotx( 60*i);

    % The Three elements of DMP
    cs        = CanonicalSystem( 'discrete', data.tau, data.alpha_s );
    fs        = NonlinearForcingTerm( cs, N );
    trans_sys = TransformationSystem( data.alpha_z, data.beta_z, cs );
    
    % Calculate the nonlinear forcing term 
    % This can be diminishing
    input_arr = fs.calc_forcing_term( t_arr( 1:end-1 ), data.weight, t0i, eye( 3 ), 'trimmed' );
    
    % Rollout, note that initial condition is set to be zeros. 
    [ y_arr, z_arr, dy_arr ] = trans_sys.rollout( zeros( 3, 1 ), zeros( 3, 1 ), tmpR*scl*data.goal, tmpR*scl * input_arr, t0i, t_arr  );    
    p0_arr( 1, :, i ) = y_arr( 1,  : );
    p0_arr( 2, :, i ) = y_arr( 2,  : );
    p0_arr( 3, :, i ) = y_arr( 3,  : );

end

marker_handle = cell( 1, 6 );
off = 0.3;
for i = 1 : 6
    plot3( anim.hAxes, off+p_init_arr( 1, i ) + squeeze( p0_arr( 1, :, i ) ), ...
                       p_init_arr( 2, i ) + squeeze( p0_arr( 2, :, i ) ), ...
                       p_init_arr( 3, i ) + squeeze( p0_arr( 3, :, i ) ), 'linewidth', 10, 'linestyle', '-', 'color', [0.6350 0.0780 0.1840] 	)
    scatter3( anim.hAxes, 0.01+off+p_init_arr( 1, i ), p_init_arr( 2, i ), p_init_arr( 3, i ), 400, 'filled', 'o', 'markerfacecolor', 'w', 'markeredgecolor',  [0.6350 0.0780 0.1840] 	, 'linewidth', 5);
    scatter3( anim.hAxes, 0.01+off+p0_arr( 1, end, i )+p_init_arr( 1, i ), p0_arr( 2, end, i )+p_init_arr( 2, i ), p0_arr( 3, end, i )+p_init_arr( 3, i ), 400, 'filled', 'd', 'markerfacecolor', 'w', 'markeredgecolor',  [0.6350 0.0780 0.1840] 	, 'linewidth', 5);

    marker_handle{ i } =  scatter3( anim.hAxes, 0.01+off+p_init_arr( 1, i ), p_init_arr( 2, i ), p_init_arr( 3, i ), 200, 'filled', 'o', 'markerfacecolor', 'w', 'markeredgecolor', [0.6350 0.0780 0.1840], 'linewidth', 3);
end


%% -- (2C) Main Simulation

% Time step for the simulation
ns = 1;

% Task-space impedances for Position
Kp = 500 * eye( 3 );
Bp = 0.1 * Kp;

% Joint-space impedance, damping
Bq = .8 * eye( robot1.nq );

% Task-space impedances for Orientation
Kr = 10.0 * eye( 3 );
Br =  0.1*Kr;

D = tau;

tmp = zeros( robot1.nq, 1 );

 q_arr = { q_init, q_init, q_init, q_init, q_init, q_init };
dq_arr = {    tmp,    tmp,    tmp,    tmp,    tmp,    tmp };
 p_arr_tmp = cell( 1, 7 );

for i = 1 : Nt

    t = t_arr( i );
    
    % The initial end-effector position 
    for j = 1 : 6 

        robot = robot_arr{ j };

        q  =  q_arr{ j };
        dq = dq_arr{ j };
        % Get the mass matrix of the Acrobot
        M = robot.getMassMatrix( q );
        M( 7, 7 ) = 10 * M( 7, 7 );
        C = robot.getCoriolisMatrix( q, dq );   
    
        % Get the Hybrid Jacobian 
        JH = robot.getHybridJacobian( q );
        
        % Get the end-effector position and velocity 
        dp = JH( 1:3, : ) * dq;
                
        H = robot.getForwardKinematics( q );
        H = robot.H_base * H;
        R = H( 1:3, 1:3 );
        p = H( 1:3,   4 );

        tau1 = JH( 1:3, : )' * ( Kp * ( p_init_arr( :, j ) +  p0_arr( :, i, j ) - p ) + Bp * ( - dp ) );
        tau2 = JH( 4:6, : )' * Kr * R * so3_to_R3( logm( R' * R_init ) ) - JH( 4:6, : )' * Br * JH( 4:6, : ) * dq ;
        tau3 = - Bq * dq;
        tau  = tau1 + tau2 + tau3;
    
        rhs = M\( -C*dq + tau ); 

    
        [ q1, dq1 ] = func_symplecticEuler( q, dq, rhs, dt );
        q_arr{ j }  =  q1;
        dq_arr{ j } = dq1;

        p_arr_tmp{ j } = p;
    end

    if round( t / anim.FrameUpdateTime ) >= ns
        % Update the linkage plot
        for j = 1 : 6
            robot_arr{ j }.updateKinematics( q_arr{ j } );
        end
        anim.update( t );    
        ns = ns + 1;

        % Set animation title
        set( mytitle, 'String', sprintf( 'Time: %2.1f sec', t ) );  
        for j = 1 : 6
            set( marker_handle{ j }, 'XData', off+0.01+p_arr_tmp{ j }( 1 ), 'YData', p_arr_tmp{ j }( 2 ), 'ZData', p_arr_tmp{ j }( 3 ))
        end
    end

                                                                 
end

anim.close( )

%% (3-) Drawing M, Rotational Invariance
%%  -- (3A) Parameters for Simulation
close all; clear; clc;

% Load the data
load('C:\Users\moses\OneDrive\Documents\GitHub\DMP-MATLAB\learned_parameters\discrete\drawM_pos.mat')


clear; close all;clc;
% Simulation settings
T   =  8.0;            % Total simulation time
t   =    0;            % The current time of simulation   
t0i =  0.3;            % Initial time start of movement
dt  = 1e-3;            % Time-step of simulation 
t_arr  = 0:dt:T;       % Time array
Nt  = length( t_arr ); % Number of time-array elements

% Create animation and Attach robot
anim = Animation( 'Dimension', 3, 'xLim', [-0.2,1.6], 'yLim', [-1.0,0.8], 'zLim', [0, 1.0], 'isSaveVideo', true, 'Title', 'sim3' );
anim.init( );

% Define robot, initialization
func_Hbase = @( x, y ) [ rotz( x ), rotz( x ) * [ 0; y; 0 ]; zeros( 1, 3 ), 1];

c = 0.3;
robot1 = iiwa14( 'high' ); robot1.init( ); robot1.H_base = func_Hbase( 0, 0 );
robot2 = iiwa14( 'high' ); robot2.init( ); robot2.H_base = func_Hbase( 0, 0 );
robot3 = iiwa14( 'high' ); robot3.init( ); robot3.H_base = func_Hbase( 0, 0 );
robot4 = iiwa14( 'high' ); robot4.init( ); robot4.H_base = func_Hbase( 0, 0 );
robot5 = iiwa14( 'high' ); robot5.init( ); robot5.H_base = func_Hbase( 0, 0 );
robot6 = iiwa14( 'high' ); robot6.init( ); robot6.H_base = func_Hbase( 0, 0 );

anim.attachRobot( robot1 ) 
anim.attachRobot( robot2 ) 
anim.attachRobot( robot3 ) 
anim.attachRobot( robot4 ) 
anim.attachRobot( robot5 ) 
anim.attachRobot( robot6 ) 

% Set the initial joint position and update
q_init = [-0.80167,  0.77555,  0.70568, -1.60438, -0.46002, -0.51510, -0.25311]';
robot1.updateKinematics( q_init );
robot2.updateKinematics( q_init );
robot3.updateKinematics( q_init );
robot4.updateKinematics( q_init );
robot5.updateKinematics( q_init );
robot6.updateKinematics( q_init );

robot_arr = { robot1, robot2, robot3, robot4, robot5, robot6};

% Get one of the initial positoin
p_init_arr = zeros( 3, 6 );
for i = 1: 6
    H_init = robot_arr{ i }.H_base*robot_arr{ i }.getForwardKinematics( q_init );
    p_init_arr( :, i ) = H_init( 1:3, 4 );
end

R_init = H_init( 1:3, 1:3 );
anim.update( 0 );

% Title: simulation time
titleString = sprintf( 'Time: %2.1f sec', 0 );
mytitle = title( titleString );
set( mytitle, 'FontSize' , 15);
set( anim.hAxes, 'view', [90 0], 'xticklabel', {}, 'yticklabel', {}, 'zticklabel', {} )

%% -- (3B) Motion Planning of end-effector Position

load('C:\Users\moses\OneDrive\Documents\GitHub\DMP-MATLAB\learned_parameters\discrete\drawM_pos.mat')

% Simply load the MAT file
p0_arr = zeros( 3, Nt, 6 );

D_arr = [ 2.0, 3.0, 4.0, 4.8, 5.5, 6.5];

for i = 1 : 6
    scl = 0.7;

    % Getting the number of basis functions from the nonlinear forcing term
    [ ~, N ] = size( data.weight );
 
    tmpR = rotx( 60*i);

    % The Three elements of DMP
    cs        = CanonicalSystem( 'discrete', D_arr( i ), data.alpha_s );
    fs        = NonlinearForcingTerm( cs, N );
    trans_sys = TransformationSystem( data.alpha_z, data.beta_z, cs );
    
    % Calculate the nonlinear forcing term 
    % This can be diminishing
    input_arr = fs.calc_forcing_term( t_arr( 1:end-1 ), data.weight, t0i, eye( 3 ), 'trimmed' );
    
    % Rollout, note that initial condition is set to be zeros. 
    [ y_arr, z_arr, dy_arr ] = trans_sys.rollout( zeros( 3, 1 ), zeros( 3, 1 ), scl*data.goal, scl * input_arr, t0i, t_arr  );    
    p0_arr( 1, :, i ) = y_arr( 1,  : );
    p0_arr( 2, :, i ) = y_arr( 2,  : );
    p0_arr( 3, :, i ) = y_arr( 3,  : );

end

marker_handle = cell( 1, 6 );
off = 0.3;
for i = 1 : 6
    plot3( anim.hAxes, off+p_init_arr( 1, i ) + squeeze( p0_arr( 1, :, i ) ), ...
                       p_init_arr( 2, i ) + squeeze( p0_arr( 2, :, i ) ), ...
                       p_init_arr( 3, i ) + squeeze( p0_arr( 3, :, i ) ), 'linewidth', 10, 'linestyle', '-', 'color', [0.6350 0.0780 0.1840] 	)
    scatter3( anim.hAxes, 0.01+off+p_init_arr( 1, i ), p_init_arr( 2, i ), p_init_arr( 3, i ), 400, 'filled', 'o', 'markerfacecolor', 'w', 'markeredgecolor',  [0.6350 0.0780 0.1840] 	, 'linewidth', 5);
    scatter3( anim.hAxes, 0.01+off+p0_arr( 1, end, i )+p_init_arr( 1, i ), p0_arr( 2, end, i )+p_init_arr( 2, i ), p0_arr( 3, end, i )+p_init_arr( 3, i ), 400, 'filled', 'd', 'markerfacecolor', 'w', 'markeredgecolor',  [0.6350 0.0780 0.1840] 	, 'linewidth', 5);

    marker_handle{ i } =  scatter3( anim.hAxes, 0.01+off+p_init_arr( 1, i ), p_init_arr( 2, i ), p_init_arr( 3, i ), 200, 'filled', 'o', 'markerfacecolor', 'w', 'markeredgecolor', [0.6350 0.0780 0.1840], 'linewidth', 3);
end


%% -- (3C) Main Simulation

% Time step for the simulation
ns = 1;

% Task-space impedances for Position
Kp = 500 * eye( 3 );
Bp = 0.1 * Kp;

% Joint-space impedance, damping
Bq = .8 * eye( robot1.nq );

% Task-space impedances for Orientation
Kr = 10.0 * eye( 3 );
Br =  0.1*Kr;

tmp = zeros( robot1.nq, 1 );

 q_arr = { q_init, q_init, q_init, q_init, q_init, q_init };
dq_arr = {    tmp,    tmp,    tmp,    tmp,    tmp,    tmp };
 p_arr_tmp = cell( 1, 7 );

for i = 1 : Nt

    t = t_arr( i );
    
    % The initial end-effector position 
    for j = 1 : 6 

        robot = robot_arr{ j };

        q  =  q_arr{ j };
        dq = dq_arr{ j };
        % Get the mass matrix of the Acrobot
        M = robot.getMassMatrix( q );
        M( 7, 7 ) = 10 * M( 7, 7 );
        C = robot.getCoriolisMatrix( q, dq );   
    
        % Get the Hybrid Jacobian 
        JH = robot.getHybridJacobian( q );
        
        % Get the end-effector position and velocity 
        dp = JH( 1:3, : ) * dq;
                
        H = robot.getForwardKinematics( q );
        H = robot.H_base * H;
        R = H( 1:3, 1:3 );
        p = H( 1:3,   4 );

        tau1 = JH( 1:3, : )' * ( Kp * ( p_init_arr( :, j ) +  p0_arr( :, i, j ) - p ) + Bp * ( - dp ) );
        tau2 = JH( 4:6, : )' * Kr * R * so3_to_R3( logm( R' * R_init ) ) - JH( 4:6, : )' * Br * JH( 4:6, : ) * dq ;
        tau3 = - Bq * dq;
        tau  = tau1 + tau2 + tau3;
    
        rhs = M\( -C*dq + tau ); 

    
        [ q1, dq1 ] = func_symplecticEuler( q, dq, rhs, dt );
        q_arr{ j }  =  q1;
        dq_arr{ j } = dq1;

        p_arr_tmp{ j } = p;
    end

    if round( t / anim.FrameUpdateTime ) >= ns
        % Update the linkage plot
        for j = 1 : 6
            robot_arr{ j }.updateKinematics( q_arr{ j } );
        end
        anim.update( t );    
        ns = ns + 1;

        % Set animation title
        set( mytitle, 'String', sprintf( 'Time: %2.1f sec', t ) );  
        for j = 1 : 6
            set( marker_handle{ j }, 'XData', off+0.01+p_arr_tmp{ j }( 1 ), 'YData', p_arr_tmp{ j }( 2 ), 'ZData', p_arr_tmp{ j }( 3 ))
        end
    end

                                                                 
end

anim.close( )


%% (4-) Lifting up and Down
%%  -- (4A) Parameters for Simulation
close all; clear; clc;

% Load the data
load('C:\Users\moses\OneDrive\Documents\GitHub\DMP-MATLAB\learned_parameters\discrete\lift_up_down_orient.mat', 'data')

% Simulation settings
T   =  4.0;            % Total simulation time
t   =    0;            % The current time of simulation   
t0i =  0.3;            % Initial time start of movement
dt  = 1e-3;            % Time-step of simulation 
t_arr  = 0:dt:T;       % Time array
Nt  = length( t_arr ); % Number of time-array elements

% Create animation and Attach robot
anim = Animation( 'Dimension', 3, 'xLim', [-0.7,0.7], 'yLim', [-0.7,0.7], 'zLim', [0, 1.0], 'isSaveVideo', true, 'Title', 'sim4' );
anim.init( );

% Define robot, initialization
func_Hbase = @( x, y ) [ rotz( x ), rotz( x ) * [ 0; y; 0 ]; zeros( 1, 3 ), 1];

c = 0.3;
robot1 = iiwa14( 'high' ); robot1.init( ); robot1.H_base = func_Hbase( 0, 0 );
robot2 = iiwa14( 'high' ); robot2.init( ); robot2.H_base = func_Hbase( 0, 0 );
robot3 = iiwa14( 'high' ); robot3.init( ); robot3.H_base = func_Hbase( 0, 0 );
robot4 = iiwa14( 'high' ); robot4.init( ); robot4.H_base = func_Hbase( 0, 0 );
robot5 = iiwa14( 'high' ); robot5.init( ); robot5.H_base = func_Hbase( 0, 0 );
robot6 = iiwa14( 'high' ); robot6.init( ); robot6.H_base = func_Hbase( 0, 0 );

anim.attachRobot( robot1 ) 
anim.attachRobot( robot2 ) 
anim.attachRobot( robot3 ) 
anim.attachRobot( robot4 ) 
anim.attachRobot( robot5 ) 
anim.attachRobot( robot6 ) 

% Set the initial joint position and update
q_init = [0.00010,  0.49694,  0.30722, -1.53026, -0.13637,  1.31887, -0.15771]';
robot1.updateKinematics( q_init );
robot2.updateKinematics( q_init );
robot3.updateKinematics( q_init );
robot4.updateKinematics( q_init );
robot5.updateKinematics( q_init );
robot6.updateKinematics( q_init );

robot_arr = { robot1, robot2, robot3, robot4, robot5, robot6};

% Get one of the initial positoin
p_init_arr = zeros( 3, 6 );
for i = 1: 6
    H_init = robot_arr{ i }.H_base*robot_arr{ i }.getForwardKinematics( q_init );
    p_init_arr( :, i ) = H_init( 1:3, 4 );
end

R_init = H_init( 1:3, 1:3 );
anim.update( 0 );

% Title: simulation time
titleString = sprintf( 'Time: %2.1f sec', 0 );
mytitle = title( titleString );
set( mytitle, 'FontSize' , 15);
set( anim.hAxes, 'view', [90 0], 'xticklabel', {}, 'yticklabel', {}, 'zticklabel', {} )

%% -- (4B) Motion Planning of end-effector Position

% Simply load the MAT file
p0_arr = zeros( 3, Nt, 6 );

% Scaling Matrix
Sr = diag( get_quat_error( data.init, data.goal) );
scl_arr = [ 0.5, 0.7, 1.0, 1.6, 2.0, 2.5];

eq_whole_arr = cell( 1, 6 );

for i = 1 : 6
    scl = scl_arr( i );

    % Getting the number of basis functions from the nonlinear forcing term
    [ ~, N ] = size( data.weight );
 
    tmpR = rotx( 60*i);

    % The Three elements of DMP
    cs        = CanonicalSystem( 'discrete', data.tau, data.alpha_s );
    fs        = NonlinearForcingTerm( cs, N );
    trans_sys = TransformationSystemQuat( data.alpha_z, data.beta_z, cs );
    
    % Calculate the nonlinear forcing term 
    % This can be diminishing
    % Calculate the nonlinear forcing term
    input_arr = fs.calc_forcing_term( t_arr( 1:end-1 ), data.weight, t0i, Sr  );

    [ eq_arr, z_arr, deq_arr ] = trans_sys.rollout( scl*get_quat_error( data.init, data.goal), scl*data.z0, scl*1*input_arr, t0i, t_arr  );
   
    eq_whole_arr{ i } = eq_arr;
end

quat_arr_gen = zeros( 4, Nt );
R_arr_gen    = zeros( 3, 3, Nt, 6 );
quat_g = data.goal;

for j = 1 : 6
    eq_arr = eq_whole_arr{ j };

    for i = 1 : Nt
   
        tmp = R3_to_quat( eq_arr( :,i ) );
        quat_arr_gen( :, i ) = quat_mul( quat_g', quat_conj( ExpQuat( 0.5 * tmp ) ) );
        ttmp = quat_arr_gen( :, i );
        R_arr_gen( :, :, i, j ) = quat2rotm( quaternion( ttmp( 1 ), ttmp( 2 ), ttmp( 3), ttmp( 4 ) ) );
    end
end


%% -- (4C) Main Simulation

% Time step for the simulation
ns = 1;

% Task-space impedances for Position
Kp = 500 * eye( 3 );
Bp = 0.1 * Kp;

% Joint-space impedance, damping
Bq = .8 * eye( robot1.nq );

% Task-space impedances for Orientation
Kr = 10.0 * eye( 3 );
Br =  0.1*Kr;

tmp = zeros( robot1.nq, 1 );

 q_arr = { q_init, q_init, q_init, q_init, q_init, q_init };
dq_arr = {    tmp,    tmp,    tmp,    tmp,    tmp,    tmp };
 p_arr_tmp = cell( 1, 7 );

for i = 1 : Nt

    t = t_arr( i );
    
    % The initial end-effector position 
    for j = 1 : 6 

        robot = robot_arr{ j };

        q  =  q_arr{ j };
        dq = dq_arr{ j };
        % Get the mass matrix of the Acrobot
        M = robot.getMassMatrix( q );
        M( 7, 7 ) = 10 * M( 7, 7 );
        C = robot.getCoriolisMatrix( q, dq );   
    
        % Get the Hybrid Jacobian 
        JH = robot.getHybridJacobian( q );
        
        % Get the end-effector position and velocity 
        dp = JH( 1:3, : ) * dq;
                
        H = robot.getForwardKinematics( q );
        H = robot.H_base * H;
        R = H( 1:3, 1:3 );
        p = H( 1:3,   4 );

        tau1 = JH( 1:3, : )' * ( Kp * ( p_init_arr( :, j ) - p )  + Bp * ( - dp ) );
        tau2 = JH( 4:6, : )' * Kr * R * so3_to_R3( logm( R' * squeeze( R_arr_gen( :, :, i, j ) ) ) ) - JH( 4:6, : )' * Br * JH( 4:6, : ) * dq ;
        tau3 = - Bq * dq;
        tau  = tau1 + tau2 + tau3;
    
        rhs = M\( -C*dq + tau ); 

    
        [ q1, dq1 ] = func_symplecticEuler( q, dq, rhs, dt );
        q_arr{ j }  =  q1;
        dq_arr{ j } = dq1;

        p_arr_tmp{ j } = p;
    end

    if round( t / anim.FrameUpdateTime ) >= ns
        % Update the linkage plot
        for j = 1 : 6
            robot_arr{ j }.updateKinematics( q_arr{ j } );
        end
        anim.update( t );    
        ns = ns + 1;

    end

                                                                 
end

anim.close( )

%% (5-) Lifting up and Down
%%  -- (5A) Parameters for Simulation
close all; clear; clc;

% Load the data
load('C:\Users\moses\OneDrive\Documents\GitHub\DMP-MATLAB\learned_parameters\discrete\lift_up_down_orient.mat', 'data')

% Simulation settings
T   =  8.0;            % Total simulation time
t   =    0;            % The current time of simulation   
t0i =  0.3;            % Initial time start of movement
dt  = 1e-3;            % Time-step of simulation 
t_arr  = 0:dt:T;       % Time array
Nt  = length( t_arr ); % Number of time-array elements

% Create animation and Attach robot
anim = Animation( 'Dimension', 3, 'xLim', [-0.7,0.7], 'yLim', [-0.7,0.7], 'zLim', [0, 1.0], 'isSaveVideo', true, 'Title', 'sim5' );
anim.init( );

% Define robot, initialization
func_Hbase = @( x, y ) [ rotz( x ), rotz( x ) * [ 0; y; 0 ]; zeros( 1, 3 ), 1];

c = 0.3;
robot1 = iiwa14( 'high' ); robot1.init( ); robot1.H_base = func_Hbase( 0, 0 );
robot2 = iiwa14( 'high' ); robot2.init( ); robot2.H_base = func_Hbase( 0, 0 );
robot3 = iiwa14( 'high' ); robot3.init( ); robot3.H_base = func_Hbase( 0, 0 );
robot4 = iiwa14( 'high' ); robot4.init( ); robot4.H_base = func_Hbase( 0, 0 );
robot5 = iiwa14( 'high' ); robot5.init( ); robot5.H_base = func_Hbase( 0, 0 );
robot6 = iiwa14( 'high' ); robot6.init( ); robot6.H_base = func_Hbase( 0, 0 );

anim.attachRobot( robot1 ) 
anim.attachRobot( robot2 ) 
anim.attachRobot( robot3 ) 
anim.attachRobot( robot4 ) 
anim.attachRobot( robot5 ) 
anim.attachRobot( robot6 ) 

% Set the initial joint position and update
q_init = [0.00010,  0.49694,  0.30722, -1.53026, -0.13637,  1.31887, -0.15771]';
robot1.updateKinematics( q_init );
robot2.updateKinematics( q_init );
robot3.updateKinematics( q_init );
robot4.updateKinematics( q_init );
robot5.updateKinematics( q_init );
robot6.updateKinematics( q_init );

robot_arr = { robot1, robot2, robot3, robot4, robot5, robot6};

% Get one of the initial positoin
p_init_arr = zeros( 3, 6 );
for i = 1: 6
    H_init = robot_arr{ i }.H_base*robot_arr{ i }.getForwardKinematics( q_init );
    p_init_arr( :, i ) = H_init( 1:3, 4 );
end

R_init = H_init( 1:3, 1:3 );
anim.update( 0 );

% Title: simulation time
titleString = sprintf( 'Time: %2.1f sec', 0 );
mytitle = title( titleString );
set( mytitle, 'FontSize' , 15);
set( anim.hAxes, 'view', [90 0], 'xticklabel', {}, 'yticklabel', {}, 'zticklabel', {} )

%% -- (5B) Motion Planning of end-effector Position

% Simply load the MAT file
p0_arr = zeros( 3, Nt, 6 );

% Scaling Matrix
Sr = diag( get_quat_error( data.init, data.goal) );
tau_arr = data.tau*[ 0.5, 0.7, 1.0, 1.2, 1.5, 2.0];

eq_whole_arr = cell( 1, 6 );

for i = 1 : 6
    scl = 1.0;

    % Getting the number of basis functions from the nonlinear forcing term
    [ ~, N ] = size( data.weight );
 
    tmpR = rotx( 60*i);

    % The Three elements of DMP
    cs        = CanonicalSystem( 'discrete', tau_arr( i ), data.alpha_s );
    fs        = NonlinearForcingTerm( cs, N );
    trans_sys = TransformationSystemQuat( data.alpha_z, data.beta_z, cs );
    
    % Calculate the nonlinear forcing term 
    % This can be diminishing
    % Calculate the nonlinear forcing term
    input_arr = fs.calc_forcing_term( t_arr( 1:end-1 ), data.weight, t0i, Sr  );

    [ eq_arr, z_arr, deq_arr ] = trans_sys.rollout( get_quat_error( data.init, data.goal), data.z0, input_arr, t0i, t_arr  );
   
    eq_whole_arr{ i } = eq_arr;
end

quat_arr_gen = zeros( 4, Nt );
R_arr_gen    = zeros( 3, 3, Nt, 6 );
quat_g = data.goal;

for j = 1 : 6
    eq_arr = eq_whole_arr{ j };

    for i = 1 : Nt
        tmp = R3_to_quat( eq_arr( :,i ) );
        quat_arr_gen( :, i ) = quat_mul( quat_g', quat_conj( ExpQuat( 0.5 * tmp ) ) );
        ttmp = quat_arr_gen( :, i );
        R_arr_gen( :, :, i, j ) = quat2rotm( quaternion( ttmp( 1 ), ttmp( 2 ), ttmp( 3), ttmp( 4 ) ) );
    end
end


%% -- (5C) Main Simulation

% Time step for the simulation
ns = 1;

% Task-space impedances for Position
Kp = 500 * eye( 3 );
Bp = 0.1 * Kp;

% Joint-space impedance, damping
Bq = .8 * eye( robot1.nq );

% Task-space impedances for Orientation
Kr = 10.0 * eye( 3 );
Br =  0.1*Kr;

tmp = zeros( robot1.nq, 1 );

 q_arr = { q_init, q_init, q_init, q_init, q_init, q_init };
dq_arr = {    tmp,    tmp,    tmp,    tmp,    tmp,    tmp };
 p_arr_tmp = cell( 1, 7 );

for i = 1 : Nt

    t = t_arr( i );
    
    % The initial end-effector position 
    for j = 1 : 6 

        robot = robot_arr{ j };

        q  =  q_arr{ j };
        dq = dq_arr{ j };
        % Get the mass matrix of the Acrobot
        M = robot.getMassMatrix( q );
        M( 7, 7 ) = 10 * M( 7, 7 );
        C = robot.getCoriolisMatrix( q, dq );   
    
        % Get the Hybrid Jacobian 
        JH = robot.getHybridJacobian( q );
        
        % Get the end-effector position and velocity 
        dp = JH( 1:3, : ) * dq;
                
        H = robot.getForwardKinematics( q );
        H = robot.H_base * H;
        R = H( 1:3, 1:3 );
        p = H( 1:3,   4 );

        tau1 = JH( 1:3, : )' * ( Kp * ( p_init_arr( :, j ) - p )  + Bp * ( - dp ) );
        tau2 = JH( 4:6, : )' * Kr * R * so3_to_R3( logm( R' * squeeze( R_arr_gen( :, :, i, j ) ) ) ) - JH( 4:6, : )' * Br * JH( 4:6, : ) * dq ;
        tau3 = - Bq * dq;
        tau  = tau1 + tau2 + tau3;
    
        rhs = M\( -C*dq + tau ); 

    
        [ q1, dq1 ] = func_symplecticEuler( q, dq, rhs, dt );
        q_arr{ j }  =  q1;
        dq_arr{ j } = dq1;

        p_arr_tmp{ j } = p;
    end

    if round( t / anim.FrameUpdateTime ) >= ns
        % Update the linkage plot
        for j = 1 : 6
            robot_arr{ j }.updateKinematics( q_arr{ j } );
        end
        anim.update( t );    
        ns = ns + 1;

    end

                                                                 
end

anim.close( )

%% (6-) Sequencing, Drawing M and then Lifting up and Down
%%  -- (6A) Parameters for Simulation
close all; clear; clc;

% Load the data
data1 = load('C:\Users\moses\OneDrive\Documents\GitHub\DMP-MATLAB\learned_parameters\discrete\lift_up_down_orient.mat', 'data'); data1 = data1.data;
data2 = load('C:\Users\moses\OneDrive\Documents\GitHub\DMP-MATLAB\learned_parameters\discrete\drawM_pos.mat', 'data'); data2 = data2.data;

% Simulation settings
T   =  8.0;            % Total simulation time
t   =    0;            % The current time of simulation   
t0i =  0.3;            % Initial time start of movement
dt  = 1e-3;            % Time-step of simulation 
t_arr  = 0:dt:T;       % Time array
Nt  = length( t_arr ); % Number of time-array elements
% Define robot, initialization
func_Hbase = @( x, y ) [ rotz( x ), rotz( x ) * [ 0; y; 0 ]; zeros( 1, 3 ), 1];

robot1 = iiwa14( 'high' ); robot1.init( ); robot1.H_base = func_Hbase( 0, 0 );


% Set the initial joint position and update
q_init = [-0.40010,  0.49694,  0.30722, -1.53026, -0.13637,  1.31887, -0.15771]';
robot1.updateKinematics( q_init );


% Get one of the initial positoin
H_init = robot1.H_base*robot1.getForwardKinematics( q_init );
p_init = H_init( 1:3, 4 );
R_init = H_init( 1:3, 1:3 );

% Getting the number of basis functions from the nonlinear forcing term
[ ~, N ] = size( data1.weight );

% The Three elements of DMP
cs        = CanonicalSystem( 'discrete', data1.tau, data1.alpha_s );
fs        = NonlinearForcingTerm( cs, N );
trans_sys = TransformationSystemQuat( data1.alpha_z, data1.beta_z, cs );
Sr = diag( get_quat_error( data1.init, data1.goal) );

% Calculate the nonlinear forcing term 
% This can be diminishing
% Calculate the nonlinear forcing term
input_arr = fs.calc_forcing_term( t_arr( 1:end-1 ), data1.weight, 4.0, Sr  );

[ eq_arr, ~, ~] = trans_sys.rollout( get_quat_error( rotm2quat( R_init )', data1.goal), data1.z0, input_arr, 4.0, t_arr  );

quat_arr_gen = zeros( 4, Nt );
R_arr_gen    = zeros( 3, 3, Nt );
quat_g = data1.goal;

for i = 1 : Nt
    tmp = R3_to_quat( eq_arr( :,i ) );
    quat_arr_gen( :, i ) = quat_mul( quat_g', quat_conj( ExpQuat( 0.5 * tmp ) ) );
    ttmp = quat_arr_gen( :, i );
    R_arr_gen( :, :, i ) = quat2rotm( quaternion( ttmp( 1 ), ttmp( 2 ), ttmp( 3), ttmp( 4 ) ) );
end

% The Three elements of DMP
cs        = CanonicalSystem( 'discrete', data2.tau, data2.alpha_s );
fs        = NonlinearForcingTerm( cs, N );
trans_sys = TransformationSystem( data2.alpha_z, data2.beta_z, cs );

% Calculate the nonlinear forcing term 
% This can be diminishing
input_arr = fs.calc_forcing_term( t_arr( 1:end-1 ), data2.weight, 0.3, eye( 3 ), 'trimmed' );

% Rollout, note that initial condition is set to be zeros. 
[ p_arr, ~, ~] = trans_sys.rollout( zeros( 3, 1 ), zeros( 3, 1 ), data2.goal, input_arr, 0.3, t_arr  );  



% Create animation and Attach robot
anim = Animation( 'Dimension', 3, 'xLim', [-0.7,0.7], 'yLim', [-0.7,0.7], 'zLim', [0, 1.0], 'isSaveVideo', true, 'Title', 'sim6' );
anim.init( );


anim.attachRobot( robot1 ) 


anim.update( 0 );

% Title: simulation time
titleString = sprintf( 'Time: %2.1f sec', 0 );
mytitle = title( titleString );
set( mytitle, 'FontSize' , 15);
set( anim.hAxes, 'view', [90 0], 'xticklabel', {}, 'yticklabel', {}, 'zticklabel', {} )



% Time step for the simulation
ns = 1;

% Task-space impedances for Position
Kp = 500 * eye( 3 );
Bp = 0.1 * Kp;

% Joint-space impedance, damping
Bq = .8 * eye( robot1.nq );

% Task-space impedances for Orientation
Kr = 10.0 * eye( 3 );
Br =  0.1*Kr;
 q = q_init;
dq = zeros( robot1.nq, 1 );

for i = 1 : Nt

    t = t_arr( i );
    

    % Get the mass matrix of the Acrobot
    M = robot1.getMassMatrix( q );
    M( 7, 7 ) = 10 * M( 7, 7 );
    C = robot1.getCoriolisMatrix( q, dq );   

    % Get the Hybrid Jacobian 
    JH = robot1.getHybridJacobian( q );
    
    % Get the end-effector position and velocity 
    dp = JH( 1:3, : ) * dq;
            
    H = robot1.getForwardKinematics( q );
    H = robot1.H_base * H;
    R = H( 1:3, 1:3 );
    p = H( 1:3,   4 );

    tau1 = JH( 1:3, : )' * ( Kp * ( p_init + p_arr( :, i ) - p )  + Bp * ( - dp ) );
    tau2 = JH( 4:6, : )' * Kr * R * so3_to_R3( logm( R' * squeeze( R_arr_gen( :, :, i ) ) ) ) - JH( 4:6, : )' * Br * JH( 4:6, : ) * dq ;
    tau3 = - Bq * dq;
    tau  = tau1 + tau2 + tau3;

    rhs = M\( -C*dq + tau ); 


    [ q1, dq1 ] = func_symplecticEuler( q, dq, rhs, dt );
    q  =  q1;
    dq = dq1;    

    if round( t / anim.FrameUpdateTime ) >= ns
        % Update the linkage plot
        robot1.updateKinematics( q );
        anim.update( t );    
        ns = ns + 1;

    end

                                                                 
end

anim.close( )