% [Title]     Initialization Script for Robot Control Example
% [Author]    Moses Chong-ook Nah
% [Email]     mosesnah@mit.edu
% [Update]    At 2023.11.27

%% (--) Initialization and Adding Paths for Robot Simulation
clear; close all; clc;

% Run the setup script 
run( './Explicit-MATLAB/setup.m' )

% Add path for Impedance Control Examples
addpath( './ImpedanceControl', './utils'  )