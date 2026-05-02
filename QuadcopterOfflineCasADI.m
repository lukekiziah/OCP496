clear; clc;
import casadi.*

%% ================= USER SETTINGS =================
NhorizonFullTrajectory  = 50;
nx = 12;   % number of state variables
nu = 4;   % number of control variables
T  = 5;
dt = T/NhorizonFullTrajectory; time_vector = 0:dt:T;

x0_val = [-1; -1; 4; 0; 0; 0; pi; 0; 0; 0; 0; 0];   % initial state vector
xf_val = [1; 1; 3; 0; 0; 0; 0; 0; 0; 0; 0; 0];   % terminal state vector
enforceTerminalState = true;
onlyPerturbFeedbackControllers = true;
%% ================= SYMBOLICS =================
X = MX.sym('X', nx, NhorizonFullTrajectory+1);
U = MX.sym('U', nu, NhorizonFullTrajectory);

%% ================= QUADCOPTER DYNAMICS =================
%Constants
C11 = .01; C22 = .01; C33 = .05; %translational aero damping consts
m = .5; % drone mass kg
D11 = .04; D22 = .04; D33 = .01; %rotational aero damping consts
g = 9.8; %gravity
Ixx = 4e-3; Iyy = 4e-3; Izz = 7e-3; % [kg*m^2]
L = .127; %from COM to rotor in meters
c1 = 1e-3; c2 = c1; c3 = c1; c4 = c1; %yaw torque coefficents
% Define the steady-state hover
U_ss = [m*g; 0; 0; 0];

x = MX.sym('x', nx);
u = MX.sym('u', nu);

% Precompute trigonometric terms for readability and performance
s7 = sin(x(7)); c7 = cos(x(7));
s8 = sin(x(8)); c8 = cos(x(8)); t8 = tan(x(8)); sec8 = sec(x(8));
s9 = sin(x(9)); c9 = cos(x(9));

% Define translational dynamics f4, f5, f6
f4 = (1/m) * ( u(1) * (s7*s9 + c7*c9*s8) ...
    - x(4) * ( C22*(c7*s9 - c9*s7*s8)*(c7*s9 - c9*s7*s8) + C33*(s7*s9 + c7*c9*s8)*(s7*s9 + c7*c9*s8) + C11*c9*c8*c9*c8 ) ...
    + x(5) * ( C22*(c7*c9 + s7*s9*s8)*(c7*s9 - c9*s7*s8) + C33*(c9*s7 - c7*s9*s8)*(s7*s9 + c7*c9*s8) - C11*c8*s9*c9*c8 ) ...
    + x(6) * ( C11*s8*c9*c8 - C33*c7*c8*(s7*s9 + c7*c9*s8) + C22*c8*s7*(c7*s9 - c9*s7*s8) ) );

f5 = (1/m) * ( -u(1) * (c9*s7 - c7*s9*s8) ...
    + x(4) * ( C22*(c7*s9 - c9*s7*s8)*(c7*c9 + s7*s9*s8) + C33*(s7*s9 + c7*c9*s8)*(c9*s7 - c7*s9*s8) - C11*c9*c8*c8*s9 ) ...
    - x(5) * ( C22*(c7*c9 + s7*s9*s8)*(c7*c9 + s7*s9*s8) + C33*(c9*s7 - c7*s9*s8)*(c9*s7 - c7*s9*s8) + C11*c8*s9*c8*s9 ) ...
    + x(6) * ( C33*c7*c8*(c9*s7 - c7*s9*s8) - C22*c8*s7*(c7*c9 + s7*s9*s8) + C11*s8*c8*s9 ) );

f6 = (1/m) * ( u(1)*c7*c8 - g*m ...
    - x(4) * ( -C11*c9*c8*s8 - C22*c7*s9*c8*s7 + C33*s7*s9*c7*c8 + C33*c7*c9*s8*c7*c8 + C22*c9*s7*s8*c8*s7 ) ...
    - x(5) * ( -C11*c8*s9*s8 + C22*c7*c9*c8*s7 - C33*c9*s7*c7*c8 + C33*c7*s9*s8*c7*c8 + C22*s7*s9*s8*c8*s7 ) ...
    - x(6) * ( C11*s8*s8 + C33*c7*c8*c7*c8 + C22*c8*s7*c8*s7 ) );

% Assemble the state derivative vector
xdot = [
    x(4);
    x(5);
    x(6);
    f4;
    f5;
    f6;
    x(10) + x(11)*s7*t8 + x(12)*c7*t8;
    x(11)*c7 - x(12)*s7;
    x(11)*s7*sec8 + x(12)*c7*sec8;
    (1/Ixx)*(u(2) - D11*x(10) + Iyy*x(11)*x(12));
    (1/Iyy)*(u(3) - D22*x(11) - Ixx*x(10)*x(12) + Izz*x(10)*x(12));
    (1/Izz)*(u(4) - D33*x(12) + Ixx*x(10)*x(11))
];
f = Function('f', {x,u}, {xdot});

%% ================= Objective COST =================
%weighting matrices
Q = 5*diag(ones(1,nx));
Qf = 10*Q;
R = .001*diag(ones(1,nu));

J = 0;
for k = 1:NhorizonFullTrajectory
    stateError = X(:,k) - xf_val; %penalize state error
    controlError = U(:,k) - U_ss; %penalize any control effort
    % ===== EDIT THIS =====
    J = J + (stateError'*Q*stateError + controlError'*R*controlError)*dt;
end
terminalStateError = X(:,NhorizonFullTrajectory+1) - xf_val;
J = J + terminalStateError'*Qf*terminalStateError;

%% ================= CONSTRAINTS =================
T_max = 2.4; %max thrust of a single rotor in Newtons;
T_min = 0; %minimum thrust of a single rotor

%---- ACTUATOR MAPPING CONSTRAINT ----
% enforce u = [F; Mx; My; Mz] limits based on lower subsystem Tvec = [T1; T2; T3; T4] = A_inv*u
% Tvec = A_inv * u must satisfy 0 <= Ti <= T_max for i = 1:4
MotorMappingMatrix = [1 1 1 1;
    0 L 0 -L;
    -L 0 L 0;
    c1 -c2 c3 -c4]; %maps individual rotor thrusts to net force and torques
A_inv = inv(MotorMappingMatrix); %enforce in g constraints

% ------ State Bounds ------
x_min = -inf*ones(nx,1);
x_max = inf*ones(nx,1);
%add ground constraint
x_min(3) = 0.01;
%add wall constraints
x_min(1) = -1.9; x_max(1) = 1.9;
x_min(2) = -1.9; x_max(2) = 1.9;

%Dynamics approximation type
CollocationType = 'HermiteSimpson'; %'Trapezoidal' or 'HermiteSimpson'

% 1. Pre-allocate with physical extremes, not zeros
lbx = -inf * ones(nx*(NhorizonFullTrajectory+1) + nu*NhorizonFullTrajectory, 1);
ubx =  inf * ones(nx*(NhorizonFullTrajectory+1) + nu*NhorizonFullTrajectory, 1);
gConstraints = []; lbg = []; ubg = [];

% Set Bounds for ALL State Nodes (1 to N+1)
for k = 1:NhorizonFullTrajectory+1
    x_idx = (k-1)*nx + (1:nx);
    if k == 1
        % Initial State
        lbx(x_idx) = x0_val;
        ubx(x_idx) = x0_val;
    elseif k == NhorizonFullTrajectory+1
        % Terminal State (Note: k is N+1 here)
        if enforceTerminalState
            lbx(x_idx) = xf_val;
            ubx(x_idx) = xf_val;
        else
            lbx(x_idx) = x_min;
            ubx(x_idx) = x_max;
        end
    else
        % Intermediate States
        lbx(x_idx) = x_min;
        ubx(x_idx) = x_max;
    end
end

% Set Bounds for Controls and Build Dynamics (1 to N)
for k = 1:NhorizonFullTrajectory
    % Control bounds
    u_idx = nx*(NhorizonFullTrajectory+1) + (k-1)*nu + (1:nu);
    %lbx(u_idx) = u_min; Use if we have direct bounds on u
    %ubx(u_idx) = u_max;
    lbx(u_idx) = -inf; ubx(u_idx) = inf;

    %Instead use Motor Mapping to enforce lower subsystem constraint
    uk = U(:,k);
    Tk = A_inv * uk;
    % 0 <= Tk <= Tmax %NO THRUST in OPPOSITE direction
    gConstraints   = [gConstraints; Tk];
    lbg = [lbg; T_min*ones(size(Tk))];
    ubg = [ubg; T_max*ones(size(Tk))];
    
    % Dynamics (Your Collocation Logic)
    xk  = X(:,k);
    xk1 = X(:,k+1);
    %uk  = U(:,k); motor mapping initializes this already
    if strcmp(CollocationType,'Trapezoidal')
        dyn = TrapezoidalCollocation(xk, xk1, uk, f, dt);
    else % Hermite-Simpson
        uk1 = U(:,k); % ZOH assumption
        dyn = HermiteSimpsonCollocation(xk, xk1, uk, uk1, f, dt);
    end
    gConstraints = [gConstraints; dyn];
    lbg = [lbg; zeros(nx,1)];
    ubg = [ubg; zeros(nx,1)];
end

%% ================= DECISION VARIABLES =================
OPT_variables = [reshape(X, nx*(NhorizonFullTrajectory+1),1);
                 reshape(U, nu*NhorizonFullTrajectory,1)];

%% ================= NLP =================
nlp_prob = struct('f', J, 'x', OPT_variables, 'g', gConstraints);

opts = struct;
opts.ipopt.max_iter = 300;
opts.ipopt.print_level = 5;
opts.print_time = 0;

solver = nlpsol('solver', 'ipopt', nlp_prob, opts);

%% ================= INITIAL GUESS =================
initialGuessType = 'linearState 0U'; % 'linearState 0U', 'linearState invUequalibrium', ''linearState invUoperatingpt'
[x_init,U_init] = initialGuess(nx,nu,NhorizonFullTrajectory,x0_val,xf_val,T,xdot,x,u,initialGuessType);

%% ================= SOLVE =================
sol = solver('x0', x_init, ...
             'lbx', lbx, 'ubx', ubx, ...
             'lbg', lbg, 'ubg', ubg);

w_opt = full(sol.x); %Full optimal trajectory from CasADI = z = [allX;allU]

%% ================= EXTRACT =================
X_opt = (reshape(w_opt(1:nx*(NhorizonFullTrajectory+1)), nx, NhorizonFullTrajectory+1))'; % is N+1 by numStateVars
U_opt = (reshape(w_opt(nx*(NhorizonFullTrajectory+1)+1:end), nu, NhorizonFullTrajectory))'; % is N by numControlVars
X_opt_col = w_opt(nx+1:nx*(NhorizonFullTrajectory+1)); %nx*Nx1 contains all x from k+1 to k+N
U_opt_col = w_opt(nx*(NhorizonFullTrajectory+1)+1:end);

%% ================= PLOT OFFLINE COMPUTED REFERENCE =================
plotTrajectory(X_opt,U_opt)
sgtitle('Reference Trajectory')

%% ============== ANIMATION OFFLINE COMPUTED REFERENCE ===================
animateTrajectory(X_opt,'RefTraj')
title('Reference Trajectory')

%% ========== Prepare for Controllers and Simulation =============
IntegratorType = 'RK2'; %choose 'ForwardEuler', 'RK2', or 'RK4'
simExtraSteps = 20;
sim_time = 0:dt:T+simExtraSteps*dt;
SimulateSteps = NhorizonFullTrajectory + simExtraSteps; %simulate beyond reference in receding horizon
shrinkingStepsLeftTransition = 4;

constVals = [m;g;L;Ixx;Iyy;Izz;C11;C22;C33;D11;D22;D33;c1;c2;c3;c4];
numConstants = height(constVals);
[eval_fContinuous,eval_continuousA,eval_continuousB, eval_fDiscrete,eval_discreteAk,eval_discreteBk,FullEOM_sym,constVec,stateVec,uVec] = prepareQuad4Feedback(constVals,IntegratorType,dt);

%% ==== Create Hover Stabilizing LQR for Transition to after horizon ends IF Receding Horizon is NOT used =======
% Get the linear matrices exactly at the terminal state
A_hover = eval_discreteAk(xf_val, U_ss);
B_hover = eval_discreteBk(xf_val, U_ss);

% Calculate the optimal feedback gain matrix K
K_lqr = dlqr(A_hover, B_hover, Q, R);

%% =========== EMPLOY OFFLINE REFERENCE IN OPEN LOOP WITHOUT PERTURBATIONS ==================
X_history = zeros(nx,SimulateSteps+1); % nx(N+1)
U_history = U_opt'; % mxN
X_history(:,1) =  x0_val;
current_X = x0_val;
for i = 1:SimulateSteps
    if i <= NhorizonFullTrajectory
        current_U = U_opt(i,:)'; % mx1
    else
        % --- MODE 2: LQR HOVER HOLD ---
        state_error = current_X - xf_val;
        current_U = U_ss - K_lqr * state_error;
    end
    T_span = [sim_time(i), sim_time(i+1)];
    %simulate without disturbances - Expect nearly equivalent reference with errors only due to collocation approximation
    [~, X_interval] = ode45(@(t, X) eval_fContinuous(X, current_U), T_span, current_X);
    %Update the state for the next loop iteration (grab the last row of output)
    current_X = X_interval(end, :)';
    % Store the state
    X_history(:, i+1) = current_X;
end
plotTrajectory(X_history',U_history')
sgtitle('Open Loop Reference Control | No Perturbations')
animateTrajectory(X_history','OpenLoopRefNoDisturb')
title('Open Loop Reference Control | No Perturbations')

%% =========== COMPUTE Discrete TVLQR Around Reference ===============
A_AllSteps = zeros(nx,nx,NhorizonFullTrajectory);
B_AllSteps = zeros(nx,nu,NhorizonFullTrajectory);
for i = 1:NhorizonFullTrajectory %Linearizations and Discretizations
    % Index Nominal Ref traj x and u
    xRef = X_opt(i,:)';
    uRef = U_opt(i,:)';
    % Linear Continuous Dynamics Around Ref Traj then Discretize
    AkContinuous = eval_continuousA(xRef,uRef);
    BkContinuous = eval_continuousB(xRef,uRef);
    [AkLin2Discrete, BkLin2Discrete] =c2dFREE(AkContinuous,BkContinuous,dt);
    A_AllSteps(:,:,i) = AkLin2Discrete;
    B_AllSteps(:,:,i) = BkLin2Discrete;
end
%Compute gain matrix through horizon
Ktvlqr = DiscreteTimeVaryingLQR(A_AllSteps,B_AllSteps,Q,R,Qf,NhorizonFullTrajectory);

%% ======== SIMULATE Discrete TVLQR Around Reference WITHOUT PERTURBATIONS ==============
%Without Perturbations expect currentX-refX=0 always so U=Uref
X_histTVLQR = zeros(nx,SimulateSteps+1); % nx(N+1)
U_histTVLQR = zeros(nu,SimulateSteps); % mxN
X_histTVLQR(:,1) =  x0_val;
current_X = x0_val;
for i = 1:SimulateSteps
    if i <= NhorizonFullTrajectory
        Know = Ktvlqr(:,:,i);
        current_U = U_opt(i,:)'- Know*(current_X-X_opt(i,:)'); % mx1
    else
        % --- MODE 2: LQR HOVER HOLD ---
        state_error = current_X - xf_val;
        current_U = U_ss - K_lqr * state_error;
    end
    U_histTVLQR(:,i) = current_U;
    T_span = [sim_time(i), sim_time(i+1)];
    %simulate without disturbances - Expect nearly equivalent reference with errors only due to collocation approximation
    [~, X_interval] = ode45(@(t, X) eval_fContinuous(X, current_U), T_span, current_X);
    %Update the state for the next loop iteration (grab the last row of output)
    current_X = X_interval(end, :)';
    % Store the state
    X_histTVLQR(:, i+1) = current_X;
end
plotTrajectory(X_histTVLQR',U_histTVLQR')
sgtitle('TVLQR | No Perturbations')
animateTrajectory(X_histTVLQR','TVLQRNoDisturb')
title('TVLQR | No Perturbations')

%% ======= COMPUTE MULTIPLE SHOOTING LINEAR MPC AROUND REFERENCE ===============
Zguess = w_opt;
%Objective
[Qh, Rh] = bigWeighting(Q,R,Qf,NhorizonFullTrajectory);
Hessian_multiShooting = multipleShootingH(nx,Qh,Rh,Q);
linearObjectivef = -2*[zeros(nx,1);Qh*X_opt_col;Rh*U_opt_col];

%Equality Constraints
%linearizations and first order dynamics correction for current trajectory
[Ad_AllSteps,Bd_AllSteps,allCk] = MPClinearizationsAndCorrections(X_opt',U_opt',eval_discreteAk,eval_discreteBk,eval_fDiscrete);
%compute Equality Constraints based on those linearizations
[Aeq, beq] = multipleShootingDynamicConstraints(Ad_AllSteps, Bd_AllSteps, allCk, x0_val, NhorizonFullTrajectory);
%add terminal state constraint
Aeq(end+1:end+nx,nx*NhorizonFullTrajectory+1:nx*(NhorizonFullTrajectory+1)) = eye(nx); 
beq(end+1:end+nx,1) = xf_val;

%Inequality Constraints
Gstate = zeros(2*(NhorizonFullTrajectory+1)*nx,(NhorizonFullTrajectory+1)*nx+NhorizonFullTrajectory*nu);
hstate = zeros(2*(NhorizonFullTrajectory+1)*nx,1);
Ginput = zeros(2*NhorizonFullTrajectory*nu,(NhorizonFullTrajectory+1)*nx+NhorizonFullTrajectory*nu);
hinput = zeros(2*NhorizonFullTrajectory*nu,1);

for i = 1:NhorizonFullTrajectory+1
    %State bounds
    stateIndx = (i-1)*nx+1:i*nx;
    %maximums
    rowMaxIndx = (2*i-2)*nx+1:(2*i-1)*nx;
    Gstate(rowMaxIndx,stateIndx) = eye(nx);
    hstate(rowMaxIndx,1) = x_max;
    %minimums
    rowMinIndx = (2*i-1)*nx+1:(2*i)*nx;
    Gstate(rowMinIndx,stateIndx) = -eye(nx);
    hstate(rowMinIndx,1) = -x_min;

    if i<=NhorizonFullTrajectory
        %Input bounds
        controlIndx = (NhorizonFullTrajectory+1)*nx+(i-1)*nu+1:(NhorizonFullTrajectory+1)*nx+i*nu;
        %maximums
        rowMaxInputIndx = (2*i-2)*nu+1:(2*i-1)*nu;
        Ginput(rowMaxInputIndx,controlIndx) = A_inv;
        hinput(rowMaxInputIndx,1) = T_max*ones(nu,1);
        %minimums
        rowMinInputIndx = (2*i-1)*nu+1:(2*i)*nu;
        Ginput(rowMinInputIndx,controlIndx) = -A_inv;
        hinput(rowMinInputIndx,1) = T_min*ones(nu,1);
    end
end
G = [Gstate;Ginput]; h= [hstate;hinput];
% Identify all rows where the constraint limit is finite
valid_constraint_rows = ~isinf(h);

% Overwrite G and h to only include the valid, finite constraints
G = G(valid_constraint_rows, :);
h = h(valid_constraint_rows);

% Solve Local QP
Options = optimoptions('quadprog', 'Algorithm','active-set','Display', 'off');
[Zstar, ~, exitFlag, ~, lambda] = quadprog(Hessian_multiShooting, linearObjectivef, G, h, Aeq, beq, [], [], Zguess, Options);

%extract inputs and states from optimized QP
X_QP = (reshape(Zstar(1:nx*(NhorizonFullTrajectory+1)), nx, NhorizonFullTrajectory+1))'; % is N+1 by numStateVars
U_QP = (reshape(Zstar(nx*(NhorizonFullTrajectory+1)+1:end), nu, NhorizonFullTrajectory))'; % is N by numControlVars
X_QP_col = Zstar(nx+1:nx*(NhorizonFullTrajectory+1)); %nx*Nx1 contains all x from k+1 to k+N
U_QP_col = Zstar(nx*(NhorizonFullTrajectory+1)+1:end);

%% ======= SIMULATE Open Loop Full Horizon Multiple Shooting Linear MPC Around Reference WIHTOUT PERTURBATIONS ============
X_historyLinearMPC = zeros(nx,SimulateSteps+1); % nx(N+1)
U_historyLinearMPC = zeros(nu,SimulateSteps); % mxN
X_historyLinearMPC(:,1) =  x0_val;
current_X = x0_val;
for i = 1:SimulateSteps
    if i <= NhorizonFullTrajectory
        current_U = U_QP(i,:)'; % mx1
    else
        % --- MODE 2: LQR HOVER HOLD ---
        state_error = current_X - xf_val;
        current_U = U_ss - K_lqr * state_error;
    end
    T_span = [sim_time(i), sim_time(i+1)];
    %simulate without disturbances - Expect nearly equivalent reference with errors only due to collocation approximation
    [~, X_interval] = ode45(@(t, X) eval_fContinuous(X, current_U), T_span, current_X);
    %Update the state for the next loop iteration (grab the last row of output)
    current_X = X_interval(end, :)';
    % Store the state
    X_historyLinearMPC(:, i+1) = current_X;
    U_historyLinearMPC(:,i) = current_U;
end
plotTrajectory(X_historyLinearMPC',U_historyLinearMPC')
sgtitle('OL Linear MPC | No Perturbations')
animateTrajectory(X_historyLinearMPC','OpenLoopLinearMPCNoDisturb')
title('OL Linear MPC | No Perturbations')

%% ====== SIMULATE Shrinking Horizon Multiple Shooting Linear MPC Around Reference WITHOUT PERTURBATIONS ========
[X_historyShrinkingLinearMPC,U_historyShrinkingLinearMPC] = simulateShrinkingHorizonLinearMPC(Aeq,beq,NhorizonFullTrajectory,nx,nu,x0_val,U_QP,sim_time,eval_fContinuous,Q,R,Qf,X_opt_col,U_opt_col,x_max,x_min,T_max,T_min,A_inv,Zstar,Options,m,g,U_ss,K_lqr,shrinkingStepsLeftTransition,SimulateSteps,xf_val);
plotTrajectory(X_historyShrinkingLinearMPC',U_historyShrinkingLinearMPC')
sgtitle('Shrinking Horizon Linear MPC | No Perturbations')
animateTrajectory(X_historyShrinkingLinearMPC','ShrinkingLinearMPCNoDisturb')
title('Shrinking Horizon Linear MPC | No Perturbations')

%% ====== COMPUTE Open Loop NMPC Around Reference ======
% Use the CasADi optimal solution as both the target reference and the initial guess
Zref   = w_opt;
Zguess = w_opt;

% Call the custom tracking SQP function
[X_NMPC, U_NMPC, Z_NMPC_opt] = solveNMPCAroundReference(...
    Zguess, Zref, Hessian_multiShooting, eval_fDiscrete, eval_discreteAk, ...
    eval_discreteBk, x0_val, xf_val, NhorizonFullTrajectory, nx, nu, G, h, Options);

%% ====== SIMULATE Open Loop NMPC Inputs WITHOUT PERTURBATIONS ======
X_historyNMPC = zeros(nx, SimulateSteps+1);
U_historyNMPC = zeros(nu,SimulateSteps); % nu x N
X_historyNMPC(:,1) = x0_val;
current_X = x0_val;

for i = 1:SimulateSteps
    if i <= NhorizonFullTrajectory
        current_U = U_NMPC(i,:)'; 
    else
        % --- MODE 2: LQR HOVER HOLD ---
        state_error = current_X - xf_val;
        current_U = U_ss - K_lqr * state_error;
    end
    T_span = [sim_time(i), sim_time(i+1)];
    
    % Simulate forward
    [~, X_interval] = ode45(@(t, X) eval_fContinuous(X, current_U), T_span, current_X);
    current_X = X_interval(end, :)';
    
    % Store
    X_historyNMPC(:, i+1) = current_X;
    U_historyNMPC(:,i) = current_U;
end

% Plot and Animate the NMPC solution
plotTrajectory(X_historyNMPC', U_historyNMPC')
sgtitle('Open Loop NMPC | No Perturbations')
animateTrajectory(X_historyNMPC', 'OpenLoopNMPCNoDisturb')
title('Open Loop NMPC | No Perturbations')

%% ====== SIMULATE Shrinking Horizon RTI NMPC WITHOUT PERTURBATIONS ======
% Using Real-Time Iteration (RTI) to track CasADi reference
Zref_full = w_opt; 

[X_hist_RTI_Shrink, U_hist_RTI_Shrink] = simulateShrinkingRTI_NMPC(...
    NhorizonFullTrajectory, nx, nu, x0_val, xf_val, Zref_full, sim_time, ...
    eval_fContinuous, eval_fDiscrete, eval_discreteAk, eval_discreteBk, ...
    Q, R, Qf, x_max, x_min, T_max, T_min, A_inv, Options, m, g, U_ss, K_lqr, SimulateSteps, shrinkingStepsLeftTransition);

% Plot and Animate
plotTrajectory(X_hist_RTI_Shrink', U_hist_RTI_Shrink')
sgtitle('Shrinking Horizon RTI NMPC | No Perturbations')
animateTrajectory(X_hist_RTI_Shrink', 'ShrinkingRTINMPCNoDisturb')
title('Shrinking Horizon RTI NMPC | No Perturbations')

%% ===== SIMULATE Receding Horizon RTI NMPC WITHOUT PERTURBATIONS =======
Nshorthorizon = NhorizonFullTrajectory;
Zguess_Receding = [w_opt(1 : nx*(Nshorthorizon+1)); w_opt(nx*(NhorizonFullTrajectory+1)+1 : nx*(NhorizonFullTrajectory+1) + nu*Nshorthorizon)];
X_historyRecedingRTI_NOperturb = zeros(nx, SimulateSteps+1);
U_historyRecedingRTI_NOperturb = zeros(nu, SimulateSteps);
current_X_RecedingRTI_NOperturb = x0_val;
X_historyRecedingRTI_NOperturb(:,1) = x0_val;
for i = 1:SimulateSteps
    [current_U, Zguess_Receding] = stepRecedingRTI_NMPC(i, Nshorthorizon, nx, nu, ...
        Zguess_Receding, w_opt, current_X_RecedingRTI_NOperturb, xf_val, ...
        eval_fDiscrete, eval_discreteAk, eval_discreteBk, ...
        Q, R, Qf, x_max, x_min, T_max, T_min, A_inv, Options, m, g);
        
    U_historyRecedingRTI_NOperturb(:, i) = current_U;
    
    T_span = [sim_time(i), sim_time(i+1)]; % Ensure sim_time vector is long enough!
    [~, X_interval] = ode45(@(t, X) eval_fContinuous(X, current_U), T_span, current_X_RecedingRTI_NOperturb);
    current_X_RecedingRTI_NOperturb = X_interval(end, :)';
    X_historyRecedingRTI_NOperturb(:, i+1) = current_X_RecedingRTI_NOperturb;
end

% Plot and Animate
plotTrajectory(X_historyRecedingRTI_NOperturb', U_historyRecedingRTI_NOperturb')
sgtitle('Receding Horizon RTI NMPC | No Perturbations')
animateTrajectory(X_historyRecedingRTI_NOperturb', 'RecedingRTINMPCNoDisturb')
title('Receding Horizon RTI NMPC | No Perturbations')

%% ============ Monte Carlo Model Paramters and Input/Process Perturbations ==================
rng(0); %repeatability
MonteCarloLength = 250; 

% Define uncertainties: 10% for aero terms
sd_vec = [0.05; .01; .01; 0.05*ones(3,1); 0.10*ones(6,1); 0.05*ones(4,1)];
%this is a bit hardcoded but ehh
u_Max = zeros(nu,1);
u_Max(1) = 4*T_max; %4 rotors at max thrust
u_Max(2) = 2*L*T_max; u_Max(3) = 2*L*T_max; u_Max(4) = 2*c1*T_max; %these are just used for input disturbance

%initialize perturbation set
input_noise = zeros(nu,NhorizonFullTrajectory,MonteCarloLength);
true_constValsSet = zeros(numConstants,MonteCarloLength);

%initial Control and State Histories
X_historyOLref_perturb = zeros(nx,SimulateSteps+1,MonteCarloLength); % nx(N+1)xMonteCarloLength
U_historyOLref_perturb = zeros(nu,SimulateSteps,MonteCarloLength); % mxN
X_historyTVLQR_perturb = zeros(nx,SimulateSteps+1,MonteCarloLength); % nx(N+1)xMonteCarloLength
U_historyTVLQR_perturb = zeros(nu,SimulateSteps,MonteCarloLength); % mxNxMonteCarloLength
X_historyOLLinearMPC_perturb = zeros(nx,SimulateSteps+1,MonteCarloLength); % nx(N+1)xMonteCarloLength
U_historyOLLinearMPC_perturb = zeros(nu,SimulateSteps,MonteCarloLength); % mxNxMonteCarloLength
X_historyShrinkingLinearMPC_perturb = X_historyOLLinearMPC_perturb;
U_historyShrinkingLinearMPC_perturb = U_historyOLLinearMPC_perturb;
X_historyShrinkingRTI_perturb = zeros(nx, SimulateSteps+1, MonteCarloLength);
U_historyShrinkingRTI_perturb = zeros(nu, SimulateSteps, MonteCarloLength);
X_history_OL_NMPC_perturb = X_historyShrinkingRTI_perturb;
U_history_OL_NMPC_perturb = U_historyShrinkingRTI_perturb;
X_historyRecedingRTI_perturb = zeros(nx, SimulateSteps+1, MonteCarloLength);
U_historyRecedingRTI_perturb = zeros(nu, SimulateSteps, MonteCarloLength);

for MonteSimNum = 1:MonteCarloLength 
    U_QPshrink = U_QP;
    Zshrinkguess = Zstar;
    % Randomized Perturbed Parameter set
    true_constVals = generatePerturbedParams(constVals, sd_vec);
    true_constValsSet(:,MonteSimNum) = true_constVals;

    % Generate a specific 'True' dynamics function for this run
    FullEOM_sym = subs(FullEOM_sym,constVec,true_constVals);
    eval_fContinuousParameterized = matlabFunction(FullEOM_sym,'Vars', {stateVec, uVec});

    %Initialize State
    current_X_OLref = x0_val; current_X_TVLQR = x0_val; current_X_OLLinearMPC = x0_val; current_X_ShrinkingLinearMPC = x0_val;
    X_historyOLref_perturb(:,1,MonteSimNum) =  x0_val;
    X_historyTVLQR_perturb(:,1,MonteSimNum) = x0_val;
    X_historyOLLinearMPC_perturb(:,1,MonteSimNum) = x0_val;
    X_historyShrinkingLinearMPC_perturb(:,1,MonteSimNum) = x0_val;
    current_X_ShrinkingRTI = x0_val;
    X_historyShrinkingRTI_perturb(:,1,MonteSimNum) = x0_val;
    current_X_OL_NMPC = x0_val;
    X_history_OL_NMPC_perturb(:,1,MonteSimNum) = x0_val;
    current_X_RecedingRTI = x0_val;
    X_historyRecedingRTI_perturb(:,1,MonteSimNum) = x0_val;

    % Initialize full guess for RTI at the start of each simulation
    Zguess_RTI = w_opt;
    % Initial guess is the first N steps of the CasADi solution
    Zguess_Receding = [w_opt(1 : nx*(Nshorthorizon+1)); w_opt(nx*(NhorizonFullTrajectory+1)+1 : nx*(NhorizonFullTrajectory+1) + nu*Nshorthorizon)];

    % --- Simulation Loop ---
    for i = 1:SimulateSteps
        %Add actuator noise (e.g., 5% of max thrust and torques)
        currentNoise = 0.05 * u_Max .* randn(4, 1);
        input_noise(:,i,MonteSimNum) = currentNoise;

        %Simulate on Controllers
        T_span = [sim_time(i), sim_time(i+1)];

        if ~onlyPerturbFeedbackControllers
            %OFFLINE REFERENCE IN OPEN LOOP
            if i<=NhorizonFullTrajectory
                current_U = U_opt(i,:)'; % mx1
            else
                % --- MODE 2: LQR HOVER HOLD ---
                state_error = current_X_OLref - xf_val;
                current_U = U_ss - K_lqr * state_error;
            end
            %Simulate forward using the TRUE model and NOISY inputs
            [~, X_interval] = ode45(@(t, X) eval_fContinuousParameterized(X, current_U+currentNoise) + continuousWindDisturbance(t, MonteSimNum), T_span, current_X_OLref);
            %Update the state for the next loop iteration (grab the last row of output)
            current_X_OLref = X_interval(end, :)';
            % Store the state
            X_historyOLref_perturb(:, i+1,MonteSimNum) = current_X_OLref;
            U_historyOLref_perturb(:,i,MonteSimNum) = current_U;
        
            % OL Linear MPC Around Reference
            if i <= NhorizonFullTrajectory
                current_U = U_QP(i,:)'; % mx1
            else
                % --- MODE 2: LQR HOVER HOLD ---
                state_error = current_X_OLLinearMPC - xf_val;
                current_U = U_ss - K_lqr * state_error;
            end
            %Simulate forward using the TRUE model and NOISY inputs
            [~, X_interval] = ode45(@(t, X) eval_fContinuousParameterized(X, current_U+currentNoise)+ continuousWindDisturbance(t, MonteSimNum), T_span, current_X_OLLinearMPC);
            %Update the state for the next loop iteration (grab the last row of output)
            current_X_OLLinearMPC = X_interval(end, :)';
            % Store the state
            X_historyOLLinearMPC_perturb(:, i+1,MonteSimNum) = current_X_OLLinearMPC;
            U_historyOLLinearMPC_perturb(:,i,MonteSimNum) = current_U;

            % OL NMPC WITH PERTURBATIONS
            if i <= NhorizonFullTrajectory
                current_U = U_NMPC(i,:)';
            else
                % --- MODE 2: LQR HOVER HOLD ---
                state_error = current_X_OL_NMPC - xf_val;
                current_U = U_ss - K_lqr * state_error;
            end
            [~,X_interval] = ode45(@(t,X) eval_fContinuousParameterized(X,current_U + currentNoise)+ continuousWindDisturbance(t, MonteSimNum), T_span, current_X_OL_NMPC);
            %Update the state for the next loop iteration
            current_X_OL_NMPC = X_interval(end, :)';
            X_history_OL_NMPC_perturb(:, i+1, MonteSimNum) = current_X_OL_NMPC;
            U_history_OL_NMPC_perturb(:,i,MonteSimNum) = current_U;
        end

        %Discrete TVLQR Around Reference
        if i <= NhorizonFullTrajectory
            Know = Ktvlqr(:,:,i);
            current_U = U_opt(i,:)'- Know*(current_X_TVLQR-X_opt(i,:)'); % mx1
        else
            % --- MODE 2: LQR HOVER HOLD ---
            state_error = current_X_TVLQR - xf_val;
            current_U = U_ss - K_lqr * state_error;
        end
        U_histTVLQR(:,i) = current_U; %same control commands history
        %Simulate forward using the TRUE model and NOISY inputs
        [~, X_interval] = ode45(@(t, X) eval_fContinuousParameterized(X, current_U+currentNoise)+ continuousWindDisturbance(t, MonteSimNum), T_span, current_X_TVLQR);
        %Update the state for the next loop iteration (grab the last row of output)
        current_X_TVLQR = X_interval(end, :)';
        % Store the state
        X_historyTVLQR_perturb(:, i+1,MonteSimNum) = current_X_TVLQR;
        U_historyTVLQR_perturb(:,i,MonteSimNum) = current_U;
   
        % Shrinking Horizon Linear MPC Around Reference
        if i < NhorizonFullTrajectory - shrinkingStepsLeftTransition
            current_U = U_QPshrink(1,:)'; % mx1
        else
            % --- MODE 2: LQR HOVER HOLD ---
            state_error = current_X_ShrinkingLinearMPC - xf_val;
            current_U = U_ss - K_lqr * state_error;
        end
        %Simulate forward using the TRUE model and NOISY inputs
        [~, X_interval] = ode45(@(t, X) eval_fContinuousParameterized(X, current_U+currentNoise)+ continuousWindDisturbance(t, MonteSimNum), T_span, current_X_ShrinkingLinearMPC);
        %Update the state for the next loop iteration (grab the last row of output)
        current_X_ShrinkingLinearMPC = X_interval(end, :)';
        % Store the state
        X_historyShrinkingLinearMPC_perturb(:, i+1,MonteSimNum) = current_X_ShrinkingLinearMPC;
        U_historyShrinkingLinearMPC_perturb(:,i,MonteSimNum) = current_U;
        %solve new QP
        if i < NhorizonFullTrajectory -shrinkingStepsLeftTransition
            % Setup new QP
            [Hessian_shrinks,linearObjectiveShrinks,AeqShrink,beqShrink,Gshrink,hShrink,Zshrinkguess,N_shrink] = setupShrinkingMPCstep(i,NhorizonFullTrajectory,Q,R,Qf,nx,nu,X_opt_col,U_opt_col,Aeq,beq,x_max,x_min,T_max,T_min,A_inv,Zshrinkguess,current_X_ShrinkingLinearMPC);
            %solve
            [Zshrinkguess, ~, exitFlag, ~, lambda] = quadprog(Hessian_shrinks, linearObjectiveShrinks, Gshrink, hShrink, AeqShrink, beqShrink, [], [], Zshrinkguess, Options);
            %extract inputs and states from optimized QP
            U_QPshrink = (reshape(Zshrinkguess(nx*(N_shrink+1)+1:end), nu, N_shrink))'; % is Nshrink by numControlVars
        end

        % Shrinking Horizon RTI NMPC Around Reference WITH PERTURBATIONS
        %Compute control via RTI and update guess for next iteration
        if i < NhorizonFullTrajectory - shrinkingStepsLeftTransition
            [current_U, Zguess_RTI] = stepShrinkingRTI_NMPC(i, NhorizonFullTrajectory, nx, nu, ...
                Zguess_RTI, w_opt, current_X_ShrinkingRTI, xf_val, ...
                eval_fDiscrete, eval_discreteAk, eval_discreteBk, ...
                Q, R, Qf, x_max, x_min, T_max, T_min, A_inv, Options);
        else
            % --- MODE 2: LQR HOVER HOLD ---
            state_error = current_X_ShrinkingRTI - xf_val;
            current_U = U_ss - K_lqr * state_error;
        end
        % Store Control
        U_historyShrinkingRTI_perturb(:, i, MonteSimNum) = current_U;
        %Simulate forward using the TRUE model and NOISY inputs
        [~, X_interval] = ode45(@(t, X) eval_fContinuousParameterized(X, current_U + currentNoise)+ continuousWindDisturbance(t, MonteSimNum), T_span, current_X_ShrinkingRTI);
        %Update the state for the next loop iteration
        current_X_ShrinkingRTI = X_interval(end, :)';
        X_historyShrinkingRTI_perturb(:, i+1, MonteSimNum) = current_X_ShrinkingRTI;
        
        %Receding Horizon RTI NMPC With Perturbations
        [current_U, Zguess_Receding] = stepRecedingRTI_NMPC(i, Nshorthorizon, nx, nu, ...
            Zguess_Receding, w_opt, current_X_RecedingRTI, xf_val, ...
            eval_fDiscrete, eval_discreteAk, eval_discreteBk, ...
            Q, R, Qf, x_max, x_min, T_max, T_min, A_inv, Options, m, g);
        U_historyRecedingRTI_perturb(:, i, MonteSimNum) = current_U;
        T_span = [sim_time(i), sim_time(i+1)]; % Ensure sim_time vector is long enough!
        [~, X_interval] = ode45(@(t, X) eval_fContinuousParameterized(X, current_U + currentNoise)+ continuousWindDisturbance(t, MonteSimNum), T_span, current_X_RecedingRTI);
        current_X_RecedingRTI = X_interval(end, :)';
        X_historyRecedingRTI_perturb(:, i+1, MonteSimNum) = current_X_RecedingRTI;
    end

    %Plots Animations and Bookeeping
    if MonteSimNum == 1
        if ~onlyPerturbFeedbackControllers
            % Open Loop Reference Control | With Perturbations
            titleOLrefSim = sprintf('Open Loop Reference Control | With Perturbation %s',num2str(MonteSimNum));
            animationOLrefNum = sprintf('OpenLoopRefWithPerturbation%s',num2str(MonteSimNum));
            plotTrajectory(X_historyOLref_perturb(:,:,MonteSimNum)',U_historyOLref_perturb(:,:,MonteSimNum)')
            sgtitle(titleOLrefSim)
            animateTrajectory(X_historyOLref_perturb(:,:,MonteSimNum)',animationOLrefNum)
            title(titleOLrefSim)

             % OL Linear MPC Around Reference With Perturbations
            titleLinearMPCSim = sprintf('OL Linear MPC | With Perturbation %s',num2str(MonteSimNum));
            animationLinearMPCNum = sprintf('OLLinearMPCWithPerturbation%s',num2str(MonteSimNum));
            plotTrajectory(X_historyOLLinearMPC_perturb(:,:,MonteSimNum)',U_historyOLLinearMPC_perturb(:,:,MonteSimNum)');
            sgtitle(titleLinearMPCSim)
            animateTrajectory(X_historyOLLinearMPC_perturb(:,:,MonteSimNum)',animationLinearMPCNum);
            title(titleLinearMPCSim)

            % OL NMPC with Perturbations
            title_OL_NMPC = sprintf('OL NNMPC | With Perturbation %s',num2str(MonteSimNum));
            animation_OL_NMPC_num = sprintf('OL_NMPCWithPerturbation%s',num2str(MonteSimNum));
            plotTrajectory(X_history_OL_NMPC_perturb(:,:,MonteSimNum)',U_history_OL_NMPC_perturb(:,:,MonteSimNum)');
            sgtitle(title_OL_NMPC)
            animateTrajectory(X_history_OL_NMPC_perturb(:,:,MonteSimNum)',animation_OL_NMPC_num);
            title(title_OL_NMPC)
        end

        %Discrete TVLQR Around Reference With Perturbations
        titleTVLQRSim = sprintf('TVLQR | With Perturbation %s',num2str(MonteSimNum));
        animationTVLQRNum = sprintf('TVLQRWithPerturbation%s',num2str(MonteSimNum));
        plotTrajectory(X_historyTVLQR_perturb(:,:,MonteSimNum)',U_historyTVLQR_perturb(:,:,MonteSimNum)');
        sgtitle(titleTVLQRSim);
        animateTrajectory(X_historyTVLQR_perturb(:,:,MonteSimNum)',animationTVLQRNum);
        title(titleTVLQRSim);
   
        %Shrinking Horizon Linear MPC with Perturbations
        titleShrinkingLinearMPCSim = sprintf('Shrinking Linear MPC | With Perturbation %s',num2str(MonteSimNum));
        animationShrinkingLinearMPCNum = sprintf('ShrinkingLinearMPCWithPerturbation%s',num2str(MonteSimNum));
        plotTrajectory(X_historyShrinkingLinearMPC_perturb(:,:,MonteSimNum)',U_historyShrinkingLinearMPC_perturb(:,:,MonteSimNum)');
        sgtitle(titleShrinkingLinearMPCSim)
        animateTrajectory(X_historyShrinkingLinearMPC_perturb(:,:,MonteSimNum)',animationShrinkingLinearMPCNum);
        title(titleShrinkingLinearMPCSim)
    
        % Shrinking Horizon RTI NMPC WITH PERTURBATIONS
        titleShrinking_RTI_NMPCSim = sprintf('Shrinking RTI NMPC | With Perturbation %s',num2str(MonteSimNum));
        animationShrinking_RTI_NMPC_Num = sprintf('Shrinking_RTI_NMPCWithPerturbation%s',num2str(MonteSimNum));
        plotTrajectory(X_historyShrinkingRTI_perturb(:,:,MonteSimNum)',U_historyShrinkingRTI_perturb(:,:,MonteSimNum)');
        sgtitle(titleShrinking_RTI_NMPCSim)
        animateTrajectory(X_historyShrinkingRTI_perturb(:,:,MonteSimNum)',animationShrinking_RTI_NMPC_Num);
        title(titleShrinking_RTI_NMPCSim)
    
        %Receding Horizon RTI NMPC With Perturbations
        titleRecedingRTIperturb = sprintf('Receding RTI NMPC | With Perturbation %s',num2str(MonteSimNum));
        animationReceding_RTI_NMPC_Num = sprintf('Receding_RTI_NMPCWithPerturbation%s',num2str(MonteSimNum));
        plotTrajectory(X_historyRecedingRTI_perturb(:,:,MonteSimNum)',U_historyRecedingRTI_perturb(:,:,MonteSimNum)');
        sgtitle(titleRecedingRTIperturb)
        animateTrajectory(X_historyRecedingRTI_perturb(:,:,MonteSimNum)',animationReceding_RTI_NMPC_Num);
        title(titleRecedingRTIperturb) 
    end
end

%% ============ Post-Processing & Statistics ==================
disp('Calculating Statistics...');
ControllerNames = {'TVLQR', 'Shrinking LinMPC', 'Shrinking RTI', 'Receding RTI'};
numControllers = length(ControllerNames);

X_All = {X_historyTVLQR_perturb, X_historyShrinkingLinearMPC_perturb, ...
         X_historyShrinkingRTI_perturb, X_historyRecedingRTI_perturb};
U_All = {U_historyTVLQR_perturb, U_historyShrinkingLinearMPC_perturb, ...
         U_historyShrinkingRTI_perturb, U_historyRecedingRTI_perturb};

Total_Cost_Data = zeros(MonteCarloLength, numControllers);
Effort_Data = zeros(MonteCarloLength, numControllers);
Terminal_Error_Data = zeros(MonteCarloLength, numControllers);

for c = 1:numControllers
    for m = 1:MonteCarloLength
        curr_X = X_All{c}(:, :, m); 
        curr_U = U_All{c}(:, :, m); 
        N_sim = size(curr_U, 2);

        if any(~isfinite(curr_X), 'all') || any(~isfinite(curr_U), 'all')
            Total_Cost_Data(m, c) = NaN; Effort_Data(m, c) = NaN; Terminal_Error_Data(m, c) = NaN;
            continue;
        end

        run_cost = 0;
        run_effort = 0; % Total absolute battery effort
        
        for k = 1:N_sim
            state_err = curr_X(:, k) - xf_val;
            u_err = curr_U(:, k) - U_ss; % LQR explicitly penalizes deviation from hover
            
            run_cost = run_cost + ((state_err' * Q * state_err) + (u_err' * R * u_err))*dt;
            run_effort = run_effort + (curr_U(:, k)' * curr_U(:, k)); % Raw thrust usage
        end
        
        term_err = curr_X(:, end) - xf_val;
        run_cost = run_cost + (term_err' * Qf * term_err);
        
        Total_Cost_Data(m, c) = run_cost;
        Effort_Data(m, c) = run_effort;
        Terminal_Error_Data(m, c) = norm(term_err(1:3)); 
    end
end
disp('Statistics Calculated.');

%% ============ Constraint Violation Analysis ==================
Max_State_Violation = zeros(MonteCarloLength, numControllers);
Max_Input_Violation = zeros(MonteCarloLength, numControllers);

for c = 1:numControllers
    for m = 1:MonteCarloLength
        curr_X = X_All{c}(:, :, m); 
        curr_U = U_All{c}(:, :, m);                   
        
        state_viol_upper = max(0, curr_X - x_max);
        state_viol_lower = max(0, x_min - curr_X);
        Max_State_Violation(m, c) = max(max(state_viol_upper + state_viol_lower));
        
        curr_T = A_inv * curr_U; 
        input_viol_upper = max(0, curr_T - T_max);
        input_viol_lower = max(0, T_min - curr_T);
        Max_Input_Violation(m, c) = max(max(input_viol_upper + input_viol_lower));
    end
end
Safety_Rate = sum(Max_State_Violation < 1e-4, 1) / MonteCarloLength * 100;

%% ============ Professional Aesthetics Setup ==================
% Define a clean, modern color palette (blue, red, green, purple)
c_map = [0.12 0.47 0.71; 0.89 0.10 0.11; 0.17 0.63 0.17; 0.60 0.20 0.80];
set(groot, 'defaultAxesTickLabelInterpreter', 'latex');
set(groot, 'defaultTextInterpreter', 'latex');
set(groot, 'defaultLegendInterpreter', 'latex');

%% Visual 1: Total Quadratic Cost (Modern Boxchart)
figure('Name', 'Total LQR Cost', 'Color', 'w', 'Position', [100, 100, 700, 500]);
hold on;
for c = 1:numControllers
    % Create categories for spacing
    b = boxchart(c * ones(MonteCarloLength, 1), Total_Cost_Data(:, c), ...
        'BoxFaceColor', c_map(c,:), 'MarkerStyle', '.', 'MarkerColor', c_map(c,:), 'BoxWidth', 0.5);
end
title('\textbf{Total Quadratic Cost to Target}', 'FontSize', 14);
ylabel('Cost $\mathbf{J}$ (Lower is Better)', 'FontSize', 12);
xticks(1:numControllers); xticklabels(ControllerNames);
grid on; ax = gca; ax.GridAlpha = 0.25; ax.LineWidth = 1.2;
hold off; drawnow;

%% Visual 2: Pareto Front (Effort vs. Terminal Error)
figure('Name', 'Pareto Front', 'Color', 'w', 'Position', [150, 150, 700, 500]);
hold on;
for c = 1:numControllers
    scatter(Effort_Data(:, c), Terminal_Error_Data(:, c), 40, c_map(c,:), 'filled', ...
        'MarkerEdgeColor', 'w', 'LineWidth', 0.5, 'MarkerFaceAlpha', 0.75);
end
title('\textbf{Total Control Effort vs. Terminal Distance Error}', 'FontSize', 14);
xlabel('Total Absolute Control Effort ($\Sigma \mathbf{u}^T \mathbf{u}$)', 'FontSize', 12);
ylabel('Terminal Distance Error (m)', 'FontSize', 12);
legend(ControllerNames, 'Location', 'best', 'FontSize', 11, 'Box', 'off');
grid on; ax = gca; ax.GridAlpha = 0.25; ax.LineWidth = 1.2;
hold off; drawnow;

%% Visual 3: Distance from Target Over Time
figure('Name', 'Distance Trajectory', 'Color', 'w', 'Position', [200, 200, 750, 500]);
hold on;
time_vec = sim_time(1:size(X_All{1},2));
time_vec = time_vec(:)'; 

for c = 1:numControllers
    dist_to_target = zeros(MonteCarloLength, length(time_vec));
    for m = 1:MonteCarloLength
        current_pos = X_All{c}(1:3, :, m);
        dist_to_target(m, :) = sqrt(sum((current_pos - xf_val(1:3)).^2, 1));
    end
    mean_dist = mean(dist_to_target, 1, 'omitnan');
    std_dist = std(dist_to_target, 0, 1, 'omitnan');
    
    upper_bound = mean_dist + std_dist; lower_bound = max(0, mean_dist - std_dist); 
    valid_idx = isfinite(upper_bound) & isfinite(lower_bound) & isfinite(time_vec);
    
    if any(valid_idx)
        fill([time_vec(valid_idx), fliplr(time_vec(valid_idx))], ...
             [upper_bound(valid_idx), fliplr(lower_bound(valid_idx))], ...
             c_map(c,:), 'FaceAlpha', 0.15, 'EdgeColor', 'none', 'HandleVisibility', 'off');
        plot(time_vec(valid_idx), mean_dist(valid_idx), 'Color', c_map(c,:), ...
             'LineWidth', 2.5, 'DisplayName', ControllerNames{c});
    end
end
title('\textbf{Mean Distance to Target State ($\pm 1\sigma$)}', 'FontSize', 14);
xlabel('Simulation Time (s)', 'FontSize', 12);
ylabel('Distance to Target (m)', 'FontSize', 12);
legend('Location', 'northeast', 'FontSize', 11, 'Box', 'off');
grid on; ax = gca; ax.GridAlpha = 0.25; ax.LineWidth = 1.2;
hold off; drawnow;

%% Visual 4: Controller Safety Rate (Clean Bar Chart)
figure('Name', 'Safety Rate', 'Color', 'w', 'Position', [250, 250, 700, 500]);
b = bar(Safety_Rate, 0.6, 'FaceColor', 'flat', 'EdgeColor', 'none');
for k = 1:numControllers
    b.CData(k,:) = c_map(k,:);
end
title('\textbf{Flights Without State Constraint Violations}', 'FontSize', 14);
ylabel('Success Rate (\%)', 'FontSize', 12);
set(gca, 'xticklabel', ControllerNames);
ylim([0 105]); grid on; ax = gca; ax.GridAlpha = 0.25; ax.LineWidth = 1.2;
for i = 1:numControllers
    text(i, Safety_Rate(i) + 4, sprintf('%.1f\\%%', Safety_Rate(i)), ...
        'HorizontalAlignment', 'center', 'FontSize', 11, 'FontWeight', 'bold', 'Interpreter', 'latex');
end
drawnow;

%% Visual 5: Rotor Thrust Violations
figure('Name', 'Rotor Thrust Violations', 'Color', 'w', 'Position', [300, 300, 700, 500]);
hold on;
for c = 1:numControllers
    boxchart(c * ones(MonteCarloLength, 1), Max_Input_Violation(:, c), ...
        'BoxFaceColor', c_map(c,:), 'MarkerStyle', '.', 'MarkerColor', c_map(c,:), 'BoxWidth', 0.5);
end
title('\textbf{Magnitude of Individual Rotor Thrust Violations}', 'FontSize', 14);
ylabel('Max Thrust Exceedance (N)', 'FontSize', 12);
xticks(1:numControllers); xticklabels(ControllerNames);
yline(0, 'k--', 'LineWidth', 1.5, 'Alpha', 0.6);
grid on; ax = gca; ax.GridAlpha = 0.25; ax.LineWidth = 1.2;
hold off; drawnow;

%% --- Data Sanitization for Visuals (Do not change Total_Cost_Data, use copies) ---
Sanitized_Cost = Total_Cost_Data;
Sanitized_Effort = Effort_Data;
Sanitized_Term_Err = Terminal_Error_Data;
Sanitized_Input_Viol = Max_Input_Violation;

% Define indices of successful (non-NaN) runs for honest legend reporting later
num_failures = sum(isnan(Total_Cost_Data(:,1)));
num_successful = MonteCarloLength - num_failures;

%% Visual 1: Total Quadratic Cost (Modern Boxchart)
figure('Name', 'Total LQR Cost', 'Color', 'w', 'Position', [100, 100, 700, 500]);
hold on;
for c = 1:numControllers
    % boxchart automatically removes NaNs
    boxchart(c * ones(MonteCarloLength, 1), Sanitized_Cost(:, c), ...
        'BoxFaceColor', c_map(c,:), 'MarkerStyle', '.', 'MarkerColor', c_map(c,:), 'BoxWidth', 0.5);
end
title('\textbf{Total Quadratic Cost (Success Population only)}', 'FontSize', 14);
ylabel('Total Cost $\mathbf{J}$', 'FontSize', 12);
xticks(1:numControllers); xticklabels(ControllerNames);

% --- LINEAR SCALE WITH UPDATED LIMITS ---
ylim([300, 475]); % Cleanly frames your 250-700 data range

grid on; ax = gca; ax.GridAlpha = 0.25; ax.LineWidth = 1.2;
hold off; drawnow;

%% Visual 2: Pareto Front (Effort vs. Terminal Error)
figure('Name', 'Pareto Front', 'Color', 'w', 'Position', [150, 150, 700, 500]);
hold on;

for c = 1:numControllers
    % 1. Filter out NaN, Inf, AND physically impossible runaway trajectories
    % (An effort of 10^27 means the controller exploded mathematically)
    valid_idx = isfinite(Sanitized_Term_Err(:, c)) & ...
                isfinite(Sanitized_Effort(:, c)) & ...
                (Sanitized_Effort(:, c) < 1e5) & ... % Hard limit: Excludes mathematical explosions
                (Sanitized_Term_Err(:, c) < 20);     % Hard limit: 20m error is a crashed flight
    
    % 2. Plot only the successful, stable dots
    scatter(Sanitized_Effort(valid_idx, c), Sanitized_Term_Err(valid_idx, c), ...
        40, c_map(c,:), 'filled', 'MarkerEdgeColor', 'w', ...
        'LineWidth', 0.5, 'MarkerFaceAlpha', 0.75);
end

title('\textbf{Control Effort vs. Terminal Distance Error (Stable Runs)}', 'FontSize', 14);
xlabel('Total Absolute Control Effort ($\Sigma \mathbf{u}^T \mathbf{u}$)', 'FontSize', 12);
ylabel('Terminal Distance Error (m)', 'FontSize', 12);
legend(ControllerNames, 'Location', 'best', 'FontSize', 11, 'Box', 'off');

% --- Auto-Scaling for Clean Data ---
axis tight; 
xl = xlim; yl = ylim;

% Add a little padding to the axes safely
if xl(2) > xl(1) && yl(2) > yl(1)
    xlim([max(0, xl(1)*0.95), xl(2)*1.05]); 
    ylim([-0.1, yl(2)*1.1]); % Error can't be less than 0
end

grid on; ax = gca; ax.GridAlpha = 0.25; ax.LineWidth = 1.2;
hold off; drawnow;

%% Visual 3: Distance Trajectory (Sanitized Mean)
figure('Name', 'Distance Trajectory', 'Color', 'w', 'Position', [200, 200, 750, 500]);
hold on;
time_vec = sim_time(1:size(X_All{1},2));
time_vec = time_vec(:)'; 

for c = 1:numControllers
    % 1. Identify which runs actually survived for this controller
    % (A run is successful if its Total Cost is not NaN)
    valid_runs = ~isnan(Sanitized_Cost(:, c)); 
    num_valid = sum(valid_runs);
    
    if num_valid == 0
        continue; % Skip if all runs for this controller crashed
    end
    
    % 2. Extract ONLY the successful trajectories
    dist_to_target = zeros(num_valid, length(time_vec));
    valid_indices = find(valid_runs); % Get the specific Monte Carlo run numbers
    
    for idx = 1:num_valid
        m = valid_indices(idx);
        current_pos = X_All{c}(1:3, :, m);
        dist_to_target(idx, :) = sqrt(sum((current_pos - xf_val(1:3)).^2, 1));
    end
    
    % 3. Calculate mean and std on the strictly clean data
    mean_dist = mean(dist_to_target, 1); 
    std_dist = std(dist_to_target, 0, 1);
    
    upper_bound = mean_dist + std_dist; 
    lower_bound = max(0, mean_dist - std_dist); 
    
    % 4. Plot the bounds and the mean line
    fill([time_vec, fliplr(time_vec)], ...
         [upper_bound, fliplr(lower_bound)], ...
         c_map(c,:), 'FaceAlpha', 0.15, 'EdgeColor', 'none', 'HandleVisibility', 'off');
         
    plot(time_vec, mean_dist, 'Color', c_map(c,:), ...
         'LineWidth', 2.5, 'DisplayName', ControllerNames{c});
end

title('\textbf{Mean Distance to Target State (Successful Runs)}', 'FontSize', 14);
xlabel('Simulation Time (s)', 'FontSize', 12);
ylabel('Distance to Target (m)', 'FontSize', 12);
legend('Location', 'northeast', 'FontSize', 11, 'Box', 'off');

% --- AUTO SCALING ---
% Because we filtered out the pre-crash spikes, MATLAB can now perfectly 
% auto-scale the axis to fit the initial distance (~3.4m) down to 0m.
ylim('auto'); 

grid on; ax = gca; ax.GridAlpha = 0.25; ax.LineWidth = 1.2;
hold off; drawnow;

%% Visual 5: Rotor Thrust Violations -> TOTAL ZOOM
figure('Name', 'Rotor Thrust Violations (Zoomed)', 'Color', 'w', 'Position', [300, 300, 700, 500]);
hold on;
for c = 1:numControllers
    boxchart(c * ones(MonteCarloLength, 1), Sanitized_Input_Viol(:, c), ...
        'BoxFaceColor', c_map(c,:), 'MarkerStyle', '.', 'MarkerColor', c_map(c,:), 'BoxWidth', 0.5);
end
title('\textbf{Maximum Individual Rotor Thrust Violations}', 'FontSize', 14);
ylabel('Max Thrust Exceedance (N)', 'FontSize', 12);
xticks(1:numControllers); xticklabels(ControllerNames);
% Red dashed line shows perfect compliance
yline(0, 'r--', 'LineWidth', 1.5, 'Alpha', 0.6, 'HandleVisibility', 'off');
% --- PURE ZOOM FOR VISIBILITY ---
% Since T_max is 2.25, a violation of 0.1N (4.4%) is useful to see.
ylim([0, 0.25]); % Magnified view. Whiskers of TVLQR will go past the roof.
grid on; ax = gca; ax.GridAlpha = 0.25; ax.LineWidth = 1.2;
hold off; drawnow;
%% ================ Local Functions =============================
%% ========== Create Initial Guess for CasADI solver ===============
function [x_init,U_init] = initialGuess(nx,nu,N,x0_val,xf_val,T,xdot,x,u,guessType)
    import casadi.*
    X_init = zeros(nx, N+1);
    U_init = zeros(nu, N);
    if strcmp(guessType,'linearState 0U')
        for k = 1:N+1
            alpha = (k-1)/N;
            X_init(:,k) = x0_val + alpha*(xf_val - x0_val);
        end
    elseif strcmp(guessType,'linearState invUequalibrium') || strcmp(guessType,'linearState invUoperatingpt')
        % Because Dynamics are control-affine -> pick 4 equations to solve the 4 control variables. 
        idx = [6, 10, 11, 12]; % 4 indices in xdot that contain u(1), u(2), u(3), u(4)
        target_xdot = MX.sym('target', 4); % The values you want these 4 eqns to equal
        relevant_eqs = xdot(idx);
        % Calculate the mapping matrix from u to xdot
        G = jacobian(relevant_eqs, u); %control-affine so xdot=f(x)+g(x)u
        % Calculate the "drift" term (everything in those equations NOT multiplied by u)
        % We get this by substituting u = 0
        F = substitute(relevant_eqs, u, zeros(nu, 1));
        % Solve: target_xdot = F + G*u  =>  u = G \ (target_xdot - F)
        u_sol = solve(G,(target_xdot - F)); 
        eval_u = Function('eval_u', {x, target_xdot}, {u_sol});
        if strcmp(guessType,'linearState invUequalibrium')
            %Solve for U by treating X as equilibrium
            target_xdot = zeros(nx,1);
            target_xdot = target_xdot(idx,1);
        else
            %Solve for U by treating X as operating pt
            target_xdot = (xf_val-x0_val)/T;
            target_xdot = target_xdot(idx,1);
        end
        for k = 1:N+1
            alpha = (k-1)/N;
            xnow = x0_val + alpha*(xf_val - x0_val);
            X_init(:,k) = xnow;
            if k<N+1
                U_init(:,k) = full(eval_u(xnow,target_xdot));
            end
        end
    else
        error('Invalid initialGuessType')
    end
    x_init = [reshape(X_init, nx*(N+1),1);
                  reshape(U_init, nu*N,1)];
end

%% ============== PLOT =============
function plotTrajectory(X,U)
    figure;
    subplot(2,1,1);
    plot(X);
    title('States');
    legend('x','y','z','vx','vy','vz','phi','theta','psi','p','q','r')
    
    subplot(2,1,2);
    stairs(U);
    title('Controls');
    legend('Thrust','Mx','My','Mz')
end

%% ============= ANIMATE ==============
function animateTrajectory(Xtraj, name)
    % Assuming Xtraj is your (N+1) x n state matrix
    rx = Xtraj(:,1); ry = Xtraj(:,2); rz = Xtraj(:,3);
    phi = Xtraj(:,7); theta = Xtraj(:,8); psi = Xtraj(:,9);
    
    numFrames = size(Xtraj, 1);
    gap = 1; % Adjust for playback speed
    
    % --- Define the Quadcopter Template ---
    L = 0.25; % Arm length (meters)
    R_prop = 0.15; % Propeller radius
    
    % Arms (X shape)
    arm1 = [-L, L; 0, 0; 0, 0]; 
    arm2 = [0, 0; -L, L; 0, 0]; 
    
    % "Up" Indicator 
    mast_height = 0.2;
    mast = [0, 0; 0, 0; 0, mast_height]; 
    
    % Propeller circle points (for the patch)
    theta_cir = linspace(0, 2*pi, 20);
    prop_circle = [R_prop*cos(theta_cir); R_prop*sin(theta_cir); zeros(1, 20)];
    centers = [L, -L, 0, 0; 0, 0, L, -L; 0, 0, 0, 0];
    
    % --- Initialize Figure ---
    fig = figure('WindowStyle', 'normal', 'Name', 'Epic Quadcopter Animation', 'Color', 'w');
    hold on; view(35, 25); axis equal;
    
    % Set fixed axes to perfectly frame the 4x4x5 box
    % X/Y from -2 to 2 (4m total), Z from 0 to 5 (5m total)
    xlim([-2.2, 2.2]); 
    ylim([-2.2, 2.2]); 
    zlim([0, 5.2]);
    xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
    
    % Turn off default grid, box handles the reference now
    grid off; 
    set(gca, 'Box', 'off');
    
    z_floor = 0; % Shadow projection plane
    
    % --- Draw the 4x4x5 Flight Arena ---
    box_color = [0.85 0.85 0.85];
    xb = [-2 2 2 -2 -2]; yb = [-2 -2 2 2 -2];
    
    % Bottom and Top perimeters
    plot3(xb, yb, z_floor*ones(1,5), 'Color', box_color, 'LineWidth', 1.5, 'LineStyle', '--');
    plot3(xb, yb, 5*ones(1,5), 'Color', box_color, 'LineWidth', 1.5, 'LineStyle', '--');
    
    % Vertical pillars
    for k = 1:4
        plot3([xb(k) xb(k)], [yb(k) yb(k)], [z_floor 5], 'Color', box_color, 'LineWidth', 1.5, 'LineStyle', '--');
    end

    % --- Initialize Plot Objects ---
    
    % 1. Faint full path
    plot3(rx, ry, rz, 'Color', [0.8 0.8 0.8], 'LineWidth', 1.5, 'LineStyle', '--');
    
    % 2. Active colored trail
    trail = plot3(rx(1), ry(1), rz(1), 'Color', [0 0.4 0.8], 'LineWidth', 2.0);
    
    % 3. Ground Shadows
    shadow_color = [0.7 0.7 0.7];
    sh_arm1 = plot3(arm1(1,:), arm1(2,:), z_floor*ones(1,2), 'Color', shadow_color, 'LineWidth', 2);
    sh_arm2 = plot3(arm2(1,:), arm2(2,:), z_floor*ones(1,2), 'Color', shadow_color, 'LineWidth', 2);
    sh_hub  = scatter3(0, 0, z_floor, 40, 'MarkerEdgeColor', 'none', 'MarkerFaceColor', shadow_color);
    
    % 4. Quadcopter Body
    p_arm1 = plot3(arm1(1,:), arm1(2,:), arm1(3,:), 'k', 'LineWidth', 3);
    p_arm2 = plot3(arm2(1,:), arm2(2,:), arm2(3,:), 'k', 'LineWidth', 3);
    p_hub  = scatter3(0, 0, 0, 60, 'k', 'filled'); 
    p_mast = plot3(mast(1,:), mast(2,:), mast(3,:), 'Color', [0 0.7 0], 'LineWidth', 2.5);
    
    % 5. Semi-Transparent Propellers (Using patch instead of plot3)
    p_props = gobjects(1,4); sh_props = gobjects(1,4);
    prop_colors = {[0.8 0.1 0.1], [0.8 0.1 0.1], [0.2 0.2 0.2], [0.2 0.2 0.2]}; 
    
    for j = 1:4
        % Actual Propellers
        p_props(j) = patch('XData', prop_circle(1,:) + centers(1,j), ...
                           'YData', prop_circle(2,:) + centers(2,j), ...
                           'ZData', prop_circle(3,:) + centers(3,j), ...
                           'FaceColor', prop_colors{j}, 'FaceAlpha', 0.4, ...
                           'EdgeColor', prop_colors{j}, 'LineWidth', 1.5);
                       
        % Shadow Propellers
        sh_props(j) = patch('XData', prop_circle(1,:) + centers(1,j), ...
                            'YData', prop_circle(2,:) + centers(2,j), ...
                            'ZData', z_floor*ones(1,20), ...
                            'FaceColor', shadow_color, 'FaceAlpha', 0.2, ...
                            'EdgeColor', 'none');
    end
    
    drawnow;
    quad_frames(numFrames) = struct('cdata', [], 'colormap', []);
    
    % --- Animation Loop ---
    for i = 1:gap:numFrames
        if ~isgraphics(fig), break; end
        
        pos = [rx(i); ry(i); rz(i)];
        
        % Rotation Matrix (Z-Y-X Euler)
        cphi = cos(phi(i)); sphi = sin(phi(i));
        cthe = cos(theta(i)); sthe = sin(theta(i));
        cpsi = cos(psi(i)); spsi = sin(psi(i));
        
        Rot = [cpsi*cthe,  cpsi*sthe*sphi - spsi*cphi,  cpsi*sthe*cphi + spsi*sphi;
               spsi*cthe,  spsi*sthe*sphi + cpsi*cphi,  spsi*sthe*cphi - cpsi*sphi;
               -sthe,      cthe*sphi,                   cthe*cphi];
        
        % Transform Body
        arm1_rot = Rot * arm1 + pos;
        arm2_rot = Rot * arm2 + pos;
        mast_rot = Rot * mast + pos;
        
        set(p_arm1, 'XData', arm1_rot(1,:), 'YData', arm1_rot(2,:), 'ZData', arm1_rot(3,:));
        set(p_arm2, 'XData', arm2_rot(1,:), 'YData', arm2_rot(2,:), 'ZData', arm2_rot(3,:));
        set(p_mast, 'XData', mast_rot(1,:), 'YData', mast_rot(2,:), 'ZData', mast_rot(3,:));
        set(p_hub, 'XData', pos(1), 'YData', pos(2), 'ZData', pos(3));
        
        % Transform Shadows
        set(sh_arm1, 'XData', arm1_rot(1,:), 'YData', arm1_rot(2,:));
        set(sh_arm2, 'XData', arm2_rot(1,:), 'YData', arm2_rot(2,:));
        set(sh_hub, 'XData', pos(1), 'YData', pos(2));
        
        % Transform Propellers
        for j = 1:4
            prop_rot = Rot * (prop_circle + centers(:,j)) + pos;
            set(p_props(j), 'XData', prop_rot(1,:), 'YData', prop_rot(2,:), 'ZData', prop_rot(3,:));
            set(sh_props(j), 'XData', prop_rot(1,:), 'YData', prop_rot(2,:));
        end
        
        % Update trail
        set(trail, 'XData', rx(1:i), 'YData', ry(1:i), 'ZData', rz(1:i));
        
        drawnow;
        quad_frames(i) = getframe(fig);
        pause(0.01); 
    end
    
    assignin('base', name, quad_frames);
    CallItThis = sprintf('Animation complete. Replay using: movie(gcf, %s, 1, 30)',name);
    disp(CallItThis);
end
%%=== and save animation
function animateAndSaveTrajectory(Xtraj, name)
    % Assuming Xtraj is your (N+1) x n state matrix
    rx = Xtraj(:,1); ry = Xtraj(:,2); rz = Xtraj(:,3);
    phi = Xtraj(:,7); theta = Xtraj(:,8); psi = Xtraj(:,9);
    
    numFrames = size(Xtraj, 1);
    gap = 1; % Adjust for playback speed
    
    % --- Define the Quadcopter Template ---
    L = 0.25; % Arm length (meters)
    R_prop = 0.15; % Propeller radius
    
    % Arms (X shape)
    arm1 = [-L, L; 0, 0; 0, 0]; 
    arm2 = [0, 0; -L, L; 0, 0]; 
    
    % "Up" Indicator 
    mast_height = 0.2;
    mast = [0, 0; 0, 0; 0, mast_height]; 
    
    % Propeller circle points (for the patch)
    theta_cir = linspace(0, 2*pi, 20);
    prop_circle = [R_prop*cos(theta_cir); R_prop*sin(theta_cir); zeros(1, 20)];
    centers = [L, -L, 0, 0; 0, 0, L, -L; 0, 0, 0, 0];
    
    % --- Initialize Figure ---
    fig = figure('WindowStyle', 'normal', 'Name', 'Epic Quadcopter Animation', 'Color', 'w');
    hold on; view(35, 25); axis equal;
    
    % Set fixed axes to perfectly frame the 4x4x5 box
    % X/Y from -2 to 2 (4m total), Z from 0 to 5 (5m total)
    xlim([-2.2, 2.2]); 
    ylim([-2.2, 2.2]); 
    zlim([0, 5.2]);
    xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
    
    % Turn off default grid, box handles the reference now
    grid off; 
    set(gca, 'Box', 'off');
    
    z_floor = 0; % Shadow projection plane
    
    % --- Draw the 4x4x5 Flight Arena ---
    box_color = [0.85 0.85 0.85];
    xb = [-2 2 2 -2 -2]; yb = [-2 -2 2 2 -2];
    
    % Bottom and Top perimeters
    plot3(xb, yb, z_floor*ones(1,5), 'Color', box_color, 'LineWidth', 1.5, 'LineStyle', '--');
    plot3(xb, yb, 5*ones(1,5), 'Color', box_color, 'LineWidth', 1.5, 'LineStyle', '--');
    
    % Vertical pillars
    for k = 1:4
        plot3([xb(k) xb(k)], [yb(k) yb(k)], [z_floor 5], 'Color', box_color, 'LineWidth', 1.5, 'LineStyle', '--');
    end
    % --- Initialize Plot Objects ---
    
    % 1. Faint full path
    plot3(rx, ry, rz, 'Color', [0.8 0.8 0.8], 'LineWidth', 1.5, 'LineStyle', '--');
    
    % 2. Active colored trail
    trail = plot3(rx(1), ry(1), rz(1), 'Color', [0 0.4 0.8], 'LineWidth', 2.0);
    
    % 3. Ground Shadows
    shadow_color = [0.7 0.7 0.7];
    sh_arm1 = plot3(arm1(1,:), arm1(2,:), z_floor*ones(1,2), 'Color', shadow_color, 'LineWidth', 2);
    sh_arm2 = plot3(arm2(1,:), arm2(2,:), z_floor*ones(1,2), 'Color', shadow_color, 'LineWidth', 2);
    sh_hub  = scatter3(0, 0, z_floor, 40, 'MarkerEdgeColor', 'none', 'MarkerFaceColor', shadow_color);
    
    % 4. Quadcopter Body
    p_arm1 = plot3(arm1(1,:), arm1(2,:), arm1(3,:), 'k', 'LineWidth', 3);
    p_arm2 = plot3(arm2(1,:), arm2(2,:), arm2(3,:), 'k', 'LineWidth', 3);
    p_hub  = scatter3(0, 0, 0, 60, 'k', 'filled'); 
    p_mast = plot3(mast(1,:), mast(2,:), mast(3,:), 'Color', [0 0.7 0], 'LineWidth', 2.5);
    
    % 5. Semi-Transparent Propellers (Using patch instead of plot3)
    p_props = gobjects(1,4); sh_props = gobjects(1,4);
    prop_colors = {[0.8 0.1 0.1], [0.8 0.1 0.1], [0.2 0.2 0.2], [0.2 0.2 0.2]}; 
    
    for j = 1:4
        % Actual Propellers
        p_props(j) = patch('XData', prop_circle(1,:) + centers(1,j), ...
                           'YData', prop_circle(2,:) + centers(2,j), ...
                           'ZData', prop_circle(3,:) + centers(3,j), ...
                           'FaceColor', prop_colors{j}, 'FaceAlpha', 0.4, ...
                           'EdgeColor', prop_colors{j}, 'LineWidth', 1.5);
                       
        % Shadow Propellers
        sh_props(j) = patch('XData', prop_circle(1,:) + centers(1,j), ...
                            'YData', prop_circle(2,:) + centers(2,j), ...
                            'ZData', z_floor*ones(1,20), ...
                            'FaceColor', shadow_color, 'FaceAlpha', 0.2, ...
                            'EdgeColor', 'none');
    end
    
    drawnow;
    
    % --- Setup Video Writer ---
    video_filename = sprintf('%s.mp4', name);
    v = VideoWriter(video_filename, 'MPEG-4');
    v.FrameRate = 15; % Adjust frame rate to match your data
    v.Quality = 100;  % Max video quality
    open(v);
    
    % Preallocate structure for workspace saving
    numSavedFrames = length(1:gap:numFrames);
    quad_frames(numSavedFrames) = struct('cdata', [], 'colormap', []);
    
    frameCount = 1;
    
    % --- Animation Loop ---
    for i = 1:gap:numFrames
        if ~isgraphics(fig), break; end
        
        pos = [rx(i); ry(i); rz(i)];
        
        % Rotation Matrix (Z-Y-X Euler)
        cphi = cos(phi(i)); sphi = sin(phi(i));
        cthe = cos(theta(i)); sthe = sin(theta(i));
        cpsi = cos(psi(i)); spsi = sin(psi(i));
        
        Rot = [cpsi*cthe,  cpsi*sthe*sphi - spsi*cphi,  cpsi*sthe*cphi + spsi*sphi;
               spsi*cthe,  spsi*sthe*sphi + cpsi*cphi,  spsi*sthe*cphi - cpsi*sphi;
               -sthe,      cthe*sphi,                   cthe*cphi];
        
        % Transform Body
        arm1_rot = Rot * arm1 + pos;
        arm2_rot = Rot * arm2 + pos;
        mast_rot = Rot * mast + pos;
        
        set(p_arm1, 'XData', arm1_rot(1,:), 'YData', arm1_rot(2,:), 'ZData', arm1_rot(3,:));
        set(p_arm2, 'XData', arm2_rot(1,:), 'YData', arm2_rot(2,:), 'ZData', arm2_rot(3,:));
        set(p_mast, 'XData', mast_rot(1,:), 'YData', mast_rot(2,:), 'ZData', mast_rot(3,:));
        set(p_hub, 'XData', pos(1), 'YData', pos(2), 'ZData', pos(3));
        
        % Transform Shadows
        set(sh_arm1, 'XData', arm1_rot(1,:), 'YData', arm1_rot(2,:));
        set(sh_arm2, 'XData', arm2_rot(1,:), 'YData', arm2_rot(2,:));
        set(sh_hub, 'XData', pos(1), 'YData', pos(2));
        
        % Transform Propellers
        for j = 1:4
            prop_rot = Rot * (prop_circle + centers(:,j)) + pos;
            set(p_props(j), 'XData', prop_rot(1,:), 'YData', prop_rot(2,:), 'ZData', prop_rot(3,:));
            set(sh_props(j), 'XData', prop_rot(1,:), 'YData', prop_rot(2,:));
        end
        
        % Update trail
        set(trail, 'XData', rx(1:i), 'YData', ry(1:i), 'ZData', rz(1:i));
        
        drawnow;
        
        % Capture frame and write to video
        current_frame = getframe(fig);
        writeVideo(v, current_frame);
        
        % Save to structure for workspace
        quad_frames(frameCount) = current_frame;
        frameCount = frameCount + 1;
        
        % Optional: Remove or reduce pause if rendering is slow
        % pause(0.01); 
    end
    
    % Close video file
    close(v);
    
    assignin('base', name, quad_frames);
    CallItThis = sprintf('Animation complete.\nVideo saved as: %s\nReplay in MATLAB using: movie(gcf, %s, 1, 30)', video_filename, name);
    disp(CallItThis);
end

%% ====== Model Paramter Perturbations ============
function true_params = generatePerturbedParams(nominal_params, sd_percentages)
    % nominal_params: 16x1 vector of nominal constants
    % sd_percentages: 16x1 vector of standard deviations (e.g., 0.10 for 10%)
    
    % Ensure repeatability if rng(0) is set in the main script
    num_params = length(nominal_params);
    
    % Generate Gaussian noise: N(mean, std^2)
    % Value = nominal + nominal * N(0,1) * sd_percentage
    noise_multipliers = randn(num_params, 1) .* sd_percentages;
    true_params = nominal_params .* (1 + noise_multipliers);

    % In a rigorous setup, you might use truncated normal distributions here
end

%% ====== MPC linearization Corrections =======
function [Ad_AllSteps,Bd_AllSteps,allCk] = MPClinearizationsAndCorrections(X,U,eval_discreteAk,eval_discreteBk,eval_fDiscrete)
    n = height(X); [m,N] = size(U);
    Ad_AllSteps = zeros(n,n,N);
    Bd_AllSteps = zeros(n,m,N);
    allCk = zeros(n,N);
    for i = 1:N %Linearizations and Discretizations
        % Index Nominal Ref traj x and u
        xRef = X(:,i);
        uRef = U(:,i);
        % Linear Continuous Dynamics Around Ref Traj then Discretize
        AkDiscrete = eval_discreteAk(xRef,uRef);
        BkDiscrete = eval_discreteBk(xRef,uRef);
        Ad_AllSteps(:,:,i) = AkDiscrete;
        Bd_AllSteps(:,:,i) = BkDiscrete;
    
        ck = eval_fDiscrete(xRef,uRef) - AkDiscrete*xRef - BkDiscrete*uRef;
        allCk(:,i) = ck;
    end
end

%% ======== Shrinking RTI NMPC Step Setup ==========
function [current_U, Zguess_next] = stepShrinkingRTI_NMPC(i, N, nx, nu, Zguess, Zref_full, current_X, xf_val, eval_fDiscrete, eval_discreteAk, eval_discreteBk, Q, R, Qf, x_max, x_min, T_max, T_min, A_inv, Options)
    % Active Feedback RTI NMPC for Shrinking Horizon
    
    % Current horizon length
    N_curr = N - i + 1;
    
    % 1. Slice Zref for current horizon
    X_ref_full = Zref_full(1 : nx*(N+1));
    U_ref_full = Zref_full(nx*(N+1) + 1 : end);
    Zref_curr = [X_ref_full((i-1)*nx+1 : end); U_ref_full((i-1)*nu+1 : end)];
    
    % 2. Setup QP Matrices (using the perturbed current_X as the initial state!)
    [H, f, Aeq, beq, G, h] = setupRTI_QP_Matrices(...
        N_curr, nx, nu, Zguess, Zref_curr, current_X, xf_val, ...
        eval_fDiscrete, eval_discreteAk, eval_discreteBk, ...
        Q, R, Qf, x_max, x_min, T_max, T_min, A_inv);
        
    % 3. Solve local QP (One single step)
    [Zstar, ~, exitFlag] = quadprog(H, f, G, h, Aeq, beq, [], [], Zguess, Options);
    
    if exitFlag <= 0
        fprintf('RTI QP failed at step %d (exitFlag = %d). Using previous guess.\n', i, exitFlag);
        Zstar = Zguess; % Fallback
    end
    
    % 4. Extract Control to Apply Right Now
    idx_u_start = nx*(N_curr + 1) + 1;
    current_U = Zstar(idx_u_start : idx_u_start + nu - 1);
    
    % 5. Shrink Zstar to form the guess for the NEXT time step
    if i < N
        X_star = Zstar(1 : nx*(N_curr + 1));
        U_star = Zstar(idx_u_start : end);
        % Drop the first state and first control
        Zguess_next = [X_star(nx+1 : end); U_star(nu+1 : end)];
    else
        Zguess_next = []; % End of horizon
    end
end

%% --- Local Helper for RTI ---
function [H, f, Aeq, beq, G, h] = setupRTI_QP_Matrices(N_curr, nx, nu, Zguess, Zref, current_X, xf_val, eval_fDiscrete, eval_discreteAk, eval_discreteBk, Q, R, Qf, x_max, x_min, T_max, T_min, A_inv)
    % Objective
    [Qh, Rh] = bigWeighting(Q, R, Qf, N_curr);
    H = multipleShootingH(nx, Qh, Rh, Q);
    f = -H * Zref; 
    
    % Linearize Dynamics
    Ad = zeros(nx, nx, N_curr);
    Bd = zeros(nx, nu, N_curr);
    Ck = zeros(nx, N_curr);
    for k = 1:N_curr
        xk = Zguess((k-1)*nx + 1 : k*nx);
        uk = Zguess(nx*(N_curr+1) + (k-1)*nu + 1 : nx*(N_curr+1) + k*nu);
        Ad(:,:,k) = eval_discreteAk(xk, uk);
        Bd(:,:,k) = eval_discreteBk(xk, uk);
        Ck(:,k)   = eval_fDiscrete(xk, uk) - Ad(:,:,k)*xk - Bd(:,:,k)*uk;
    end
    
    % Dynamic Constraints (Injects current_X into beq)
    [Aeq, beq] = multipleShootingDynamicConstraints(Ad, Bd, Ck, current_X, N_curr);
    
    % Terminal Constraint
    %Aeq(end+1:end+nx, nx*N_curr+1 : nx*(N_curr+1)) = eye(nx); 
    %beq(end+1:end+nx, 1) = xf_val;
    
    % Inequality Constraints
    n_vars = (N_curr + 1)*nx + N_curr*nu;
    Gstate = zeros(2*(N_curr+1)*nx, n_vars);
    hstate = zeros(2*(N_curr+1)*nx, 1);
    Ginput = zeros(2*N_curr*nu, n_vars);
    hinput = zeros(2*N_curr*nu, 1);
    
    for k = 1:(N_curr + 1)
        stateIndx = (k-1)*nx+1 : k*nx;
        rowMaxIndx = (2*k-2)*nx+1 : (2*k-1)*nx;
        Gstate(rowMaxIndx, stateIndx) = eye(nx);
        hstate(rowMaxIndx, 1) = x_max;
        
        rowMinIndx = (2*k-1)*nx+1 : (2*k)*nx;
        Gstate(rowMinIndx, stateIndx) = -eye(nx);
        hstate(rowMinIndx, 1) = -x_min;
        
        if k <= N_curr
            controlIndx = (N_curr+1)*nx + (k-1)*nu+1 : (N_curr+1)*nx + k*nu;
            rowMaxInputIndx = (2*k-2)*nu+1 : (2*k-1)*nu;
            Ginput(rowMaxInputIndx, controlIndx) = A_inv;
            hinput(rowMaxInputIndx, 1) = T_max*ones(nu,1);
            
            rowMinInputIndx = (2*k-1)*nu+1 : (2*k)*nu;
            Ginput(rowMinInputIndx, controlIndx) = -A_inv;
            hinput(rowMinInputIndx, 1) = T_min*ones(nu,1);
        end
    end
    G = [Gstate; Ginput]; 
    h = [hstate; hinput];
    valid_rows = ~isinf(h);
    G = G(valid_rows, :);
    h = h(valid_rows);
end

function [current_U, Zguess_next] = stepRecedingRTI_NMPC(i, N, nx, nu, Zguess, Zref_full, current_X, xf_val, eval_fDiscrete, eval_discreteAk, eval_discreteBk, Q, R, Qf, x_max, x_min, T_max, T_min, A_inv, Options, m_nom, g)
    % Active Feedback RTI NMPC for Receding Horizon (Constant N)
    
    % Nominal hover control used ONLY for the objective reference penalty
    U_ref_ss = [m_nom*g; 0; 0; 0]; 
    
    % --- 1. Slice and Pad Reference ---
    N_full_ref = (length(Zref_full) - nx) / (nx + nu); 
    X_ref_full = Zref_full(1 : nx*(N_full_ref+1));
    U_ref_full = Zref_full(nx*(N_full_ref+1) + 1 : end);
    
    Zref_curr_X = zeros(nx*(N+1), 1);
    Zref_curr_U = zeros(nu*N, 1);
    
    for k = 1:(N+1)
        ref_idx = i + k - 1;
        if ref_idx <= N_full_ref + 1
            Zref_curr_X((k-1)*nx+1 : k*nx) = X_ref_full((ref_idx-1)*nx+1 : ref_idx*nx);
        else
            Zref_curr_X((k-1)*nx+1 : k*nx) = xf_val; % Pad with exact target state
        end
        
        if k <= N
            if ref_idx <= N_full_ref
                Zref_curr_U((k-1)*nu+1 : k*nu) = U_ref_full((ref_idx-1)*nu+1 : ref_idx*nu);
            else
                Zref_curr_U((k-1)*nu+1 : k*nu) = U_ref_ss; % Penalize deviation from nominal hover
            end
        end
    end
    Zref_curr = [Zref_curr_X; Zref_curr_U];
    
    % --- 2. Setup QP Matrices (using perturbed current_X) ---
    [H, f, Aeq, beq, G, h] = setupRecedingRTI_QP_Matrices(...
        N, nx, nu, Zguess, Zref_curr, current_X, ...
        eval_fDiscrete, eval_discreteAk, eval_discreteBk, ...
        Q, R, Qf, x_max, x_min, T_max, T_min, A_inv);
        
    % --- 3. Solve local QP ---
    [Zstar, ~, exitFlag] = quadprog(H, f, G, h, Aeq, beq, [], [], Zguess, Options);
    
    if exitFlag <= 0
        % If noise causes brief infeasibility, fallback to shifted guess
        Zstar = Zguess; 
    end
    
    % --- 4. Extract Control ---
    current_U = Zstar(nx*(N+1) + 1 : nx*(N+1) + nu);
    
    % --- 5. Shift Zstar for NEXT time step (Constant N) ---
    X_star = Zstar(1 : nx*(N+1));
    U_star = Zstar(nx*(N+1) + 1 : end);
    
    % ADAPTIVE PADDING: Use the QP's last planned control to pad the guess
    U_adaptive_pad = U_star(end-nu+1 : end);
    
    U_next = zeros(nu*N, 1);
    U_next(1 : nu*(N-1)) = U_star(nu+1 : end);
    U_next(nu*(N-1)+1 : end) = U_adaptive_pad; 
    
    X_next = zeros(nx*(N+1), 1);
    X_next(1 : nx*N) = X_star(nx+1 : end); 
    
    x_prev = X_next(nx*(N-1)+1 : nx*N);
    X_next(nx*N+1 : end) = eval_fDiscrete(x_prev, U_adaptive_pad);
    
    Zguess_next = [X_next; U_next];
end

%% --- Local Helper specifically for Receding Horizon RTI ---
function [H, f, Aeq, beq, G, h] = setupRecedingRTI_QP_Matrices(N_curr, nx, nu, Zguess, Zref, current_X, eval_fDiscrete, eval_discreteAk, eval_discreteBk, Q, R, Qf, x_max, x_min, T_max, T_min, A_inv)
    % Objective
    [Qh, Rh] = bigWeighting(Q, R, Qf, N_curr);
    H = multipleShootingH(nx, Qh, Rh, Q);
    f = -H * Zref; 
    
    % Linearize Dynamics
    Ad = zeros(nx, nx, N_curr);
    Bd = zeros(nx, nu, N_curr);
    Ck = zeros(nx, N_curr);
    for k = 1:N_curr
        xk = Zguess((k-1)*nx + 1 : k*nx);
        uk = Zguess(nx*(N_curr+1) + (k-1)*nu + 1 : nx*(N_curr+1) + k*nu);
        Ad(:,:,k) = eval_discreteAk(xk, uk);
        Bd(:,:,k) = eval_discreteBk(xk, uk);
        Ck(:,k)   = eval_fDiscrete(xk, uk) - Ad(:,:,k)*xk - Bd(:,:,k)*uk;
    end
    
    % Dynamic Constraints (Injects current_X into beq)
    [Aeq, beq] = multipleShootingDynamicConstraints(Ad, Bd, Ck, current_X, N_curr);
    
    % CRITICAL FIX: No terminal equality constraint added here! 
    % The QP is free to find a feasible path near the origin without crashing.
    
    % Inequality Constraints
    n_vars = (N_curr + 1)*nx + N_curr*nu;
    Gstate = zeros(2*(N_curr+1)*nx, n_vars);
    hstate = zeros(2*(N_curr+1)*nx, 1);
    Ginput = zeros(2*N_curr*nu, n_vars);
    hinput = zeros(2*N_curr*nu, 1);
    
    for k = 1:(N_curr + 1)
        stateIndx = (k-1)*nx+1 : k*nx;
        rowMaxIndx = (2*k-2)*nx+1 : (2*k-1)*nx;
        Gstate(rowMaxIndx, stateIndx) = eye(nx);
        hstate(rowMaxIndx, 1) = x_max;
        
        rowMinIndx = (2*k-1)*nx+1 : (2*k)*nx;
        Gstate(rowMinIndx, stateIndx) = -eye(nx);
        hstate(rowMinIndx, 1) = -x_min;
        
        if k <= N_curr
            controlIndx = (N_curr+1)*nx + (k-1)*nu+1 : (N_curr+1)*nx + k*nu;
            rowMaxInputIndx = (2*k-2)*nu+1 : (2*k-1)*nu;
            Ginput(rowMaxInputIndx, controlIndx) = A_inv;
            hinput(rowMaxInputIndx, 1) = T_max*ones(nu,1);
            
            rowMinInputIndx = (2*k-1)*nu+1 : (2*k)*nu;
            Ginput(rowMinInputIndx, controlIndx) = -A_inv;
            hinput(rowMinInputIndx, 1) = -T_min*ones(nu,1); 
        end
    end
    G = [Gstate; Ginput]; 
    h = [hstate; hinput];
    valid_rows = ~isinf(h);
    G = G(valid_rows, :);
    h = h(valid_rows);
end

%% ====== Continuous Environmental Disturbance (Wind) ======
function d_dot = continuousWindDisturbance(t, MonteSimNum)
    % t: current continuous time inside ode45
    % MonteSimNum: used as a seed offset to ensure different wind per run
    
    % 1. Randomize Base Wind Direction (0 to 2*pi)
    % Multiplying by an arbitrary irrational-like number creates a pseudo-random angle
    wind_angle = mod(MonteSimNum * 137.508, 2*pi); 
    
    % 2. Set Base Wind Magnitude (e.g., 0.25 Newtons)
    base_mag = 0.08; 
    
    % X and Y base winds will average to 0 over many Monte Carlo runs
    base_wind_X = base_mag * cos(wind_angle); 
    base_wind_Y = base_mag * sin(wind_angle);
    base_wind_Z = 0.01 * sin(MonteSimNum * 42.1); % Slight random updraft/downdraft
    
    % 3. Randomize Gust Frequencies and Phases
    % This ensures the gusts don't constructively interfere at the exact same time every run
    freq1 = 1.0 + 0.4 * sin(MonteSimNum * 11.1); % Range: ~[0.6, 1.4] Hz
    freq2 = 3.0 + 1.0 * sin(MonteSimNum * 22.2); % Range: ~[2.0, 4.0] Hz
    freq3 = 0.8 + 0.3 * sin(MonteSimNum * 33.3); % Range: ~[0.5, 1.1] Hz
    
    phase1 = mod(MonteSimNum * 1.23, 2*pi);
    phase2 = mod(MonteSimNum * 4.56, 2*pi);
    phase3 = mod(MonteSimNum * 7.89, 2*pi);
    
    % 4. Sum of sines to create "chaotic but smooth" gusts
    gust_X = 0.2 * sin(freq1 * t + phase1) + 0.2 * cos(freq2 * t);
    gust_Y = 0.2 * cos(freq1 * t + phase2) + 0.2 * sin(freq3 * t);
    gust_Z = 0.1 * sin(freq2 * t + phase3); 
    
    % The disturbance vector must be the same size as your state derivative (12x1)
    d_dot = zeros(12, 1);
    
    % Mass of the quadcopter
    m_nominal = 0.5; 
    
    % Apply forces (F = ma -> a = F/m)
    d_dot(4) = (base_wind_X + gust_X) / m_nominal; % Acceleration in X
    d_dot(5) = (base_wind_Y + gust_Y) / m_nominal; % Acceleration in Y
    d_dot(6) = (base_wind_Z + gust_Z) / m_nominal; % Acceleration in Z
    
    % Optional: Add random aerodynamic torques if you want to stress the attitude controller
    % torque_mag = 0.01;
    % d_dot(10) = (torque_mag * sin(freq1*t + phase2)) / Ixx; % Roll disturbance
    % d_dot(11) = (torque_mag * cos(freq2*t + phase1)) / Iyy; % Pitch disturbance
    % d_dot(12) = (torque_mag * sin(freq3*t + phase3)) / Izz; % Yaw disturbance
end