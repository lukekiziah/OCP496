function [X_history, U_history] = simulateShrinkingRTI_NMPC(N, nx, nu, x0_val, xf_val, Zref_full, sim_time, eval_fContinuous, eval_fDiscrete, eval_discreteAk, eval_discreteBk, Q, R, Qf, x_max, x_min, T_max, T_min, A_inv, Options, m, g, U_ss, K_lqr, SimulateSteps,shrinkingStepsLeftTransition)
    
    % Initialize histories
    X_history = zeros(nx, SimulateSteps+1);
    U_history = zeros(nu, SimulateSteps);
    X_history(:,1) = x0_val;
    current_X = x0_val;
    
    % Initial guess is the full CasADi offline optimal solution
    Zguess = Zref_full; 
    
    for i = 1:SimulateSteps
        if i < N - shrinkingStepsLeftTransition
            % Current horizon length
            N_curr = N - i + 1; 
            
            % Extract the very first control from our current guess
            % Controls start immediately after the (N_curr + 1) states
            idx_u_start = nx*(N_curr + 1) + 1;
            if i<N
                current_U = Zguess(idx_u_start : idx_u_start + nu - 1); 
            else
                current_U = [m*g;0;0;0];
            end
        else
            % --- MODE 2: LQR HOVER HOLD ---
            state_error = current_X - xf_val;
            current_U = U_ss - K_lqr * state_error;
        end
        
        % Simulate Continuous Plant Without Disturbances
        T_span = [sim_time(i), sim_time(i+1)];
        [~, X_interval] = ode45(@(t, X) eval_fContinuous(X, current_U), T_span, current_X);
        current_X = X_interval(end, :)';
        
        % Store the state and input
        X_history(:, i+1) = current_X;
        U_history(:, i) = current_U;
        
        if i < N - shrinkingStepsLeftTransition
            % 1. SHRINK THE HORIZON
            N_shrink = N - i;
            
            % Shrink Zguess (Drop the first state and first control)
            X_guess_old = Zguess(1 : nx*(N_curr + 1));
            U_guess_old = Zguess(nx*(N_curr + 1) + 1 : end);
            Zguess = [X_guess_old(nx+1 : end); U_guess_old(nu+1 : end)];
            
            % Shrink Zref (Slice directly from the full reference)
            X_ref_full = Zref_full(1 : nx*(N+1));
            U_ref_full = Zref_full(nx*(N+1) + 1 : end);
            Zref_shrink = [X_ref_full(i*nx+1 : end); U_ref_full(i*nu+1 : end)];
            
            % 2. BUILD RTI QP MATRICES 
            % Linearize around the newly shrunk Zguess
            [Hshrink, fshrink, AeqShrink, beqShrink, Gshrink, hShrink] = setupRTI_QP_Matrices(...
                N_shrink, nx, nu, Zguess, Zref_shrink, current_X, xf_val, ...
                eval_fDiscrete, eval_discreteAk, eval_discreteBk, ...
                Q, R, Qf, x_max, x_min, T_max, T_min, A_inv);
            
            % 3. SOLVE LOCAL QP (One Iteration)
            [Zstar, ~, exitFlag] = quadprog(Hshrink, fshrink, Gshrink, hShrink, AeqShrink, beqShrink, [], [], Zguess, Options);
            
            if exitFlag <= 0
                fprintf('RTI QP failed at step %d (exitFlag = %d). Using fallback!\n', i, exitFlag);
                % FALLBACK: Instead of breaking and filling the rest with zeros,
                % just use the shifted guess. It's safe and keeps the drone flying!
                Zstar = Zguess; 
            end
            
            % 4. FULL NEWTON STEP
            Zguess = Zstar; 
        end
    end
end

%% --- LOCAL HELPER FUNCTION ---
function [H, f, Aeq, beq, G, h] = setupRTI_QP_Matrices(N_shrink, nx, nu, Zguess, Zref, current_X, xf_val, eval_fDiscrete, eval_discreteAk, eval_discreteBk, Q, R, Qf, x_max, x_min, T_max, T_min, A_inv)
    
    % 1. Cost Function (Tracking Zref)
    [Qh, Rh] = bigWeighting(Q, R, Qf, N_shrink);
    H = multipleShootingH(nx, Qh, Rh, Q);
    f = -H * Zref; % Linear objective term for tracking
    
    % 2. Linearize Dynamics around Zguess
    Ad = zeros(nx, nx, N_shrink);
    Bd = zeros(nx, nu, N_shrink);
    Ck = zeros(nx, N_shrink);
    
    for k = 1:N_shrink
        xk = Zguess((k-1)*nx + 1 : k*nx);
        uk = Zguess(nx*(N_shrink+1) + (k-1)*nu + 1 : nx*(N_shrink+1) + k*nu);
        
        Ad(:,:,k) = eval_discreteAk(xk, uk);
        Bd(:,:,k) = eval_discreteBk(xk, uk);
        Ck(:,k)   = eval_fDiscrete(xk, uk) - Ad(:,:,k)*xk - Bd(:,:,k)*uk;
    end
    
    % 3. Equality Constraints
    [Aeq, beq] = multipleShootingDynamicConstraints(Ad, Bd, Ck, current_X, N_shrink);
    
    % Append Terminal State Constraint (since horizon shrinks, terminal node is preserved)
    % Aeq(end+1:end+nx, nx*N_shrink+1 : nx*(N_shrink+1)) = eye(nx); 
    % beq(end+1:end+nx, 1) = xf_val;
    
    % 4. Inequality Constraints (State and Input Bounds)
    n_vars = (N_shrink + 1)*nx + N_shrink*nu;
    Gstate = zeros(2*(N_shrink+1)*nx, n_vars);
    hstate = zeros(2*(N_shrink+1)*nx, 1);
    Ginput = zeros(2*N_shrink*nu, n_vars);
    hinput = zeros(2*N_shrink*nu, 1);
    
    for k = 1:(N_shrink + 1)
        stateIndx = (k-1)*nx+1 : k*nx;
        
        % State Max
        rowMaxIndx = (2*k-2)*nx+1 : (2*k-1)*nx;
        Gstate(rowMaxIndx, stateIndx) = eye(nx);
        hstate(rowMaxIndx, 1) = x_max;
        
        % State Min
        rowMinIndx = (2*k-1)*nx+1 : (2*k)*nx;
        Gstate(rowMinIndx, stateIndx) = -eye(nx);
        hstate(rowMinIndx, 1) = -x_min;
        
        if k <= N_shrink
            controlIndx = (N_shrink+1)*nx + (k-1)*nu+1 : (N_shrink+1)*nx + k*nu;
            
            % Input Max
            rowMaxInputIndx = (2*k-2)*nu+1 : (2*k-1)*nu;
            Ginput(rowMaxInputIndx, controlIndx) = A_inv;
            hinput(rowMaxInputIndx, 1) = T_max*ones(nu,1);
            
            % Input Min
            rowMinInputIndx = (2*k-1)*nu+1 : (2*k)*nu;
            Ginput(rowMinInputIndx, controlIndx) = -A_inv;
            hinput(rowMinInputIndx, 1) = T_min*ones(nu,1);
        end
    end
    
    G = [Gstate; Ginput]; 
    h = [hstate; hinput];
    
    % Filter out infinite bounds
    valid_rows = ~isinf(h);
    G = G(valid_rows, :);
    h = h(valid_rows);
end