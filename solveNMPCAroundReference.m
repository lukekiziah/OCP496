function [Xsol, Usol, Zsol] = solveNMPCAroundReference(Zguess, Zref, H, eval_fDiscrete, eval_discreteAk, eval_discreteBk, x0_val, xf_val, N, nx, nu, G, h, Options)
    % NMPC SQP Solver for tracking a non-zero reference trajectory
    % -----------------------------------------------------------
    tol_grad = 1e-4;
    tol_eq   = 1e-4;
    tol_step = 1e-4;
    MAXiterations = 200;
    mu = 10; % Initial penalty parameter for merit function
    
    % Linear objective term for tracking Zref
    f_obj = -H * Zref; 
    
    % Evaluate initial nonlinear gaps
    [CurrentObjCost, sum_gaps, ~] = evalTrackingGaps(Zguess, Zref, H, eval_fDiscrete, x0_val, N, nx, nu);
    CurrentMeritCost = CurrentObjCost + mu * sum_gaps;
    
    for iter = 1:MAXiterations
        % 1. Linearize discrete dynamics around the current Zguess
        Ad_AllSteps = zeros(nx, nx, N);
        Bd_AllSteps = zeros(nx, nu, N);
        
        allCk = zeros(nx, N);
        
        for k = 1:N
            xk = Zguess((k-1)*nx + 1 : k*nx);
            uk = Zguess(nx*(N+1) + (k-1)*nu + 1 : nx*(N+1) + k*nu);
            Ad_AllSteps(:,:,k) = eval_discreteAk(xk, uk);
            Bd_AllSteps(:,:,k) = eval_discreteBk(xk, uk);
            % Compute the residual (affine term)
            allCk(:,k) = eval_fDiscrete(xk, uk) - Ad_AllSteps(:,:,k)*xk - Bd_AllSteps(:,:,k)*uk;
        end
        
        % 2. Build Equality Constraints for Multiple Shooting
        [Aeq, beq] = multipleShootingDynamicConstraints(Ad_AllSteps, Bd_AllSteps, allCk, x0_val, N);
        
        % Append Terminal State Constraint (optional but matching your Linear MPC script)
        Aeq(end+1:end+nx, nx*N+1:nx*(N+1)) = eye(nx); 
        beq(end+1:end+nx, 1) = xf_val;
        
        % 3. Solve the local QP
        [Zstar, ~, exitFlag, ~, lambda] = quadprog(H, f_obj, G, h, Aeq, beq, [], [], Zguess, Options);
        
        if exitFlag <= 0
            fprintf('SQP QP failed at iteration %d (exitFlag = %d)\n', iter, exitFlag);
            break;
        end
        
        % 4. Update Penalty Parameter (mu) based on dual variables
        max_lambda_eq = max(norm(lambda.eqlin, 'inf'));
        if ~isempty(lambda.ineqlin)
            max_lambda_ineq = max(norm(lambda.ineqlin,'inf'));
        else
            max_lambda_ineq = 0;
        end
        mu = max(mu, max(max_lambda_eq, max_lambda_ineq) + 0.1);
        
        % 5. Line Search (Armijo condition on Merit function)
        alpha = 1.0;
        betterMerit = false;
        for ls = 1:20
            Ztrial = Zguess + alpha * (Zstar - Zguess);
            [trialObj, trial_sum_gaps, trial_max_gap] = evalTrackingGaps(Ztrial, Zref, H, eval_fDiscrete, x0_val, N, nx, nu);
            NewMeritCost = trialObj + mu * trial_sum_gaps;
            
            % Sufficient decrease condition
            if NewMeritCost < CurrentMeritCost - 1e-4 * alpha * abs(CurrentMeritCost)
                Zguess = Ztrial;
                CurrentMeritCost = NewMeritCost;
                eq_res = trial_max_gap;
                betterMerit = true;
                break;
            end
            alpha = alpha * 0.5;
        end
        
        if ~betterMerit
            % Take small conservative step if line search fails
            alpha = 0.1;
            Zguess = Zguess + alpha * (Zstar - Zguess);
            [trialObj, trial_sum_gaps, trial_max_gap] = evalTrackingGaps(Zguess, Zref, H, eval_fDiscrete, x0_val, N, nx, nu);
            CurrentMeritCost = trialObj + mu * trial_sum_gaps;
            eq_res = trial_max_gap;
            fprintf('Line search failed - taking safe step (alpha=0.1), mu=%.2e\n', mu);
        end
        
        % 6. Check Convergence
        % KKT Stationarity residual
        gradL_res = norm(H*(Zguess - Zref) + Aeq'*lambda.eqlin + G'*lambda.ineqlin, 'inf');
        step_res = norm(Zstar - Zguess, 'inf');
        
        if mod(iter,10)==0 || iter <= 5
            fprintf('SQP %d: abs_grad=%.2e abs_eq=%.2e step=%.2e\n', iter, gradL_res, eq_res, step_res);
        end
        
        if gradL_res < tol_grad && eq_res < tol_eq && step_res < tol_step
            fprintf('NMPC Converged successfully at iteration %d!\n', iter);
            break;
        end
    end
    
    % Extract Trajectories
    Zsol = Zguess;
    Xsol = (reshape(Zsol(1:nx*(N+1)), nx, N+1))';
    Usol = (reshape(Zsol(nx*(N+1)+1:end), nu, N))';
end

%% --- Local Helper Function ---
function [objCost, sum_gaps, max_gap] = evalTrackingGaps(Z, Zref, H, eval_fDiscrete, X0, N, nx, nu)
    % 1. Evaluate objective tracking cost
    dZ = Z - Zref;
    objCost = 0.5 * dZ' * H * dZ;
    
    X = Z(1:nx*(N+1));
    U = Z(nx*(N+1)+1:end);
    
    % 2. Evaluate Dynamic Gaps
    sum_gaps = 0;
    max_gap = 0;
    
    % Initial Condition Gap
    gap0 = X(1:nx) - X0;
    sum_gaps = sum_gaps + sum(abs(gap0));
    max_gap = max(abs(gap0));
    
    % Dynamic Gaps
    for k = 1:N
        x_k = X((k-1)*nx+1 : k*nx);
        u_k = U((k-1)*nu+1 : k*nu);
        x_kp1 = X(k*nx+1 : (k+1)*nx);
        
        gap_k = eval_fDiscrete(x_k, u_k) - x_kp1;
        sum_gaps = sum_gaps + sum(abs(gap_k));
        max_gap = max(max_gap, max(abs(gap_k)));
    end
end