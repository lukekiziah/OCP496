function [Hessian_shrinks,linearObjectiveShrinks,AeqShrink,beqShrink,Gshrink,hShrink,Zstar,N_shrink] = setupShrinkingMPCstep(i,N,Q,R,Qf,nx,nu,X_opt_col,U_opt_col,Aeq,beq,x_max,x_min,T_max,T_min,A_inv,Zstar,current_X)
%resolve LinearMPC at new state in shrinking horizon fashion
    % Calculate remaining horizon steps instead of letting N_shrink go all the way to 1:
    N_shrink = N - i;

    %resolve LinearMPC at new state in shrinking horizon fashion
    [Qhshrink,RhShrink] = bigWeighting(Q,R,Qf, N_shrink);
    
    %update Hessian
    Hessian_shrinks = multipleShootingH(nx,Qhshrink,RhShrink,Q);
    
    % --- FIX 1: Dynamically shrink the reference trajectory ---
    % Drop the first i+1 states (we want predictions from x_{i+1} to x_N)
    X_opt_col_shrink = X_opt_col(i*nx + 1 : end);  
    % Drop the first i inputs
    U_opt_col_shrink = U_opt_col(i*nu + 1 : end);
    
    %update linear objective
    linearObjectiveShrinks = -2*[zeros(nx,1); Qhshrink*X_opt_col_shrink; RhShrink*U_opt_col_shrink];
    
    % Total number of states and inputs in the ORIGINAL full trajectory
    total_states_original = (N + 1) * nx;
    
    % 1. State columns to keep: Skip the first i*nx columns
    state_cols_to_keep = (i * nx) + 1 : total_states_original;
    
    % 2. Input columns to keep: 
    % These start AFTER all the original states. 
    % We skip the first i*nu control vectors.
    input_start_original = total_states_original + (i * nu) + 1;
    input_cols_to_keep = input_start_original : size(Aeq, 2);
    
    % Combine them for the AeqShrink matrix
    AeqShrink = Aeq(i*nx + 1 : end, [state_cols_to_keep, input_cols_to_keep]);
    beqShrink = beq(i*nx + 1 : end);
    beqShrink(1:nx) = current_X;

    % --- UPDATED INEQUALITY CONSTRAINTS ---
    % Total number of decision variables in the current shrinking step
    n_vars_shrink = (N_shrink + 1) * nx + N_shrink * nu;
    
    % Preallocate matrices using the shrinking horizon size
    GstateShrink = zeros(2 * (N_shrink + 1) * nx, n_vars_shrink);
    hstateShrink = zeros(2 * (N_shrink + 1) * nx, 1);
    GinputShrink = zeros(2 * N_shrink * nu, n_vars_shrink);
    hinputShrink = zeros(2 * N_shrink * nu, 1);
    
    for k = 1:(N_shrink + 1)
        % State bounds
        stateIndx = (k - 1) * nx + 1 : k * nx;
        
        % maximums
        rowMaxIndx = (2 * k - 2) * nx + 1 : (2 * k - 1) * nx;
        GstateShrink(rowMaxIndx, stateIndx) = eye(nx);
        hstateShrink(rowMaxIndx, 1) = x_max;
        
        % minimums
        rowMinIndx = (2 * k - 1) * nx + 1 : (2 * k) * nx;
        GstateShrink(rowMinIndx, stateIndx) = -eye(nx);
        hstateShrink(rowMinIndx, 1) = -x_min;
    
        if k <= N_shrink
            % Input bounds
            controlIndx = (N_shrink + 1) * nx + (k - 1) * nu + 1 : (N_shrink + 1) * nx + k * nu;
            
            % maximums
            rowMaxInputIndx = (2 * k - 2) * nu + 1 : (2 * k - 1) * nu;
            GinputShrink(rowMaxInputIndx, controlIndx) = A_inv;
            hinputShrink(rowMaxInputIndx, 1) = T_max * ones(nu, 1);
            
            % minimums
            rowMinInputIndx = (2 * k - 1) * nu + 1 : (2 * k) * nu;
            GinputShrink(rowMinInputIndx, controlIndx) = -A_inv;
            hinputShrink(rowMinInputIndx, 1) = -T_min * ones(nu, 1);
        end
    end
    
    Gshrink = [GstateShrink; GinputShrink]; 
    hShrink = [hstateShrink; hinputShrink];
    
    % Identify all rows where the constraint limit is finite
    valid_constraint_rows = ~isinf(hShrink);
    
    % Overwrite G and h to only include the valid, finite constraints
    Gshrink = Gshrink(valid_constraint_rows, :);
    hShrink = hShrink(valid_constraint_rows);
    
    %shrink previous Zstar (Your logic here was correct!)
    Zstar = Zstar([nx+1:(N_shrink+2)*nx,(N_shrink+2)*nx+nu+1:end], 1);
end