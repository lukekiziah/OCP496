function H = multipleShootingH(NumStateVars, Qh, Rh, Q)
    % MULTIPLESHOOTINGH Creates the Hessian matrix for Multiple Shooting MPC
    %
    % Inputs:
    % NumStateVars - (n) Number of state variables
    % Qh           - (n*N x n*N) Concatenated state weighting matrix (x1 to xN)
    % Rh           - (m*N x m*N) Concatenated control weighting matrix
    % Q            - (unused - kept for backward compatibility)
    %
    % Output:
    % H - The large block-diagonal Hessian matrix
    %     [ 0     0    0 ]
    %     [ 0    Qh    0 ]
    %     [ 0     0   Rh ]
    H_initial = zeros(NumStateVars, NumStateVars);  % x0 is fixed -> no cost on x0
    H = 2 * blkdiag(H_initial, Qh, Rh);
    % Optional: sparse for large N (N=50 is still tiny)
    % H = sparse(H);
end