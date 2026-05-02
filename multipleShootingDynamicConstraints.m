function [Aeq, beq] = multipleShootingDynamicConstraints(allA, allB, allCk, xinitial, N)
    % Inputs:
    % allA    - (n x n x N) 3D array of linearizations
    % allB    - (n x m x N) 3D array of linearizations
    % allCk   - (n x N) Constant offset terms from linearization
    % xinitial - (n x 1) The current measured state of the plant
    % N       - Scalar horizon length
    
    n = size(allA, 1);
    m = size(allB, 2);
    
    % Decision vector Z is [x0; x1; ...; xN; u0; ...; uN-1]
    % Total variables: n*(N+1) + m*N
    numVars = n*(N+1) + m*N;
    % Total equations: n*(N+1)  (One for x0 constraint, N for dynamics)
    numEqs = n*(N+1);
    
    Aeq = zeros(numEqs, numVars);
    beq = zeros(numEqs, 1);
    
    % 1. Initial State Constraint: x0 = xinitial
    % Corresponds to: [I 0 0 ... 0] * Z = xinitial
    Aeq(1:n, 1:n) = eye(n);
    beq(1:n) = xinitial;
    
    % 2. Dynamics Constraints: -A_k*x_k + I*x_{k+1} - B_k*u_k = c_k
    for k = 1:N
        rowIdx = (k*n) + 1 : (k+1)*n;
        
        % Indices for x_k and x_{k+1}
        xk_colIdx = ((k-1)*n) + 1 : k*n;
        xnext_colIdx = (k*n) + 1 : (k+1)*n;
        
        % Indices for u_k
        % Note: u_k starts after all (N+1) states
        uk_colIdx = (n*(N+1)) + (k-1)*m + 1 : (n*(N+1)) + k*m;
        
        % Fill Aeq
        Aeq(rowIdx, xk_colIdx) = -allA(:,:,k);      % -Ak
        Aeq(rowIdx, xnext_colIdx) = eye(n);         %  I
        Aeq(rowIdx, uk_colIdx) = -allB(:,:,k);      % -Bk
        
        % Fill beq
        beq(rowIdx) = allCk(:, k);
    end
end