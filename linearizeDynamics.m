function [dfdx, dfdu, f_sym, xvec, uvec] = linearizeDynamics(eq_cells, numstatevars, numcontrolvars)
    % linearizeDynamics Linearizes a system of nonlinear equations of motion.
    % 
    % Inputs:
    %   eq_cells       - nx1 cell array of chars or string array containing EOMs
    %   numstatevars   - Integer number of state variables (x)
    %   numcontrolvars - Integer number of control variables (u)
    %
    % Outputs:
    %   dfdx  - Symbolic Jacobian matrix of f with respect to x (A matrix)
    %   dfdu  - Symbolic Jacobian matrix of f with respect to u (B matrix)
    %   f_sym - The symbolic column vector of the dynamics
    %   xvec  - Symbolic state vector [x1; x2; ...]
    %   uvec  - Symbolic control vector [u1; u2; ...]

    % 1. Create symbolic state and control vectors dynamically
    % This automatically generates column vectors [x1; x2; ...] and [u1; u2; ...]
    xvec = sym('x', [numstatevars, 1]);
    uvec = sym('u', [numcontrolvars, 1]);

    % 2. Convert the cell array of strings into symbolic expressions
    % str2sym will automatically recognize 'x1', 'u1', etc., as symbolic variables
    f_sym = str2sym(eq_cells);

    % Ensure f_sym is a column vector to match the dimensions of xvec and uvec
    f_sym = f_sym(:);

    % 3. Compute the Jacobians
    dfdx = jacobian(f_sym, xvec);
    dfdu = jacobian(f_sym, uvec);
end