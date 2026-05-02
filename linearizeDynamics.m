function [dfdx, dfdu, f_sym, xvec, uvec, constvec] = linearizeDynamics(eq_cells, numstatevars, numcontrolvars, const_cells)
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

    % 1. Create symbolic state and control vectors
    xvec = sym('x', [numstatevars, 1]);
    uvec = sym('u', [numcontrolvars, 1]);
    
    % 2. Create the symbolic constants vector
    % The sym() function can take a cell array of chars/strings and 
    % instantly convert it into a vector of symbolic variables.
    if nargin > 3 && ~isempty(const_cells)
        constvec = sym(const_cells);
        constvec = constvec(:); % Ensure it's a column vector
    else
        constvec = sym([]); % Empty if no constants provided
    end

    % 3. Convert the cell array of strings into symbolic expressions
    % str2sym will automatically match 'm', 'g', etc. to the variables in constvec
    f_sym = str2sym(eq_cells);
    f_sym = f_sym(:);

    % 4. Compute the Jacobians
    dfdx = jacobian(f_sym, xvec);
    dfdu = jacobian(f_sym, uvec);
end