function [symDiscreteAk, symDiscreteBk,f_discrete] = discreteLinearization(fContinuous_sym, xVec, uVec, IntegratorType, dt)
    % numericalLinearization Computes the discrete-time numerical Jacobians
    %
    % Inputs:
    %   fContinuous_sym - Symbolically defined vector expression for continuous dynamics (xdot = f(x,u))
    %   xVec            - Symbolic vector with the names of each state variable
    %   uVec            - Symbolic vector with the names of each control variable
    %   Xbar            - Numeric column vector defining the state linearization point
    %   Ubar            - Numeric column vector defining the control linearization point
    %   IntegratorType  - 'ForwardEuler', 'RK2', or 'RK4'
    %   dt              - Numeric time step for the discretization
    %
    % Outputs:
    %   symDiscreteAk      - symbolic discrete-time state Jacobian (A matrix)
    %   symDiscreteBk      - symbolic discrete-time input Jacobian (B matrix)
    
    % 1. Formulate the explicit discrete-time nonlinear dynamics symbolically
    % x_{k+1} = f_discrete(x_k, u_k)
    switch IntegratorType
        case 'ForwardEuler'
            f_discrete = xVec + fContinuous_sym * dt;
            
        case 'RK2' % Also known as Midpoint or Heun's method
            k1 = fContinuous_sym;
            
            % Evaluate continuous dynamics at (x_k + k1*dt/2)
            k2 = subs(fContinuous_sym, xVec, xVec + k1 * (dt / 2));
            
            f_discrete = xVec + k2 * dt;
            
        case 'RK4' % Standard 4th Order Runge-Kutta
            k1 = fContinuous_sym;
            k2 = subs(fContinuous_sym, xVec, xVec + k1 * (dt / 2));
            k3 = subs(fContinuous_sym, xVec, xVec + k2 * (dt / 2));
            k4 = subs(fContinuous_sym, xVec, xVec + k3 * dt);
            
            f_discrete = xVec + (dt / 6) * (k1 + 2*k2 + 2*k3 + k4);
            
        otherwise
            error('Invalid IntegratorType. Choose ''ForwardEuler'', ''RK2'', or ''RK4''.');
    end
    
    % 2. Compute the exact symbolic Jacobians of the DISCRETE dynamics
    symDiscreteAk = jacobian(f_discrete, xVec);
    symDiscreteBk = jacobian(f_discrete, uVec);
end