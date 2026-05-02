function dyn = HermiteSimpsonCollocation(xk,xk1,uk,uk1,f,dt)
% Dynamics at endpoints
fk  = f(xk,  uk);
fk1 = f(xk1, uk1);

% 1. Calculate the Midpoint State (Hermite Interpolation)
% This is the predicted state at the center of the interval
xc = 0.5*(xk + xk1) + (dt/8)*(fk - fk1);

% 2. Calculate the Midpoint Control (Linear Interpolation)
uc = 0.5*(uk + uk1); 

% 3. Evaluate dynamics at the midpoint
fc = f(xc, uc);

% 4. Simpson's Quadrature Constraint
% This forces the interval to follow a cubic spline
dyn = xk - xk1 + (dt/6)*(fk + 4*fc + fk1);
end