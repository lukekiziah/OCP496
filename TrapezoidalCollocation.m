function dyn = TrapezoidalCollocation(xk,xk1,uk,f,dt)
fk  = f(xk,  uk);
fk1 = f(xk1, uk);

dyn = xk1 - (xk + dt/2*(fk + fk1));
end