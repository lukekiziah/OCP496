function [X_history,U_history] = simulateShrinkingHorizonLinearMPC(Aeq,beq,N,nx,nu,x0_val,U_QP,sim_time,eval_fContinuous,Q,R,Qf,X_opt_col,U_opt_col,x_max,x_min,T_max,T_min,A_inv,Zstar,Options, m, g, U_ss,K_lqr,shrinkingStepsLeftTransition, SimulateSteps, xf_val)

X_history = zeros(nx,SimulateSteps+1); % nx(N+1)
U_history = zeros(nu,SimulateSteps); % mxN
X_history(:,1) =  x0_val;
current_X = x0_val;

for i = 1:SimulateSteps
    if i < N - shrinkingStepsLeftTransition
        current_U = U_QP(1,:)'; % mx1
    else
        % --- MODE 2: LQR HOVER HOLD ---
        state_error = current_X - xf_val;
        current_U = U_ss - K_lqr * state_error;
    end
    T_span = [sim_time(i), sim_time(i+1)];
    %simulate without disturbances - Expect nearly equivalent reference with errors only due to collocation approximation
    [~, X_interval] = ode45(@(t, X) eval_fContinuous(X, current_U), T_span, current_X);
    %Update the state for the next loop iteration (grab the last row of output)
    current_X = X_interval(end, :)';
    % Store the state
    X_history(:, i+1) = current_X;
    U_history(:,i) = current_U;
    if i<N-shrinkingStepsLeftTransition
        % Setup new QP
        [Hessian_shrinks,linearObjectiveShrinks,AeqShrink,beqShrink,Gshrink,hShrink,Zstar,N_shrink] = setupShrinkingMPCstep(i,N,Q,R,Qf,nx,nu,X_opt_col,U_opt_col,Aeq,beq,x_max,x_min,T_max,T_min,A_inv,Zstar,current_X);
        
        %solve
        [Zstar, ~, exitFlag, ~, lambda] = quadprog(Hessian_shrinks, linearObjectiveShrinks, Gshrink, hShrink, AeqShrink, beqShrink, [], [], Zstar, Options);
     
        %extract inputs and states from optimized QP
        U_QP = (reshape(Zstar(nx*(N_shrink+1)+1:end), nu, N_shrink))'; % is Nshrink by numControlVars
    end
end
end