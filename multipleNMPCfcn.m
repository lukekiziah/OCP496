function [Usol, Xsol] = multipleNMPCfcn(prob)
    try
        N = prob.N;
        n = prob.n;          % number of states
        m = prob.m;          % number of controls
        X0 = prob.x0;

        % Dynamics setup
        FullEOM = prob.eq_cells;
        [FullEOM_sym, stateVec, uVec, constVec] = makeEOMsymbolic(FullEOM, n, m, prob.const_names);
        FullEOM_sym = subs(FullEOM_sym, constVec, str2double(prob.const_vals));

        if strcmp(prob.timeType, 'Continuous')
            [symDiscreteAk, symDiscreteBk, fullEOMdiscrete] = discreteLinearization(FullEOM_sym, stateVec, uVec, 'ForwardEuler', prob.dt);
        else
            fullEOMdiscrete = FullEOM_sym;
            symDiscreteAk = jacobian(fullEOMdiscrete, stateVec);
            symDiscreteBk = jacobian(fullEOMdiscrete, uVec);
        end

        eval_fullEOMdiscrete = matlabFunction(fullEOMdiscrete, 'Vars', {stateVec, uVec});
        eval_discreteAk = matlabFunction(symDiscreteAk, 'Vars', {stateVec, uVec});
        eval_discreteBk = matlabFunction(symDiscreteBk, 'Vars', {stateVec, uVec});

        % Objective
        Q = zeros(n,n);
        if prob.useQ
            for i = 1:n
                Q(i,i) = prob.Q_diag(i,1);
            end
        end
        if prob.useQf
            Qf = zeros(n,n);
            for i = 1:n
                Qf(i,i) = prob.Qf_diag(i,1);
            end
        else
            Qf = 10*Q;
        end
        R = zeros(m,m);
        if prob.useR
            for i = 1:m
                R(i,i) = prob.R_diag(i,1);
            end
        end

        [Qh, Rh] = bigWeighting(Q, R, Qf, N);
        H = multipleShootingH(n, Qh, Rh, Q);
        H = H + 1e-2 * eye(size(H));   % regularization

        TerminalState = [];
        if prob.useTerminal
            TerminalState = str2double(prob.x_terminal);
        end
        StateMin = str2double(prob.x_min);
        StateMax = str2double(prob.x_max);
        InputMin = str2double(prob.u_min);
        InputMax = str2double(prob.u_max);
        % === BUILD LINEAR INEQUALITY CONSTRAINTS G z <= h ===
        % Decision vector Z = [x0; x1; ...; xN; u0; ...; u_{N-1}]
        % Total variables: n*(N+1) + m*N

        G = [];
        h = [];

        % 1. State bounds (applied to x1, x2, ..., xN — x0 is fixed by equality)
        if any(isfinite(StateMin)) || any(isfinite(StateMax))
            numStateIneq = sum(isfinite(StateMin)) + sum(isfinite(StateMax));
            if numStateIneq > 0
                G_states = zeros(numStateIneq, n*(N+1) + m*N);
                h_states = zeros(numStateIneq, 1);
                row = 0;

                for k = 1:N   % for each node x1 to xN
                    x_start_col = (k * n) + 1;   % column index of x_k in Z

                    for i = 1:n   % for each state variable
                        % Lower bound:  -x_{k,i}  <= -StateMin(i)   =>   G row = -e_i
                        if isfinite(StateMin(i))
                            row = row + 1;
                            G_states(row, x_start_col + i - 1) = -1;
                            h_states(row,1) = -StateMin(i);
                        end

                        % Upper bound:   x_{k,i}  <=  StateMax(i)
                        if isfinite(StateMax(i))
                            row = row + 1;
                            G_states(row, x_start_col + i - 1) = 1;
                            h_states(row,1) = StateMax(i);
                        end
                    end
                end
                G = [G; G_states];
                h = [h; h_states];
            end
        end

        % 2. Input bounds (applied to u0, u1, ..., u_{N-1})
        if any(isfinite(InputMin)) || any(isfinite(InputMax))
            numInputIneq = sum(isfinite(InputMin)) + sum(isfinite(InputMax));
            if numInputIneq > 0
                G_inputs = zeros(numInputIneq, n*(N+1) + m*N);
                h_inputs = zeros(numInputIneq, 1);
                row = 0;

                for k = 0:N-1   % for each control u_k
                    u_start_col = n*(N+1) + (k * m) + 1;   % column index of u_k in Z

                    for j = 1:m
                        % Lower bound:  -u_{k,j}  <= -InputMin(j)
                        if isfinite(InputMin(j))
                            row = row + 1;
                            G_inputs(row, u_start_col + j - 1) = -1;
                            h_inputs(row) = -InputMin(j);
                        end

                        % Upper bound:   u_{k,j}  <=  InputMax(j)
                        if isfinite(InputMax(j))
                            row = row + 1;
                            G_inputs(row, u_start_col + j - 1) = 1;
                            h_inputs(row) = InputMax(j);
                        end
                    end
                end
                G = [G; G_inputs];
                h = [h; h_inputs];
            end
        end

        % LQR initial guess (your version)
        x_eq = zeros(n,1);
        u_eq = zeros(m,1);
        A0 = eval_discreteAk(x_eq, u_eq);
        B0 = eval_discreteBk(x_eq, u_eq);
        K = DiscreteFiniteHorizonLQR(A0,B0,Q,R,Qf,N);
        %send(q, sprintf('LQR gain computed successfully (norm(K) = %.2f)', norm(K,'fro')));
        fprintf('LQR gain computed successfully (norm(K) = %.2f)\n', norm(K,'fro'))
        Zguess = sim_horizonK_on_fd(eval_fullEOMdiscrete, K, N, X0);

        % Merit
        mu = 10;
        [CurrentObjCost, sum_gaps, ~] = evaluateNonlinearGaps(Zguess, H, eval_fullEOMdiscrete, X0, prob, N, n, m, TerminalState);
        CurrentMeritCost = CurrentObjCost + mu * sum_gaps;

        Options = optimoptions('quadprog', 'Display', 'off');
        %Options = [];

        tol_grad = 1e-6;
        tol_eq   = 1e-6;
        tol_step = 1e-6;
        MAXiterations = 1000;

        for numSQPsteps = 1:MAXiterations
            % Linearize
            allA = zeros(n,n,N);
            allB = zeros(n,m,N);
            allCk = zeros(n,N);
            for k = 1:N
                xk = Zguess((k-1)*n + 1 : k*n);
                uk = Zguess(n*(N+1) + (k-1)*m + 1 : n*(N+1) + k*m);
                allA(:,:,k) = eval_discreteAk(xk, uk);
                allB(:,:,k) = eval_discreteBk(xk, uk);
                allCk(:,k) = eval_fullEOMdiscrete(xk, uk) - allA(:,:,k)*xk - allB(:,:,k)*uk;
            end

            [AeqDynamics, beqDynamics] = multipleShootingDynamicConstraints(allA, allB, allCk, X0, N);
            if prob.useTerminal
                [Atc, btc] = multipleTerminalConstraint(TerminalState, N, m);
                Aeq = [AeqDynamics; Atc];
                beq = [beqDynamics; btc];
            else
                Aeq = AeqDynamics;
                beq = beqDynamics;
            end

            % QP
            [Zstar, ~, exitFlag, ~, lambda] = quadprog(H, [], G, h, Aeq, beq, [], [], Zguess, Options);

            if exitFlag <= 0
                %if nargin > 1 && ~isempty(q)
                    %send(q, sprintf('QP failed (exitFlag = %d)', exitFlag));
                %end
                fprintf('QP failed (exitFlag = %d)\n', exitFlag)
                break;
            end

            max_lambda_eq = max(norm(lambda.eqlin, 'inf'));
            if ~isempty(lambda.ineqlin)
                max_lambda_ineq = max(norm(lambda.ineqlin,'inf'));
            else
                max_lambda_ineq = 0;
            end
            max_lambda = max([max_lambda_ineq,max_lambda_eq]);
            mu = max(mu, max_lambda + 0.1);

            % === IMPROVED LINE SEARCH - prevents huge steps ===
            learningRate = 1.0;
            betterMerit = false;
            for ls = 1:20   % more attempts
                Ztrial = Zguess + learningRate * (Zstar - Zguess);
                [trialObj, trial_sum_gaps, trial_max_gap] = evaluateNonlinearGaps(Ztrial, H, eval_fullEOMdiscrete, X0, prob, N, n, m, TerminalState);
                NewMeritCost = trialObj + mu * trial_sum_gaps;

                if NewMeritCost < CurrentMeritCost - 1e-4 * learningRate * abs(CurrentMeritCost)
                    Zguess = Ztrial;
                    CurrentMeritCost = NewMeritCost;
                    eq_res = trial_max_gap;
                    betterMerit = true;
                    break;
                end
                learningRate = learningRate * 0.5;
            end

            if ~betterMerit
                % Take a small safe step instead of rejecting completely
                learningRate = 0.1;   % conservative step when line search fails
                Zguess = Zguess + learningRate * (Zstar - Zguess);
                [~, trial_sum_gaps, trial_max_gap] = evaluateNonlinearGaps(Zguess, H, eval_fullEOMdiscrete, X0, prob, N, n, m, TerminalState);
                CurrentMeritCost = 0.5*Zguess'*H*Zguess + mu * trial_sum_gaps;
                eq_res = trial_max_gap;
                % if nargin > 1 && ~isempty(q)
                %     send(q, sprintf('Line search failed - taking small safe step (alpha=0.1), mu=%.2e', mu));
                % end
                fprintf('Line search failed - taking small safe step (alpha=0.1), mu=%.2e\n', mu)
            end

            % Convergence
            if isempty(G)
                gradL_res = norm(H*Zguess + Aeq'*lambda.eqlin, 'inf');
            else
                gradL_res = norm(H*Zguess + Aeq'*lambda.eqlin +G'*lambda.ineqlin, 'inf');
            end
            step_res = norm(Zstar - Zguess, 'inf');   % use attempted step size

            %if nargin > 1 && ~isempty(q)
            if mod(numSQPsteps,10)==0 || numSQPsteps <= 5
                % send(q, sprintf('SQP %d: opt->%.2f eq->%.2f delta->%.2f | abs_grad=%.2e abs_eq=%.2e', ...
                %     numSQPsteps, gradL_res/tol_grad, eq_res/tol_eq, step_res/tol_step, gradL_res, eq_res));
                fprintf('SQP %d: opt->%.2f eq->%.2f delta->%.2f | abs_grad=%.2e abs_eq=%.2e\n',numSQPsteps, gradL_res/tol_grad, eq_res/tol_eq, step_res/tol_step, gradL_res, eq_res)
            end
            %end

            if gradL_res < tol_grad && eq_res < tol_eq && step_res < tol_step
                % if nargin > 1 && ~isempty(q)
                %     send(q, 'Converged successfully!');
                % end
                fprintf('Converged successfully!\n')
                break;
            end
        end

        % Extract results
        Usol = reshape(Zguess(n*(N+1)+1:end), m, N)';
        Xsol = reshape(Zguess(1:n*(N+1)), n, N+1)';

    catch ME
        % if nargin > 1 && ~isempty(q)
        %     send(q, sprintf('ERROR in multipleNMPC: %s', ME.message));
        % end
        fprintf('ERROR in multipleMNMPC: %s\n',ME.message);
        disp(ME.message);
        disp(ME.stack);
    end
end
%% --- LOCAL HELPER (unchanged) ---
function [objCost, sum_gaps, max_gap] = evaluateNonlinearGaps(Z, H, eval_fullEOMdiscrete, X0, prob, N, NumStateVars, NumControlVars, TerminalState)
    objCost = 0.5 * Z' * H * Z;
    X = Z(1:NumStateVars*(N+1));
    U = Z(NumStateVars*(N+1)+1:end);
    sum_gaps = 0;
    max_gap = 0;
    gap0 = X(1:NumStateVars) - X0;
    sum_gaps = sum_gaps + sum(abs(gap0));
    max_gap = max(max_gap, max(abs(gap0)));
    for k = 1:N
        x_k = X((k-1)*NumStateVars+1 : k*NumStateVars);
        u_k = U((k-1)*NumControlVars+1 : k*NumControlVars);
        x_kp1 = X(k*NumStateVars+1 : (k+1)*NumStateVars);
        gap_k = eval_fullEOMdiscrete(x_k, u_k) - x_kp1;
        sum_gaps = sum_gaps + sum(abs(gap_k));
        max_gap = max(max_gap, max(abs(gap_k)));
    end
    if prob.useTerminal
        gapT = TerminalState - X(end-NumStateVars+1:end);
        sum_gaps = sum_gaps + sum(abs(gapT));
        max_gap = max(max_gap, max(abs(gapT)));
    end
end