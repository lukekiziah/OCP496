function [Qh,Rh] = bigWeighting(Q,R,Qf,N)
%%%%% Notes %%%%%%
% bigWeighting concatenates weighting matrices over the horizon
% for an objective J=X'QhX+U'RhU using single shooting, REQUIRES Q

%%%% Inputs %%%%%
% Q is nxn state error weighting matrix applied to x_(k+1):x_(k+N-1)
% R is mxm control effort weighting matrix applied through horizon u_k:u_(k+N-1)
% Qf is nxn terminal state error weighting matrix applied to x_(k+N)
% N is the scalar horizon

%%%% Outputs %%%%%%
% Qh is (N*n)x(N*n) the concatenated state weighting matrix over trajectory
% Rh is (N*m)x(N*m) the concatenated control effort weighting matrix over trajectory

n = size(Q,1); % number of state variables
m = size(R,1); % number of inputs
Qh = zeros(N*n,N*n);
Rh = zeros(N*m,N*m);
for r = 1:N
    for c = 1:N
        if c==r
            Rh((r-1)*m+1:r*m,(c-1)*m+1:c*m) = R;
            if c == N
                Qh((r-1)*n+1:r*n,(c-1)*n+1:c*n) = Qf;
            else
                Qh((r-1)*n+1:r*n,(c-1)*n+1:c*n) = Q;
            end
        end
    end
end
end