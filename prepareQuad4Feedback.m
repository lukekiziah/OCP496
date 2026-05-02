function [eval_fContinuous,eval_continuousA,eval_continuousB, eval_fDiscrete,eval_discreteAk,eval_discreteBk,FullEOM_sym,constVec,stateVec,uVec] = prepareQuad4Feedback(constVals,IntegratorType,dt)
NumStateVars = 12; NumControlVars = 4;
ConstCell = {'m';'g';'L';'Ixx';'Iyy';'Izz';'C11';'C22';'C33';'D11';'D22';'D33';'c1';'c2';'c3';'c4'};

% Define translational dynamics f4, f5, f6
f4 = '(1/m) * ( u1 * (sin(x7)*sin(x9) + cos(x7)*cos(x9)*sin(x8)) - x4 * ( C22*(cos(x7)*sin(x9) - cos(x9)*sin(x7)*sin(x8))^2 + C33*(sin(x7)*sin(x9) + cos(x7)*cos(x9)*sin(x8))^2 + C11*(cos(x9)*cos(x8))^2 ) + x5 * ( C22*(cos(x7)*cos(x9) + sin(x7)*sin(x9)*sin(x8))*(cos(x7)*sin(x9) - cos(x9)*sin(x7)*sin(x8)) + C33*(cos(x9)*sin(x7) - cos(x7)*sin(x9)*sin(x8))*(sin(x7)*sin(x9) + cos(x7)*cos(x9)*sin(x8)) - C11*cos(x8)*sin(x9)*cos(x9)*cos(x8) ) + x6 * ( C11*sin(x8)*cos(x9)*cos(x8) - C33*cos(x7)*cos(x8)*(sin(x7)*sin(x9) + cos(x7)*cos(x9)*sin(x8)) + C22*cos(x8)*sin(x7)*(cos(x7)*sin(x9) - cos(x9)*sin(x7)*sin(x8)) ) )';
f5 = '(1/m) * ( -u1 * (cos(x9)*sin(x7) - cos(x7)*sin(x9)*sin(x8)) + x4 * ( C22*(cos(x7)*sin(x9) - cos(x9)*sin(x7)*sin(x8))*(cos(x7)*cos(x9) + sin(x7)*sin(x9)*sin(x8)) + C33*(sin(x7)*sin(x9) + cos(x7)*cos(x9)*sin(x8))*(cos(x9)*sin(x7) - cos(x7)*sin(x9)*sin(x8)) - C11*cos(x9)*cos(x8)*cos(x8)*sin(x9) ) - x5 * ( C22*(cos(x7)*cos(x9) + sin(x7)*sin(x9)*sin(x8))^2 + C33*(cos(x9)*sin(x7) - cos(x7)*sin(x9)*sin(x8))^2 + C11*(cos(x8)*sin(x9))^2 ) + x6 * ( C33*cos(x7)*cos(x8)*(cos(x9)*sin(x7) - cos(x7)*sin(x9)*sin(x8)) - C22*cos(x8)*sin(x7)*(cos(x7)*cos(x9) + sin(x7)*sin(x9)*sin(x8)) + C11*sin(x8)*cos(x8)*sin(x9) ) )';
f6 = '(1/m) * ( u1*cos(x7)*cos(x8) - g*m - x4 * ( -C11*cos(x9)*cos(x8)*sin(x8) - C22*cos(x7)*sin(x9)*cos(x8)*sin(x7) + C33*sin(x7)*sin(x9)*cos(x7)*cos(x8) + C33*cos(x7)*cos(x9)*sin(x8)*cos(x7)*cos(x8) + C22*cos(x9)*sin(x7)*sin(x8)*cos(x8)*sin(x7) ) - x5 * ( -C11*cos(x8)*sin(x9)*sin(x8) + C22*cos(x7)*cos(x9)*cos(x8)*sin(x7) - C33*cos(x9)*sin(x7)*cos(x7)*cos(x8) + C33*cos(x7)*sin(x9)*sin(x8)*cos(x7)*cos(x8) + C22*sin(x7)*sin(x9)*sin(x8)*cos(x8)*sin(x7) ) - x6 * ( C11*sin(x8)^2 + C33*(cos(x7)*cos(x8))^2 + C22*(cos(x8)*sin(x7))^2 ) )';

FULLEOM = {'x4';'x5';'x6';
    f4; f5; f6;
    'x10 + x11*sin(x7)*tan(x8) + x12*cos(x7)*tan(x8)';
    'x11*cos(x7) - x12*sin(x7)';
    'x11*sin(x7)*sec(x8) + x12*cos(x7)*sec(x8)';
    '(1/Ixx)*(u2 - D11*x10 + Iyy*x11*x12)';
    '(1/Iyy)*(u3 - D22*x11 - Ixx*x10*x12 + Izz*x10*x12)';
    '(1/Izz)*(u4 - D33*x12 + Ixx*x10*x11)'};


[dfdx_continuous, dfdu_continuous, FullEOM_sym, stateVec, uVec, constVec] = linearizeDynamics(FULLEOM, NumStateVars, NumControlVars, ConstCell);
symA = subs(dfdx_continuous,constVec,full(constVals)); %USE symA and symB for subing x,u linearizations
symB = subs(dfdu_continuous,constVec,constVals); %symA and symB are the continuous A and B matrices
eval_continuousA = matlabFunction(symA,'Vars',{stateVec, uVec}); %define linearizations as functions
eval_continuousB = matlabFunction(symB,'Vars',{stateVec, uVec});

FullEOM_sym = subs(FullEOM_sym,constVec,constVals);
eval_fContinuous = matlabFunction(FullEOM_sym,'Vars', {stateVec, uVec}); %efficiently evaluate full dynamics at x,u
[symDiscreteAk, symDiscreteBk, fDiscrete] = discreteLinearization(FullEOM_sym, stateVec, uVec, IntegratorType, dt);
eval_discreteAk = matlabFunction(symDiscreteAk,'Vars', {stateVec, uVec}); %define linearizations as functions
eval_discreteBk = matlabFunction(symDiscreteBk,'Vars', {stateVec, uVec});
eval_fDiscrete = matlabFunction(fDiscrete,'Vars',{stateVec, uVec});

end