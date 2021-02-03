function Ydot = computeOpenSimModelDerivatives(t, y, model, state)
% Compute the dynamics of the OpenSim model and return the derivative of 
% all its continuous state variables.

ts = state.updTime();
ts = t;
state.setY(y);
Ydot = model.computeStateVariableDerivatives(state);

end

