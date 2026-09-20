function [out, state] = pid_update(gains, err, dt, state, out_min, out_max)

    if nargin < 5, out_min = -Inf; end
    if nargin < 6, out_max =  Inf; end

    Kp = gains(1); Ki = gains(2); Kd = gains(3);

    deriv = (err - state.prev_err) / dt;

    out_unsat = Kp*err + Ki*(state.integral + err*dt) + Kd*deriv;

    if out_unsat > out_max
        out = out_max;
    elseif out_unsat < out_min
        out = out_min;
    else
        out = out_unsat;
        state.integral = state.integral + err*dt;
    end

    state.prev_err = err;

end
