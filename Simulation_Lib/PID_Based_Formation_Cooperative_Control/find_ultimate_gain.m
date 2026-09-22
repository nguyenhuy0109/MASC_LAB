function [Ku, Pu, info] = find_ultimate_gain(sim_fn, dt, Kp_lo, Kp_hi, opts)

    if nargin < 5, opts = struct(); end
    if ~isfield(opts,'max_expand'), opts.max_expand = 40; end
    if ~isfield(opts,'max_bisect'), opts.max_bisect = 30; end
    if ~isfield(opts,'tol'),        opts.tol = 1e-3; end

   
    growth_hi = classify(sim_fn(Kp_hi));
    n_exp = 0;
    while growth_hi <= 1.02 && n_exp < opts.max_expand
        Kp_hi = Kp_hi * 1.7;
        growth_hi = classify(sim_fn(Kp_hi));
        n_exp = n_exp + 1;
    end

    growth_lo = classify(sim_fn(Kp_lo));
    n_shr = 0;
    while growth_lo >= 0.98 && n_shr < opts.max_expand
        Kp_lo = Kp_lo * 0.6;
        growth_lo = classify(sim_fn(Kp_lo));
        n_shr = n_shr + 1;
    end

    
    for it = 1:opts.max_bisect
        Kp_mid = sqrt(Kp_lo * Kp_hi);   
        g = classify(sim_fn(Kp_mid));
        if g > 1.0
            Kp_hi = Kp_mid;
        else
            Kp_lo = Kp_mid;
        end
        if (Kp_hi/Kp_lo - 1) < opts.tol
            break;
        end
    end

    Ku = sqrt(Kp_lo * Kp_hi);

   
    err = sim_fn(Ku);
    [Pu, is_osc] = measure_period(err, dt);

    info.oscillatory = is_osc;
    info.iterations  = it;

end


function g = classify(err)
    N = length(err);
    i0 = max(1, round(0.2*N));
    seg = err(i0:end);
    M = length(seg);
    if M < 8
        g = 1; return;
    end
    half = floor(M/2);
    rms1 = sqrt(mean(seg(1:half).^2));
    rms2 = sqrt(mean(seg(end-half+1:end).^2));
    if ~isfinite(rms1) || ~isfinite(rms2) || isnan(rms1) || isnan(rms2)
        g = Inf; return;
    end
    if rms1 < 1e-12
        if rms2 < 1e-12
            g = 1;
        else
            g = Inf;
        end
    else
        g = rms2 / rms1;
    end
end


function [Pu, is_osc] = measure_period(err, dt)
    N = length(err);
    i0 = max(1, round(0.5*N));
    seg = err(i0:end);
    s = sign(seg);
    s(s==0) = 1;
    crossings = find(diff(s) ~= 0);
    if length(crossings) >= 2
       
        gaps = diff(crossings) * dt;
        Pu = 2 * mean(gaps);
        is_osc = true;
    else
        Pu = 2*dt;
        is_osc = false;
    end
end
