%% tune_IB_SMC_PID.m
% =========================================================================
% Tu dong do bo so PID cho model IB_SMC.slx (cascade: PID Position ->
% PID Attitude(outer angle) -> PID Attitude(inner rate)) bang toi uu hoa
% dua tren mo phong (simulation-based optimization).
%
% CACH DUNG:
%   1) Mo/copy file nay vao cung thu muc voi IB_SMC.slx (hoac sua bien MDL_FILE).
%   2) Dam bao cac bien workspace ma model can de chay duoc BINH THUONG
%      (m, g, Iv, kt, kd, Lroll, Lpitch, Ir, k123, k456, drag, gyro_eff,
%      aero_friction, va cac gia tri KHOI TAO cua Kp_pos/Ki_pos/... ) da
%      ton tai trong base workspace (vi du qua mot script init rieng cua ban).
%      Neu chua co, script se gan gia tri mac dinh o Section 1 (SUA LAI CHO PHU HOP!).
%   3) Chinh BOUNDS o Section 3 cho phu hop voi thang do vat ly cua may bay ban.
%   4) Chay script. Ket qua toi uu se duoc gan vao workspace + luu file .mat
%      + ve do thi so sanh truoc/sau.
%
% LUU Y QUAN TRONG (rat nen doc):
%   - Script gia dinh moi khoi Gain (Kp_att, Ki_att, Kd_att, Kp_att_dot,
%     Ki_att_dot, Kd_att_dot, Kp_pos, Ki_pos, Kd_pos) la BIEN WORKSPACE
%     dung chung cho ca 3 truc (dung nhu trong file .slx ban da upload).
%     Neu ban tach rieng gain cho tung truc (vd Kp_att = [Kp_phi Kp_theta Kp_psi]),
%     hay sua VAR_NAMES/ so bien thiet ke (nvars) va cach gan gia tri cho phu hop.
%   - Model hien tai KHONG co anti-windup tren cac khoi Integrator va dung
%     Derivative ly tuong (khong loc) tren sai so. Toi uu hoa co the "ne"
%     duoc mot phan van de nay bang cach chon Ki/Kd nho, nhung ve lau dai
%     BAN NEN SUA TRUC TIEP TRONG MODEL (xem ghi chu cuoi file).
%   - Script nay CHUA duoc chay thu tren MATLAB/Simulink that (moi truong
%     nay khong co Simulink) - hay chay thu voi so vong lap nho truoc
%     (POP_SIZE, MAX_ITER nho) de kiem tra khong loi cu phap/duong dan
%     truoc khi chay toi uu day du.
% =========================================================================

clear; clc;

%% ------------------- Section 0: Cau hinh model -------------------------
MDL_FILE = 'IB_SMC';   % ten model (khong co .slx)
if ~bdIsLoaded(MDL_FILE)
    load_system(MDL_FILE);
end
mdl = MDL_FILE;

% Duong dan cac khoi quan trong (SUA LAI neu ten khoi trong model cua ban khac)
BLK_UAV      = [mdl '/UAV Model'];      % out1=Position, out2=att_dot, out3=att
BLK_PIDPOS   = [mdl '/PID Position'];   % out1=thrust,  out2=att_r
BLK_PIDATT   = [mdl '/PID Attitude '];  % out1=Torque   (chu y co dau cach cuoi ten, giong file goc)
BLK_REF_MUX  = [mdl '/Mux1'];           % out1 = [x_ref;y_ref;z_ref;psi_ref]

T_SIM = 12;   % thoi gian mo phong khi tune (s) - nen >= thoi gian quy dao ban muon test

%% ------------------- Section 1: Gia tri vat ly / gain ban dau -----------
% Neu cac bien nay DA co san trong base workspace (vi du ban co script
% init rieng), COMMENT phan gan gia tri mac dinh ben duoi de khoi ghi de.
defaultsNeeded = {'m','g','Iv','kt','kd','Lroll','Lpitch'};
for i = 1:numel(defaultsNeeded)
    if ~evalin('base', ['exist(''' defaultsNeeded{i} ''',''var'')'])
        warning('Bien "%s" chua co trong base workspace - dang gan gia tri MAC DINH tam thoi. Hay kiem tra lai cho dung may bay cua ban!', defaultsNeeded{i});
    end
end
% Gia tri mac dinh THAM KHAO (theo thang do trong bai bao Processes 2021,9,1951)
% SUA LAI cho dung thong so may bay/thang do luc-momen cua ban (vd Command
% Authority ~1 Nm trong model cua ban lon hon nhieu so voi bai bao => co
% the may bay cua ban lon hon, hoac don vi khac).
assignin_if_missing('m',       1.121);
assignin_if_missing('g',       9.81);
assignin_if_missing('Iv',      diag([1e-2, 8.2e-3, 1.48e-2]));
assignin_if_missing('kt',      5.11);
assignin_if_missing('kd',      0.0487);
assignin_if_missing('Lroll',   0.2136);
assignin_if_missing('Lpitch',  0.1758);

% Gia tri KHOI DIEM cho toi uu hoa (dung gia tri hien co trong model neu co,
% neu khong dung gia tri tham khao ben duoi)
x0 = get_initial_gains();

%% ------------------- Section 2: Thiet lap Signal Logging ----------------
enableSignalLogging(BLK_UAV,     1, 'log_pos_actual');
enableSignalLogging(BLK_UAV,     3, 'log_att_actual');
enableSignalLogging(BLK_PIDPOS,  2, 'log_att_ref');
enableSignalLogging(BLK_REF_MUX, 1, 'log_pos_ref');
enableSignalLogging(BLK_PIDATT,  1, 'log_torque_cmd');
enableSignalLogging(BLK_PIDPOS,  1, 'log_thrust_cmd');

set_param(mdl, 'SignalLogging', 'on');
set_param(mdl, 'SignalLoggingName', 'logsout');
set_param(mdl, 'StopTime', num2str(T_SIM));

%% ------------------- Section 3: Bien thiet ke va gioi han ---------------
% Thu tu bien: [Kp_pos Ki_pos Kd_pos  Kp_att Ki_att Kd_att  Kp_att_dot Ki_att_dot Kd_att_dot]
varNames = {'Kp_pos','Ki_pos','Kd_pos', ...
            'Kp_att','Ki_att','Kd_att', ...
            'Kp_att_dot','Ki_att_dot','Kd_att_dot'};
nvars = numel(varNames);

% GIOI HAN TIM KIEM - SUA LAI cho phu hop thang do he thong cua ban.
% Cac gia tri duoi day chi la khoang rong mang tinh khoi diem.
lb = [0.2   0    0     ...   % Kp_pos Ki_pos Kd_pos
      1.0   0    0     ...   % Kp_att Ki_att Kd_att
      0.001 0    0    ];     % Kp_att_dot Ki_att_dot Kd_att_dot
ub = [20    5    10    ...
      30    8    8     ...
      2     1    0.6  ];

% Dam bao x0 nam trong bien (neu khong, keo vao)
x0 = min(max(x0, lb), ub);

%% ------------------- Section 4: Toi uu hoa ------------------------------
costFcn = @(x) simCost(x, mdl, T_SIM, varNames);

optsFound = false;
bestX = x0; bestCost = costFcn(x0);
fprintf('Cost tai gia tri khoi diem: %.4f\n', bestCost);

% --- Uu tien 1: particleswarm (Global Optimization Toolbox) ---
if exist('particleswarm', 'file') == 2
    try
        popSize  = 24;    % tang len (vd 40-60) neu may du manh / co Parallel Toolbox
        maxIter  = 40;    % tang len de tim nghiem tot hon, giam de test nhanh
        useParallel = (exist('parpool','file') == 2);

        psOpts = optimoptions('particleswarm', ...
            'SwarmSize', popSize, ...
            'MaxIterations', maxIter, ...
            'Display', 'iter', ...
            'UseParallel', useParallel, ...
            'InitialSwarmMatrix', repmat(x0, popSize, 1) + ...
                (rand(popSize, nvars) - 0.5) .* (ub - lb) * 0.3);

        [bestX, bestCost] = particleswarm(costFcn, nvars, lb, ub, psOpts);
        optsFound = true;
        fprintf('Da toi uu bang particleswarm.\n');
    catch ME
        warning('particleswarm loi: %s. Chuyen sang phuong an du phong.', ME.message);
    end
end

% --- Uu tien 2: patternsearch (Optimization Toolbox) ---
if ~optsFound && exist('patternsearch', 'file') == 2
    try
        psearchOpts = optimoptions('patternsearch', ...
            'Display', 'iter', 'MaxIterations', 200, 'UseCompletePoll', true);
        [bestX, bestCost] = patternsearch(costFcn, x0, [], [], [], [], lb, ub, [], psearchOpts);
        optsFound = true;
        fprintf('Da toi uu bang patternsearch.\n');
    catch ME
        warning('patternsearch loi: %s. Chuyen sang phuong an du phong.', ME.message);
    end
end

% --- Uu tien 3: fminsearch da khoi tao (khong can Optimization Toolbox) ---
if ~optsFound
    warning('Khong tim thay particleswarm/patternsearch. Dung fminsearch (multi-start) - it manh hon, nen cai Optimization/Global Optimization Toolbox neu co the.');
    nStarts = 6;
    fmsOpts = optimset('Display', 'iter', 'MaxIter', 150, 'MaxFunEvals', 400);
    penalizedCost = @(x) costFcn(clampBounds(x, lb, ub)) + 1e3 * sum(max(0, lb - x).^2 + max(0, x - ub).^2);
    for s = 1:nStarts
        xs = lb + rand(1, nvars) .* (ub - lb);
        if s == 1, xs = x0; end
        [xOut, cOut] = fminsearch(penalizedCost, xs, fmsOpts);
        xOut = clampBounds(xOut, lb, ub);
        cOut = costFcn(xOut);
        fprintf('  Start %d: cost = %.4f\n', s, cOut);
        if cOut < bestCost
            bestCost = cOut; bestX = xOut;
        end
    end
end

fprintf('\n=== KET QUA TOI UU ===\n');
for i = 1:nvars
    fprintf('  %-12s = %.6g\n', varNames{i}, bestX(i));
end
fprintf('  Cost cuoi cung  = %.4f (so voi cost ban dau %.4f)\n\n', ...
    costFcn(bestX), costFcn(x0));

%% ------------------- Section 5: Ap dung ket qua & kiem tra lai ----------
applyGains(bestX, varNames);
simOut = sim(mdl, 'StopTime', num2str(T_SIM), 'ReturnWorkspaceOutputs', 'on');
plotResult(simOut, 'Ket qua SAU khi toi uu');

save(fullfile(pwd, 'IB_SMC_best_pid_gains.mat'), 'bestX', 'varNames', 'bestCost');
fprintf('Da luu bo so gain toi uu vao IB_SMC_best_pid_gains.mat\n');


%% =========================================================================
%%                          CAC HAM PHU (LOCAL FUNCTIONS)
%% =========================================================================

function assignin_if_missing(name, val)
% Chi gan gia tri mac dinh neu bien CHUA ton tai trong base workspace,
% de khong ghi de len thong so that ban da co san.
    if ~evalin('base', ['exist(''' name ''',''var'')'])
        assignin('base', name, val);
    end
end

function x0 = get_initial_gains()
% Lay gia tri hien co trong base workspace lam diem khoi dau; neu bien nao
% chua co, dung gia tri mac dinh tham khao.
    names = {'Kp_pos','Ki_pos','Kd_pos','Kp_att','Ki_att','Kd_att', ...
             'Kp_att_dot','Ki_att_dot','Kd_att_dot'};
    defaults = [4  0.2 2   ...   % Kp_pos Ki_pos Kd_pos
                8  0.5 1.5 ...   % Kp_att Ki_att Kd_att
                0.15 0.02 0.02]; % Kp_att_dot Ki_att_dot Kd_att_dot
    x0 = zeros(1, numel(names));
    for i = 1:numel(names)
        if evalin('base', ['exist(''' names{i} ''',''var'')'])
            v = evalin('base', names{i});
            x0(i) = v(1); % neu la vector, lay phan tu dau lam dai dien
        else
            x0(i) = defaults(i);
            assignin('base', names{i}, defaults(i));
        end
    end
end

function applyGains(x, varNames)
% Gan bo gain x vao base workspace theo dung ten bien varNames.
    for i = 1:numel(varNames)
        assignin('base', varNames{i}, x(i));
    end
end

function xC = clampBounds(x, lb, ub)
    xC = min(max(x, lb), ub);
end

function enableSignalLogging(blockPath, outportIdx, logName)
% Bat signal logging tren mot outport cu the cua mot khoi (khong lam
% thay doi so do noi day, an toan de goi nhieu lan).
    try
        ph = get_param(blockPath, 'PortHandles');
        h = ph.Outport(outportIdx);
        set_param(h, 'DataLogging', 'on');
        set_param(h, 'DataLoggingNameMode', 'Custom');
        set_param(h, 'DataLoggingName', logName);
    catch ME
        warning('Khong the bat logging cho %s (outport %d): %s\nHay kiem tra lai duong dan/ten khoi trong model cua ban.', ...
            blockPath, outportIdx, ME.message);
    end
end

function sig = getLoggedSignal(simOut, name)
% Lay du lieu [time, values] tu logsout theo ten da dat trong enableSignalLogging.
    try
        el = simOut.logsout.getElement(name);
        sig.time = el.Values.Time;
        sig.data = el.Values.Data;
    catch
        sig.time = [];
        sig.data = [];
    end
end

function cost = simCost(x, mdl, T_sim, varNames)
% Ham cost dung cho toi uu hoa: chay mo phong voi bo gain x, tinh sai so
% bam quy dao (vi tri + goc), phat (penalty) qua trinh dieu khien lon va
% overshoot, tra ve gia tri lon neu mo phong khong on dinh / loi.
    PENALTY_UNSTABLE = 1e6;

    applyGains(x, varNames);

    try
        simOut = sim(mdl, 'StopTime', num2str(T_sim), ...
                          'ReturnWorkspaceOutputs', 'on', ...
                          'SrcWorkspace', 'base');
    catch
        cost = PENALTY_UNSTABLE;
        return;
    end

    posAct = getLoggedSignal(simOut, 'log_pos_actual');
    posRef = getLoggedSignal(simOut, 'log_pos_ref');
    attAct = getLoggedSignal(simOut, 'log_att_actual');
    attRef = getLoggedSignal(simOut, 'log_att_ref');
    torque = getLoggedSignal(simOut, 'log_torque_cmd');

    if isempty(posAct.time) || isempty(attAct.time)
        cost = PENALTY_UNSTABLE;
        return;
    end

    if any(~isfinite(posAct.data(:))) || any(~isfinite(attAct.data(:)))
        cost = PENALTY_UNSTABLE;
        return;
    end

    % --- Dua ve luoi thoi gian chung ---
    tGrid = linspace(0, T_sim, round(T_sim*100))';

    ePos = interpErr(posRef, posAct, tGrid);   % [N x 3]
    eAtt = interpErr(attRef, attAct, tGrid, 2); % chi lay roll & pitch (cot 1-2)

    % --- ITAE (nhay theo thoi gian, phat sai so keo dai) ---
    itae_pos = trapz(tGrid, tGrid .* sum(abs(ePos), 2));
    itae_att = trapz(tGrid, tGrid .* sum(abs(eAtt), 2));

    % --- Overshoot / dao dong: dung do lech chuan cua sai so tren nua cuoi ---
    half = floor(numel(tGrid)/2);
    osc_pos = mean(std(ePos(half:end, :), 0, 1));
    osc_att = mean(std(eAtt(half:end, :), 0, 1));

    % --- Phat khi lenh dieu khien cham nguong bao hoa qua nhieu (chi bao thu) ---
    satPenalty = 0;
    if ~isempty(torque.data)
        limitApprox = 1.0; % SUA LAI dung gia tri Command Authority that su neu can chinh xac hon
        fracSat = mean(any(abs(torque.data) > 0.95*limitApprox, 2));
        satPenalty = 50 * fracSat;
    end

    w_pos = 5.0; w_att = 1.0; w_osc = 20.0;
    cost = w_pos*itae_pos + w_att*itae_att + w_osc*(osc_pos + osc_att) + satPenalty;

    if ~isfinite(cost)
        cost = PENALTY_UNSTABLE;
    end
end

function e = interpErr(refSig, actSig, tGrid, nCols)
% Noi suy ref va actual ve cung luoi thoi gian roi tra ve sai so ref-actual.
    if nargin < 4
        nCols = size(actSig.data, 2);
    end
    if isempty(refSig.time) || isempty(actSig.time)
        e = zeros(numel(tGrid), nCols);
        return;
    end
    refI = interp1(refSig.time, refSig.data(:,1:nCols), tGrid, 'linear', 'extrap');
    actI = interp1(actSig.time, actSig.data(:,1:nCols), tGrid, 'linear', 'extrap');
    e = refI - actI;
end

function plotResult(simOut, ttl)
    posAct = getLoggedSignal(simOut, 'log_pos_actual');
    posRef = getLoggedSignal(simOut, 'log_pos_ref');
    attAct = getLoggedSignal(simOut, 'log_att_actual');
    attRef = getLoggedSignal(simOut, 'log_att_ref');

    figure('Name', ttl);
    subplot(2,1,1);
    plot(posRef.time, posRef.data, '--'); hold on;
    set(gca, 'ColorOrderIndex', 1);
    plot(posAct.time, posAct.data, '-');
    grid on; xlabel('t (s)'); ylabel('Vi tri (m)');
    legend('x_{ref}','y_{ref}','z_{ref}','x','y','z');
    title([ttl ' - Vi tri']);

    subplot(2,1,2);
    plot(attRef.time, rad2deg(attRef.data(:,1:2)), '--'); hold on;
    set(gca, 'ColorOrderIndex', 1);
    plot(attAct.time, rad2deg(attAct.data(:,1:2)), '-');
    grid on; xlabel('t (s)'); ylabel('Goc (deg)');
    legend('\phi_{ref}','\theta_{ref}','\phi','\theta');
    title([ttl ' - Goc nghieng (roll/pitch)']);
end