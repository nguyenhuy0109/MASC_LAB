%% tune_PID.m
% =========================================================================
% Tu dong do bo so PID cho model PID.slx (cascade: PID Position ->
% PID Attitude (outer angle) -> PID Attitude (inner rate)) bang toi uu hoa
% dua tren mo phong (simulation-based optimization).
%
% MUC TIEU TOI UU: uu tien SAI SO TINH (steady-state error) va THOI GIAN
% HOI TU (settling time) la NHO NHAT, cac tieu chi khac (ITAE, dao dong,
% bao hoa dieu khien) chi la phu tro de tranh nghiem "an gian" (vi du
% gain rat lon lam settle nhanh nhung rung lac/bao hoa lien tuc).
%
% =========================================================================
% CAC LOI DA SUA SO VOI BAN GOC (tune_IB_SMC_PID.m):
%   1) [LOI CHINH - GAY CRASH NGAY LAP TUC]
%      MDL_FILE = 'IB_SMC' -> SUA THANH 'PID' (dung ten file PID.slx ban
%      upload). load_system('IB_SMC') se bao loi "Unable to find/load"
%      vi khong co file nay.
%      Da kiem tra: tat ca ten khoi/cong/bien con lai ('UAV Model',
%      'PID Position', 'PID Attitude ' co dau cach cuoi, 'Mux1', cac chi
%      so outport, cac ten bien Kp_pos/Ki_pos/.../Kd_att_dot, va cac bien
%      vat ly m,g,Iv,kt,kd,Lroll,Lpitch) DA KHOP CHINH XAC voi model that,
%      khong can sua gi them o phan do.
%   2) Gioi han tim kiem (lb/ub) duoc MO RONG dang ke (~2.5-3 lan) de
%      thuat toan co khong gian tim nghiem tot hon.
%   3) Ham cost (simCost) duoc VIET LAI: tinh truc tiep THOI GIAN HOI TU
%      (settling time, dua vao bang dung sai +/-2% cho vi tri va +/-1.5 do
%      cho goc) va SAI SO TINH (gia tri trung binh |error| tren cua so
%      cuoi cua mo phong), la 2 thanh phan trong so LON NHAT trong cost.
%      ITAE va do dao dong van giu lai voi trong so nho hon de tranh
%      nghiem gain cuc lon gay rung/bao hoa lien tuc.
%   4) Phat bao hoa (saturation penalty) dung DUNG gioi han mo-men that
%      cua tung truc lay tu khoi Saturate trong model ([1.0915, 0.8984,
%      0.0984] Nm cho roll/pitch/yaw) thay vi mot so 1.0 chung chung nhu
%      ban goc (truc yaw co gioi han rat nho ~0.0984 Nm nen de bao hoa).
%   5) Neu he KHONG hoi tu kip trong thoi gian mo phong T_SIM, cost bi
%      phat rat nang de loai bo cac bo gain khong on dinh/qua cham.
%   6) Tang so vong lap/quan the cua bo toi uu (vi khong gian tim kiem
%      rong hon) va them bao cao settling time + sai so tinh sau khi tune.
%
% CACH DUNG:
%   1) Dat file nay CUNG THU MUC voi PID.slx.
%   2) Neu ban co script init rieng dat cac bien m, g, Iv, kt, kd, Lroll,
%      Lpitch (va Kp_pos/Ki_pos/... khoi tao), chay script do TRUOC, hoac
%      de tune_PID.m tu dong dung gia tri mac dinh tham khao o Section 1.
%   3) Kiem tra/chinh BOUNDS o Section 3 va cac gioi han vat ly o Section 0
%      (TORQUE_LIMIT, RATE_LIMIT, ATT_LIMIT, cac dung sai settling) cho
%      dung may bay/thang do cua ban.
%   4) Chay script. Ket qua toi uu duoc gan vao workspace + luu file .mat
%      + ve do thi so sanh truoc/sau + in bao cao settling time/sai so tinh.
%
% LUU Y (van con ton tai tu ban goc, khong thuoc pham vi sua doi lan nay):
%   - Model hien tai KHONG co anti-windup tren cac khoi Integrator va dung
%     Derivative ly tuong (khong loc) tren sai so. Toi uu hoa co the "ne"
%     duoc mot phan van de nay bang cach chon Ki/Kd phu hop, nhung ve lau
%     dai NEN SUA TRUC TIEP TRONG MODEL (them anti-windup + loc dao ham).
%   - Ca 3 truc x/y/z (vi tri) va roll/pitch/yaw (goc) dung CHUNG mot bo
%     gain scalar (dung nhu cau truc that trong PID.slx: khoi Gain nhan
%     voi tin hieu vector 3 phan tu). Neu muon gain rieng cho tung truc,
%     phai sua ca cau truc .slx (them Gain vector [Kx Ky Kz] rieng), khong
%     the sua chi trong file .m nay.
% =========================================================================

clear; clc;

%% ------------------- Section 0: Cau hinh model -------------------------
MDL_FILE = 'PID';   % SUA: ten file that la PID.slx (truoc day ghi nham 'IB_SMC')
if ~(exist([MDL_FILE '.slx'], 'file') == 4 || exist([MDL_FILE '.slx'], 'file') == 2)
    error(['Khong tim thay file "%s.slx" trong thu muc hien tai (%s).\n' ...
           'Hay dat tune_PID.m CUNG THU MUC voi %s.slx, hoac sua bien MDL_FILE ' ...
           'cho dung ten file model cua ban.'], MDL_FILE, pwd, MDL_FILE);
end
if ~bdIsLoaded(MDL_FILE)
    load_system(MDL_FILE);
end
mdl = MDL_FILE;

% Duong dan cac khoi quan trong (da doi chieu va khop voi PID.slx)
BLK_UAV      = [mdl '/UAV Model'];      % out1=Position, out2=att_dot, out3=att
BLK_PIDPOS   = [mdl '/PID Position'];   % out1=thrust,  out2=att_r
BLK_PIDATT   = [mdl '/PID Attitude '];  % out1=Torque   (chu y co dau cach cuoi ten, giong file goc)
BLK_REF_MUX  = [mdl '/Mux1'];           % out1 = [x_ref;y_ref;z_ref;psi_ref]

T_SIM = 15;   % thoi gian mo phong khi tune (s) - tang tu 12->15s de gain
              % o khoang rong hon van co du thoi gian hoi tu on dinh

% --- Gioi han vat ly THAT lay tu cac khoi Saturate trong PID.slx ---
% (dung de tinh phat bao hoa chinh xac hon, va de tham khao khi dat BOUNDS
% o Section 3). SUA LAI neu ban thay doi cac khoi Saturate trong model.
TORQUE_LIMIT = [1.0915, 0.8984, 0.0984];  % Nm, gioi han "Saturation Command Authority" (roll,pitch,yaw)
RATE_LIMIT   = 10*pi/3;                   % rad/s, gioi han "Saturation Max Rate"
ATT_LIMIT    = pi/4;                      % rad, gioi han "Saturate Stabilization Command" (roll,pitch)

% --- Dung sai (tolerance band) de tinh settling time ---
TOL_POS_REL  = 0.02;   % +-2% cua bien do buoc tham chieu vi tri
TOL_POS_MIN  = 0.02;   % san toi thieu 2 cm (tranh chia cho so qua nho)
TOL_ATT_DEG  = 1.5;    % +-1.5 do cho roll/pitch

%% ------------------- Section 1: Gia tri vat ly / gain ban dau -----------
defaultsNeeded = {'m','g','Iv','kt','kd','Lroll','Lpitch'};
for i = 1:numel(defaultsNeeded)
    if ~evalin('base', ['exist(''' defaultsNeeded{i} ''',''var'')'])
        warning('Bien "%s" chua co trong base workspace - dang gan gia tri MAC DINH tam thoi. Hay kiem tra lai cho dung may bay cua ban!', defaultsNeeded{i});
    end
end
assignin_if_missing('m',       1.121);
assignin_if_missing('g',       9.81);
assignin_if_missing('Iv',      diag([1e-2, 8.2e-3, 1.48e-2]));
assignin_if_missing('kt',      5.11);
assignin_if_missing('kd',      0.0487);
assignin_if_missing('Lroll',   0.2136);
assignin_if_missing('Lpitch',  0.1758);

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

% GIOI HAN TIM KIEM - DA MO RONG (~2.5-3 lan) so voi ban goc.
% Ly do chon muc tran:
%  - Vong ngoai vi tri/goc: gioi han tren du rong de toi uu co the "danh
%    doi" giua dap ung nhanh va do on dinh, nhung khong qua lon toi muc
%    lam nghiem so mat on dinh so hoc.
%  - Vong trong toc do goc (Kp/Ki/Kd_att_dot): mo-men bao hoa rat nho
%    (~1 Nm, rieng yaw ~0.1 Nm) nen gain qua lon se lam tin hieu dieu
%    khien bao hoa/chattering lien tuc - da gioi han o muc hop ly hon.
lb = [0   0     0      ...   % Kp_pos Ki_pos Kd_pos
      0   0     0      ...   % Kp_att Ki_att Kd_att
      0 0     0     ];     % Kp_att_dot Ki_att_dot Kd_att_dot
ub = [10    10    10     ...
      10    10    10     ...
      10     10     10     ];

% Dam bao x0 nam trong bien (neu khong, keo vao)
x0 = min(max(x0, lb), ub);

%% ------------------- Section 4: Toi uu hoa ------------------------------
costCtx = struct('T_sim', T_SIM, 'torqueLimit', TORQUE_LIMIT, ...
                  'rateLimit', RATE_LIMIT, 'attLimit', ATT_LIMIT, ...
                  'tolPosRel', TOL_POS_REL, 'tolPosMin', TOL_POS_MIN, ...
                  'tolAttDeg', TOL_ATT_DEG);
costFcn = @(x) simCost(x, mdl, varNames, costCtx);

optsFound = false;
bestX = x0; bestCost = costFcn(x0);
fprintf('Cost tai gia tri khoi diem: %.4f\n', bestCost);

% --- Uu tien 1: particleswarm (Global Optimization Toolbox) ---
if exist('particleswarm', 'file') == 2
    try
        popSize  = 40;    % tang tu ban goc (24 -> 40) vi khong gian tim kiem rong hon
        maxIter  = 60;    % tang tu ban goc (40 -> 60); giam de test nhanh (vd 8/10)
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
            'Display', 'iter', 'MaxIterations', 300, 'UseCompletePoll', true);
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
    nStarts = 8;   % tang tu 6 -> 8 vi khong gian tim kiem rong hon
    fmsOpts = optimset('Display', 'iter', 'MaxIter', 200, 'MaxFunEvals', 600);
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
reportMetrics(simOut, T_SIM, TOL_POS_REL, TOL_POS_MIN, TOL_ATT_DEG);
plotResult(simOut, 'Ket qua SAU khi toi uu', TOL_POS_REL, TOL_POS_MIN, TOL_ATT_DEG);

save(fullfile(pwd, 'PID_best_pid_gains.mat'), 'bestX', 'varNames', 'bestCost');
fprintf('Da luu bo so gain toi uu vao PID_best_pid_gains.mat\n');


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
% Lay tin hieu da log tu logsout theo ten da dat trong enableSignalLogging.
% Luu lai CA doi tuong timeseries goc (sig.ts) de noi suy an toan sau nay
% (xem tsDataAtN/interpErr) - khong tu doan hinh dang mang thu cong nua,
% vi Simulink co the tra ve du lieu o nhieu dang khac nhau tuy loai tin
% hieu (dac biet tin hieu HANG SO nhu tham chieu tu khoi Constant thuong
% chi co DUNG 1 MAU thoi gian, khac han voi tin hieu lien tuc co hang
% tram/nghin mau - day chinh la nguyen nhan gay loi "X and V must be of
% the same length" truoc do).
    try
        el = simOut.logsout.getElement(name);
        sig.ts   = el.Values;                          % doi tuong timeseries goc
        sig.time = el.Values.Time;
        sig.data = tsDataAtN(el.Values, el.Values.Time); % du lieu da chuan hoa, dung cho ve do thi/kiem tra
    catch
        sig.ts   = [];
        sig.time = [];
        sig.data = [];
    end
end

function y = tsDataAtN(ts, wantedTime)
% Lay gia tri cua mot timeseries tai cac moc thoi gian wantedTime, tra ve
% dang [numel(wantedTime) x width]. Dung CHINH co che resample cua lop
% timeseries trong MATLAB (thay vi tu doan hinh dang mang bang tay) nen xu
% ly dung MOI truong hop: tin hieu vector nhieu chieu, mang 3-D voi chieu
% thoi gian o vi tri bat ky, va DAC BIET la tin hieu HANG SO chi co 1 mau
% thoi gian (vi du tham chieu ghep tu cac khoi Constant qua Mux1).
    if isempty(ts) || isempty(ts.Time)
        y = [];
        return;
    end

    if numel(ts.Time) < 2
        % Tin hieu hang so thuc su (chi 1 mau) - khong interp1/resample duoc,
        % "trai" gia tri do ra toan bo luoi thoi gian yeu cau.
        val = squeeze(ts.Data);
        val = val(:).';                          % ep ve dang 1 hang [1 x width]
        y = repmat(val, numel(wantedTime), 1);
        return;
    end

    tsOut = resample(ts, wantedTime);            % MATLAB tu xu ly dung hinh dang du lieu
    y = tsOut.Data;
    if ndims(y) >= 3
        y = squeeze(y);
    end
    % Dam bao dung dang [numel(wantedTime) x width]
    if size(y, 1) ~= numel(wantedTime) && size(y, 2) == numel(wantedTime)
        y = y.';
    end
end

function y = padOrTrimCols(y, nCols)
% Cat bot hoac dem them cot bang 0 cho du y co dung nCols cot yeu cau.
    if isempty(y)
        y = zeros(0, nCols);
        return;
    end
    w = size(y, 2);
    if w >= nCols
        y = y(:, 1:nCols);
    else
        y = [y, zeros(size(y,1), nCols - w)];
    end
end

function e = interpErr(refSig, actSig, tGrid, nCols)
% Lay ref va actual tai cung luoi thoi gian tGrid (qua tsDataAtN, an toan
% voi moi hinh dang du lieu) roi tra ve sai so ref-actual.
%   nCols: BAT BUOC chi ro so cot can lay (khong con tu doan tu actSig.data
%   nhu ban truoc, vi kich thuoc do co the sai lech neu tin hieu co hinh
%   dang dac biet). Vi du: 3 cho vi tri (x,y,z), 2 cho goc (roll,pitch).
    if nargin < 4 || isempty(nCols)
        nCols = 3;
    end
    refI = tsDataAtN(refSig.ts, tGrid);
    actI = tsDataAtN(actSig.ts, tGrid);
    if isempty(refI), refI = zeros(numel(tGrid), nCols); end
    if isempty(actI), actI = zeros(numel(tGrid), nCols); end
    refI = padOrTrimCols(refI, nCols);
    actI = padOrTrimCols(actI, nCols);
    e = refI - actI;
end

function [tSettle, sse, osc] = settleMetrics(errMat, tGrid, tol)
% Tinh thoi gian hoi tu (settling time) va sai so tinh (steady-state error)
% cho mot tap sai so nhieu kenh (errMat: N x m).
%   tol: dung sai (scalar hoac 1xm) - he "hoi tu" khi TAT CA cac kenh nam
%        trong bang +-tol va GIU NGUYEN trong bang do cho den het mo phong.
%   tSettle: thoi diem (s) tu do tro di sai so luon nam trong bang. Neu
%            khong bao gio hoi tu, tra ve tGrid(end) (worst-case).
%   sse: sai so tinh = trung binh |error| tren 15% cuoi cua cua so mo phong.
%   osc: do dao dong (std) tren cung cua so cuoi, dung de phat rung lac.
    N = size(errMat, 1);
    if isscalar(tol)
        tol = repmat(tol, 1, size(errMat, 2));
    end
    withinBand = abs(errMat) <= tol;      % N x m logical
    allWithin  = all(withinBand, 2);      % N x 1
    idxViol = find(~allWithin);
    if isempty(idxViol)
        tSettle = tGrid(1);               % da nam trong bang tu dau
    elseif idxViol(end) >= N
        tSettle = tGrid(end);             % khong bao gio hoi tu -> worst case
    else
        tSettle = tGrid(idxViol(end) + 1);
    end

    winLen = max(round(0.15 * N), 5);
    winLen = min(winLen, N);
    tailIdx = (N - winLen + 1):N;
    sse = mean(mean(abs(errMat(tailIdx, :)), 2));
    osc = mean(std(errMat(tailIdx, :), 0, 1));
end

function cost = simCost(x, mdl, varNames, ctx)
% Ham cost dung cho toi uu hoa. Uu tien CHINH: thoi gian hoi tu va sai so
% tinh nho nhat. Cac thanh phan phu (ITAE, dao dong, bao hoa) giu vai tro
% "phanh" de tranh nghiem gain cuc doan (rung lac/bao hoa lien tuc) du co
% the co settling time/sai so tinh danh gia tot tren luoi roi rac.
    PENALTY_UNSTABLE = 1e6;
    T_sim = ctx.T_sim;

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

    tGrid = linspace(0, T_sim, round(T_sim * 100))';

    ePos = interpErr(posRef, posAct, tGrid, 3);     % [N x 3]  (x,y,z)
    eAtt = interpErr(attRef, attAct, tGrid, 2);    % [N x 2]  (roll,pitch)

    % --- Dung sai settling ---
    if ~isempty(posRef.data)
        refAmp = max(abs(posRef.data(end, :)));
    else
        refAmp = 0;
    end
    tolPos = max(ctx.tolPosRel * refAmp, ctx.tolPosMin);
    tolAtt = deg2rad(ctx.tolAttDeg);

    [tsPos, ssePos, oscPos] = settleMetrics(ePos, tGrid, tolPos);
    [tsAtt, sseAtt, oscAtt] = settleMetrics(eAtt, tGrid, tolAtt);
    tSettle = max(tsPos, tsAtt);   % he chi coi la "hoi tu" khi CA HAI da on dinh

    % --- ITAE (chi la thanh phan phu, trong so nho) ---
    itae_pos = trapz(tGrid, tGrid .* sum(abs(ePos), 2)) / T_sim^2;
    itae_att = trapz(tGrid, tGrid .* sum(abs(eAtt), 2)) / T_sim^2;

    % --- Phat bao hoa dieu khien, dung DUNG gioi han moment tung truc ---
    satPenalty = 0;
    if ~isempty(torque.data)
        limitVec = ctx.torqueLimit;   % [Nm] roll,pitch,yaw
        fracSat = mean(abs(torque.data) > 0.95 * limitVec, 1); % 1x3
        satPenalty = 30 * mean(fracSat);
    end

    % --- Trong so: SAI SO TINH va THOI GIAN HOI TU la uu tien hang dau ---
    w_ts      = 5.0;     % diem cho moi giay chua hoi tu
    w_sse_pos = 200.0;   % sai so tinh vi tri (m) rat nhay cam
    w_sse_att = 100.0;   % sai so tinh goc (rad)
    w_osc     = 8.0;     % phat dao dong/rung o giai doan cuoi
    w_itae    = 1.0;     % phu tro, dam bao qua trinh qua do cung muot

    cost = w_ts * tSettle ...
         + w_sse_pos * ssePos + w_sse_att * sseAtt ...
         + w_osc * (oscPos + oscAtt) ...
         + w_itae * (itae_pos + itae_att) ...
         + satPenalty;

    % Phat nang neu khong hoi tu kip trong T_sim (loai bo gain qua cham/khong on dinh)
    if tSettle >= 0.99 * T_sim
        cost = cost + 100;
    end

    if ~isfinite(cost)
        cost = PENALTY_UNSTABLE;
    end
end

function reportMetrics(simOut, T_sim, tolPosRel, tolPosMin, tolAttDeg)
% In ra console: settling time va sai so tinh dat duoc voi bo gain cuoi cung.
    posAct = getLoggedSignal(simOut, 'log_pos_actual');
    posRef = getLoggedSignal(simOut, 'log_pos_ref');
    attAct = getLoggedSignal(simOut, 'log_att_actual');
    attRef = getLoggedSignal(simOut, 'log_att_ref');

    tGrid = linspace(0, T_sim, round(T_sim * 100))';
    ePos = interpErr(posRef, posAct, tGrid, 3);
    eAtt = interpErr(attRef, attAct, tGrid, 2);

    if ~isempty(posRef.data)
        refAmp = max(abs(posRef.data(end, :)));
    else
        refAmp = 0;
    end
    tolPos = max(tolPosRel * refAmp, tolPosMin);
    tolAtt = deg2rad(tolAttDeg);

    [tsPos, ssePos, ~] = settleMetrics(ePos, tGrid, tolPos);
    [tsAtt, sseAtt, ~] = settleMetrics(eAtt, tGrid, tolAtt);

    fprintf('\n=== BAO CAO SETTLING TIME & SAI SO TINH (bo gain cuoi cung) ===\n');
    fprintf('  Vi tri : settling time = %.3f s (dung sai +-%.3f m) | sai so tinh trung binh = %.4f m\n', ...
        tsPos, tolPos, ssePos);
    fprintf('  Goc    : settling time = %.3f s (dung sai +-%.2f do) | sai so tinh trung binh = %.4f rad (%.3f do)\n', ...
        tsAtt, tolAttDeg, sseAtt, rad2deg(sseAtt));
    fprintf('  => He duoc coi la "hoi tu" tai t = %.3f s (max cua 2 gia tri tren)\n\n', max(tsPos, tsAtt));
end

function plotResult(simOut, ttl, tolPosRel, tolPosMin, tolAttDeg)
    posAct = getLoggedSignal(simOut, 'log_pos_actual');
    posRef = getLoggedSignal(simOut, 'log_pos_ref');
    attAct = getLoggedSignal(simOut, 'log_att_actual');
    attRef = getLoggedSignal(simOut, 'log_att_ref');

    if ~isempty(posRef.data)
        refAmp = max(abs(posRef.data(end, :)));
    else
        refAmp = 0;
    end
    tolPos = max(tolPosRel * refAmp, tolPosMin);
    tolAtt = deg2rad(tolAttDeg);

    figure('Name', ttl);
    subplot(2,1,1);
    plot(posRef.time, posRef.data, '--'); hold on;
    set(gca, 'ColorOrderIndex', 1);
    plot(posAct.time, posAct.data, '-');
    if ~isempty(posRef.data)
        yline(posRef.data(end,:) + tolPos, ':k', 'HandleVisibility','off');
        yline(posRef.data(end,:) - tolPos, ':k', 'HandleVisibility','off');
    end
    grid on; xlabel('t (s)'); ylabel('Vi tri (m)');
    legend('x_{ref}','y_{ref}','z_{ref}','x','y','z');
    title([ttl ' - Vi tri (duong cham: bang dung sai settling)']);

    subplot(2,1,2);
    plot(attRef.time, rad2deg(attRef.data(:,1:2)), '--'); hold on;
    set(gca, 'ColorOrderIndex', 1);
    plot(attAct.time, rad2deg(attAct.data(:,1:2)), '-');
    yline(rad2deg(tolAtt), ':k', 'HandleVisibility','off');
    yline(-rad2deg(tolAtt), ':k', 'HandleVisibility','off');
    grid on; xlabel('t (s)'); ylabel('Goc (deg)');
    legend('\phi_{ref}','\theta_{ref}','\phi','\theta');
    title([ttl ' - Goc nghieng (roll/pitch)']);
end