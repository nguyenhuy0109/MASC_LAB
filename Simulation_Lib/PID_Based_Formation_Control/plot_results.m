function plot_results(matfile)
    if nargin < 1
        matfile = 'formation_sim_results.mat';
    end
    S = load(matfile);
    tvec = S.tvec;
    LT = S.LEADER_TRAJ; F1 = S.F1_TRAJ; F2 = S.F2_TRAJ;
    LD = S.LEADER_DES;
    figure('Name','3D UAV Formation Trajectory','Position',[100 100 900 750]);
    hold on; grid on; box on;
    plot3(LD(:,1), LD(:,2), LD(:,3), 'k--', 'LineWidth', 1.0, 'DisplayName', 'Desired Trajectory (Leader)');
    plot3(LT(:,1), LT(:,2), LT(:,3), 'r-', 'LineWidth', 2.0, 'DisplayName', 'Leader (Actual)');
    plot3(F1(:,1), F1(:,2), F1(:,3), 'b-', 'LineWidth', 1.5, 'DisplayName', 'Follower 1 (Actual)');
    plot3(F2(:,1), F2(:,2), F2(:,3), 'g-', 'LineWidth', 1.5, 'DisplayName', 'Follower 2 (Actual)');
    plot3(LT(1,1), LT(1,2), LT(1,3), 'ko', 'MarkerFaceColor','k', 'MarkerSize', 8, 'DisplayName','Starting Point');
    
    
    plot3(LT(end,1), LT(end,2), LT(end,3), 'r^', 'MarkerFaceColor','r', 'MarkerSize', 9, 'DisplayName','Leader - End');
    plot3(F1(end,1), F1(end,2), F1(end,3), 'b^', 'MarkerFaceColor','b', 'MarkerSize', 9, 'DisplayName','Follower 1 - End');
    plot3(F2(end,1), F2(end,2), F2(end,3), 'g^', 'MarkerFaceColor','g', 'MarkerSize', 9, 'DisplayName','Follower 2 - End');

    n_snap = 6;
    idxs = round(linspace(1, length(tvec), n_snap+1));
    idxs = idxs(2:end);   
    

    for i = 1:length(idxs)
        k = idxs(i);
        Xtri = [LT(k,1) F1(k,1) F2(k,1) LT(k,1)];
        Ytri = [LT(k,2) F1(k,2) F2(k,2) LT(k,2)];
        Ztri = [LT(k,3) F1(k,3) F2(k,3) LT(k,3)];
        plot3(Xtri, Ytri, Ztri, '-', 'Color', [0.55 0.55 0.55], 'LineWidth', 1.0, 'HandleVisibility','off');
    end
    xlabel('X (m)'); 
    ylabel('Y (m)'); 
    zlabel('Z (m) - Altitude');
    title('Leader-Follower Formation Trajectory (1 Leader + 2 Followers, Triangle)');
    
    legend('Location','northeastoutside');
    view(35, 25);   
    axis equal;      
    hold off;
end