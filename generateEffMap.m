
clear; clc;

load('effmap/efficiency_map_all.mat');  % 加载 eff_data


% 提取数据点
X_raw = eff_data.rpm;
Y_raw = eff_data.torque;
Z_raw = eff_data.eta;
% === 步骤 1: 原始数据点等高线图绘制 (散点图和简单的等高线) ===

X_zero_point = 1;
Y_zero_point = 1;
Z_zero_point = 70; % 虚拟低效率



X_lowTorque = 1 :1 :16000;
Y_lowTorque = zeros([1,length(X_lowTorque)]);
Z_lowTorque = [linspace(80, 94, length(X_lowTorque)*0.125),linspace(94, 98, length(X_lowTorque)*0.875)];


% 合并数据
X_raw_ext = [X_raw, X_zero_point, X_lowTorque];
Y_raw_ext = [Y_raw, Y_zero_point, Y_lowTorque];
Z_raw_ext = [Z_raw, Z_zero_point, Z_lowTorque];

fprintf('数据提取完成。已添加虚拟零点。总数据点数量: %d\n', length(X_raw_ext));

figure('Name', '电机效率特性 - 原始数据点', 'Color', 'w');

% 绘制原始散点图，用颜色表示效率
scatter3(X_raw, Y_raw, Z_raw, 100, Z_raw, 'filled', 'MarkerEdgeColor', 'k');
colormap(jet);
cbar1 = colorbar('Location', 'EastOutside', 'FontSize', 10); % <<<<<< 修正点 1: 分离 ColorBar 的设置
set(get(cbar1, 'Label'), 'String', '效率 (%)', 'FontSize', 12); % <<<<<< 修正点 1: 单独设置 Label
title('原始效率描点数据 (3D 视图)');
xlabel('转速 (rpm)', 'FontSize', 12);
ylabel('扭矩 (Nm)', 'FontSize', 12);
zlabel('效率 (%)', 'FontSize', 12);
grid on;
view(3); % 切换到3D视图
axis tight;

%% === 步骤 2: 插值和平滑，生成效率 Map (修正版) ===

% ... (步骤 2.1 和 2.2 保持不变，确保 F 的 Extrapolation 是 'none') ...

% ... (步骤 2.1 和 2.2 保持不变) ...

%% === 步骤 2: 插值和平滑，生成效率 Map (最终修正版) ===

% 2.1 定义新的网格空间 (Map的范围和分辨率)
X_min = 0;                     % 最低转速设为0
X_max = max(X_raw) * 1.1;
Y_min = 0;
Y_max = max(Y_raw) * 1.1;
Map_resolution = 800;

X_map = linspace(X_min, X_max, Map_resolution);
Y_map = linspace(Y_min, Y_max, Map_resolution);
[X_mesh, Y_mesh] = meshgrid(X_map, Y_map);

% 2.2 使用 Scattered Interpolant 进行插值，外推方式为 'none'
Z_map = griddata(X_raw_ext, Y_raw_ext, Z_raw_ext, X_mesh, Y_mesh, 'cubic');
% Z_map = F(X_mesh, Y_mesh); 

% --- 2.3 修正点：确定平滑的包络线 (使用 Alpha Shape) ---
% 选择合适的 alpha 值来控制包络线的松紧度。
% 较小的 alpha 值会使边界更紧密地贴合数据点，但可能导致边界不光滑。
% 这里的 alpha 取值需要根据您的具体数据进行微调。
alpha_value = 800; % 示例值，请根据您的数据规模调整
shp = alphaShape(X_raw_ext', Y_raw_ext', alpha_value);

% 使用 inShape 判断网格点是否在 Alpha Shape 区域内
IN = inShape(shp, X_mesh, Y_mesh); 

% 2.4 对 Map 进行平滑处理 (仅对包络内的数据进行平滑)
sigma = 3; 

Z_temp = Z_map;
% 临时填充 NaN 值。注意：如果 NaN 区域大，这可能导致边界出现平滑的"拖尾"，
% 故我们严格依赖 IN Mask。
Z_temp(isnan(Z_temp)) = 0; 

Z_map_smooth = imgaussfilt(Z_temp, sigma);

% 2.5 重新应用包络掩膜 (Mask)
Z_final_map = NaN(size(Z_map_smooth)); 
Z_final_map(IN) = Z_map_smooth(IN); 

% --- 2.6 修正点：效率值限定 (Clipping) ---
eta_min_limit = 70;
eta_max_limit = 98;

% 设定下限：低于 80% 的效率点，如果是有效数据，则设置为 80%
Z_final_map(Z_final_map < eta_min_limit) = eta_min_limit;

% 设定上限：高于 98% 的效率点，设置为 98% (防止插值过冲)
Z_final_map(Z_final_map > eta_max_limit) = eta_max_limit;


% 2.7 强制在原始数据点处保持精确效率值 (可选，推荐)
for i = 1:length(X_raw_ext)
    [~, ix] = min(abs(X_map - X_raw_ext(i)));
    [~, iy] = min(abs(Y_map - Y_raw_ext(i)));
    
    % 将原始值 Z_raw_ext(i) 也进行上下限限定，再赋值
    clamped_raw_eta = max(eta_min_limit, min(eta_max_limit, Z_raw_ext(i)));
    
    Z_final_map(iy, ix) = clamped_raw_eta; 
end

fprintf('Map 生成完成。已使用 Alpha Shape 限定包络并对效率值进行了限定。\n');

% 存储最终 Map 数据 (MotorEfficiencyMap 结构体定义略)
MotorEfficiencyMap.efficiency_data = Z_final_map; 
MotorEfficiencyMap.rpm_axis = X_map;
MotorEfficiencyMap.torque_axis = Y_map;
save('MotorEfficiencyMap.mat', 'MotorEfficiencyMap');
fprintf('效率 Map 数据已保存为 MotorEfficiencyMap.mat\n');

%% === 步骤 3: 绘制最终的效率 Map (等高线图) ===

figure('Name', '电机效率 Map (Alpha Shape 包络)', 'Color', 'w');

% 3.1 绘制效率 Map
[C, h] = contourf(X_mesh, Y_mesh, Z_final_map);

colormap(jet);
cbar3 = colorbar('Location', 'EastOutside', 'FontSize', 12);
set(get(cbar3, 'Label'), 'String', '效率 (%)', 'FontSize', 14);
hold on;

% 绘制 Alpha Shape 包络边界
[bf, p] = boundaryFacets(shp);
plot(p(bf, 1), p(bf, 2), 'k--', 'LineWidth', 1.5); % 绘制黑色虚线边界线
% text(mean(X_raw), mean(Y_raw)/2, 'Alpha Shape 包络线', 'Color', 'k', 'HorizontalAlignment', 'center');

% --- 3.2 修正点：自定义标注等效率曲线 ---
levels = [80, 85, 88, 90, 92, 93, 94, 95, 96, 97];
% 确保 levels 中的值都在 [eta_min_limit, eta_max_limit] 范围内。

[C, h] = contour(X_mesh, Y_mesh, Z_final_map, levels, 'k-', 'LineWidth', 1);
clabel(C, h, 'FontSize', 9, 'Color', 'k', 'Rotation', 0, 'LabelSpacing', 200);

% ... (3.3 和 3.4 保持不变) ...

% 3.3 标记原始数据点 
% scatter(X_raw_ext', Y_raw_ext', 50, 'w', 'Marker', 'o', 'LineWidth', 1.5); 
% scatter(X_raw_ext', Y_raw_ext', 10, 'k', 'filled');