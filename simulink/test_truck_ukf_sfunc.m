function test_truck_ukf_sfunc()
% TEST_TRUCK_UKF_SFUNC 测试半挂卡车UKF S-Function
%
% 此脚本创建一个简单的Simulink测试模型来验证UKF S-Function的功能
%
% 使用方法:
%   1. 确保已编译S-Function: compile_truck_ukf_sfunc()
%   2. 运行此脚本: test_truck_ukf_sfunc()
%
% 作者: UKF Speed Estimation Team
% 日期: 2025年7月

fprintf('创建半挂卡车UKF测试模型...\n');

% 模型名称
model_name = 'truck_ukf_test_model';

% 检查S-Function是否存在
if ~exist('truck_ukf_sfunc', 'file')
    error(['S-Function未找到！请先编译:\n' ...
           'compile_truck_ukf_sfunc()']);
end

% 关闭已存在的同名模型
if bdIsLoaded(model_name)
    close_system(model_name, 0);
end

% 创建新模型
new_system(model_name);
open_system(model_name);

% 设置仿真参数
set_param(model_name, 'SolverType', 'Fixed-step');
set_param(model_name, 'Solver', 'ode4');
set_param(model_name, 'FixedStep', '0.01');
set_param(model_name, 'StartTime', '0');
set_param(model_name, 'StopTime', '30');

fprintf('添加仿真模块...\n');

% 添加输入信号源
% 1. 轮速信号 (模拟渐进加速)
add_block('simulink/Sources/Signal Generator', [model_name '/WheelSpeed']);
set_param([model_name '/WheelSpeed'], 'WaveForm', 'sawtooth');
set_param([model_name '/WheelSpeed'], 'Amplitude', '20');
set_param([model_name '/WheelSpeed'], 'Frequency', '0.1');
set_param([model_name '/WheelSpeed'], 'Units', 'rad/sec');
set_param([model_name '/WheelSpeed'], 'Position', [50 50 100 80]);

% 2. 加速度计X (模拟纵向加速度)
add_block('simulink/Sources/Signal Generator', [model_name '/AccX']);
set_param([model_name '/AccX'], 'WaveForm', 'square');
set_param([model_name '/AccX'], 'Amplitude', '1.5');
set_param([model_name '/AccX'], 'Frequency', '0.2');
set_param([model_name '/AccX'], 'Position', [50 100 100 130]);

% 3. 加速度计Y (模拟侧向加速度)
add_block('simulink/Sources/Signal Generator', [model_name '/AccY']);
set_param([model_name '/AccY'], 'WaveForm', 'sine');
set_param([model_name '/AccY'], 'Amplitude', '0.5');
set_param([model_name '/AccY'], 'Frequency', '0.3');
set_param([model_name '/AccY'], 'Position', [50 150 100 180]);

% 4. 陀螺仪 (模拟航向角速度)
add_block('simulink/Sources/Signal Generator', [model_name '/YawRate']);
set_param([model_name '/YawRate'], 'WaveForm', 'sine');
set_param([model_name '/YawRate'], 'Amplitude', '0.2');
set_param([model_name '/YawRate'], 'Frequency', '0.15');
set_param([model_name '/YawRate'], 'Position', [50 200 100 230]);

% 5. 油门信号
add_block('simulink/Sources/Constant', [model_name '/Throttle']);
set_param([model_name '/Throttle'], 'Value', '0.3');
set_param([model_name '/Throttle'], 'Position', [50 250 100 280]);

% 6. 制动信号
add_block('simulink/Sources/Constant', [model_name '/Brake']);
set_param([model_name '/Brake'], 'Value', '0.0');
set_param([model_name '/Brake'], 'Position', [50 300 100 330]);

% 7. 方向盘转角
add_block('simulink/Sources/Constant', [model_name '/Steering']);
set_param([model_name '/Steering'], 'Value', '0.0');
set_param([model_name '/Steering'], 'Position', [50 350 100 380]);

% 添加信号合并器
add_block('simulink/Signal Routing/Mux', [model_name '/InputMux']);
set_param([model_name '/InputMux'], 'Inputs', '7');
set_param([model_name '/InputMux'], 'Position', [150 150 180 320]);

% 添加UKF S-Function
add_block('simulink/User-Defined Functions/S-Function', [model_name '/TruckUKF']);
set_param([model_name '/TruckUKF'], 'FunctionName', 'truck_ukf_sfunc');
set_param([model_name '/TruckUKF'], 'Parameters', '[3.8, 15000, 10.0, 0.6, 0.0, 0.01, 0]');
set_param([model_name '/TruckUKF'], 'Position', [220 200 300 260]);

% 添加输出信号分离器
add_block('simulink/Signal Routing/Demux', [model_name '/OutputDemux']);
set_param([model_name '/OutputDemux'], 'Outputs', '7');
set_param([model_name '/OutputDemux'], 'Position', [340 150 370 320]);

% 添加示波器显示
add_block('simulink/Sinks/Scope', [model_name '/SpeedScope']);
set_param([model_name '/SpeedScope'], 'Position', [400 50 450 100]);
set_param([model_name '/SpeedScope'], 'NumInputPorts', '2');

add_block('simulink/Sinks/Scope', [model_name '/PositionScope']);
set_param([model_name '/PositionScope'], 'Position', [400 150 450 200]);
set_param([model_name '/PositionScope'], 'NumInputPorts', '2');

add_block('simulink/Sinks/Scope', [model_name '/AccelScope']);
set_param([model_name '/AccelScope'], 'Position', [400 250 450 300]);
set_param([model_name '/AccelScope'], 'NumInputPorts', '2');

add_block('simulink/Sinks/Scope', [model_name '/UncertaintyScope']);
set_param([model_name '/UncertaintyScope'], 'Position', [400 350 450 400]);

% 添加数据记录
add_block('simulink/Sinks/To Workspace', [model_name '/DataLogger']);
set_param([model_name '/DataLogger'], 'VariableName', 'ukf_results');
set_param([model_name '/DataLogger'], 'SaveFormat', 'Structure With Time');
set_param([model_name '/DataLogger'], 'Position', [400 420 450 450]);

fprintf('连接信号线...\n');

% 连接输入信号到合并器
add_line(model_name, 'WheelSpeed/1', 'InputMux/1');
add_line(model_name, 'AccX/1', 'InputMux/2');
add_line(model_name, 'AccY/1', 'InputMux/3');
add_line(model_name, 'YawRate/1', 'InputMux/4');
add_line(model_name, 'Throttle/1', 'InputMux/5');
add_line(model_name, 'Brake/1', 'InputMux/6');
add_line(model_name, 'Steering/1', 'InputMux/7');

% 连接合并器到UKF
add_line(model_name, 'InputMux/1', 'TruckUKF/1');

% 连接UKF到分离器
add_line(model_name, 'TruckUKF/1', 'OutputDemux/1');

% 连接输出到示波器
add_line(model_name, 'OutputDemux/1', 'SpeedScope/1');  % 估计速度
add_line(model_name, 'WheelSpeed/1', 'SpeedScope/2');   % 轮速参考

add_line(model_name, 'OutputDemux/2', 'PositionScope/1');  % X位置
add_line(model_name, 'OutputDemux/3', 'PositionScope/2');  % Y位置

add_line(model_name, 'OutputDemux/5', 'AccelScope/1');  % 估计加速度X
add_line(model_name, 'OutputDemux/6', 'AccelScope/2');  % 估计加速度Y

add_line(model_name, 'OutputDemux/7', 'UncertaintyScope/1');  % 速度不确定度

% 连接数据记录
add_line(model_name, 'OutputDemux/1', 'DataLogger/1');

% 配置示波器
set_param([model_name '/SpeedScope'], 'YLimits', '[0 25]');
set_param([model_name '/SpeedScope'], 'TimeRange', '30');

% 保存模型
save_system(model_name);

fprintf('✓ 测试模型创建完成！\n\n');
fprintf('=== 模型说明 ===\n');
fprintf('模型名称: %s\n', model_name);
fprintf('仿真时间: 30秒\n');
fprintf('采样时间: 0.01秒\n\n');

fprintf('=== 输入信号 ===\n');
fprintf('1. 轮速: 锯齿波 (0-20 m/s)\n');
fprintf('2. 加速度X: 方波 (±1.5 m/s²)\n');
fprintf('3. 加速度Y: 正弦波 (±0.5 m/s²)\n');
fprintf('4. 航向角速度: 正弦波 (±0.2 rad/s)\n');
fprintf('5. 油门: 常量 (0.3)\n');
fprintf('6. 制动: 常量 (0.0)\n');
fprintf('7. 方向盘转角: 常量 (0.0)\n\n');

fprintf('=== 输出显示 ===\n');
fprintf('1. 速度示波器: 估计速度 vs 轮速\n');
fprintf('2. 位置示波器: X-Y位置轨迹\n');
fprintf('3. 加速度示波器: 估计加速度\n');
fprintf('4. 不确定度示波器: 速度估计不确定度\n\n');

fprintf('=== 下一步操作 ===\n');
fprintf('1. 检查模型连接\n');
fprintf('2. 运行仿真: sim(''%s'')\n', model_name);
fprintf('3. 查看结果: 检查示波器和workspace变量''ukf_results''\n');
fprintf('4. 与TruckSim集成: 替换输入信号源为TruckSim输出\n\n');

% 询问是否立即运行仿真
answer = questdlg('是否立即运行测试仿真？', '运行仿真', '是', '否', '是');
if strcmp(answer, '是')
    fprintf('开始仿真...\n');
    try
        sim(model_name);
        fprintf('✓ 仿真完成！请检查示波器结果。\n');
        
        % 显示简单的结果分析
        if exist('ukf_results', 'var')
            fprintf('\n=== 仿真结果摘要 ===\n');
            fprintf('最终估计速度: %.2f m/s\n', ukf_results.Data(end));
            fprintf('平均速度: %.2f m/s\n', mean(ukf_results.Data));
            fprintf('速度变化范围: %.2f - %.2f m/s\n', min(ukf_results.Data), max(ukf_results.Data));
        end
    catch ME
        fprintf('✗ 仿真失败: %s\n', ME.message);
    end
end

end
