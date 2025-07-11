function compile_truck_ukf_sfunc()
% COMPILE_TRUCK_UKF_SFUNC 编译半挂卡车UKF S-Function
%
% 此脚本用于编译truck_ukf_sfunc.cpp为MEX文件，以便在Simulink中使用
%
% 使用方法:
%   1. 确保MATLAB已安装C++编译器 (运行 mex -setup 检查)
%   2. 确保已安装Eigen库
%   3. 在MATLAB中运行此脚本: compile_truck_ukf_sfunc()
%
% 依赖:
%   - Eigen3 库 (用于矩阵运算)
%   - C++11 或更高版本编译器
%
% 作者: UKF Speed Estimation Team
% 日期: 2025年7月

fprintf('正在编译半挂卡车UKF S-Function...\n');

% 获取当前脚本路径
script_path = fileparts(mfilename('fullpath'));
project_root = fullfile(script_path, '..');

% 源文件路径
src_files = {
    fullfile(script_path, 'truck_ukf_sfunc.cpp')
    fullfile(project_root, 'src', 'truck_ukf.cpp')
};

% 头文件路径
include_dirs = {
    fullfile(project_root, 'include')
    '/usr/include/eigen3'  % Ubuntu/Debian Eigen路径
    '/usr/local/include/eigen3'  % 自编译Eigen路径
    '/opt/homebrew/include/eigen3'  % macOS Homebrew路径
};

% 检查Eigen库是否存在
eigen_found = false;
for i = 1:length(include_dirs)
    eigen_path = fullfile(include_dirs{i}, 'Eigen', 'Dense');
    if exist(eigen_path, 'file')
        fprintf('找到Eigen库: %s\n', include_dirs{i});
        eigen_found = true;
        break;
    end
end

if ~eigen_found
    error(['未找到Eigen库！请安装Eigen3:\n' ...
           'Ubuntu/Debian: sudo apt-get install libeigen3-dev\n' ...
           'macOS: brew install eigen\n' ...
           '或从 https://eigen.tuxfamily.org 下载源码编译']);
end

% 编译选项
compile_flags = {
    '-std=c++11'
    '-O2'
    '-DMATLAB_MEX_FILE'
    '-largeArrayDims'
};

% 构建MEX命令
mex_cmd = 'mex';

% 添加编译标志
for i = 1:length(compile_flags)
    mex_cmd = [mex_cmd ' CXXFLAGS="$CXXFLAGS ' compile_flags{i} '"'];
end

% 添加头文件路径
for i = 1:length(include_dirs)
    if exist(include_dirs{i}, 'dir')
        mex_cmd = [mex_cmd ' -I' include_dirs{i}];
    end
end

% 添加源文件
for i = 1:length(src_files)
    if exist(src_files{i}, 'file')
        mex_cmd = [mex_cmd ' ' src_files{i}];
    else
        error('源文件不存在: %s', src_files{i});
    end
end

% 设置输出目录
output_dir = script_path;
mex_cmd = [mex_cmd ' -outdir ' output_dir];

fprintf('执行编译命令:\n%s\n\n', mex_cmd);

try
    % 执行编译
    eval(mex_cmd);
    fprintf('\n✓ 编译成功！\n');
    fprintf('S-Function已生成: %s\n', fullfile(output_dir, 'truck_ukf_sfunc.mexw64'));
    
    % 验证编译结果
    if ispc
        mex_file = fullfile(output_dir, 'truck_ukf_sfunc.mexw64');
    elseif ismac
        mex_file = fullfile(output_dir, 'truck_ukf_sfunc.mexmaci64');
    else
        mex_file = fullfile(output_dir, 'truck_ukf_sfunc.mexa64');
    end
    
    if exist(mex_file, 'file')
        fprintf('验证通过: MEX文件已生成\n');
        
        % 提供使用说明
        fprintf('\n=== 使用说明 ===\n');
        fprintf('1. 在Simulink模型中添加S-Function块\n');
        fprintf('2. 设置S-Function名称为: truck_ukf_sfunc\n');
        fprintf('3. 设置S-Function参数 (7个参数):\n');
        fprintf('   - 轴距 (m): 3.8\n');
        fprintf('   - 质量 (kg): 15000\n');
        fprintf('   - 迎风面积 (m²): 10.0\n');
        fprintf('   - 风阻系数: 0.6\n');
        fprintf('   - 初始速度 (m/s): 0.0\n');
        fprintf('   - 采样时间 (s): 0.01\n');
        fprintf('   - 启用日志: 0\n');
        fprintf('4. 配置输入输出端口\n');
        fprintf('5. 运行仿真\n\n');
        
    else
        warning('MEX文件未找到，编译可能失败');
    end
    
catch ME
    fprintf('\n✗ 编译失败！\n');
    fprintf('错误信息: %s\n', ME.message);
    
    % 提供故障排除建议
    fprintf('\n=== 故障排除 ===\n');
    fprintf('1. 检查是否安装了C++编译器:\n');
    fprintf('   运行: mex -setup\n\n');
    fprintf('2. 检查Eigen库安装:\n');
    fprintf('   Ubuntu/Debian: sudo apt-get install libeigen3-dev\n');
    fprintf('   macOS: brew install eigen\n\n');
    fprintf('3. 检查源文件是否存在:\n');
    for i = 1:length(src_files)
        fprintf('   %s: %s\n', src_files{i}, ...
            char("存在" * exist(src_files{i}, 'file') + "不存在" * ~exist(src_files{i}, 'file')));
    end
    
    rethrow(ME);
end

end

% 辅助函数：检查编译器
function check_compiler()
    fprintf('检查MATLAB编译器配置...\n');
    try
        cc = mex.getCompilerConfigurations('C++', 'Selected');
        if isempty(cc)
            fprintf('警告: 未配置C++编译器\n');
            fprintf('请运行: mex -setup C++\n');
        else
            fprintf('C++编译器: %s\n', cc.Name);
        end
    catch
        fprintf('无法获取编译器信息\n');
    end
end
