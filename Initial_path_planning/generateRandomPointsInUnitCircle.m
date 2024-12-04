
function randomPoints = generateRandomPointsInUnitCircle(numPoints)
    % 生成位于单位圆内的随机点
    % 输入参数：
    %   numPoints: 要生成的随机点的数量
    % 输出参数：
    %   randomPoints: 包含随机点坐标的矩阵，每行是一个点的坐标（x, y）

    % 生成随机角度
    theta = 2 * pi * rand(1, numPoints);

    % 生成随机半径（在0到1之间）
    radius = sqrt(rand(1, numPoints));

    % 将极坐标转换为直角坐标
    x = radius .* cos(theta);
    y = radius .* sin(theta);

    % 构建包含随机点坐标的矩阵
    randomPoints = [x; y]';
    % 
    % % 显示生成的随机点
    % scatter(x, y, 'filled');
    % axis square;
    % title('Random Points in Unit Circle');
    % xlabel('X-axis');
    % ylabel('Y-axis');
end