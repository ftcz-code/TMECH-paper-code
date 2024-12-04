function [Highriskarea,Lowriskareas] = feature_extraction(img)

% img = imread('bendnew3.bmp');

% 检查图像是否为 RGB
if ndims(img) == 3
    grayImg = rgb2gray(img);
else
    grayImg = img;
end

% 转换为 double 类型
grayImg = double(grayImg);

% 使用高斯滤波器进行去噪
smoothImg = imgaussfilt(grayImg, 3);
% 正常化到 [0, 1]
smoothImg = (smoothImg - min(smoothImg(:))) / (max(smoothImg(:)) - min(smoothImg(:)));

% figure(1);
% imshow(smoothImg); % 显示高斯滤波后的图像
% hold on;

% 使用 Canny 算子进行边缘检测
edges = edge(smoothImg, 'Canny');
% figure(2);
% imshow(edges);
% title('Canny edge detection results');

%% 区域划分
% 膨胀操作，使边缘封闭
se = strel('disk', 5); % 使用圆形结构元素
dilated_edges = imdilate(edges, se);

% 填充封闭区域
filled_img = imfill(dilated_edges, 'holes');

% 标记连通区域
[labeled_img, num] = bwlabel(filled_img);

% 获取区域属性
props = regionprops(labeled_img, 'Area', 'BoundingBox', 'Centroid');

% 筛选出较大区域，仅保留主要的两个区域
min_area = 2000;  % 根据图像内容调整此值
filtered_props = props([props.Area] > min_area);

% 检查是否检测到至少两个区域
if numel(filtered_props) < 2
    disp('未能检测到足够的区域，请调整最小面积阈值或图像处理步骤。');
else
    % 按垂直位置排序
    % [~, order] = sort([filtered_props.Centroid]); % 按中心点 y 值排序
    centroids_y = arrayfun(@(x) x.Centroid(2), filtered_props);
    [~, order] = sort(centroids_y); % 按 y 值排序

    first_region = filtered_props(order(1));
    second_region = filtered_props(order(2));

    % 获取第一个区域轮廓并提取最上和最下端坐标
    first_boundary = bwboundaries(labeled_img == order(1));
    first_boundary = first_boundary{1};
    [~, min_idx1] = min(first_boundary(:,1));  % 最上端
    [~, max_idx1] = max(first_boundary(:,1));  % 最下端
    first_region_top = first_boundary(min_idx1, :);
    first_region_bottom = first_boundary(max_idx1, :);
    
    % 获取第二个区域轮廓并提取最上和最下端坐标
    second_boundary = bwboundaries(labeled_img == order(2));
    second_boundary = second_boundary{1};
    [~, min_idx2] = min(second_boundary(:,1));  % 最上端
    [~, max_idx2] = max(second_boundary(:,1));  % 最下端
    second_region_top = second_boundary(min_idx2, :);
    second_region_bottom = second_boundary(max_idx2, :);

    % % 显示结果
    % disp('第一块区域最上端坐标:');
    % disp(first_region_top);
    % disp('第一块区域最下端坐标:');
    % disp(first_region_bottom);
    % disp('第二块区域最上端坐标:');
    % disp(second_region_top);
    % disp('第二块区域最下端坐标:');
    % disp(second_region_bottom);

    % % 可视化结果
    % figure(3);
    % imshow(img); % 显示原始图像
    % hold on;
    % 
    % % 绘制区域边界框
    % rectangle('Position', first_region.BoundingBox, 'EdgeColor', 'blue', 'LineWidth', 2);
    % rectangle('Position', second_region.BoundingBox, 'EdgeColor', 'magenta', 'LineWidth', 2);
    % 
    % % 标出第一块区域的最上端和最下端坐标，使用红色和绿色标记
    % plot(first_region_top(2), first_region_top(1), 'ro', 'MarkerSize', 10, 'LineWidth', 2);
    % text(first_region_top(2), first_region_top(1), 'First Top', 'Color', 'red', 'FontSize', 12);
    % plot(first_region_bottom(2), first_region_bottom(1), 'go', 'MarkerSize', 10, 'LineWidth', 2);
    % text(first_region_bottom(2), first_region_bottom(1), 'First Bottom', 'Color', 'green', 'FontSize', 12);
    % 
    % % 标出第二块区域的最上端和最下端坐标，使用红色和绿色标记
    % plot(second_region_top(2), second_region_top(1), 'ro', 'MarkerSize', 10, 'LineWidth', 2);
    % text(second_region_top(2), second_region_top(1), 'Second Top', 'Color', 'red', 'FontSize', 12);
    % plot(second_region_bottom(2), second_region_bottom(1), 'go', 'MarkerSize', 10, 'LineWidth', 2);
    % text(second_region_bottom(2), second_region_bottom(1), 'Second Bottom', 'Color', 'green', 'FontSize', 12);

    % hold off;
end

%% 分类
% 使用 Sobel 算子计算梯度方向和梯度强度
[Gx, Gy] = imgradientxy(smoothImg, 'sobel');
[~, Gdir] = imgradient(Gx, Gy); % 计算边缘方向

% 定义滑动窗口大小和滑动步长
windowSize = 3; % 窗口大小可以调整
stepSize = 2; % 窗口的滑动步长
[rows, cols] = size(edges);
features = []; % 用于存储窗口特征
directions = []; % 存储窗口方向
positions = []; % 存储每个窗口的位置

% 滑动窗口遍历图像
for i = 1:stepSize:rows-windowSize+1
    for j = 1:stepSize:cols-windowSize+1
        % 获取窗口内的边缘区域
        window = edges(i:i+windowSize-1, j:j+windowSize-1);
        windowDir = Gdir(i:i+windowSize-1, j:j+windowSize-1);
        
        % 如果窗口内没有边缘则跳过
        if sum(window(:)) == 0
            continue;
        end
        
        % 计算平均斜率方向
        meanDir = mean(windowDir(window)); % 仅计算边缘像素的平均方向
        
        % 根据斜率方向初步分类
        if (abs(meanDir) <= 15) || (abs(meanDir) >= 165)
            direction = 1; % 水平
        elseif (abs(meanDir) >= 75 && abs(meanDir) <= 105)
            direction = 2; % 垂直
        else
            direction = 3; % 斜向
        end

        % 存储窗口的方向和位置
        directions = [directions; direction];
        positions = [positions; i, j]; % 记录窗口位置
    end
end

% 自定义位置划分规则，每个方向分成多类
positionClasses = cell(3, 1); % 存储每个方向的分类结果


  % midpointY = second_region_top(1)+50;
  % midpointY1 = second_region_top(1)+230;
  % midpointY2 = first_region_bottom(1)-50;
  % midpointY3 = first_region_bottom(1)-250;
   
  midpointY = second_region_top(1)+80;
  midpointY1 = second_region_top(1)+215;
  midpointY2 = first_region_bottom(1)-50;
  midpointY3 = first_region_bottom(1)-255;


for d = 1:3
    % 获取当前方向的窗口位置
    directionPositions = positions(directions == d, :);
    
    % 根据方向定义位置范围
    if d == 1  % 水平方向：划分为上、中、下部分
        left = cols / 3;
        right = 2 * cols / 3;
      
        
        classes = zeros(size(directionPositions, 1), 1);
        classes(directionPositions(:, 1) < midpointY2 & directionPositions(:, 1)> midpointY3) = 1; % 上部区域
        classes(directionPositions(:, 1) < midpointY & directionPositions(:, 1) >= midpointY2) = 2; % 中部区域
        classes(directionPositions(:, 1) >= midpointY & directionPositions(:, 1) < midpointY1) = 3; % 下部区域
        
    elseif d == 2  % 垂直方向：划分为上、中、下
        top = rows / 3;
        bottom = 2 * rows / 3;
        classes = zeros(size(directionPositions, 1), 1);
        classes(directionPositions(:, 1) < midpointY2 & directionPositions(:, 1)> midpointY3) = 1; % 上部区域
        classes(directionPositions(:, 1) < midpointY & directionPositions(:, 1) >= midpointY2) = 2; % 中部区域
        classes(directionPositions(:, 1) >= midpointY & directionPositions(:, 1) < midpointY1) = 3; % 下部区域
        
    else  % 斜向方向：划分为上、中、下部分
        midpointX = cols / 2;
        classes = zeros(size(directionPositions, 1), 1);
        classes(directionPositions(:, 1) < midpointY2 & directionPositions(:, 1)> midpointY3) = 1; % 上部区域
        classes(directionPositions(:, 1) < midpointY & directionPositions(:, 1) >= midpointY2) = 2; % 中部区域
        classes(directionPositions(:, 1) >= midpointY & directionPositions(:, 1) < midpointY1) = 3; % 下部区域
    end
    
    % 存储分类结果
    positionClasses{d} = classes;
end

% 可视化分类结果，每个方向的不同位置类别用不同颜色表示
% figure;
% colors = {[1, 0.7, 0.7], [1, 0.5, 0.5], [1, 0, 0]; ...     % 水平类别 1 (浅红) - 3 (深红)
%           [0.7, 1, 0.7], [0.5, 1, 0.5], [0, 1, 0]; ...     % 垂直类别 1 (浅绿) - 3 (深绿)
%           [0.7, 0.7, 1], [0.5, 0.5, 1], [0, 0, 1]};        % 斜向类别 1 (浅蓝) - 3 (深蓝)

Highriskarea = [];
Lowriskareas = [];
a = 1;
j = 1;

for d = 1:3
    % 创建子图并显示原始图像
    % subplot(2, 3, d);
    % imshow(img);
    % hold on;
    % title(['Edge regions in direction ', num2str(d)]);
    
    % 获取当前方向的分类结果
    directionPositions = positions(directions == d, :);
    directionClasses = positionClasses{d};
    
    % 可视化当前方向的分类
    for k = 1:length(directionClasses)
        % 获取窗口位置和类别索引
        pos = directionPositions(k, :);
        class = directionClasses(k);

        % 获取高风险区域
        if class == 2
           Highriskarea(a,:) = pos;
           a=a+1;
        end
        
        % 获取低风险区域
        if class ~= 2  && class ~= 0
           Lowriskareas(j,:) = pos;
           j=j+1;
        end    
        
        % % 判断 class 是否为 0
        % if class ~= 0
        %     % 当 class 不为 0 时，使用对应的颜色
        %     color = colors{d, class};
        %     rectangle('Position', [pos(2), pos(1), windowSize, windowSize], ...
        %           'EdgeColor', color, 'LineWidth', 1.5);
        % else
        %     % 当 class 为 0 时，跳过或设置默认值
        %     color = []; % 或者设置为某种默认颜色，例如：[0 0 0] 表示黑色
        % end

    end

    % hold off;
end

%% 可视化高风险区域
% subplot(2, 3, 4);
% figure(1);
% imshow(img);
% hold on;
% title('High-Risk Edge Regions');
% 
% color1 = [1, 0.5, 0];
% 
% for m = 1:length(Highriskarea)
%     rectangle('Position', [Highriskarea(m,2), Highriskarea(m,1), windowSize, windowSize], ...
%                   'EdgeColor', color1, 'LineWidth', 1.5);
% end
% 
% %% 可视化低风险区域
% % subplot(2, 3, 5);
% figure(2);
% imshow(img);
% hold on;
% title('Low-Risk Edge Regions');
% 
% color2 = [1, 0.75, 0.8];
% 
% for m = 1:length(Lowriskareas)
%     rectangle('Position', [Lowriskareas(m,2), Lowriskareas(m,1), windowSize, windowSize], ...
%                   'EdgeColor', color2, 'LineWidth', 1.5);
% % end
% hold off;
end