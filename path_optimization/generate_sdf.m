function sdf_map = generate_sdf(binary_map)
    % binary_map: 二值化地图，0 表示障碍物，1 表示自由空间

    % 使用 bwdist 函数计算 8SSEDT
    dist_transform1 = bwdist(~binary_map); % 使用 ~binary_map 来反转二值化地图

    % sdf_map_out = -dist_transform1 .* (binary_map - 1) + dist_transform1 .* binary_map;

    dist_transform2 = bwdist(binary_map);

    % sdf_map_in = -dist_transform2 .* (binary_map - 1) + dist_transform2 .* binary_map;

    sdf_map = dist_transform1-dist_transform2;
    % sdf_map = dist_transform2;

end

