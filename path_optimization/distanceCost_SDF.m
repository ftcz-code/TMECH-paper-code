%中文版
function [cost]=distanceCost_SDF(path,pixels,pixels2)
%% 路径成本
map=imread('bendair.bmp');
map1 = imread('bendnew1.bmp');
alfa1=4;
cost1=0;
for i=1:size(path)-1
  cost1 = cost1+sqrt((path(i,1)-path(i+1,1)).^2 + (path(i,2)-path(i+1,2)).^2)+alfa1.*(path(i,3)-path(i+1,3)).^2;
end
%% 计算扫描体积
[mm,n]=size(path);
allPoints = [];
%% 实验2工件
for i=1:3:length(path)-650
    k=0;
    kk=0;
    kkk=0;
    kkkk=0;
    %右侧
    alfa=pi/6-path(i,3);
    xr=uint16(115.5*cos(alfa));
    bend=[];
    bend(1,1)=path(i,1);
    bend(1,2)=path(i,2);
    %右侧2
    alfa1=pi/6+path(i,3);
    xrr=uint16(115.5*cos(alfa1));
    %左侧
    alfa3=pi/6+path(i,3);
    xl=uint16(442.75*cos(alfa3));
    bend1=[];
    bend1(1,1)=path(i,1);
    bend1(1,2)=path(i,2);
     for  rr=1:1:xr
        bend(rr+1,2) = bend(rr,2)+1;
        k=k+1;
        bend(rr+1,1) = -(k)*tan(alfa)+bend(1,1);
     end
    
     for  rr=1:1:xrr
        bend(rr+xr+1,2) = bend(rr+xr,2)+1;
        kk=kk+1;
        bend(rr+xr+1,1) = (kk)*tan(alfa1)+bend(xr+1,1);
     end
    
      for  rr=1:1:xl
        bend1(rr+1,2) = bend1(rr,2)-1;
        kkkk=kkkk+1;
        bend1(rr+1,1) = -(kkkk)*tan(alfa3)+bend1(1,1);
      end
    allPoints = [allPoints; bend; bend1];
end
point = round(allPoints);
% 将指定像素设置为黑色
for j = 1:size(point, 1)
    x = point(j, 1);
    y = point(j, 2);
    % 设置指定像素为黑色
    map(x, y) = 0; % 黑色
    % 设置周围一圈像素为黑色
    for dx = -1:1
        for dy = -1:1
            map(x + dx, y + dy) =0; % 黑色
        end
    end
end
%% 计算扫描体积的SDF
sdf_map = generate_sdf(map);

for i = 1:size(pixels, 1)
    y = pixels(i, 1);
    x = pixels(i, 2);
    sdf_value1(i)=sdf_map(y+1,x+1);
end
sdf_value_sum1 = 0;
for i=1:length(sdf_value1)
sdf_value_sum1 = sdf_value_sum1+sdf_value1(i);
end
sdf_value_sum1= sdf_value_sum1/length(sdf_value1);
mu1 = 28; % 均值，即 sdf_value_sum1在最优时的值
sigma =1.2; % 标准差，用于控制正态分布的形状
pdf_value_specific1 = 1- normpdf(sdf_value_sum1, mu1, sigma);
cost2 = pdf_value_specific1 *200;

for i = 1:size(pixels2, 1)
    y = pixels2(i, 1);
    x = pixels2(i, 2);
    sdf_value2(i)=sdf_map(y+1,x+1);
end
sdf_value_sum2 = 0;
for i=1:length(sdf_value2)
sdf_value_sum2 = sdf_value_sum2+sdf_value2(i);
end
sdf_value_sum2= sdf_value_sum2/length(sdf_value2);
mu2 = 100; % 均值，即 sdf_value_sum1在最优时的值
sigma =1.2; % 标准差，用于控制正态分布的形状
pdf_value_specific2 = 1- normpdf(sdf_value_sum2, mu2, sigma);
cost3 = pdf_value_specific2 *200;

% for i=1:1:length(sdf_value1)
%     if sdf_value1(i) < -1.5
%         cost2 = 2000;
%         break;  
%     end
% end
% for i=1:1:length(sdf_value2)
%     if sdf_value2(i) < -1
%         cost3 = 2000;
%         break;  
%     end
% end

for i=2:1:length(path)
    if checkPath4(path(i,:),map1)~=1
        cost1 = 2000;
        break;  
    end
end

cost = cost1+1.5*(cost2+cost3);
