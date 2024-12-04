%% 总体框图用
path =Xmin;
% path =pop(23).position;
% path =path30(23).position;
map=imread('bendair.bmp');
img = imread('bendnew3.bmp');
map1 = imread('bendnew1.bmp');
% 检查图像是否为 RGB
if ndims(img) == 3
    grayImg = rgb2gray(img);
else
    grayImg = img;
end
allPoints = [];
pixels = [];
pixels2 = [];
[pixels,pixels2]=feature_extraction(img);

for i=1:1:length(path) 
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
    %右侧1
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

% 定义感兴趣区域像素点
% pixels = [655, 913;
%           655, 914;
%           655, 915;
%           655, 916;
%           655, 917;
% 
%           654, 912;
%           654, 913;
%           654, 914;
%           654, 915;
%           654, 916;
%           654, 917;
%           654, 918;
% 
%           653, 911;
%           653, 912;
%           653, 913;
%           653, 917;
%           653, 918;
%           653, 919;
% 
%           652, 910;
%           652, 911;
%           652, 912;
%           652, 918;
%           652, 919;
%           652, 920;
% 
%           651, 909;
%           651, 910;
%           651, 911;
%           651, 919;
%           651, 920;
%           651, 921;
% 
%           650, 908;
%           650, 909;
%           650, 910;
%           650, 920;
%           650, 921;
%           650, 922;
% 
%           649, 907;
%           649, 908;
%           649, 909;
%           649, 921;
%           649, 922;
%           649, 923;
% 
%           648, 906;
%           648, 907;
%           648, 908;
%           648, 922;
%           648, 923;
%           648, 924;
% 
%           647, 905;
%           647, 906;
%           647, 907;
%           647, 923;
%           647, 924;
%           647, 925;
% 
%           646, 904;
%           646, 905;
%           646, 906;
%           646, 924;
%           646, 925;
%           646, 926;
% 
%           645, 903;
%           645, 904;
%           645, 905;
%           645, 925;
%           645, 926;
%           645, 927;
% 
% 
%           644, 902;
%           644, 903;
%           644, 904;
%           644, 926;
%           644, 927;
%           644, 928;
% 
%           643, 901;
%           643, 902;
%           643, 903;
%           643, 927;
%           643, 928;
%           643, 929;
% 
%           642, 900;
%           642, 901;
%           642, 902;
%           642, 928;
%           642, 929;
% 
% 
%           726, 898;
%           726, 899;
%           726, 900;
%           726, 901;
%           726, 902;
%           726, 903;
%           726, 904;
%           726, 905;
%           726, 906;
% 
%           727, 898;
%           727, 899;
%           727, 900;
%           727, 901;
%           727, 902;
%           727, 903;
%           727, 904;
%           727, 905;
%           727, 906;
%           727, 907;
%           727, 924;
%           727, 925;
%           727, 926;
%           727, 927;
%           727, 928;
%           727, 929;
%           727, 930;
% 
%           728, 899;
%           728, 908;
%           728, 907;
%           728, 906;
%           728, 921;
%           728, 922;
%           728, 923;
% 
%           729, 899;
%           729, 909;
%           729, 908;
%           729, 907;
%           729, 920;
%           729, 921;
%           729, 922;
% 
%           730, 899;
%           730, 910;
%           730, 909;
%           730, 908;
%           730, 919;
%           730, 920;
%           730, 921;
% 
%           731, 899;
%           731, 911;
%           731, 910;
%           731, 909;
%           731, 918;
%           731, 919;
%           731, 920;
% 
%           732, 899;
%           732, 912;
%           732, 911;
%           732, 910;
%           732, 917;
%           732, 918;
%           732, 919;
% 
% 
%           733, 899;
%           733, 913;
%           733, 912;
%           733, 911;
%           733, 916;
%           733, 917;
%           733, 918;
% 
%           734, 917;
%           734, 916;
%           734, 915;
%           734, 914;
%           734, 913;
%           734, 912;
% 
%           735, 916;
%           735, 914;
%           735, 915;
%           735, 913;
% 
%           736, 914;
%           736, 915;
% 
%           ];

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

mu1 = 5; % 均值，即 sdf_value_sum1在最优时的值
sigma =1.2; % 标准差，用于控制正态分布的形状

pdf_value_specific1 = 1- normpdf(sdf_value_sum1, mu1, sigma);
cost2 = pdf_value_specific1 *200;

%%
% pixels2 =[641, 901;
%           641, 902;
%           641, 903;
%           641, 928;
%           641, 929;
% 
%           640, 929;
%           640, 928;
%           640, 927;
%           640, 900;
%           640, 901;
%           640, 902;
% 
%           639, 899;
%           639, 900;
%           639, 901;
%           639, 926;
%           639, 927;          
%           639, 928;
% 
%           638, 898;
%           638, 899;
%           638, 900;
%           638, 926;
%           638, 927;
% 
%           637, 897;
%           637, 898;
%           637, 899;
%           637, 925;
%           637, 926;
%           637, 927;
% 
%           636, 896;
%           636, 897;
%           636, 898;
%           636, 924;
%           636, 925;
%           636, 926;
% 
%           635, 895;
%           635, 896;
%           635, 897;
%           635, 923;
%           635, 924;
%           635, 925;
% 
%           634, 894;
%           634, 895;
%           634, 896;
%           634, 922;
%           634, 923;
%           634, 924;
% 
%           633, 893;
%           633, 894;
%           633, 895;
%           633, 922;
%           633, 923;
% 
%           632, 892;
%           632, 893;
%           632, 894;
%           632, 922;
%           632, 923;
% 
%           631, 891;
%           631, 892;
%           631, 893;
%           631, 922;
%           631, 923;
% 
%           630, 890;
%           630, 891;
%           630, 892;
% 
%           629, 889;
%           629, 890;
%           629, 891;
% 
%           628, 888;
%           628, 889;
%           628, 890;
% 
%           627, 887;
%           627, 888;
%           627, 889;
% 
%           626, 886;
%           626, 887;
%           626, 888;
% 
%           625, 885;
%           625, 886;
%           625, 887;
% 
%           624, 884;
%           624, 885;
%           624, 886;
% 
%           623, 883;
%           623, 884;
%           623, 885;
% 
%           622, 882;
%           622, 883;
%           622, 884;
% 
%           621, 881;
%           621, 882;
%           621, 883;
% 
%           620, 880;
%           620, 881;
%           620, 882;
% 
%           619, 879;
%           619, 880;
%           619, 881;
% 
%           618, 878;
%           618, 879;
%           618, 880;
% 
%           617, 877;
%           617, 878;
%           617, 879;
% 
%           616, 876;
%           616, 877;
%           616, 878;
% 
%           615, 875;
%           615, 876;
%           615, 877;
% 
%           614, 874;
%           614, 875;
%           614, 876;
% 
%           613, 874;
%           613, 875;
% 
%     ];

for i= 571 :1: 612
    pixels2 =[pixels2;i,875];
    pixels2 =[pixels2;i,874];
end

for i= 728 :1: 776
    pixels2 =[pixels2;i,899];
    pixels2 =[pixels2;i,898];
end

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

mu2 = 18; % 均值，即 sdf_value_sum1在最优时的值
sigma =1.2; % 标准差，用于控制正态分布的形状

pdf_value_specific2 = 1- normpdf(sdf_value_sum2, mu2, sigma);
cost3 = pdf_value_specific2 *200;

%% 路径成本
alfa=4;
cost1=0;
for i=1:size(path)-1
  cost1 = cost1+sqrt((path(i,1)-path(i+1,1)).^2 + (path(i,2)-path(i+1,2)).^2)+alfa.*(path(i,3)-path(i+1,3)).^2;
end


for i=2:1:length(path)
    feasiable = checkPath4(path(i,:),map1)
    if feasiable ~=1
        cost1 = 2000;
        break;  
    end
end

