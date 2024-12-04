close all;
figure(1)
map=imread('bend_map.bmp');
index =23;
   	imshow(map);
    % hold on;
    % scatter(qinit(2),qinit(1),50,'sm','filled');
    % hold on;
    % scatter(qgoal(2),qgoal(1),50,'sb','filled');
    hold on
%% 优化前的图
for i=1:1:length(pop(index).position) 
    k=0;
    kk=0;
    kkk=0;
    kkkk=0;
    %右侧
    alfa=pi/6-pop(index).position(i,3);
    xr=uint16(115.5*cos(alfa));
    bend=[];
    bend(1,1)=pop(index).position(i,1);
    bend(1,2)=pop(index).position(i,2);
    %右侧1
    alfa1=pi/6+pop(index).position(i,3);
    xrr=uint16(115.5*cos(alfa1));
    % %右侧2
    % alfa2=52.5*pi/180-path(i,3);
    % xrrr=uint16(115.5*cos(alfa2));
    %左侧
    alfa3=pi/6+pop(index).position(i,3);
    xl=uint16(442.75*cos(alfa3));
    bend1=[];
    bend1(1,1)=pop(index).position(i,1);
    bend1(1,2)=pop(index).position(i,2);

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

      % for  rr=1:1:xrrr
      %   bend(rr+xrr+xr+1,2) = bend(rr+xrr+xr,2)+1;
      %   kkk=kkk+1;
      %   bend(rr+xrr+xr+1,1) = -(kkk)*tan(alfa2)+bend(xrr+xr+1,1);
      % end

      for  rr=1:1:xl
        bend1(rr+1,2) = bend1(rr,2)-1;
        kkkk=kkkk+1;
        bend1(rr+1,1) = -(kkkk)*tan(alfa3)+bend1(1,1);
      end

    if i==1
     % line(bend(:,2),bend(:,1),'linestyle','-','linewidth',2,'color','k');
     plot(bend(:,2),bend(:,1),'color','#FF0000','linewidth',3);
     plot(bend1(:,2),bend1(:,1),'color','#FF0000','linewidth',3);
     

    elseif i==length(Xmin)
     plot(bend(:,2),bend(:,1),'color','#0000FF','linewidth',3);
     plot(bend1(:,2),bend1(:,1),'color','#0000FF','linewidth',3); 

    else
     plot(bend(:,2),bend(:,1),'color',[0, 0.45, 0.74],'linestyle',':');
     plot(bend1(:,2),bend1(:,1),'color',[0, 0.45, 0.74],'linestyle',':');
    end
end

line(pop(index).position(:,2),pop(index).position(:,1),'linestyle','-','linewidth',5,'color','#F5A827');

%% 绘制优化后的图
figure(2)

   	imshow(map);
    % hold on;
    % scatter(qinit(2),qinit(1),50,'sm','filled');
    % hold on;
    % scatter(qgoal(2),qgoal(1),50,'sb','filled');
    hold on;
%% 总体框图用
for i=1:1:length(Xmin) 
    k=0;
    kk=0;
    kkk=0;
    kkkk=0;
    %右侧
    alfa=pi/6-Xmin(i,3);
    xr=uint16(115.5*cos(alfa));
    bend=[];
    bend(1,1)=Xmin(i,1);
    bend(1,2)=Xmin(i,2);
    %右侧1
    alfa1=pi/6+Xmin(i,3);
    xrr=uint16(115.5*cos(alfa1));
    % %右侧2
    % alfa2=52.5*pi/180-path(i,3);
    % xrrr=uint16(115.5*cos(alfa2));
    %左侧
    alfa3=pi/6+Xmin(i,3);
    xl=uint16(442.75*cos(alfa3));
    bend1=[];
    bend1(1,1)=Xmin(i,1);
    bend1(1,2)=Xmin(i,2);

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

    if i==1
     % line(bend(:,2),bend(:,1),'linestyle','-','linewidth',2,'color','k');
     plot(bend(:,2),bend(:,1),'color','#FF0000','linewidth',3);
     plot(bend1(:,2),bend1(:,1),'color','#FF0000','linewidth',3);
     

    elseif i==length(Xmin)
     plot(bend(:,2),bend(:,1),'color','#0000FF','linewidth',3);
     plot(bend1(:,2),bend1(:,1),'color','#0000FF','linewidth',3); 

    else
     plot(bend(:,2),bend(:,1),'color',[0, 0.45, 0.74],'linestyle',':');
     plot(bend1(:,2),bend1(:,1),'color',[0, 0.45, 0.74],'linestyle',':');
    end
end

line(Xmin(:,2),Xmin(:,1),'linestyle','-','linewidth',5,'color','#F5A827');


figure(3)
plot(pop(index).position(:, 3), 'LineWidth', 2, 'Color', 'r'); % 设置线条为红色
xlabel('Path Point Index', 'FontSize', 12); % 设置X轴标签
ylabel('Value of attitude', 'FontSize', 12); % 设置Y轴标签
title('Attitude change process', 'FontSize', 14); % 添加标题
set(gca, 'XLim', [0 55], 'YLim', [-1 1], 'FontSize', 12); % 设置坐标轴范围及字体
grid on; % 打开网格
% 可以选择在图形上添加水平线或参考线
% 例如添加 y = 0 的水平线
yline(0, 'k--', 'LineWidth', 1.5); 
% 调整图形的背景颜色
set(gca, 'Color', [0.95 0.95 0.95]);  % 淡灰色背景
hold off;