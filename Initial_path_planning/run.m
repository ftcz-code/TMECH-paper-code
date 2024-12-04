%--------------------------------------------------------------------------
% File Name: run.m
% Date: December 4, 2024
% Author: Miao Yiyang
% Affiliation: NJUPT
% Description: This program implements the initial path planning for bending feeding 
%              and unloading of workpieces based on the improved SBG-RRT-Connect algorithm.
%              The main functions include:
%              1. Path planning using RRT-Connect, SBG-RRT-Connect, and improved SBG-RRT-Connect algorithms.
%              2. Outputting the motion path of the workpiece and the posture change curve.
% Dependencies:
%       - bend_map.bmp: Obstacle map
%       - rrt_connect_bend.m: RRT-Connect algorithm
%       - Synchronized_bend.m: SBG-RRT-Connect algorithm
%       - improved_Synchronized_bend.m: Improved SBG-RRT-Connect algorithm
%       - checkpath4.m: Path collision detection
%       - distanceCost2.m: Path cost calculation
%       - feasiblePoint_bend.m: Workpiece collision detection function
%       - informed_new_node.m: Elliptical region sampling
%--------------------------------------------------------------------------

close all;clc;
%% Obtain the bending map
map=imread('bend_map1.bmp');
map2=imread('bend_mapshow.bmp');

%% Define initialization parameters, 
% counting from the top left corner as the origin, 
% where the first number in the matrix represents y and the second number represents x
qinit = [730  916 0];          %start point
qgoal = [480 460 -0.523];      %Target point
stepsize = 4;                  %Displacement step size
disTh = 2;                     %threshold 
zstepsize=0.0349;              %Rotating sampling step size 0.01744; 0.0349  0.0523 0.0872
maxFailAttempts = 15000;       %Search limit
counter = 0;                   %Counting times
display = 1;                   %Display the above data as true
check4 = @checkPath4;          %Select the collision detection function for the corresponding shape
time=[];
plength=[];
iterition=[];
points=[];

%% Initialization of population structure for repeated path planning
ind.position=[];
ind.cost=[];
path30=repmat(ind,30,1);

%% Determine whether the starting point and target point meet the requirements
if feasiblePoint(qinit,map) ==0
	error('起始点不符合地图要求');
end

if feasiblePoint(qgoal,map) ==0
	error('目标点不符合地图要求');
end
%% Display map
if display
	imshow(map2);
    %rectangle('position',[1 1 size(map)-1],'LineWidth', 2,'edgecolor','k');
    hold on;
    scatter(qinit(2),qinit(1),'sm','filled');
    hold on;
    scatter(qgoal(2),qgoal(1),'sb','filled');
end
% for ii=1:1:30
%% Program starts and timing
tic;
RRTree1 = double([qinit 0]);    %Starting point random tree
RRTree2 = double([qgoal -1]);   %Termination point random tree
Tree1ExpansionFail = 0;
Tree2ExpansionFail = 0;
while ~Tree1ExpansionFail || ~Tree2ExpansionFail
    %% Expand the first lesson tree
    if ~Tree1ExpansionFail
        % [RRTree1,pathFound,Tree1ExpansionFail] = rrt_connect_bend(check4,RRTree1,RRTree2,qgoal,stepsize,zstepsize,maxFailAttempts,disTh,map);
        % [RRTree1,pathFound,Tree1ExpansionFail] = Synchronized_bend(check4,RRTree1,RRTree2,qgoal,stepsize,zstepsize,maxFailAttempts,disTh,map);
        [RRTree1,pathFound,Tree1ExpansionFail] = improved_Synchronized_bend(check4,RRTree1,RRTree2,qgoal,stepsize,zstepsize,maxFailAttempts,disTh,map,qinit(1:2),qgoal(1:2));%扩展Tree1，Tree1ExpansionFail的返回值永远为0
         if ~Tree1ExpansionFail && isempty(pathFound) && display   %%If the tree extension is successful but has not reached the final point
           % Tree1 connects the new node to the parent node
             line([RRTree1(end,2);RRTree1(RRTree1(end,4),2)],[RRTree1(end,1);RRTree1(RRTree1(end,4),1)],'color','#D9958F','linewidth',1.5,'marker','.','linestyle',':','markersize',10);
             counter=counter+1;M(counter)=getframe;
         end
    end
    %% Expand the second lesson tree
    if ~Tree2ExpansionFail
         % [RRTree2,pathFound,Tree2ExpansionFail] = rrt_connect_bend(check4,RRTree2,RRTree1,qinit,stepsize,zstepsize,maxFailAttempts,disTh,map);
         % [RRTree2,pathFound,Tree2ExpansionFail] = Synchronized_bend(check4,RRTree2,RRTree1,qinit,stepsize,zstepsize,maxFailAttempts,disTh,map);
         [RRTree2,pathFound,Tree2ExpansionFail] = improved_Synchronized_bend(check4,RRTree2,RRTree1,qinit,stepsize,zstepsize,maxFailAttempts,disTh,map,qinit(1:2),qgoal(1:2));
       %% Swap positions
        if ~isempty(pathFound)
            pathFound(4:5)=pathFound(5:-1:4); 
        end 
    end
    %% Exchange search order
    Swap(RRTree1,RRTree2);
    %% Find the path
    if ~isempty(pathFound)  
        if display
            line([RRTree1(pathFound(1,4),2);pathFound(1,2);RRTree2(pathFound(1,5),2)],[RRTree1(pathFound(1,4),1);pathFound(1,1);RRTree2(pathFound(1,5),1)],'color','g','linewidth',6);
            counter=counter+1;M(counter)=getframe;
        end
        path=[pathFound(1,1:3)]; % compute path
        prev=pathFound(1,4);     % add nodes from RRT 1 first
        while prev > 1
            path=[RRTree1(prev,1:3);path];
            prev=RRTree1(prev,4);
        end
        prev=pathFound(1,5);     % then add nodes from RRT 2
        while prev > 0
            path=[path;RRTree2(prev,1:3)];
            prev=RRTree2(prev,4);  
        end
         path=[qinit;path];
        break;
    end
end
pathLength=0;
for i=1:length(path)-1, pathLength=pathLength+distanceCost2(path(i,:),path(i+1,:)); end
line(path(:,2),path(:,1),'linestyle','-','linewidth',2,'color','#C00000');
toc;

% path30(ii).position = path;
% 
% time=[time; toc];
% pathLength
% [a,b]=size(RRTree1)
% [c,d]=size(RRTree2)
% a+c
% [e,f]=size(path)
% 
% 
% plength=[plength;pathLength]
% iterition=[iterition;a+c]
% points=[points;e]
% end

%% Draw a motion diagram of obstacle avoidance for sheet metal parts
[mm,n]=size(path);
for i=1:1:size(path) 
k=0;
kk=0;
kkk=0;
kkkk=0;
%Right side
alfa=pi/6-path(i,3);
xr=uint16(115.5*cos(alfa));
bend=[];
bend(1,1)=path(i,1);
bend(1,2)=path(i,2);
%Right side 2
alfa1=pi/6+path(i,3);
xrr=uint16(115.5*cos(alfa1));
%left
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

if i==1
 % line(bend(:,2),bend(:,1),'linestyle','-','linewidth',2,'color','k');
 plot(bend(:,2),bend(:,1),'color',[0.85, 0.33, 0.1],'linewidth',1.8);
 plot(bend1(:,2),bend1(:,1),'color',[0.85, 0.33, 0.1],'linewidth',1.8);
elseif i==mm
 plot(bend(:,2),bend(:,1),'color',[0.47, 0.67, 0.19],'linewidth',1.8);
 plot(bend1(:,2),bend1(:,1),'color',[0.47, 0.67, 0.19],'linewidth',1.8); 
else
 plot(bend(:,2),bend(:,1),'color',[0, 0.45, 0.74],'linestyle',':');
 plot(bend1(:,2),bend1(:,1),'color',[0, 0.45, 0.74],'linestyle',':');
end

end

figure(2);  
plot(path(:, 3), 'r-', 'LineWidth', 2);  
hold on; 
set(gca, 'XLim', [0 55], 'YLim', [-1 1]);
grid on;
xlabel('Path point number', 'FontSize', 12, 'FontWeight', 'bold');  
ylabel('Attitude angle / rad', 'FontSize', 12, 'FontWeight', 'bold');  
title('Change in attitude angle of path points', 'FontSize', 14, 'FontWeight', 'bold'); 


set(gca, 'FontSize', 12, 'FontWeight', 'bold');
yline(0, 'k--', 'LineWidth', 1.5); 
set(gca, 'Color', [0.95 0.95 0.95]); 

% figure(3)
% plot(path(:,2),path(:,1));

% dlmwrite('path2.txt', path, 'delimiter', ' ');