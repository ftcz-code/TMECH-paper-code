%--------------------------------------------------------------------------
% File Name: run.m
% Version: 2.0
% Date: December 4, 2024
% Author: Miao Yiyang
% Affiliation: NJUPT
% Description: This program implements robot path planning optimized using the Water Cycle Algorithm (WCA).
%              The main functions include:
%              1. Path optimization using the Water Cycle Algorithm (WCA);
%              2. Outputting the optimal path and related performance metrics.
% Dependencies:
%       - bend_map.bmp: Obstacle map
%       - bendair.bmp: Blank map for constructing swept volume
%       - Initial_path_population.mat: Initial path population data
%       - distanceCost_SDF.m: Path fitness calculation function
%       - generate_sdf.m: Generate SVSDF
%       - checkpath4.m: Path collision detection
%       - plot.m: Draw a comparison chart before and after optimization
%--------------------------------------------------------------------------
clear
close  all
clc
load("Initial_path_population");
map=imread('bend_map.bmp');
img = imread('bendnew3.bmp');
% Check if the image is in RGB format
if ndims(img) == 3
    grayImg = rgb2gray(img);
else
    grayImg = img;
end

qinit = [730 916 0];
qgoal = [480 460 -0.52];
zLB=-1;
zUB=1;
LB=460;
UB=916;
nvars=3;
Npop=30;
Nsr=5;
dmax=1e-16;
max_it=15;
flag=1;
last=length(pop(1).position)-1;
tic;

%% Image feature extraction (classifier)
Highriskarea = [];
Lowriskareas = [];
[Highriskarea,Lowriskareas]=feature_extraction(img);

%% population initialization
% while flag==1
N_stream=Npop-Nsr;
ind.position=[];
ind.cost=[];
% Initialize the interpolated matrix
result=repmat(ind,max_it,1);

for i=1:Npop
    % pop(i).position=path;
     pop(i).cost = distanceCost_SDF(pop(i).position,Highriskarea,Lowriskareas);
end

[~, index]=sort([pop.cost]); %Cost sorting
%------------- Generate ocean ------------------------------------------------
sea=pop(index(1));
precost = sea.cost;
%-------------Generate Rivers ----------------------------------------------
river=repmat(ind,Nsr-1,1);
for i=1:Nsr-1
    river(i)=pop(index(1+i));
end

%------------ Generate Stream----------------------------------------------
stream=repmat(ind,N_stream,1);
for i=1:N_stream
    stream(i)=pop(index(Nsr+i));
end

%--------- Designate streams to rivers and sea ------------------------
cs=[sea.cost;[river.cost]';stream(1).cost];
CN=cs-max(cs); 

NS=round(abs(CN/sum(CN))*N_stream);
NS(end)=[];

% ------------------------- Modification on NS -----------------------
i=Nsr;
while sum(NS)>N_stream
    if NS(i)>1
        NS(i)=NS(i)-1;
    else
        i=i-1;
    end
end

i=1;
while sum(NS)<N_stream
    NS(i)=NS(i)+1;
end

% if find(NS==0)
%     index=find(NS==0);
%     for i=1:size(index,1)
%         while NS(index(i))==0
%             % NS(index(i))=NS(index(i))+round(NS(i)/6);
%             NS(index(i))=NS(index(i))+round(NS(i)/6);
%             NS(i)=NS(i)-round(NS(i)/6);
%         end
%     end
% end

NS=sort(NS,'descend');
NB=NS(2:end);

%%
%----------- Main Loop for WCA --------------------------------------------
disp('******************** Water Cycle Algorithm (WCA)********************');
disp('*Iterations     Function Values *');
disp('********************************************************************');
FF=zeros(max_it,1);
for i=1:max_it
    %---------- Moving stream to sea---------------------------------------
    rand1=randi([1, 50]);
    rand2=0.2 + (1-0.2)*rand;
    a= 2;
    b= last;
    for j=1:NS(1)
        stream(j).position(a:b,1:2)=stream(j).position(a:b,1:2)+2.*rand(1).*(sea.position(a:b,1:2)-stream(j).position(a:b,1:2));
        stream(j).position(a:b,3)=stream(j).position(a:b,3)+rand(1).*(sea.position(a:b,3)-stream(j).position(a:b,3));


        stream(j).position(a:b,1:2)=min(stream(j).position(a:b,1:2),UB);
        stream(j).position(a:b,1:2)=max(stream(j).position(a:b,1:2),LB);
        stream(j).position(a:b,3)=min(stream(j).position(a:b,3),zUB);
        stream(j).position(a:b,3)=max(stream(j).position(a:b,3),zLB);

        stream(j).cost=distanceCost_SDF(stream(j).position,Highriskarea,Lowriskareas);

        if stream(j).cost<sea.cost
            new_sea=stream(j);
            stream(j)=sea;
            sea=new_sea;
        end
    end
    %---------- Moving Streams to rivers-----------------------------------
    rand1=randi([1, 50]);
    rand2=0.2 + (1-0.2)*rand;
    a= 2;
    b= last;
    for k=1:Nsr-1
        for j=1:NB(k)
            % stream(j+sum(NS(1:k))).position(1:2)=stream(j+sum(NS(1:k))).position(1:2)+2.*rand(1,nvars).*(river(k).position(1:2)-stream(j+sum(NS(1:k))).position(1:2));
            stream(j+sum(NS(1:k))).position(a:b,1:2)=stream(j+sum(NS(1:k))).position(a:b,1:2)+2.*rand(1,2).*(river(k).position(a:b,1:2)-stream(j+sum(NS(1:k))).position(a:b,1:2));
            stream(j+sum(NS(1:k))).position(a:b,3)=stream(j+sum(NS(1:k))).position(a:b,3)+rand(1).*(river(k).position(a:b,3)-stream(j+sum(NS(1:k))).position(a:b,3));

            stream(j+sum(NS(1:k))).position(a:b,1:2)=min(stream(j+sum(NS(1:k))).position(a:b,1:2),UB);
            stream(j+sum(NS(1:k))).position(a:b,1:2)=max(stream(j+sum(NS(1:k))).position(a:b,1:2),LB);
            stream(j+sum(NS(1:k))).position(a:b,3)=min(stream(j+sum(NS(1:k))).position(a:b,3),zUB);
            stream(j+sum(NS(1:k))).position(a:b,3)=max(stream(j+sum(NS(1:k))).position(a:b,3),zLB);

            stream(j+sum(NS(1:k))).cost=distanceCost_SDF(stream(j+sum(NS(1:k))).position,Highriskarea,Lowriskareas);

            if stream(j+sum(NS(1:k))).cost<river(k).cost
                new_river=stream(j+sum(NS(1:k)));
                stream(j+sum(NS(1:k)))=river(k);
                river(k)=new_river;

                if river(k).cost<sea.cost
                    new_sea=river(k);
                    river(k)=sea;
                    sea=new_sea;
                end
            end
        end
    end
     %---------- Moving rivers to Sea --------------------------------------
    rand1=randi([1, 50]);
    rand2=0.2 + (1-0.2)*rand;
    a= 2;
    b= last;
    for j=1:Nsr-1
        river(j).position(a:b,1:2)=river(j).position(a:b,1:2)+2.*rand(1,2).*(sea.position(a:b,1:2)-river(j).position(a:b,1:2));
        river(j).position(a:b,3)=river(j).position(a:b,3)+rand(1).*(sea.position(a:b,3)-river(j).position(a:b,3));

        river(j).position(a:b,1:2)=min(river(j).position(a:b,1:2),UB);
        river(j).position(a:b,1:2)=max(river(j).position(a:b,1:2),LB);
        river(j).position(a:b,3)=min(river(j).position(a:b,3),zUB);
        river(j).position(a:b,3)=max(river(j).position(a:b,3),zLB);

        river(j).cost=distanceCost_SDF(river(j).position,Highriskarea,Lowriskareas);

        if river(j).cost<sea.cost
            new_sea=river(j);
            river(j)=sea;
            sea=new_sea;
        end
    end
    %-------------- Evaporation condition and raining process--------------
    % Check the evaporation condition for rivers and sea
    for k=1:Nsr-1
        if ((norm(river(k).position-sea.position)<dmax) || rand<0.2)
            for j=1:NB(k)
                stream(j+sum(NS(1:k))).position(2:last,1:2)=LB+rand(length(pop(1).position)-2,2).*(UB-LB);
                stream(j+sum(NS(1:k))).position(2:last,3)=0+rand(1).*(zUB-zLB);
            end
        end
    end
    % Check the evaporation condition for streams and sea
    for j=1:NS(1)
        if ((norm(stream(j).position-sea.position)<dmax))
             stream(j).position(2:last,1:2)=LB+rand(length(pop(1).position)-2,2).*(UB-LB);
             stream(j).position(2:last,3)=0+rand(1).*(zUB-zLB);
            % stream(j).position(2:last,1:2) = sea.position(2:last,1:2) + 2*rand(length(path1)-2,2);
            % stream(j).position(2:last,3)= sea.position(2:last,3)+0.8*rand;
        end
    end
    % 拥有较少溪流的河流之间的蒸发
    % ER = (sum(NS(2:Nsr))/(Nsr-1))*rand;
    % for f=2:Nsr-1
    %     if (exp(-i/max_it)<rand && NS(f)<ER)
    %        stream(f).position(2:last,1:2) = LB+rand(length(path1)-2,2).*(UB-LB);
    %        stream(f).position(2:last,3)=0+rand(1).*(zUB-zLB);
    %     end
    % end

    %----------------------------------------------------------------------
    dmax=dmax-(dmax/max_it);

    disp(['Iteration: ',num2str(i),'   Fmin= ',num2str(sea.cost)]);
    FF(i)=sea.cost;
    result(i).position = sea.position;
    result(i).cost = sea.cost;
end

NFEs=Npop*max_it;
Xmin=sea.position;
Fmin=distanceCost_SDF(Xmin,Highriskarea,Lowriskareas);

%% Results and Plot
toc;
% FF=[pop(2).cost ;FF];
FF=[precost ;FF];
Elapsed_Time=toc;

figure(3)
plot(FF, 'LineWidth', 2, 'Color', 'b', 'LineStyle', '-');
xlabel('Number of Iterations', 'FontSize', 12);
ylabel('Function Values', 'FontSize', 12);
title('Convergence of Function Values', 'FontSize', 14);
grid on; 
set(gca, 'FontSize', 12);
set(gca, 'Color', [0.95 0.95 0.95]);  
hold off;

figure(4)
plot(Xmin(:, 3), 'LineWidth', 2, 'Color', 'r'); 
xlabel('Path Point Index', 'FontSize', 12);
ylabel('Value of attitude', 'FontSize', 12); 
title('Attitude change process', 'FontSize', 14);
set(gca, 'XLim', [0 55], 'YLim', [-1 1], 'FontSize', 12);
grid on; 
yline(0, 'k--', 'LineWidth', 1.5); 
set(gca, 'Color', [0.95 0.95 0.95]);  
hold off;


%% Draw an optimized graph
figure(5)
   	imshow(map);
    % hold on;
    % scatter(qinit(2),qinit(1),50,'sm','filled');
    % hold on;
    % scatter(qgoal(2),qgoal(1),50,'sb','filled');
    hold on;
%% Draw planning results
for i=1:1:length(Xmin) 
    k=0;
    kk=0;
    kkk=0;
    kkkk=0;
    %right side
    alfa=pi/6-Xmin(i,3);    xr=uint16(115.5*cos(alfa));
    bend=[];
    bend(1,1)=Xmin(i,1);
    bend(1,2)=Xmin(i,2);
    %Right side 2
    alfa1=pi/6+Xmin(i,3);
    xrr=uint16(115.5*cos(alfa1));
    %left
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

% dlmwrite('Finalpath.txt', Xmin, 'delimiter', ' ');







