%%函数作用：为随机树扩充一个节点，并返回extendFail=0作为标志，表示树扩展完成或者找到了最终点
function [RRTree,pathFound,extendFail] = improved_Synchronized_bend(Checkpath,RRTreeA,RRTreeB,qgoal,stepsize,zstepsize,maxFailedAttempts,disTh,map,qinitl,qgoall)
pathFound=[]; 
failedAttempts=0;
Index = RRTreeA(1,4);
while failedAttempts <= maxFailedAttempts
    if Index == 0 %%********************************扩展RRTree1的方式************************************
        %% 获取随机采样点
        if rand < 0.8
           randnode = informed_new_node(qinitl,qgoall) ;
           sample = [randnode  rand*2 - 1];
        else
           sample = qgoal;  % sample taken as goal to bias tree generation to goal
        end
        %% 找到随机树中离采样点最近的节点
        [A, I] = min(distanceCost2(RRTreeA(:,1:3),sample) ,[],1); 
        closestNode = RRTreeA(I(1),:);
        %% 基于障碍物与目标点导向的new点生成
        %% 指向最近点指向随机采样点的向量
        r0(1) = sample(1)-closestNode(1);  
        r0(2) = sample(2)-closestNode(2);
        r0_a = norm(r0, 2);
        r0 = r0/r0_a;
        %% 指向目标点的向量
        rg(1) = qgoal(1)-sample(1);  
        rg(2) = qgoal(2)-sample(2);
        rg_a = norm(rg, 2);
        rg = rg/rg_a;
        %% 障碍物指向随机点的向量
        %%随机点5x5矩阵 1行
        p(1,:) = sample(1:2)-2;
        p(2,:) = [sample(1)-2,sample(2)-1];
        p(3,:) = [sample(1)-2,sample(2)];
        p(4,:) = [sample(1)-2,sample(2)+1];
        p(5,:) = [sample(1)-2,sample(2)+2];
        %%2行
        p(6,:) = [sample(1)-1,sample(2)-2];
        p(7,:) = [sample(1)-1,sample(2)-1];
        p(8,:) = [sample(1)-1,sample(2)];
        p(9,:) = [sample(1)-1,sample(2)+1];
        p(10,:) = [sample(1)-1,sample(2)+2];
        %%3行
        p(11,:) = [sample(1),sample(2)-2];
        p(12,:) = [sample(1),sample(2)-1];
        p(13,:) = [sample(1),sample(2)];
        p(14,:) = [sample(1),sample(2)+1];
        p(15,:) = [sample(1),sample(2)+2];
        %%4行
        p(16,:) = [sample(1)+1,sample(2)-2];
        p(17,:) = [sample(1)+1,sample(2)-1];
        p(18,:) = [sample(1)+1,sample(2)];
        p(19,:) = [sample(1)+1,sample(2)+1];
        p(20,:) = [sample(1)+1,sample(2)+2];
        %%5行
        p(21,:) = [sample(1)+2,sample(2)-2];
        p(22,:) = [sample(1)+2,sample(2)-1];
        p(23,:) = [sample(1)+2,sample(2)];
        p(24,:) = [sample(1)+2,sample(2)+1];
        p(25,:) = [sample(1)+2,sample(2)+2];

        pp = [];
        k = 0;
        for i=1:1:25
        new=int32(p(i,:));
        if feasiblePoint(new,map)  
          pp = [pp;new];
          if new(1)<650
              k=k+0.01744;
          elseif new(1)>726
              k=k-0.01744;
          end
        end      
        end

       [m,n] =  size(pp);
        robj=0;
       for i=1:1:m
       rob(1)=sample(1)-double(pp(i,1));
       rob(2)=sample(2)-double(pp(i,2));
       rob_a = norm(rob, 2);
       robj = robj+rob/rob_a;
       end

       dir = 0.8*r0+0.2*robj;

        % dir = 1*r0;
        newPoint(1:2) = double(int32(closestNode(1:2) + stepsize * dir));
        newPoint(3)= zstepsize*(sample(3)-closestNode(3))+closestNode(3);
        if Checkpath(closestNode(1:3), newPoint(1:3), map)~=1 % if extension of closest node in tree to the new point is feasible
             failedAttempts = failedAttempts + 1;
        %% 根据碰撞检测启发式搜索拓展程序
             continue;
        end
       % if checkPath2(closestNode(1:3), newPoint(1:3), map)~=1
       %      failedAttempts = failedAttempts + 1;
       %      continue;
       % end
        %% 检测新节点与另一个树的节点是否满足阈值
        [A, I2] = min(distanceCost2(RRTreeB(:,1:3),newPoint) ,[],1); 
        if distanceCost2(RRTreeB(I2(1),1:3),newPoint) < disTh         
            %% 检测两棵树相连的线段之间是否存在障碍物
            if  Checkpath(RRTreeB(I2(1),1:3), newPoint, map)
                pathFound = [newPoint I I2];
                extendFail = 0;
                RRTree = RRTreeA;
                break;
            else
                failedAttempts = failedAttempts + 1;
                continue;
            end
        end
        %% 检测新节点是否已经在树中存在
        [A, I3] = min(distanceCost2(RRTreeA(:,1:3),newPoint) ,[],1); 
        if distanceCost2(newPoint,RRTreeA(I3(1),1:3)) < disTh 
            failedAttempts=failedAttempts+1;
            continue; 
        end
        RRTree = [RRTreeA;newPoint I];
        extendFail = 0;
    else %%********************************扩展RRTree2的方式************************************
        %% 获取随机采样点
        if rand < 0.8
            
           randnode = informed_new_node(qinitl,qgoall) ;          
            sample = [randnode  rand*2 - 1];
        else
            sample = qgoal;  % sample taken as goal to bias tree generation to goal
        end
        %% 找到随机树中离采样点最近的节点
        [A, I] = min( distanceCost2(RRTreeA(:,1:3),sample) ,[],1);% find the minimum value of each column
        closestNode = RRTreeA(I(1),:);
        %% 基于障碍物与目标点导向的new点生成
        %% 指向最近点指向随机采样点的向量
        r0(1) = sample(1)-closestNode(1);  
        r0(2) = sample(2)-closestNode(2);
        r0_a = norm(r0, 2);
        r0 = r0/r0_a;
        %% 指向目标点的向量
        rg(1) = qgoal(1)-sample(1);  
        rg(2) = qgoal(2)-sample(2);
        rg_a = norm(rg, 2);
        rg = rg/rg_a;
        %% 障碍物指向随机点的向量
        %%随机点5x5矩阵 1行
        p(1,:) = sample(1:2)-2;
        p(2,:) = [sample(1)-2,sample(2)-1];
        p(3,:) = [sample(1)-2,sample(2)];
        p(4,:) = [sample(1)-2,sample(2)+1];
        p(5,:) = [sample(1)-2,sample(2)+2];
        %%2行
        p(6,:) = [sample(1)-1,sample(2)-2];
        p(7,:) = [sample(1)-1,sample(2)-1];
        p(8,:) = [sample(1)-1,sample(2)];
        p(9,:) = [sample(1)-1,sample(2)+1];
        p(10,:) = [sample(1)-1,sample(2)+2];
        %%3行
        p(11,:) = [sample(1),sample(2)-2];
        p(12,:) = [sample(1),sample(2)-1];
        p(13,:) = [sample(1),sample(2)];
        p(14,:) = [sample(1),sample(2)+1];
        p(15,:) = [sample(1),sample(2)+2];
        %%4行
        p(16,:) = [sample(1)+1,sample(2)-2];
        p(17,:) = [sample(1)+1,sample(2)-1];
        p(18,:) = [sample(1)+1,sample(2)];
        p(19,:) = [sample(1)+1,sample(2)+1];
        p(20,:) = [sample(1)+1,sample(2)+2];
        %%5行
        p(21,:) = [sample(1)+2,sample(2)-2];
        p(22,:) = [sample(1)+2,sample(2)-1];
        p(23,:) = [sample(1)+2,sample(2)];
        p(24,:) = [sample(1)+2,sample(2)+1];
        p(25,:) = [sample(1)+2,sample(2)+2];

        pp = [];
        k=0;
        for i=1:1:25
        new=int32(p(i,:));
        if feasiblePoint(new,map)  
          pp = [pp;new];
          if new(1)<650
              k=k+0.01744;
          elseif new(1)>726
              k=k-0.01744;
          end
        end      
        end

       [m,n] =  size(pp);
       robj=0;
       for i=1:1:m
       rob(1)=sample(1)-double(pp(i,1));
       rob(2)=sample(2)-double(pp(i,2));
       rob_a = norm(rob, 2);
       robj = robj+rob/rob_a;
       end

        dir1 = 0.8*r0+0.2*robj;

        dir = 1*r0;
        newPoint(1:2) = double(int32(closestNode(1:2) + stepsize * dir));
        newPoint(3)= zstepsize*(sample(3)-closestNode(3))+closestNode(3);
        if Checkpath(closestNode(1:3), newPoint(1:3), map)~=1 % if extension of closest node in tree to the new point is feasible
             failedAttempts = failedAttempts + 1;
    
            continue;
        end
        
        
        % [A, I2] = min( distanceCost2(RRTreeB(:,1:3),newPoint) ,[],1); 
        % if distanceCost2(RRTreeB(I2,1:3),newPoint) < disTh
        %     extendFail = 0;
        %     break;
        % end
        if sample == qgoal
            %% 贪婪搜索
            flag = 1; 
            RRTreeA = [RRTreeA;newPoint I];%%先储存起来
            line([RRTreeA(end,2);RRTreeA(RRTreeA(end,4),2)],[RRTreeA(end,1);RRTreeA(RRTreeA(end,4),1)],'color','#789440','linewidth',0.5,'linestyle','-','marker','.','markersize',10);
         while flag == 1
          
            Temp = newPoint;
            r0(1) = sample(1)-Temp(1);  
            r0(2) = sample(2)-Temp(2);
            r0_a = norm(r0, 2);
            r0 = r0/r0_a;
            dir = 1*r0;
            newPoint(1:2) = double(int32(Temp(1:2) + stepsize * dir));
            % if(newPoint(3)-sample(3)>0.03)
                newPoint(3)= zstepsize*(sample(3)-Temp(3))+Temp(3);
            % end
            
            %newPoint(3)= sample(3);
            I = size(RRTreeA,1);
           %% 检测新节点与另一个树的节点是否满足阈值
            [A, I2] = min( distanceCost2(RRTreeB(:,1:3),newPoint) ,[],1); 
            if distanceCost2(RRTreeB(I2,1:3),newPoint) < disTh  
                %% 检测两棵树相连的线段之间是否存在障碍物
                if  Checkpath(RRTreeB(I2,1:3), newPoint, map)
                    pathFound= [newPoint I I2];
                    extendFail = 0;
                    RRTreeA = [RRTreeA;newPoint I];
                    RRTree = RRTreeA;
                    line([RRTree(end,2);RRTree(RRTree(end,4),2)],[RRTree(end,1);RRTree(RRTree(end,4),1)],'color','#789440','linewidth',0.5,'linestyle','-','marker','.','markersize',10);
                    break;
                else
                    break;
                end
            else
                if  Checkpath(Temp, newPoint, map)~=1  %%如果有碰撞
                    RRTree = RRTreeA;
                    extendFail = 0;
                    break;
                else
                     RRTreeA = [RRTreeA;newPoint I];
                     RRTree = RRTreeA;
                     extendFail = 0;
                     line([RRTree(end,2);RRTree(RRTree(end,4),2)],[RRTree(end,1);RRTree(RRTree(end,4),1)],'color','#789440','linewidth',0.5,'linestyle','-','marker','.','markersize',10);
                end  
            end
        end   
        else 
            %% 随机采样
            flag = 0;  
            
            %% 检测新节点与另一个树的节点是否满足阈值
        [A, I2] = min(distanceCost2(RRTreeB(:,1:3),newPoint) ,[],1); 
        if distanceCost2(RRTreeB(I2(1),1:3),newPoint) < disTh         
            %% 检测两棵树相连的线段之间是否存在障碍物
            if  Checkpath(RRTreeB(I2(1),1:3), newPoint, map)
                pathFound = [newPoint I I2];
                extendFail = 0;
                RRTreeA = [RRTreeA;newPoint I];
                RRTree = RRTreeA;
                break;
            else
                failedAttempts = failedAttempts + 1;
                continue;
            end
        end
        %% 检测新节点是否已经在树中存在
        [A, I3] = min(distanceCost2(RRTreeA(:,1:3),newPoint) ,[],1); 
        if distanceCost2(newPoint,RRTreeA(I3(1),1:3)) < disTh 
            failedAttempts=failedAttempts+1;
            continue; 
        end
        RRTree = [RRTreeA;newPoint I];
        line([RRTree(end,2);RRTree(RRTree(end,4),2)],[RRTree(end,1);RRTree(RRTree(end,4),1)],'color','#789440','linewidth',0.5,'linestyle','-','marker','.','markersize',10);
        extendFail = 0;
        end
        
    
    
    end
    break;
end

