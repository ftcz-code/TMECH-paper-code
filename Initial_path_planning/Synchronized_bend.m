%%函数作用：为随机树扩充一个节点，并返回extendFail=0作为标志，表示树扩展完成或者找到了最终点
function [RRTree,pathFound,extendFail] = Synchronized_bend(Checkpath,RRTreeA,RRTreeB,qgoal,stepsize,zstepsize,maxFailedAttempts,disTh,map)
pathFound=[]; 
failedAttempts=0;
Index = RRTreeA(1,4);
while failedAttempts <= maxFailedAttempts
    if Index == 0 %%********************************扩展RRTree1的方式************************************
        %% 获取随机采样点
        if rand < 0.8
            sample = [rand(1,2) .* size(map)  rand*2 - 1];
        else
            sample = qgoal;  % sample taken as goal to bias tree generation to goal
        end
        %% 找到随机树中离采样点最近的节点
        [A, I] = min(distanceCost2(RRTreeA(:,1:3),sample) ,[],1); 
        closestNode = RRTreeA(I(1),:);
        %% 从qnearest向qrand扩展一段距离到达xnew，并进行碰撞检测
        theta = atan2((sample(1)-closestNode(1)),(sample(2)-closestNode(2)));  % direction to extend sample to produce new node
        newPoint(1:2) = double(int32(closestNode(1:2) + stepsize * [sin(theta)  cos(theta)]));
        newPoint(3)= zstepsize*(sample(3)-closestNode(3))+closestNode(3);
        if Checkpath(closestNode(1:3), newPoint(1:3), map)~=1 % if extension of closest node in tree to the new point is feasible
             failedAttempts = failedAttempts + 1;
        %% 根据碰撞检测启发式搜索拓展程序
             continue;
        end
       % if Checkpath(closestNode(1:3), newPoint(1:3), map)~=1
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
        if rand < 0.995
            sample = [rand(1,2) .* size(map)  rand*2 - 1];
            
        else
            sample = qgoal;  % sample taken as goal to bias tree generation to goal

        end
        %% 找到随机树中离采样点最近的节点
        [A, I] = min( distanceCost2(RRTreeA(:,1:3),sample) ,[],1);% find the minimum value of each column
        closestNode = RRTreeA(I(1),:);
        %% 从qnearest向qrand扩展一段距离到达xnew，并进行碰撞检测
        theta = atan2((sample(1)-closestNode(1)),(sample(2)-closestNode(2)));  % direction to extend sample to produce new node
        newPoint(1:2) = double(int32(closestNode(1:2) + stepsize * [sin(theta)  cos(theta)]));
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
            newPoint(1:2) = double(int32(Temp(1:2) + stepsize * [sin(theta)  cos(theta)]));
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

