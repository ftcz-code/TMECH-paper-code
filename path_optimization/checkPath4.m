% Rahul Kala, IIIT Allahabad, Creative Commons Attribution-ShareAlike 4.0 International License. 
% The use of this code, its parts and all the materials in the text; creation of derivatives and their publication; and sharing the code publically is permitted without permission. 

% Please cite the work in all materials as: R. Kala (2014) Code for Robot Path Planning using Rapidly-exploring Random Trees, Indian Institute of Information Technology Allahabad, Available at: http://rkala.in/codes.html

function feasible=checkPath4(newPos,map)
feasible=1;
newPos1=newPos;
newPos2=newPos;
newPos3=newPos;
NewPos=newPos;

%1cm对应大约19.25个像素点 向上旋转为负，向下为正
%判断钣金件右侧是否在图中且与障碍物是否碰撞
%右1
kr=0;
alfa=pi/6-newPos(3);
xr=uint16(115.5*cos(alfa));
bend=newPos;
for r=1:1:xr
    bend(2) = bend(2)+1;
    kr=kr+1;
    bend(1) = -(kr)*tan(alfa)+NewPos(1);
    posCheck2=uint16([bend(1) bend(2)]);
    if ~feasiblePoint_bend(posCheck2,map)       
     feasible = 0;
    end
end 

%右2
krr=0;
alfa1=pi/6+newPos1(3);
bend1=[];
xrr=uint16(115.5*cos(alfa1));
for rr=1:1:xrr
    bend(2) = bend(2)+1;
    krr=krr+1;
    bend1(1) = (krr)*tan(alfa1)+bend(1);
    posCheck2=uint16([bend1(1) bend(2)]);
if ~feasiblePoint_bend(posCheck2,map), feasible=0; end 
end
 
%判断钣金件左侧是否在图中且与障碍物是否碰撞
kl=0;
alfa3=pi/6+newPos2(3);
xl=uint16(442.75*cos(alfa3));
for l=1:1:xl
    newPos3(2) = newPos3(2)-1;
    kl=kl+1;
    newPos3(1) = -(kl)*tan(alfa3)+NewPos(1);
    posCheck3=uint16([newPos3(1) newPos3(2)]);
    if ~feasiblePoint_bend(posCheck3,map), feasible=0; end 
end




