function randnode = informed_new_node(xstart,xend)
%UNTITLED2 此处显示有关此函数的摘要
%   此处显示详细说明

    alpha =atan(abs(xstart(1)-xend(1))/abs(xstart(2)-xend(2)));
    b = 320; 
    c = sqrt((xstart(1)-xend(1))^2+(xstart(2)-xend(2))^2);

    a= sqrt(b*b + c*c);
    % if a>c
    %     b = sqrt(a*a - c*c);
    % else
    %     fprintf("a<c\n");
    % end

node = generateRandomPointsInUnitCircle(1);   
randnode1(1) = node(2)*b/2; %y
randnode1(2) = node(1)*a/2; %x

% randnode(1) = (randnode1(1)*cos(alpha) + randnode1(2)*sin(alpha))+((xstart(1)+xend(1))/2); %y
% randnode(2) = (-randnode1(1)*sin(alpha) + randnode1(2)*cos(alpha))+((xstart(2)+xend(2))/2); %x
   
randnode(1) = (randnode1(1)*cos(alpha) - randnode1(2)*sin(alpha))+((xstart(1)+xend(1))/2); %y
randnode(2) = (randnode1(1)*sin(alpha) + randnode1(2)*cos(alpha))+((xstart(2)+xend(2))/2); %x

end

