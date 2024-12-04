
function h=distanceCost2(a,b)%
alfa=2;
h = sqrt((a(:,1)-b(:,1)).^2 + (a(:,2)-b(:,2)).^2)+alfa.*(a(:,3)-b(:,3)).^2;