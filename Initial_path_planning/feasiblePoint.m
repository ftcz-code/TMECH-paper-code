%%feasiblePoint.m �ж���ʼ���Ŀ����Ƿ����Ҫ��
function flag = feasiblePoint(point,map)
if ~(point(1)>=1 && point(1)<=size(map,1) && point(2)>=1 && point(2)<=size(map,2) && map(point(1),point(2))==1)
    
	flag = 0; %����ײ
else
    flag = 1; %����ײ
end

%%ע��
%% ~()�����������Ҫȫ������ſ���
%% map�������Ͻ�Ϊԭ�㿪ʼ������