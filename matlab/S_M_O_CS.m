function [queue_message,value_message] = S_M_O_CS( n_neural,v_max,v_min,car_den )%Ԫ����������ٶȣ���С�ٶȣ������ܶ�
%����Ԫ���Զ����Ķ�;����ģ��
%   �˴���ʾ��ϸ˵��
total_t = 60;%��ѭ��ʱ�䣬�ɱ�Ϊ�������
queue_message = zeros(n_neural,3,total_t);%������Ϣ���У�1~~Ԫ���������У�1~~3����ʱ�䣨1~~��ʱ�䣩��
value_message = zeros(n_neural,3,total_t);%�ٶ���Ϣ���У�1~~Ԫ���������У�1~~3����ʱ�䣨1~~��ʱ�䣩��
queue = zeros(1,3,2);%����������Ϣ��ÿһ�������ʱ�����
n_road = [ 1 , 0 ];%��״̬���䡾�г���û������
prop = [ car_den , 1-car_den ];%�������ָ��ʡ������ܶȣ�1-�����ܶȡ�
n_queue = zeros(n_neural,3,2);%Ԫ�����У��У�1~~Ԫ����)���У�1~~3����ҳ��1�����޳���2��ʵʱ���٣�����
n_queue(:,:,1) = randsrc(n_neural,3,[n_road;prop]);%���ݳ����ܶ�������ɳ�����Ϣ
prop = [ 0.9 , 0.1 ];%���ٸ���
v_last = 0;%�����ٶ�

for i = 1:n_neural%��������Ԫ����
    for j = 1:3   %
        if( n_queue(i,j,1) == 1)%����Ǹ�λ���г�
            n_queue(i,j,2) = randsrc(1,1,v_min:v_max);%�����ٶ�����С����ȸ���ѡ��һ���ٶ�
        end
    end
end
%ÿһ��ʱ��仯%
for time = 1:total_t  %ʱ��ѭ��
    for i = 1:n_neural%��������Ԫ��
        for j = 1:3%���к���
            if(n_queue(i,j,1) == 1)
                slow_pro = randsrc( 1,1,[n_road;prop]);%���ɼ����������1Ϊ�����٣�0Ϊ���٣�
                v_last = v_max;%ѭ��ǰ��v_last��ʼ��ֵ����ǰ��û�г���ʱ�������ٶȿ���Ϊ����ٶ�
                break%�˳�ѭ��
                %���ǰ������ٶ�
                for s = i-1:0%ѭ�������ó�����ǰ��ĳ�
                    if(n_queue(s,j,1) == 1)%����г�
                        v_last = min(max(s-1,1),v_max);%�����ٶ�Ϊ��ǰ���ľ���-1�������ֵС��1��ȡ1����������ٶ���ȡ����ٶ�
                    end
                end
                %�ٶȱ仯����%
                if(slow_pro == 0)%������
                    n_queue(i,j,2) = randsrc(1,1,1:n_queue(i,j,2));%�ٶ�Ϊ1�͵�ǰ�ٶ�֮��������
                else%����
                    n_queue(i,j,2) = randsrc(1,1,n_queue(i,j,2):v_last);%�ٶ�Ϊ��ǰ�ٶȺ������ٶ�֮��������
                end
            end
        end
    end
    %λ�ñ仯����%
    for i = 1:n_neural    %��������Ԫ��
        for j = 1:3       %���к���
            if(n_queue(i,j,1)==1)%�����λ���г�
                place = i - n_queue(i,j,2);%����λ��Ϊ��ǰλ����ǰ�ƶ���ǰ�ٶ�
                if( place < 1 )%���Խ�磬������1
                    n_queue(i,j,1) = 0;%Ԫ������ɾ��ԭλ�ã������ӵ�
                    n_queue(i,j,2) = 0;%ɾ��ԭ�ٶ�
                else%ûԽ��
                    n_queue(place,j,1) = 1;%����λ��
                    n_queue(place,j,2) = n_queue(i,j,2);%����λ�������ٶ���Ϣ
                    n_queue(i,j,1) = 0;%ɾ��ԭλ����Ϣ
                    n_queue(i,j,2) = 0;%ɾ��ԭλ���ٶ�
                end
            end
        end
    end
    %�����������%
    queue(1,:,1) = randsrc(1,3,[[1,0];[car_den,1-car_den]]);%ͨ�������ܶ������µģ�1*3*2�����󣬼�������һ�ŵĳ�
    queue(1,:,2) = randsrc(1,3,v_min:v_max);%�������ǵ��ٶ�
    n_queue(n_neural,:,:) = queue;%�������ɵĳ��������
    %���м�¼����%
    queue_message(:,:,time) = n_queue(:,:,1);%���ܱ��¼������Ϣ
    value_message(:,:,time) = n_queue(:,:,2);%���ܱ��¼�ٶ���Ϣ
end   
end
