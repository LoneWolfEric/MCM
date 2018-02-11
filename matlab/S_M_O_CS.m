function [queue_message,value_message] = S_M_O_CS( n_neural,v_max,v_min,car_den )%元胞数，最大速度，最小速度，车辆密度
%基于元胞自动机的短途车流模拟
%   此处显示详细说明
total_t = 60;%总循环时间，可变为输入参数
queue_message = zeros(n_neural,3,total_t);%队列信息（行（1~~元胞数），列（1~~3），时间（1~~总时间））
value_message = zeros(n_neural,3,total_t);%速度信息（行（1~~元胞数），列（1~~3），时间（1~~总时间））
queue = zeros(1,3,2);%新增队列信息，每一秒结束的时候产生
n_road = [ 1 , 0 ];%（状态区间【有车：没车】）
prop = [ car_den , 1-car_den ];%车辆出现概率【车辆密度，1-车辆密度】
n_queue = zeros(n_neural,3,2);%元胞队列（行（1~~元胞数)，列（1~~3），页（1，有无车；2，实时车速）））
n_queue(:,:,1) = randsrc(n_neural,3,[n_road;prop]);%依据车辆密度随机生成车辆信息
prop = [ 0.9 , 0.1 ];%减速概率
v_last = 0;%允许速度

for i = 1:n_neural%遍历整个元胞集
    for j = 1:3   %
        if( n_queue(i,j,1) == 1)%如果那个位置有车
            n_queue(i,j,2) = randsrc(1,1,v_min:v_max);%按照速度最大对小区间等概率选择一个速度
        end
    end
end
%每一秒时间变化%
for time = 1:total_t  %时间循环
    for i = 1:n_neural%便利所有元胞
        for j = 1:3%先行后列
            if(n_queue(i,j,1) == 1)
                slow_pro = randsrc( 1,1,[n_road;prop]);%生成减速随机数（1为不减速，0为减速）
                v_last = v_max;%循环前将v_last初始赋值，当前面没有车的时候，允许速度可以为最高速度
                break%退出循环
                %检测前车最低速度
                for s = i-1:0%循环遍历该车辆和前面的车
                    if(n_queue(s,j,1) == 1)%如果有车
                        v_last = min(max(s-1,1),v_max);%允许速度为，前车的距离-1，若这个值小于1则取1，大于最大速度则取最大速度
                    end
                end
                %速度变化过程%
                if(slow_pro == 0)%若减速
                    n_queue(i,j,2) = randsrc(1,1,1:n_queue(i,j,2));%速度为1和当前速度之间的随机数
                else%加速
                    n_queue(i,j,2) = randsrc(1,1,n_queue(i,j,2):v_last);%速度为当前速度和允许速度之间的随机数
                end
            end
        end
    end
    %位置变化过程%
    for i = 1:n_neural    %遍历所有元胞
        for j = 1:3       %先行后列
            if(n_queue(i,j,1)==1)%如果该位置有车
                place = i - n_queue(i,j,2);%更新位置为当前位置向前移动当前速度
                if( place < 1 )%如果越界，即超过1
                    n_queue(i,j,1) = 0;%元胞组中删除原位置，不增加点
                    n_queue(i,j,2) = 0;%删除原速度
                else%没越界
                    n_queue(place,j,1) = 1;%更新位置
                    n_queue(place,j,2) = n_queue(i,j,2);%在新位置增加速度信息
                    n_queue(i,j,1) = 0;%删除原位置信息
                    n_queue(i,j,2) = 0;%删除原位置速度
                end
            end
        end
    end
    %车辆进入过程%
    queue(1,:,1) = randsrc(1,3,[[1,0];[car_den,1-car_den]]);%通过车辆密度生成新的（1*3*2）矩阵，即新来了一排的车
    queue(1,:,2) = randsrc(1,3,v_min:v_max);%生成他们的速度
    n_queue(n_neural,:,:) = queue;%将新生成的车加入队列
    %队列记录过程%
    queue_message(:,:,time) = n_queue(:,:,1);%在总表记录队伍信息
    value_message(:,:,time) = n_queue(:,:,2);%在总表记录速度信息
end   
end
