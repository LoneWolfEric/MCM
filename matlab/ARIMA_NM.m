y_NM = xlsread('C:\Users\yuhui\Desktop\���������\matlab\������16.xlsx','D2:D34');
Data_NM=y_NM;              %��50������
SourceData_NM=Data_NM(1:23,1); %ǰ250��ѵ����
step=10;                  %��50������
TempData=SourceData_NM;
TempData=detrend(TempData);%ȥ������
TrendData=SourceData_NM-TempData;%���ƺ���
%--------��֣�ƽ�Ȼ�ʱ������---------
H=adftest(TempData);
difftime=0;
SaveDiffData=[];
while ~H
SaveDiffData=[SaveDiffData,TempData(1,1)];
TempData=diff(TempData);%��֣�ƽ�Ȼ�ʱ������
difftime=difftime+1;%��ִ���
H=adftest(TempData);%adf���飬�ж�ʱ�������Ƿ�ƽ�Ȼ�
end
%---------ģ�Ͷ��׻�ʶ��--------------
u = iddata(TempData);
test = [];
for p = 1:5                       %�Իع��ӦPACF,�����ͺ󳤶�����p��q��һ��ȡΪT/10��ln(T)��T^(1/2),����ȡT/10=12
for q = 1:5                    %�ƶ�ƽ����ӦACF
m = armax(u,[p q]);        
AIC = aic(m);              %armax(p,q),����AIC
test = [test;p q AIC];
end
end
for k = 1:size(test,1)
if test(k,3) == min(test(:,3)) %ѡ��AICֵ��С��ģ��
p_test = test(k,1);
q_test = test(k,2);
break;
end
end
%------1��Ԥ��-----------------
TempData=[TempData;zeros(step,1)];
n=iddata(TempData); 
m = armax(u,[p_test q_test]);
                                 %m = armax(u(1:ls),[p_test q_test]);        %armax(p,q),[p_test q_test]��ӦAICֵ��С���Զ��ع黬��ƽ��ģ��
P1=predict(m,n,1);
PreR=P1.OutputData;
PreR=PreR';
%----------��ԭ���-----------------
if size(SaveDiffData,2)~=0
for index=size(SaveDiffData,2):-1:1
PreR=cumsum([SaveDiffData(index),PreR]);
end
end 
  %-------------------Ԥ�����Ʋ����ؽ��----------------
mp1=polyfit([1:size(TrendData',2)],TrendData',1);
xt=[];
for j=1:step
xt=[xt,size(TrendData',2)+j];
end
TrendResult=polyval(mp1,xt);
PreData=TrendResult+PreR(size(SourceData_NM',2)+1:size(PreR,2));
tempx_NM=[TrendData',TrendResult]+PreR;    % tempxΪԤ����
hold on
plot(tempx_NM,'m'); 
hold on
plot(Data_NM,'y');
