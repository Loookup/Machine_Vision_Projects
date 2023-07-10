clc; close all; clear all;

Data = xlsread("Test02");
len = (length(Data)+1)/4;
X_Acc = zeros(len,1); 
Y_Acc = zeros(len,1);
delt = 0.1;

for i=1:1:len
    X_Acc(i) = Data(4*i-3);
    Y_Acc(i) = Data(4*i-2);
end

X_Vel = zeros(len-1,1);  

for i=1:1:length(X_Vel)
    
    if (X_Acc(i)*X_Acc(i+1)>=0)
        Vel_Temp = (X_Acc(i))*delt + ((X_Acc(i+1)-X_Acc(i))*delt)/2;
    else
        X_Intercept = delt*X_Acc(i+1)/(X_Acc(i+1)-X_Acc(i));
        Vel_Temp = (X_Acc(i)*(delt-X_Intercept))/2 + (X_Acc(i+1)*X_Intercept)/2;
    end
    X_Vel(i) = Vel_Temp;
end

X_Dis = zeros(len-2,1);
Total_Dis_X = 0;
Sum_Dis_X = zeros(len-2,1);

for i=1:1:length(X_Dis)
    
    if (X_Vel(i)*X_Vel(i+1)>=0)
        Dis_Temp = (X_Vel(i))*delt + ((X_Vel(i+1)-X_Vel(i))*delt)/2;
    else
        X_Intercept = delt*X_Vel(i+1)/(X_Vel(i+1)-X_Vel(i));
        Dis_Temp = (X_Vel(i)*(delt-X_Intercept))/2 + (X_Vel(i+1)*X_Intercept)/2;
    end
    X_Dis(i) = Dis_Temp;
    Total_Dis_X = Total_Dis_X + Dis_Temp;
    Sum_Dis_X(i) = sum(X_Dis(1:i));
end

Y_Vel = zeros(len-1,1);  

for i=1:1:length(Y_Vel)
    
    if (Y_Acc(i)*Y_Acc(i+1)>=0)
        Vel_Temp = (Y_Acc(i))*delt + ((Y_Acc(i+1)-Y_Acc(i))*delt)/2;
    else
        X_Intercept = delt*Y_Acc(i+1)/(Y_Acc(i+1)-Y_Acc(i));
        Vel_Temp = (Y_Acc(i)*(delt-X_Intercept))/2 + (Y_Acc(i+1)*X_Intercept)/2;
    end
    Y_Vel(i) = Vel_Temp;
end

Y_Dis = zeros(len-2,1);
Total_Dis_Y = 0;
Sum_Dis_Y = zeros(len-2,1);

for i=1:1:length(Y_Dis)
    
    if (Y_Vel(i)*Y_Vel(i+1)>=0)
        Dis_Temp = (Y_Vel(i))*delt + ((Y_Vel(i+1)-Y_Vel(i))*delt)/2;
    else
        X_Intercept = delt*Y_Vel(i+1)/(Y_Vel(i+1)-Y_Vel(i));
        Dis_Temp = (Y_Vel(i)*(delt-X_Intercept))/2 + (Y_Vel(i+1)*X_Intercept)/2;
    end
    Y_Dis(i) = Dis_Temp;
    Total_Dis_Y = Total_Dis_Y + Dis_Temp;
    Sum_Dis_Y(i) = sum(Y_Dis(1:i));
end

disp(Total_Dis_X);
disp(Total_Dis_Y);

T = [0:1:length(Sum_Dis_X)-1];

plot(T,Sum_Dis_X);
title('Displacement from Starting Point')
xlabel("Time (0.1 sec)")
ylabel("Displacement")
grid on
hold on
plot(T,Sum_Dis_Y);
grid on
hold on
legend('X-axis','Y-axis')
