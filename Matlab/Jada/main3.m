%% MATH 5546: Final Project
% Submission by Sandeep k JADA

clear; clc; close all;

%% C(A,B) and O(A,C)

load SYS3

disp(['The rank of controllablity Matrix is ',num2str(rank(ctrb(A_sys3,B_sys3))),'/',num2str(length(A_sys3))]);
disp(['The rank of observablity Matrix is ',num2str(rank(obsv(A_sys3,C_sys3))),'/',num2str(length(A_sys3))]);


%% Simulation of Open-loop

tfinal = 20;
dt = 0.001;

t = 0:dt:tfinal;

% Just doing an Euler-Cauchy 1st order

X = IC3;

for n=2:length(t)
    X(:,n) = X(:,n-1) + dt.*A_sys3*X(:,n-1);
end

figure; 
for n = 1:length(A_sys3)
    plot(t,X(n,:)');hold on
end
title('Open Loop Response of the state in sys3');


poles_OLP = eig(A_sys3);
figure(2); plot(real(poles_OLP),imag(poles_OLP),'k*'); hold on
title('Poles of sys3');


[num,den]=ss2tf(A_sys3,B_sys3,C_sys3,0);
sys3_TF = tf(num,den);

figure;margin(sys3_TF)


%% LQR

[K,P,poles_CLP] = lqr(A_sys3,B_sys3,Q_sys3,R_sys3);

X = IC3;

for n=2:length(t)
    X(:,n) = X(:,n-1) + dt.*(A_sys3-B_sys3*K)*X(:,n-1);
end

figure; 
for n = 1:length(A_sys3)
    plot(t,X(n,:)');hold on
end
title('Closed Loop Response of the state in sys3');


figure(2); plot(real(poles_CLP),imag(poles_CLP),'go')
% legend('Open Loop - A','LQR - A-B*K')

%% LQR - Prescribe degree of stablity

[K,P,poles_CLP] = lqr(A_sys3,B_sys3,Q_sys3.*exp(3),R_sys3);

X = IC3;

for n=2:length(t)
    X(:,n) = X(:,n-1) + dt.*(A_sys3-B_sys3*K)*X(:,n-1);
end

figure; 
for n = 1:length(A_sys3)
    plot(t,X(n,:)');hold on
end
title('Closed Loop Response of the state in sys3');


figure(2); plot(real(poles_CLP),imag(poles_CLP),'r+')
legend('Open Loop - A','LQR - A-B*K','LQR - A-B*K')