%% MATH 5546: Final Project
% Submission by Sandeep k JADA

clear; clc; close all;

%% C(A,B) and O(A,C)

load SYS1

disp(['The rank of controllablity Matrix is ',num2str(rank(ctrb(A_sys1,B_sys1))),'/',num2str(length(A_sys1))]);
disp(['The rank of observablity Matrix is ',num2str(rank(obsv(A_sys1,C_sys1))),'/',num2str(length(A_sys1))]);


%% Simulation of Open-loop

tfinal = 10;
dt = 0.01;

t = 0:dt:tfinal;

% Just doing an Euler-Cauchy 1st order

X = IC1;

for n=2:length(t)
    X(:,n) = X(:,n-1) + dt.*A_sys1*X(:,n-1);
end

figure; 
for n = 1:length(A_sys1)
    plot(t,X(n,:)');hold on
end
title('Open Loop Response of the state in sys1');


poles_OLP = eig(A_sys1);
figure(2); plot(real(poles_OLP),imag(poles_OLP),'k*'); hold on
title('Poles of sys1');


[num,den]=ss2tf(A_sys1,B_sys1,C_sys1,0);
sys1_TF = tf(num,den);

figure;margin(sys1_TF)


%% LQR

[K,P,poles_CLP] = lqr(A_sys1,B_sys1,Q_sys1,R_sys1);

X = IC1;

for n=2:length(t)
    X(:,n) = X(:,n-1) + dt.*(A_sys1-B_sys1*K)*X(:,n-1);
end

figure; 
for n = 1:length(A_sys1)
    plot(t,X(n,:)');hold on
end
title('Closed Loop Response of the state in sys1');


figure(2); plot(real(poles_CLP),imag(poles_CLP),'go')
% legend('Open Loop - A','LQR - A-B*K')

%% LQR - Prescribe degree of stablity

[K,P,poles_CLP] = lqr(A_sys1,B_sys1,Q_sys1.*exp(3),R_sys1);

X = IC1;

for n=2:length(t)
    X(:,n) = X(:,n-1) + dt.*(A_sys1-B_sys1*K)*X(:,n-1);
end

figure; 
for n = 1:length(A_sys1)
    plot(t,X(n,:)');hold on
end
title('Closed Loop Response of the state in sys1');


figure(2); plot(real(poles_CLP),imag(poles_CLP),'r+')
legend('Open Loop - A','LQR - A-B*K','LQR - A-B*K')