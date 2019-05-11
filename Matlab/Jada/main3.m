%% MATH 5546: Final Project
% Submission by Sandeep k JADA

clear; clc; close all;

%% C(A,B) and O(A,C)

load SYS3

disp(['The rank of controllablity Matrix is ',num2str(rank(ctrb(A_sys3,B_sys3))),'/',num2str(length(A_sys3))]);
disp(['The rank of observablity Matrix is ',num2str(rank(obsv(A_sys3,C_sys3))),'/',num2str(length(A_sys3))]);


%% Simulation of Open-loop

tfinal = 10;
dt = 0.001;

t = 0:dt:tfinal;

% Just doing an Euler-Cauchy 1st order

X = IC3;

for n=2:length(t)
    X(:,n) = X(:,n-1) + dt.*A_sys3*X(:,n-1);
end

figure; 
for n = 1:length(A_sys3)
    plot(t,X(n,:)','DisplayName',['state',num2str(n)]);hold on
end
legend show
title('Open Loop Response of the states in sys3');
xlabel('Time (s)')
ylabel('States')

poles_OLP = eig(A_sys3);
figure(2); plot(real(poles_OLP),imag(poles_OLP),'k*'); hold on
title('Poles of sys3');
xlabel('Real')
ylabel('Imaginary')

figure(3); plot(t,X(1,:)','k');hold on
title('Time History of state1 in sys3');
xlabel('Time (s)')
ylabel('States')

[num,den]=ss2tf(A_sys3,B_sys3,C_sys3,0);
sys3_TFOLP = tf(num,den);




%% LQR

[K,P,poles_CLP] = lqr(A_sys3,B_sys3,Q_sys3,R_sys3);

X = IC3;



for n=2:length(t)
    X(:,n) = X(:,n-1) + dt.*(A_sys3-B_sys3*K)*X(:,n-1);
end

figure(4); plot(t,-K*X,'r'); hold on;
title('Control history for sys3')
xlabel('Time (s)')
ylabel('Control')

figure; 
for n = 1:length(A_sys3)
    plot(t,X(n,:)','DisplayName',['state',num2str(n)]);hold on
end
legend show
title('LQR Response of the all states in sys3');
xlabel('Time (s)')
ylabel('States')


figure(2); plot(real(poles_CLP),imag(poles_CLP),'r*')

figure(3); plot(t,X(1,:)','r');

[num,den]=ss2tf(A_sys3,B_sys3,K,0);
sys3_TFLQR = tf(num,den);

% figure;margin(sys3_TF)



%% LQG
[K,P,~] = lqr(A_sys3,B_sys3,Q_sys3,R_sys3);
[F,PI,~] = lqr(A_sys3',C_sys3',M_sys3,N_sys3);
F=F';

X = IC3;
X_e = IC3.*0;

X_aug = [X;X-X_e];

A_aug = [A_sys3-B_sys3*K B_sys3*K; 0.*A_sys3 A_sys3-F*C_sys3];

for n=2:length(t)
    X_aug(:,n) = X_aug(:,n-1) + dt.*(A_aug)*X_aug(:,n-1);
end
X = X_aug(1:length(A_sys3),:);
X_e = X - X_aug(end-length(A_sys3)+1:end,:);



poles_LQG = eig([A_sys3 -B_sys3*K;F*C_sys3 (A_sys3-F*C_sys3-B_sys3*K)]);

figure(2); plot(real(poles_LQG),imag(poles_LQG),'bo')
legend('Open Loop','LQR','LQG')

[num,den]=ss2tf(A_sys3,B_sys3,C_sys3,0);
sys3_TF_LP = tf(num,den);

% figure;margin(sys3_TF)

[num,den]=ss2tf(A_sys3-B_sys3*K-F*C_sys3,F,K,0);
sys3_TF_K = tf(-1*num,den);


figure(3); plot(t,X(1,:)','b');
plot(t,X_e(1,:)','b--');
legend('State1 - Open Loop','State1 - LQR','State1 - LQG (Actual)','State1 - LQG (Estimted)')

figure(4); plot(t,-K*X_e,'b');
legend('LQR','LQG')



sys3_TFLQG = sys3_TF_LP*sys3_TF_K;

% figure;margin(sys3_TF)

%% LQR - Prescribe degree of stablity

[K,P,poles_CLP] = lqr(A_sys3,B_sys3,Q_sys3.*exp(6),R_sys3);
[~,~,poles_LQR] = lqr(A_sys3,B_sys3,Q_sys3,R_sys3);

X = IC3;

for n=2:length(t)
    X(:,n) = X(:,n-1) + dt.*(A_sys3-B_sys3*K)*X(:,n-1);
end

figure; 
for n = 1:length(A_sys3)
    plot(t,X(n,:)','DisplayName',['state',num2str(n)]);hold on
end
title('LQR Presec Response of the all states in sys3');
legend show

figure; plot(real(poles_LQR),imag(poles_LQR),'ko'); hold on;
plot(real(poles_CLP),imag(poles_CLP),'k*')
legend('LQR','LQR (With Presc Deg of Stab)')
xlabel('Real')
ylabel('Imaginary')

[num,den]=ss2tf(A_sys3,B_sys3,K,0);
sys3_TFLQR2 = tf(num,den);

% figure(3); plot(t,-K*X);

%% Plot for GM and PM

figure; margin(sys3_TFOLP); legend('Open-Loop - TF: U to Z');
figure; margin(sys3_TFLQR); legend('LQR - TF: U to U');
figure; margin(sys3_TFLQG); legend('LQG - TF: U to U');

X = X_aug(1:length(A_sys3),:);
X_e = X - X_aug(end-length(A_sys3)+1:end,:);



figure;
title('LQG Response of the all states in sys3');
for n = 1:length(A_sys3)
    subplot(4,2,n)
    plot(t,X(n,:)','b','DisplayName',['x',num2str(n),' (Act)']);hold on
    plot(t,X_e(n,:)','b--','DisplayName',['x',num2str(n),' (Est)']);
    set(gca,'FontSize',6)
    legend show
end
