%% MATH 5546: Final Project
% Submission by Sandeep k JADA

clear; clc; close all;

%% C(A,B) and O(A,C)

load SYS2

disp(['The rank of controllablity Matrix is ',num2str(rank(ctrb(A_sys2,B_sys2))),'/',num2str(length(A_sys2))]);
disp(['The rank of observablity Matrix is ',num2str(rank(obsv(A_sys2,C_sys2))),'/',num2str(length(A_sys2))]);


%% Simulation of Open-loop

tfinal = 10;
dt = 0.001;

t = 0:dt:tfinal;

% Just doing an Euler-Cauchy 1st order

X = IC2;

for n=2:length(t)
    X(:,n) = X(:,n-1) + dt.*A_sys2*X(:,n-1);
end

figure; 
for n = 1:length(A_sys2)
    plot(t,X(n,:)','DisplayName',['state',num2str(n)]);hold on
end
legend show
title('Open Loop Response of the states in sys2');
xlabel('Time (s)')
ylabel('States')

poles_OLP = eig(A_sys2);
figure(2); plot(real(poles_OLP),imag(poles_OLP),'k*'); hold on
title('Poles of sys2');
xlabel('Real')
ylabel('Imaginary')

figure(3); plot(t,X(1,:)','k');hold on
title('Time History of state1 in sys2');
xlabel('Time (s)')
ylabel('States')

[num,den]=ss2tf(A_sys2,B_sys2,C_sys2,0);
sys2_TFOLP = tf(num,den);




%% LQR

[K,P,poles_CLP] = lqr(A_sys2,B_sys2,Q_sys2,R_sys2);

X = IC2;



for n=2:length(t)
    X(:,n) = X(:,n-1) + dt.*(A_sys2-B_sys2*K)*X(:,n-1);
end

figure(4); plot(t,-K*X,'r'); hold on;
title('Control history for sys2')
xlabel('Time (s)')
ylabel('Control')

figure; 
for n = 1:length(A_sys2)
    plot(t,X(n,:)','DisplayName',['state',num2str(n)]);hold on
end
legend show
title('LQR Response of the all states in sys2');
xlabel('Time (s)')
ylabel('States')


figure(2); plot(real(poles_CLP),imag(poles_CLP),'r*')

figure(3); plot(t,X(1,:)','r');

[num,den]=ss2tf(A_sys2,B_sys2,K,0);
sys2_TFLQR = tf(num,den);

% figure;margin(sys2_TF)



%% LQG
[K,P,~] = lqr(A_sys2,B_sys2,Q_sys2,R_sys2);
[F,PI,~] = lqr(A_sys2',C_sys2',M_sys2,N_sys2);
F=F';

X = IC2;
X_e = IC2.*0;

X_aug = [X;X-X_e];

A_aug = [A_sys2-B_sys2*K B_sys2*K; 0.*A_sys2 A_sys2-F*C_sys2];

for n=2:length(t)
    X_aug(:,n) = X_aug(:,n-1) + dt.*(A_aug)*X_aug(:,n-1);
end
X = X_aug(1:length(A_sys2),:);
X_e = X - X_aug(end-length(A_sys2)+1:end,:);



poles_LQG = eig([A_sys2 -B_sys2*K;F*C_sys2 (A_sys2-F*C_sys2-B_sys2*K)]);

figure(2); plot(real(poles_LQG),imag(poles_LQG),'bo')
legend('Open Loop','LQR','LQG')

[num,den]=ss2tf(A_sys2,B_sys2,C_sys2,0);
sys2_TF_LP = tf(num,den);

% figure;margin(sys2_TF)

[num,den]=ss2tf(A_sys2-B_sys2*K-F*C_sys2,F,K,0);
sys2_TF_K = tf(-1*num,den);


figure(3); plot(t,X(1,:)','b');
plot(t,X_e(1,:)','b--');
legend('State1 - Open Loop','State1 - LQR','State1 - LQG (Actual)','State1 - LQG (Estimted)')

figure(4); plot(t,-K*X_e,'b');
legend('LQR','LQG')



sys2_TFLQG = sys2_TF_LP*sys2_TF_K;

% figure;margin(sys2_TF)

%% LQR - Prescribe degree of stablity

[K,P,poles_CLP] = lqr(A_sys2,B_sys2,Q_sys2.*exp(6),R_sys2);
[~,~,poles_LQR] = lqr(A_sys2,B_sys2,Q_sys2,R_sys2);

X = IC2;

for n=2:length(t)
    X(:,n) = X(:,n-1) + dt.*(A_sys2-B_sys2*K)*X(:,n-1);
end

figure; 
for n = 1:length(A_sys2)
    plot(t,X(n,:)','DisplayName',['state',num2str(n)]);hold on
end
title('LQR Presec Response of the all states in sys2');
legend show

figure; plot(real(poles_LQR),imag(poles_LQR),'ko'); hold on;
plot(real(poles_CLP),imag(poles_CLP),'k*')
legend('LQR','LQR (With Presc Deg of Stab)')
xlabel('Real')
ylabel('Imaginary')

[num,den]=ss2tf(A_sys2,B_sys2,K,0);
sys2_TFLQR2 = tf(num,den);

% figure(3); plot(t,-K*X);

%% Plot for GM and PM

figure; margin(sys2_TFOLP); legend('Open-Loop - TF: U to Z');
figure; margin(sys2_TFLQR); legend('LQR - TF: U to U');
figure; margin(sys2_TFLQG); legend('LQG - TF: U to U');

X = X_aug(1:length(A_sys2),:);
X_e = X - X_aug(end-length(A_sys2)+1:end,:);



figure;
title('LQG Response of the all states in sys2');
for n = 1:length(A_sys2)
    subplot(4,2,n)
    plot(t,X(n,:)','b','DisplayName',['x',num2str(n),' (Act)']);hold on
    plot(t,X_e(n,:)','b--','DisplayName',['x',num2str(n),' (Est)']);
    set(gca,'FontSize',6)
    legend show
end
