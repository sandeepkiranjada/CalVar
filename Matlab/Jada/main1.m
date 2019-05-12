%% MATH 5546: Final Project
% Submission by Sandeep k JADA

clear; clc; close all;

%% C(A,B) and O(A,C)

load SYS1

disp(['The rank of controllablity Matrix is ',num2str(rank(ctrb(A_sys1,B_sys1))),'/',num2str(length(A_sys1))]);
disp(['The rank of observablity Matrix is ',num2str(rank(obsv(A_sys1,C_sys1))),'/',num2str(length(A_sys1))]);


%% Simulation of Open-loop

tfinal = 10;
dt = 0.001;

t = 0:dt:tfinal;

% Just doing an Euler-Cauchy 1st order

X = IC1;

for n=2:length(t)
    X(:,n) = X(:,n-1) + dt.*A_sys1*X(:,n-1);
end

figure; 
for n = 1:length(A_sys1)
    plot(t,X(n,:)','DisplayName',['state',num2str(n)]);hold on
end
legend show
title('Open Loop Response of the states in sys1');
xlabel('Time (s)')
ylabel('States')

poles_OLP = eig(A_sys1);
figure(2); plot(real(poles_OLP),imag(poles_OLP),'k*'); hold on
title('Poles of sys1');
xlabel('Real')
ylabel('Imaginary')

figure(3); plot(t,X(1,:)','k');hold on
title('Time History of state1 in sys1');
xlabel('Time (s)')
ylabel('States')

[num,den]=ss2tf(A_sys1,B_sys1,C_sys1,0);
sys1_TFOLP = tf(num,den);




%% LQR

[K,P,poles_CLP] = lqr(A_sys1,B_sys1,Q_sys1,R_sys1);

X = IC1;



for n=2:length(t)
    X(:,n) = X(:,n-1) + dt.*(A_sys1-B_sys1*K)*X(:,n-1);
end

figure(4); plot(t,-K*X,'r'); hold on;
title('Control history for sys1')
xlabel('Time (s)')
ylabel('Control')

figure; 
for n = 1:length(A_sys1)
    plot(t,X(n,:)','DisplayName',['state',num2str(n)]);hold on
end
legend show
title('LQR Response of the all states in sys1');
xlabel('Time (s)')
ylabel('States')


figure(2); plot(real(poles_CLP),imag(poles_CLP),'r*')

figure(3); plot(t,X(1,:)','r');

[num,den]=ss2tf(A_sys1,B_sys1,K,0);
sys1_TFLQR = tf(num,den);

% figure;margin(sys1_TF)



%% LQG
[K,P,~] = lqr(A_sys1,B_sys1,Q_sys1,R_sys1);
[F,PI,~] = lqr(A_sys1',C_sys1',M_sys1,N_sys1);
F=F';

X = IC1;
X_e = IC1.*0;

X_aug = [X;-5*ones(length(IC1),1)];

A_aug = [A_sys1-B_sys1*K B_sys1*K; 0.*A_sys1 A_sys1-F*C_sys1];

for n=2:length(t)
    X_aug(:,n) = X_aug(:,n-1) + dt.*(A_aug)*X_aug(:,n-1);
end
X = X_aug(1:length(A_sys1),:);
X_e = X - X_aug(end-length(A_sys1)+1:end,:);



poles_LQG = eig([A_sys1 -B_sys1*K;F*C_sys1 (A_sys1-F*C_sys1-B_sys1*K)]);

figure(2); plot(real(poles_LQG),imag(poles_LQG),'bo')
legend('Open Loop','LQR','LQG')

[num,den]=ss2tf(A_sys1,B_sys1,C_sys1,0);
sys1_TF_LP = tf(num,den);

% figure;margin(sys1_TF)

[num,den]=ss2tf(A_sys1-B_sys1*K-F*C_sys1,F,K,0);
sys1_TF_K = tf(-1*num,den);


figure(3); plot(t,X(1,:)','b');
plot(t,X_e(1,:)','b--');
legend('State1 - Open Loop','State1 - LQR','State1 - LQG (Actual)','State1 - LQG (Estimted)')

figure(4); plot(t,-K*X_e,'b');
legend('LQR','LQG')



sys1_TFLQG = sys1_TF_LP*sys1_TF_K;

% figure;margin(sys1_TF)

%% LQR - Prescribe degree of stablity

[K,P,poles_CLP] = lqr(A_sys1,B_sys1,Q_sys1.*exp(6),R_sys1);
[~,~,poles_LQR] = lqr(A_sys1,B_sys1,Q_sys1,R_sys1);

X = IC1;

for n=2:length(t)
    X(:,n) = X(:,n-1) + dt.*(A_sys1-B_sys1*K)*X(:,n-1);
end

figure; 
for n = 1:length(A_sys1)
    plot(t,X(n,:)','DisplayName',['state',num2str(n)]);hold on
end
title('LQR Presec Response of the all states in sys1');
legend show

figure; plot(real(poles_LQR),imag(poles_LQR),'ko'); hold on;
plot(real(poles_CLP),imag(poles_CLP),'k*')
legend('LQR','LQR (With Presc Deg of Stab)')
xlabel('Real')
ylabel('Imaginary')

[num,den]=ss2tf(A_sys1,B_sys1,K,0);
sys1_TFLQR2 = tf(num,den);

% figure(1); plot(t,-K*X);

%% Plot for GM and PM

figure; margin(sys1_TFOLP); legend('Open-Loop - TF: U to Z');
figure; margin(sys1_TFLQR); legend('LQR - TF: U to U');
figure; margin(sys1_TFLQG); legend('LQG - TF: U to U');

X = X_aug(1:length(A_sys1),:);
X_e = X - X_aug(end-length(A_sys1)+1:end,:);



figure;
title('LQG Response of the all states in sys1');
for n = 1:length(A_sys1)
    subplot(4,2,n)
    plot(t,X(n,:)','b','DisplayName',['x',num2str(n),' (Act)']);hold on
    plot(t,X_e(n,:)','b--','DisplayName',['x',num2str(n),' (Est)']);
    set(gca,'FontSize',6)
    legend show
end
