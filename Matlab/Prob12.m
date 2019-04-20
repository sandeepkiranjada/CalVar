clear; clc; close all;


N = 8;

A = diag(ones(1,N-1),-1)+ -2*eye(N) + diag(ones(1,N-1),1);
A = A.*(N+1)^2;

B=(1:N)'./(N+1);

Q = eye(N);
R = 1;

[K,P,E] = lqr(A,B,Q,R);


X0 = ones(N,1);

tf = 10; 
dt = 0.1;

[t,X] = ode45(@(t,X) Xdot(t,X,A,B,K),[0 1],X0);

plot(X')
% 
% t = 0:dt:tf;
% 
% for n = 1:length(t)
%     X(:,n) = exp((A-B*K).*-t(n))*X0;
% end


