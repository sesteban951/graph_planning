clear all; clc; clf;
figure(1)
hold on; axis equal; grid on;

% ------------------------------ G Zonotope ------------------------------
G = [1 0 1;0 1 1];  % Generator matrix with 3 generators
c = zeros(2,1);     % Center at the origin
Z = zono(G,c)       % Creates a zonotope

plot(Z,'b',0.1)                % Plots hybrid zonotope in transparent blue

% ------------------------------ CG Zonotope ------------------------------
G = [1 0 1;0 1 1];   % Generator matrix with 3 generators
c = zeros(2,1);      % Center at the origin
A = ones(1,3);       % Constraint matrix with 1 constraint
b = 1;               % Constraint offset vector
C = conZono(G,c,A,b) % Creates a constrained zonotope

plot(C,'r',0.1)                % Plots hybrid zonotope in transparent red

% ------------------------------ H Zonotope ------------------------------
Gc = [1 0 1;0 1 1];          % Continuous generator matrix with 3 generators
Gb = [1 2;2 1];              % Binary generator matrix with 2 generators
c = zeros(2,1);              % Center at the origin
Ac = ones(1,3);              % Continuous constraint matrix with 1 constraint
Ab = ones(1,2);              % Binary constraint matrix with 1 constraint
b = 1;                       % Constraint offset vector
H = hybZono(Gc,Gb,c,Ac,Ab,b) % Creates a hybrid zonotope

plot(H,'g',0.1)                % Plots hybrid zonotope in transparent green


for i = 1:size(G,2)            % Plots the generators in red
    quiver(c(1),c(2),G(1,i),G(2,i),'r','LineWidth',2)
end
plot(c(1),c(2),'k*')           % Plots the center in black

legend('G Zonotope','CG Zonotope','H Zonotope')