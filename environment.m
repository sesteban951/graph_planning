%% Search graph for path
clear;clf;
axis equal
axis off
tic

% add path to polytope library
addpath('./polytopes_2017_10_04_v1.9');

density = 50;
obstacle_number = 5;
dt = 3;
dt_short = 0.25;
N = 2*ceil(dt/dt_short);
u_max = 1;
% N = 5;
% u_max = 1;
no_overlap = true;

O{1} = 1*[-1 1; -1 -1; 1 -1; 1 1]'+[0; 0];

for i = 1:size(O,2)
    [A_iris,b_iris] = vert2lcon(O{i}');
    A_O{i} = A_iris;
    b_O{i} = b_iris;
end

hold on;
line([-2 -2], [2 -2],'color','k')
line([2 2], [2 -2],'color','k')
line([-2 2], [2 2],'color','k')
line([-2 2], [-2 -2],'color','k')

A_x = [1 0 0 0; -1 0 0 0; 0 1 0 0; 0 -1 0 0];
b_x = [2; 2; 2; 2];

for i = 1:size(O,2)
    patch(O{i}(1,:),O{i}(2,:),'r','facealpha',0.05)
end
axis equal

IC = [-1.9 -1.9];
EC = [1.9 1.9];

scatter(IC(1), IC(2),50,'r','filled')
scatter(EC(1), EC(2),50,'g','filled')