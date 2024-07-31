%% Search graph for path
clear;clf;clc;
axis equal
axis off
tic

% add path to polytope library
addpath('./polytopes_2017_10_04_v1.9');

% problem parameters
density = 50;
obstacle_number = 5;
dt = 3;
dt_short = 0.25;
N = 2*ceil(dt/dt_short);
u_max = 1;
% N = 5;
% u_max = 1;
no_overlap = true;

% Define the obstacles
O{1} = 1*[-1 1; -1 -1; 1 -1; 1 1]'+[0; 0];

% convert obstacles to H-representation
for i = 1:size(O,2)
    [A_iris,b_iris] = vert2lcon(O{i}');
    A_O{i} = A_iris;
    b_O{i} = b_iris;
end

% plot the walls of the environment
hold on;
line([-2 -2], [2 -2],'color','k')
line([2 2], [2 -2],'color','k')
line([-2 2], [2 2],'color','k')
line([-2 2], [-2 -2],'color','k')

% double integrator allowable states (no obstacles)
A_x = [1 0 0 0; -1 0 0 0; 0 1 0 0; 0 -1 0 0];
b_x = [2; 2; 2; 2];

% plot each obstacle
for i = 1:size(O,2)
    patch(O{i}(1,:),O{i}(2,:),'r','facealpha',0.05)
end
axis equal

% sample a polytope in the free space
N_polys = 10;
n_verts = 4;
for i = 1:N_polys
    V = sample_V_polytope(n_verts, A_x, b_x, A_O, b_O);
    for j = 1:size(V,2)
        patch(V(1,:),V(2,:),'b','facealpha',0.05)
    end
end

% define the intial and end conditions
IC = [-1.9 -1.9];
EC = [1.9 1.9];

% plot the initial and end conditions
scatter(IC(1), IC(2),50,'r','filled')
scatter(EC(1), EC(2),50,'g','filled')

disp("Done.")
disp(toc)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Auxillary functions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% sample V_polytope
function V = sample_V_polytope(n_verts, A_x, b_x, A_O, b_O)
    
    % get the number of obstacles
    n_obs = size(A_O,2);
    
    % get enviornment bounds
    x1_max =  b_x(1);
    x1_min = -b_x(2);
    x2_max =  b_x(3);
    x2_min = -b_x(4);
    
    % find a polytope whose vertices are in the free sapce and dies not intersect any obstacles
    counter = 0;
    V_valid = false;
    while V_valid == false
        
        % generate a random polytope whos vertices are in the free space
        V = zeros(2, n_verts);
        n = 1;
        while n <= n_verts

            % sample the free space
            x1_rand = x1_min + (x1_max - x1_min)*rand(1);
            x2_rand = x2_min + (x2_max - x2_min)*rand(1);
            x_sample = [x1_rand; x2_rand];

            V(:,n) = x_sample;
            n = n + 1;

        end

        % take the convex hull of the sampled points
        K = convhull(V(1,:), V(2,:)); 
        V = V(:,K);

        % convert to H-representation
        [A_sample, b_sample] = vert2lcon(V');

        % check if the polytope intersects any obstacles via linear program
        for i = 1:n_obs
            A = [A_sample; A_O{i}];
            b = [b_sample; b_O{i}];
            [~, ~, exitflag] = linprog(ones(size(A,2), 1), A, b);
    
            % the problem is infeasible, the polytope does not intersect the obstacle
            if exitflag ~= -2
                break
            end
            
            % went though all obstacles and the polytope does not intersect any
            if i == n_obs
                V_valid = true;
                break;
            end
        end

        % counter to prevent infinite loop
        counter = counter + 1;
        if counter > 1500
            break
        end
    end
end
