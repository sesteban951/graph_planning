%% Search graph for paths
clear;clf;clc;
axis equal
axis off
tic

% add path to libraries
addpath('/home/sergio/projects/graph_planning'); 
addpath('/home/sergio/repos/mosek/10.2/toolbox/r2017a');

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

% Define the environment
E = 2*[-1 1; -1 -1; 1 -1; 1 1]' + [0; 0];

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

% sample some polytopes in the free space
N_polys = 10;  % number of poytopes in free space
n_verts = 4;  % number of vertices of each polytope
for i = 1:N_polys

    % rejection sample
    % V{i} = sample_V_polytope(n_verts, A_x, b_x, A_O, b_O);

    % IRIS sample
    initial_point = rand(2,1)*2-1;
    V{i} = sample_V_polytope_IRIS(E, O, initial_point);

    % plot the polytope
    for j = 1:size(V,2)
        V_ = V{j};
        patch(V_(:,1), V_(:,2),'b','facealpha',0.05)
        scatter(V_(:,1), V_(:,2),50,'b','filled')
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

% sample V polytope via rejection sampling
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
        
        % generate a random polytope whos vertices are within environment bounds
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
        opt_options = optimoptions('linprog', 'Display', 'none'); % Create options structure
        for i = 1:n_obs

            % solve LP feasibility problem
            A = [A_sample; A_O{i}];
            b = [b_sample; b_O{i}];
            [~, ~, exitflag] = linprog(ones(size(A,2), 1), A, b, [], [], [], [], opt_options);
    
            % the problem is infeasible ==> the polytope does not intersect the obstacle
            if exitflag ~= -2
                break
            end
            
            % went through all obstacles and the polytope does not intersect any
            if i == n_obs
                V_valid = true;
                V = V';
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

% sample V polytope via IRIS
function V_iris = sample_V_polytope_IRIS(E, O, initial_point)

    % convert enviorment to H-representation
    [A_E, b_E] = vert2lcon(E');

    % inflate a region around the intial point
    [A, b, ~, ~, ~]  = iris.inflate_region(O, A_E, b_E, initial_point);
    
    % convert to V-representation
    V = lcon2vert(A, b);
    K = convhull(V);
    V_iris = V(K,:);
end
