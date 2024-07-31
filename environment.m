%% Search graph for paths
clear;clf;clc;
axis equal
axis off

% add path for libraries
addpath('/home/sergio/projects/graph_planning'); 
addpath('/home/sergio/repos/mosek/10.2/toolbox/r2017a');
addpath("./polytopes_2017_10_04_v1.9");
addpath("./chebycenter");

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
tic
N_polys = 15;  % number of poytopes in free space
n_verts = 4;   % number of vertices of each polytope
for i = 1:N_polys

    % rejection sample
    V{i} = sample_V_polytope(n_verts, A_x, b_x, A_O, b_O);

    % IRIS sample
    % initial_point = rand(2,1)*2-1;
    % V{i} = sample_V_polytope_IRIS(E, O, initial_point);
end
fprintf("Time to generate polytopes: %f [sec]\n", toc)

% define the intial and end conditions
IC = [-1.7 -1.7];
EC = [1.7 1.7];

% manually add a polytope for the initial and end conditions
V_IC =  0.3 * [-1 1; -1 -1; 1 -1; 1 1]' + IC';
V_EC =  0.3 * [-1 1; -1 -1; 1 -1; 1 1]' + EC';
V{end+1} = V_IC;
V{end+1} = V_EC;

% compute chebychev circles of the generated polytopes
[C, ~] = chebychev_circles(V);

% plot polytope info
for j = 1:size(V,2)
    V_ = V{j};
    patch(V_(1,:), V_(2,:),'b','facealpha',0.05) % plot the polytope
    % scatter(V_(1,:), V_(2,:),50,'b','filled')    % plot the vertices
    scatter(C{j}(1), C{j}(2),50,'m','filled')    % plot the chebychev center
end

% build the graph
tic
G = build_graph(V, C);
e = G.Edges;
msg = sprintf("Graph has %d nodes and %d edges", numnodes(G), numedges(G));
disp(msg)
disp(e)
fprintf("Time to build graph: %f [sec]\n", toc)

% plot all the edges
for i = 1:size(e,1)

    % extract the edge information
    edge_vert_idx = e(i,"EndNodes").EndNodes;
    edge_weight = e(i,"Weight").Weight;

    % draw the edges
    C1 = C{edge_vert_idx(1)};
    C2 = C{edge_vert_idx(2)};

    % plot the edge
    line([C1(1) C2(1)], [C1(2) C2(2)], 'color', 'm', 'linestyle', '--', 'linewidth', 1)
end

% find the shortest path (if feasible)
tic
P = shortestpath(G, size(V,2)-1, size(V,2), "Method", "positive");
fprintf("Time to find shortest path: %f [sec]\n", toc)

% plot the shortest path
if isempty(P)
    msg = sprintf("No path found");
    disp(msg)
else
    msg = sprintf("Shortest path found");
    disp(msg)
    P
    for i = 1:size(P,2)-1
        C1 = C{P(i)};
        C2 = C{P(i+1)};
        line([C1(1) C2(1)], [C1(2) C2(2)], 'color', 'k', 'linewidth', 3)
    end
end

% plot the initial and end conditions
scatter(IC(1), IC(2),100,'r','filled')
scatter(EC(1), EC(2),100,'g','filled')

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Auxillary functions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% build a graph given a list of V-polytopes
function G = build_graph(V, C)

    % define an index for each polytope
    V_index = 1:size(V,2);
    V_combos = nchoosek(V_index,2);
    num_combos = size(V_combos,1);

    % instantiate the graph
    G = graph();

    % add all the nodes to the graph
    G = addnode(G, size(V,2));

    % check if there is interesection between each pair of polytopes
    for i = 1:num_combos

        % get the two polytopes
        V1 = V{V_combos(i,1)};
        V2 = V{V_combos(i,2)};

        % check if the polytopes intersect
        V_intersection = intersection_polytopes(V1, V2);
        
        % add an edge to the graph if there is an intersection
        if ~isempty(V_intersection)
            weight = norm(C{V_combos(i,1)} - C{V_combos(i,2)});
            G = addedge(G, V_combos(i,1), V_combos(i,2), weight);
        end
    end

end

% compute the intersection of two polytopes
function V = intersection_polytopes(V1, V2)
    I = intersectionHull('vert', V1', 'vert', V2');
    V = I.vert';
end

% function to compute chebychev centers
function [C, R] = chebychev_circles(V)
    
    % get the chebychev centers and radii of the polytopes
    num_polytopes = size(V,2);
    C = cell(1, num_polytopes);
    R = cell(1, num_polytopes);
    for i = 1:num_polytopes
        [A, b] = vert2lcon(V{i}');
        [c, r] = chebycenter(A, b);
        C{i} = c;
        R{i} = r;
    end
end

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
    
            % the problem is feasible ==> the polytope intersects an obstacle
            if exitflag ~= -2
                break
            end
            
            % went through all obstacles and the polytope does not intersect any
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

% sample V polytope via IRIS
function V_iris = sample_V_polytope_IRIS(E, O, initial_point)

    % convert enviorment to H-representation
    [A_E, b_E] = vert2lcon(E');

    % inflate a region around the intial point
    [A, b, ~, ~, ~]  = iris.inflate_region(O, A_E, b_E, initial_point);
    
    % convert to V-representation
    V = lcon2vert(A, b);
    K = convhull(V);
    V_iris = V(K,:)';
end
