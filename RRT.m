
classdef RRT
    properties
        q_start
        q_goal
        max_iter
    end
    methods
        function obj = RRT(q_start,q_goal,max_iter)
            obj.q_start = q_start;
            obj.q_goal = q_goal;
            obj.max_iter = max_iter;
        end
        function distance = dist_fn(obj,node1,node2)
            distance = sqrt((node1.q(1)-node2.q(1))^2 + (node1.q(2)-node2.q(2))^2 + (node1.q(3)-node2.q(3))^2);
        end
        function q_random = sample_fn(obj)
            q_random = [-0.2 + (0.8)*rand(),-0.1 + (0.2)*rand(),-0.3 + (1.1)*rand(),0,-pi/3 + (pi)*rand(),0];
        end
        function nearest_node = nearest_neighbour(obj,rand_node, tree)
            nearest_node = [];
            min_distance = inf;
            
            for i = 1:length(tree)
                tree_node = tree(i);
                distance = obj.dist_fn(tree_node, rand_node);
                if distance < min_distance
                    min_distance = distance;
                    nearest_node = tree_node;
                end
            end
        end
        function node_new = extend(obj,node_ontree,rand_node,step_size)
            node_distance = obj.dist_fn(node_ontree,rand_node);
            if node_distance < step_size
                node_new = rand_node;
                node_new.parent = node_ontree;           
            else
                q_new = node_ontree.q + (rand_node.q - node_ontree.q).*step_size;
                node_new = node(q_new);
                node_new.parent = node_ontree;
            end
        end
        function path = run_bidirectional(obj)
            start_node = node(obj.q_start);
            goal_node = node(obj.q_goal);
            start_tree(1) = start_node;
            goal_tree(1) = goal_node;
             %---------------------------------------------------
            model = model_create([0,0,0,0,0,0]);
            walls = model.create_countershaft();
            fixed_shaft = model.create_case();   
            %--------------------------------------------- 
            for i = 1:obj.max_iter
                
                %------------From start node direction--------------------
                if mod(i,10) == 0
                    q_random_start = goal_node.q;
                    random_node_start = goal_node;
                else
                    q_random_start = obj.sample_fn();
                    random_node_start = node(q_random_start);
                end

                % plot the node
                plot3(q_random_start(1),q_random_start(2),q_random_start(3),'x','Color',[0 0.4470 0.7410])
                nearest_node_start = obj.nearest_neighbour(random_node_start,start_tree);
                node_new_start = obj.extend(nearest_node_start,random_node_start,0.05);
                %--Collision Check------------------------------------
                temp_shaft = model.move(node_new_start.q);
                collides = model.collision_check(temp_shaft);
                if collides == 1
                    disp('Pose not considered due to collision');
                    continue
                end
                
                line([nearest_node_start.q(1),node_new_start.q(1)],[nearest_node_start.q(2),node_new_start.q(2)],[nearest_node_start.q(3),node_new_start.q(3)]...
                    ,'Color','red','LineWidth',2)
                drawnow;
                start_tree(end+1) = node_new_start;

                %--------- From goal node direction------------------------
                
                if mod(i,10) == 0
                    q_random_goal = start_node.q;
                    random_node_goal = start_node;
                else
                    q_random_goal = obj.sample_fn();
                    random_node_goal = node(q_random_goal);
                end
                
                % plot the node
                plot3(q_random_goal(1),q_random_goal(2),q_random_goal(3),'x','Color',[0 0.4470 0.7410])
                nearest_node_goal = obj.nearest_neighbour(random_node_goal,goal_tree);
                node_new_goal = obj.extend(nearest_node_goal,random_node_goal,0.05);
                
                % Ensure that the 3d pose generated is possible
                % and does not create collisions
                temp_shaft = model.move(node_new_goal.q);
                collides = model.collision_check(temp_shaft);
                if collides == 1
                    disp('Pose not considered due to collision');
                    continue
                end
                %--------------------------------------------------------
                
                line([nearest_node_goal.q(1),node_new_goal.q(1)],[nearest_node_goal.q(2),node_new_goal.q(2)],[nearest_node_goal.q(3),node_new_goal.q(3)]...
                    ,'Color','blue','LineWidth',2)
                drawnow;
                goal_tree(end+1) = node_new_goal;

                if obj.dist_fn(node_new_goal,node_new_start)<0.05 && abs(node_new_start.q(5)-node_new_goal.q(5))<pi/5
                    break;
                end
            end
            
            %Creating path from start node
            path_start(1) = start_tree(end);
            parent_start = start_tree(end).parent;

            while isequal(class(parent_start),'node')
                path_start(end+1) = parent_start;
                parent_start = parent_start.parent;
            end
            path_start = flip(path_start);
            
            % Creating path from goal node
            path_goal(1) = goal_tree(end);
            path_goal(1).parent = -1;
            parent_goal = goal_tree(end).parent;

            while isequal(class(parent_goal),'node')
                path_goal(end+1) = parent_goal;
                path_goal(end).parent = path_goal(end-1);
                parent_goal = parent_goal.parent;
            end
            path_goal(1).parent = path_start(end);
            path = [path_start,path_goal];
            return
        end

        function path = run_start(obj)
            start_node = node(obj.q_start);
            goal_node = node(obj.q_goal);
            start_tree(1) = start_node;
            %---------------------------------------------------
            model = model_create([0,0,0,0,0,0]);
            walls = model.create_countershaft();
            fixed_shaft = model.create_case();   
            %---------------------------------------------
            for i = 1:obj.max_iter
                % From start node direction--------------------
                if mod(i,10) == 0
                    q_random_start = goal_node.q;
                    random_node_start = goal_node;
                else
                    q_random_start = obj.sample_fn();
                    random_node_start = node(q_random_start);
                end
                

                %------------------------------------------------
                % plot the node
                plot3(q_random_start(1),q_random_start(2),q_random_start(3),'x','Color',[0 0.4470 0.7410]);
                nearest_node_start = obj.nearest_neighbour(random_node_start,start_tree); 
                node_new_start = obj.extend(nearest_node_start,random_node_start,0.05);

                % ---------------collision check-----------------------------
                % Ensure that the 3d pose generated is possible
                % and does not create collisions
                temp_shaft = model.move(node_new_start.q);
                collides = model.collision_check(temp_shaft);
                if collides == 1
                    disp('Pose not considered due to collision');
                    continue
                end
                %--------------------------------------------------------
                line([nearest_node_start.q(1),node_new_start.q(1)],[nearest_node_start.q(2),node_new_start.q(2)],[nearest_node_start.q(3),node_new_start.q(3)]...
                    ,'Color','red','LineWidth',2); 
                drawnow;
                start_tree(end+1) = node_new_start;

                if obj.dist_fn(node_new_start,goal_node)<0.05
                    goal_node.parent = node_new_start;
                    break;
                end

            end
            
            if ~isequal(class(goal_node.parent),'node')
                path = [];
                return
            end
            %Creating path from start node
            path(1) = goal_node;
            parent_start = goal_node.parent;

            while isequal(class(parent_start),'node')
                path(end+1) = parent_start;
                parent_start = parent_start.parent;
            end
            path = flip(path);
            return
        end
    end
end

