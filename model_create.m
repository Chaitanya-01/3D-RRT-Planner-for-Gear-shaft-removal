classdef model_create
    properties
        q
    end
    methods
        function obj = model_create(q)
            obj.q = q;
        end
        function main_shaft = create_mainshaft(obj)
            main_shaft = rigidBodyTree("DataFormat", "column");
            basename = main_shaft.BaseName;
            
            % Create rigid bodies and joints
            segment1 = rigidBody('segment1');
            segment2 = rigidBody('segment2');
            segment3 = rigidBody('segment3');
            segment4 = rigidBody('segment4');
            segment5 = rigidBody('segment5');
            segment6 = rigidBody('segment6');
            segment7 = rigidBody('segment7');
            segment8 = rigidBody('segment8');
            segment9 = rigidBody('segment9');
            segment10 = rigidBody('segment10');
            
            
            joint1 = rigidBodyJoint('joint1','revolute'); % joins first segment with the base of the tree
            joint2 = rigidBodyJoint('joint2','fixed');
            joint3 = rigidBodyJoint('joint3','fixed');
            joint4 = rigidBodyJoint('joint4','fixed');
            joint5 = rigidBodyJoint('joint5','fixed');
            joint6 = rigidBodyJoint('joint6','fixed');
            joint7 = rigidBodyJoint('joint7','fixed');
            joint8 = rigidBodyJoint('joint8','fixed');
            joint9 = rigidBodyJoint('joint9','fixed');
            joint10 = rigidBodyJoint('joint10','fixed');
            % joint axis and joining bodies
            joint1.JointAxis = [1,0,0];
            segment1.Joint = joint1;
            segment2.Joint = joint2;
            segment3.Joint = joint3;
            segment4.Joint = joint4;
            segment5.Joint = joint5;
            segment6.Joint = joint6;
            segment7.Joint = joint7;
            segment8.Joint = joint8;
            segment9.Joint = joint9;
            segment10.Joint = joint10;
            
            %set transforms to the joints
            setFixedTransform(joint1,[0, 0, 0, pi/2],"dh")
            setFixedTransform(joint2,[0.04,0,0,0],"dh")
            setFixedTransform(joint3,[0.056,0,0,0],"dh")
            setFixedTransform(joint4,[0.116,0,0,0],"dh")
            setFixedTransform(joint1,[0.182, 0, 0, 0],"dh")
            setFixedTransform(joint2,[0.232,0,0,0],"dh")
            setFixedTransform(joint3,[0.248,0,0,0],"dh")
            setFixedTransform(joint4,[0.358,0,0,0],"dh")
            setFixedTransform(joint3,[0.374,0,0,0],"dh")
            setFixedTransform(joint4,[0.424,0,0,0],"dh")
            
            %connect bodies
            
            addBody(main_shaft,segment1,basename)
            addBody(main_shaft,segment2,'segment1')
            addBody(main_shaft,segment3,'segment2')
            addBody(main_shaft,segment4,'segment3')
            addBody(main_shaft,segment5,'segment4')
            addBody(main_shaft,segment6,'segment5')
            addBody(main_shaft,segment7,'segment6')
            addBody(main_shaft,segment8,'segment7')
            addBody(main_shaft,segment9,'segment8')
            addBody(main_shaft,segment10,'segment9')
            
            
            geom = {};
            parameters = [0.036, 0.04,0.02, 0, 0;...
                            0.09,0.016,0.048, 0, 0;...
                            0.12,0.06,0.086, 0, 0;...
                            0.09,0.066,0.149, 0, 0;...
                            0.106,0.05,0.207, 0, 0;...
                            0.1, 0.016,0.24 , 0, 0;...
                            0.12, 0.11,0.303, 0, 0;...
                            0.1, 0.016,0.366, 0, 0;...
                            0.13, 0.05,0.399, 0, 0;...
                            0.036, 0.236,0.542, 0, 0];
            % collision geometries
            for i = 1:1:10
                gm = collisionCylinder(parameters(i,1),parameters(i,2));
                tform = trvec2tform([parameters(i,3),parameters(i,4),parameters(i,5)])*axang2tform([0 1 0 pi/2]);
                gm.Pose = tform;
                addCollision(main_shaft.Bodies{i},gm)      
                geom{i} = gm;
            end
        end
        function counter_shaft = create_countershaft(obj)
            counter_shaft = {};
            parameters = [0.036, 0.06,-0.16+0.03, 0, -0.231;...
                            0.14,0.05,-0.16+0.085,0, -0.231;...
                            0.036,0.182,-0.16+0.201, 0, -0.231;...
                            0.1395,0.05,-0.16+0.316, 0, -0.231;...
                            0.1235,0.052,-0.16+0.367, 0, -0.231;...
                            0.036, 0.016,-0.16+0.4 , 0, -0.231;...
                            0.08, 0.05,-0.16+0.433, 0, -0.231;...
                            0.036, 0.076,-0.16+0.496, 0, -0.231;...
                            0.1, 0.05,-0.16+0.559, 0, -0.231;...
                            0.036, 0.076,-0.16+0.622, 0, -0.231];
            % collision geometries
            for i = 1:1:10
                gm = collisionCylinder(parameters(i,1),parameters(i,2));
                tform = trvec2tform([parameters(i,3),parameters(i,4),parameters(i,5)])*axang2tform([0 1 0 pi/2]);
                gm.Pose = tform;
                counter_shaft{i} = gm;
            end

        end
        function transmission_case = create_case(obj)
            transmission_case = {};            
            parameters = [0.192, 0.114, 0.025, -0.16+0.66+0.0125, 0, -0.231-0.153;...
                   0.117, 0.114, 0.025, -0.16+0.66+0.0125, 0, -0.1155;...
                   0.113, 0.114, 0.025, -0.16+0.66+0.0125, 0, 0.1135;
                   0.65, 0.153, 0.025, -0.16+0.66+0.0125, 0.1335, -0.155;...
                   0.65, 0.153, 0.025, -0.16+0.66+0.0125, -0.1335, -0.155;...
                   0.192, 0.114, 0.025, -0.16+0.0125, 0, -0.231-0.153;...
                   0.117, 0.114, 0.025, -0.16+0.0125, 0, -0.1155;...
                   0.113, 0.114, 0.025, -0.16+0.0125, 0, 0.1135;
                   0.65, 0.153, 0.025, -0.16+0.0125, 0.1335, -0.155;...
                   0.65, 0.153, 0.025, -0.16+0.0125, -0.1335, -0.155;...
                   0.65, 0.025, 0.66, 0.17, -0.1975, -0.155];%...
%                    0.65, 0.025, 0.66, 0.17, 0.1975, -0.155]; commented
%                    out the side plate
            % collision geometries
            for i = 1:1:11
                gm = collisionBox(parameters(i,1),parameters(i,2),parameters(i,3));
                tform = trvec2tform([parameters(i,4),parameters(i,5),parameters(i,6)])*axang2tform([0 1 0 pi/2]);
                gm.Pose = tform;
                transmission_case{i} = gm;
            end
        end
        function draw(obj,collision_geom, model_color,num_bodies)
            for i = 1:1:num_bodies
                [~, patch_object] = show(collision_geom{i});
                patch_object.FaceColor = model_color;
                patch_object.EdgeColor = 'none';
            end
        end
        function new_location = move(obj,q_new)
            old_tree = obj.create_mainshaft();
            new_location = rigidBodyTree("DataFormat", "column");
            new_segment1 = rigidBody('new_segment1');
            tform = trvec2tform([q_new(1)-0.66,q_new(2),q_new(3)+0.66*sin(q_new(5))])*axang2tform([1 0 0 q_new(4)])*axang2tform([0 1 0 q_new(5)])*axang2tform([0 0 1 q_new(6)]);
            new_segment1.Joint.setFixedTransform(tform)
            new_location.addBody(new_segment1, 'base')
            old_location =  removeBody(old_tree,'segment1');
            new_location.addSubtree('new_segment1',old_location)
        end
        function collision_status = collision_check(obj,mainshaft_tree)
            collision_status = 0;
            countershaft_tree = obj.create_countershaft();
            case_tree = obj.create_case();
            mainshaft_config = homeConfiguration(mainshaft_tree);
            collision_withcountershaft = checkCollision(mainshaft_tree,mainshaft_config,countershaft_tree);
            collision_withcase = checkCollision(mainshaft_tree,mainshaft_config,case_tree);
            if collision_withcase(2) == 1 || collision_withcountershaft(2) == 1
                collision_status = 1;
            end
            
        end 

    end
end