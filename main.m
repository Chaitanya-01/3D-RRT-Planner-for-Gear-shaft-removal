clc; close all;clear;
%----plotting RRT tree/graph-------------------------------
figure
model = model_create([0,0,0,0,0,0]);
shaft = model.create_mainshaft();
show(shaft, 'Collisions',"on",'Visuals',"off");
hold on;
casing = model.create_case();
secondshaft = model.create_countershaft();
model.draw(secondshaft,[0.5,0.5,0.5],10)
model.draw(casing,[0.8,0.4,0.1],11)
light("Style","infinite","Position",[50 10 -1]);
xlim([-0.5 0.8])
ylim([-0.9 0.9])
zlim([-0.4 0.8])
view([1.579989510725831e+02,3.300687319312016])
 
q_start = [0.66 0 0 0 0 0];
q_goal = [0.5 0 0.4 0 0 0];
planner = RRT(q_start,q_goal,1500);
% To run unidirectional RRT uncomment below line
finalpath = planner.run_start()
% To run bidirectional RRT uncomment below line
% finalpath = planner.run_bidirectional()
hold off;

%---------------------------------------------------------------------
% Drawing the final path
figure
goal_shaft = model.move(q_goal);
show(goal_shaft, 'Collisions',"on",'Visuals',"off");
light("Style","infinite","Position",[50 10 -1]);
xlim([-0.5 0.8])
ylim([-0.9 0.9])
zlim([-0.4 0.8])
view([1.579989510725831e+02,3.300687319312016])
hold on;
casing = model.create_case();
secondshaft = model.create_countershaft();
model.draw(secondshaft,[0.5,0.5,0.5],10)
model.draw(casing,[0.8,0.4,0.1],11)
for i = 1:1:length(finalpath)-1
    line([finalpath(i+1).q(1),finalpath(i).q(1)],[finalpath(i+1).q(2),finalpath(i).q(2)],[finalpath(i+1).q(3),finalpath(i).q(3)],'Color','red','LineWidth',2)
end
hold off;

%-------------------------------------------------------------------------
% creating a gif video
figure
gif('sideview.gif');
casing = model.create_case();
secondshaft = model.create_countershaft();
for i = 1:length(finalpath)
    pos = model.move(finalpath(i).q);
    show(pos, 'Collisions',"on",'Visuals',"off");
    light("Style","infinite","Position",[50 10 -1]);
    xlim([-0.5 0.8])
    ylim([-0.9 0.9])
    zlim([-0.4 0.8])
    view([1.579989510725831e+02,3.300687319312016])
    hold on;
    model.draw(secondshaft,[0.5,0.5,0.5],10)
    model.draw(casing,[0.8,0.4,0.1],11)
    gif
    hold off;
end
%---------Uncomment to plot the mainshaft at different positions along the final path----------------------------------------------------------
% load('unidirectionpath.mat', 'finalpath')
% length(finalpath)
% figure
% model = model_create([0,0,0,0,0,0]);
% shaft = model.create_mainshaft();
% goal_shaft = model.move(finalpath(35).q);
% show(goal_shaft, 'Collisions',"on",'Visuals',"off");
% % show(shaft, 'Collisions',"on",'Visuals',"off");
% hold on;
% casing = model.create_case();
% secondshaft = model.create_countershaft();
% model.draw(secondshaft,[0.5,0.5,0.5],10)
% model.draw(casing,[0.8,0.4,0.1],11)
% light("Style","infinite","Position",[50 10 -1]);
% xlim([-0.5 0.8])
% ylim([-0.9 0.9])
% zlim([-0.4 0.8])
% view([1.579989510725831e+02,3.300687319312016])
% for i = 1:1:length(finalpath)-1
%     line([finalpath(i+1).q(1),finalpath(i).q(1)],[finalpath(i+1).q(2),finalpath(i).q(2)],[finalpath(i+1).q(3),finalpath(i).q(3)],'Color','red','LineWidth',2)
% end
% hold off;

