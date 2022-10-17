clc;clear;close all;

%% add relative path
addpath(genpath('./agent'))
addpath(genpath('./voronoi'))

%% param declaration
robot = agent;

f1 = @robot.dense_function_1;
f2 = @robot.dense_function_2;
f3 = @robot.dense_function_3;

S.df = {f1 f2 f3};
S.agent_number = 3;

vor = voronoi(0,0,50,robot);
pv = plot_voronoi();
n_p = [1.0 1.0 0.0;2.0 2.0 0.0;4.0 4.0 0.0];
vor = vor.update_uav_pos(n_p);
vor.swarm.pos

% values = [12.7, 45.4, 98.9, 26.6, 53.1]; 
% for i = 1 : length(S) 
%     c = S.df{i}(values);
% end

%% main function

execute_time = 100;
r = rateControl(1/(vor.dt));
vor = vor.update_partition(S.df);

% fig = image(vor.map_partition());
% colormap(jet(10));
% saveas(fig,'result_0.jpg');


for i = 1 : execute_time



    target_pos = [3+cos(i*10*pi/180) 3+sin(i*10*pi/180)];
    vor = vor.update_target_pos(target_pos);
    vor = vor.reset_density();

    vor = vor.update_partition(S.df);
    if mod(i,2)== 1
        close all;
        file_name = sprintf('result_%d.jpg',i);
        pv.plot_map(vor,file_name);
    end
    vor = vor.find_cv();



    vor = vor.upadte();
%     vor = vor.new_upadte();
    waitfor(r);
end
% close all;
%% plot voronoi

