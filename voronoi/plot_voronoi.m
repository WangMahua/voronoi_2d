classdef plot_voronoi 
    properties
        ImageFolder = '/home/ncrl/ws/matlab/voronoi_2d/result/'
        cmap = zeros(10,3)                    % Red for 10
    end

    methods(Static)
        function obj = plot_voronoi()
%             obj.cmap = [0, 0, 1; ...            % region cover by agent1 
%               0.3010, 0.7450, 0.9330; ...   % region cover by agent2
%               0, 0.75, 0.75; ...            % region cover by agent3
%               1, 1, 0; ...                  % target
%               0.5, 0.5, 0; ...              % agent1
%               0.5, 0.5, 0.5; ...            % agent2
%               0, 0, 0; ...                  % agent3
%               0, 0, 0; ...                  % Black for 8
%               0, 0, 0; ...                  % Black for 9
%               1, 0, 0];                      % Red for 10
%             obj.cmap = [0, 0, 1; ...            % region cover by agent1 
%               0.3010, 0.7450, 0.9330; ...   % region cover by agent2
%               0, 0.75, 0.75]; ...            % region cover by agent3
            obj.cmap = [0.8500, 0.3250, 0.0980; ...            % region cover by agent1 
              0.9290, 0.6940, 0.1250; ...   % region cover by agent2
              1, 1, 0]; ...            % region cover by agent3              
        end
    end
    methods 
        function obj = plot_map(obj,vor,filename)
            new_map = vor.map_partition;
            
            figure;
            image(new_map);
            colormap(obj.cmap);
            axis on;
            hold on;

            % target 
            scatter(vor.target(1)/vor.pixel_size,vor.target(2)/vor.pixel_size,'square','black');
            hold on ;

            % agent 
            scatter(vor.swarm.pos(1,1)/vor.pixel_size,vor.swarm.pos(1,2)/vor.pixel_size,'filled','MarkerEdgeColor',[0 .5 .5],'MarkerFaceColor',[0.8500 0.3250 0.0980]);
%             legend('agent 1');
            hold on ;

            scatter(vor.swarm.pos(2,1)/vor.pixel_size,vor.swarm.pos(2,2)/vor.pixel_size,'filled','MarkerEdgeColor',[0 .5 .5],'MarkerFaceColor',[0.9290 0.6940 0.1250]);
            hold on ;

            scatter(vor.swarm.pos(3,1)/vor.pixel_size,vor.swarm.pos(3,2)/vor.pixel_size,'filled','MarkerEdgeColor',[0 .5 .5],'MarkerFaceColor',[1 1 0]);
            hold on ;

            legend('target','agent 1','agent 2','agent 3','Location', 'EastOutside');

            colorbar
            saveas(figure(1),append(obj.ImageFolder,filename));
            hold off;
      
        end

        function obj = plot_map_with_dense(obj,vor,filename)            
            new_map = vor.map_partition;
            
            figure;
            image(new_map);
            colormap(obj.cmap);
            axis on;
            hold on;

            % target 
            scatter(vor.target(1)/vor.pixel_size,50-vor.target(2)/vor.pixel_size,'square','black');
            hold on ;


            % agent 
            scatter(vor.swarm.pos(1,1)/vor.pixel_size,vor.swarm.pos(1,2)/vor.pixel_size,'filled','MarkerFaceColor',[0.8500 0.3250 0.0980]);
            hold on ;

            scatter(vor.swarm.pos(2,1)/vor.pixel_size,vor.swarm.pos(2,2)/vor.pixel_size,'filled','MarkerFaceColor',[0.9290 0.6940 0.1250]);
            hold on ;

            scatter(vor.swarm.pos(3,1)/vor.pixel_size,vor.swarm.pos(3,2)/vor.pixel_size,'filled','yellow');
            hold on ;
           


            % cv 
            scatter(vor.cv(1,1)/vor.pixel_size,vor.cv(1,2)/vor.pixel_size,'d','MarkerEdgeColor',[0 .5 .5],'MarkerFaceColor',[0.8500 0.3250 0.0980]);
            hold on;
           

            scatter(vor.cv(2,1)/vor.pixel_size,vor.cv(2,2)/vor.pixel_size,'d','MarkerEdgeColor',[0 .5 .5],'MarkerFaceColor',[0.9290 0.6940 0.1250]);
            hold on;
         

            scatter(vor.cv(3,1)/vor.pixel_size,vor.cv(3,2)/vor.pixel_size,'d','MarkerEdgeColor',[0 .5 .5],'MarkerFaceColor',[1 1 0]);
            hold on;
        
            legend('target','agent 1','agent 2','agent 3','cv 1','cv 2','cv 3','Location', 'EastOutside');

            colorbar
            name = append(obj.ImageFolder,'partition_');
            saveas(figure(1),append(name,filename));
            hold off;
%             figure(2);
%             image(vor.map_density);
% 
%             name = append(obj.ImageFolder,'density_');
%             saveas(figure(2),append(name,filename));

        end
    end
end

