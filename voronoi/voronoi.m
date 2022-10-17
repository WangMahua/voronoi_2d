classdef voronoi 
    %UAV with camera
    
    properties
        start_x = 0; % x-coordinate where map start  
        start_y = 0; % y-coordinate where map start  
        pixel_size = 0.1; % map pixel size 
        map_size = 50; 
        map 
        map_partition % group of every pixel 
        map_density
        swarm % UAV informatiom 
        target = [3.0 3.0 0.0] 
        last_cv = zeros(3,3);
        cv = zeros(3,3);
        mass = zeros(3,1);
        inertialx = zeros(3,1);
        inertialy = zeros(3,1);
        dt = 0.1

        % test matrix
        dense_for_all = zeros(3,50,50);


    end
    
    methods (Static)
        % initialize
        function obj = voronoi(a,b,map_size,robot)
             obj.start_x= a;
             obj.start_y= b;
             obj.map = zeros(map_size,map_size);
             obj.map_partition = zeros(50,50);
             obj.map_density = zeros(50,50);
             obj.swarm = robot;
        end
    end

    methods

        % init dense function 
        function obj = init_density(obj)
            for i = 1: length(obj.map_partition)
                for j = 1: length(obj.map_partition)
                    % test code 
                    if j >34
                        obj.map_partition(i,j)=3;
                    elseif j>16
                        obj.map_partition(i,j)=2;
                    else
                        obj.map_partition(i,j)=1;
                    end
                end
            end
        end  

        % update uav pos 
        function obj = update_uav_pos(obj,new_pos)
            obj.swarm.pos = new_pos;
        end 

        % update uav pos 
        function obj = update_target_pos(obj,new_pos)
            obj.target = new_pos;
        end 

        % Since target moved, the density of every particle would change.
        function obj = reset_density(obj) 
            for i = 1 : obj.map_size 
                for j = 1 : obj.map_size
                    
                    ux = obj.target(1);
                    uy = obj.target(2);

                    sigma = 0.7;

                    qx = obj.start_x + obj.pixel_size*i ;
                    qy = obj.start_y + obj.pixel_size*j ;
                    partition_num = obj.map_partition(i,j);

                    k = 1;
                    alpha =1;

                    px = obj.swarm.pos(partition_num,1);
                    py = obj.swarm.pos(partition_num,2); 

                    dist_square = (px-qx) * (px-qx) + (py-qy) * (py-qy);
                    gamma =(1+k*dist_square)/(1+alpha*dist_square);
                    obj.map_density(i,j)= exp(-sigma*((qx-ux)*(qx-ux)+(qy-uy)*(qy-uy)));
                    obj.map_density(i,j)= obj.map_density(i,j);
                end
            end
        end 

        % update partition 
        function obj = update_partition(obj,f)
            for i = 1 : obj.map_size 

                for j = 1 : obj.map_size
                    min_dense = -1;

                    for k = 1 : length(f)
                        
                        q_pos = [obj.start_x+obj.pixel_size*i  obj.start_y+obj.pixel_size*j];
                        p_pos = [obj.swarm.pos(k,1) obj.swarm.pos(k,2)];
%                         p_pos = [obj.cv(k,1) obj.cv(k,2)];
                        now_dense = f{k}(q_pos,p_pos);
%                         obj.dense_for_all(k,i,j)= now_dense;


                        if min_dense < 0 || now_dense < min_dense 
                            obj.map_partition(j,i) = k; 
                            min_dense = now_dense;  
                            
                        end
                    end
                end
            end
        end 

        % find cv 
        function obj = find_cv(obj)       
            obj.mass = zeros(3,1);
            obj.inertialx = zeros(3,1);
            obj.inertialy = zeros(3,1);
            for i = 1 : obj.map_size 
                for j = 1 : obj.map_size 
                    partition_num = obj.map_partition(i,j);
                    dq = obj.pixel_size;
                    x = obj.start_x + i*obj.pixel_size;
                    y = obj.start_y + j*obj.pixel_size;
                    obj.mass(partition_num) = obj.mass(partition_num) + obj.map_density(i,j)*dq*dq;
                    obj.inertialx(partition_num) = obj.inertialx(partition_num) + x*obj.map_density(i,j)*dq*dq;
                    obj.inertialy(partition_num) = obj.inertialy(partition_num) + y*obj.map_density(i,j)*dq*dq;
                end  
            end

            obj.last_cv = obj.cv;
            obj.cv(:,1)= obj.inertialx./obj.mass;
            obj.cv(:,2)= obj.inertialy./obj.mass;
           
        end 

        % find neighbor 
        function obj = find_neighbor(obj)

            
        end

        % UAV move
        function obj = upadte(obj)
            desire_vel = (obj.cv - obj.swarm.pos)/obj.dt;
%             original_vel = (obj.cv - obj.last_cv)/obj.dt;
%             obj.swarm.vel = obj.swarm.last_vel + desire_vel;
            obj.swarm.vel = desire_vel;
            if norm(obj.swarm.vel)>10
                obj.swarm.vel = 10*obj.swarm.vel/(norm(obj.swarm.vel));
            end

            obj.swarm.pos = obj.swarm.pos + obj.swarm.vel*obj.dt;
            obj.swarm.last_pos = obj.swarm.pos;
            obj.swarm.last_vel = obj.swarm.vel;

        end 

        % UAV move
        function obj = new_upadte(obj)
           for i =1:obj.swarm.agent_number
                cvt_x = -1*(obj.swarm.pos(i,1)-obj.cv(i,1));
                cvt_y = -1*(obj.swarm.pos(i,2)-obj.cv(i,2));
                diff_cx =(obj.cv(i,1) - obj.last_cv(i,1))/obj.dt;
                diff_cy =(obj.cv(i,2) - obj.last_cv(i,2))/obj.dt;
                obj.swarm.vel(i,1) = cvt_x +diff_cx;
                obj.swarm.vel(i,2) = cvt_y +diff_cy;
                obj.swarm.last_pos(i,2) = obj.swarm.pos(i,1) ;
                obj.swarm.last_pos(i,2) = obj.swarm.pos(i,2) ;
                obj.swarm.pos(i,1) =obj.swarm.pos(i,1) + ( obj.swarm.vel(i,1) *0.5* obj.dt);
                obj.swarm.pos(i,2) =obj.swarm.pos(i,2) + ( obj.swarm.vel(i,2) *0.5* obj.dt);
           end

        end 

        % plot
        function obj = plot_map(obj,filename)
            fig = image(obj.map_partition());
            colormap(jet(10));
            saveas(fig,filename);
            
        end 
    end

end


