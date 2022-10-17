classdef agent
 % 3 UAV property (camera/bearing/range sensor)  

    properties
        % state of 3 UAV
        pos = zeros(3,3);
        last_pos = zeros(3,3);
        vel = zeros(3,3);
        last_vel = zeros(3,3);
        alpha = [1.8 1.9 1.3];
        k = [1.1 1 0.5];

        agent_number = 3;
    end
    
   methods (Static)
        % obseverability of 3 UAV, which are also dense function in voronoi
        
        % camera
        function dense = dense_function_1(q_pos,u_pos)

            alpha = 1.8;
            k = 1.1;
            % uav pos 
            px = u_pos(1);  py = u_pos(2);
            % particle pos      
            qx = q_pos(1);  qy = q_pos(2);

            dense = (((px-qx)*(px-qx) + (py-qy)*(py-qy)));
        end

        % bearing
        function dense = dense_function_2(q_pos,u_pos)
            
            alpha = 1.9;
            k = 1;
            % uav pos 
            px = u_pos(1);  py = u_pos(2);
            % particle pos      
            qx = q_pos(1);  qy = q_pos(2);

            dense = (((px-qx)*(px-qx) + (py-qy)*(py-qy)));
        end

        % range sensor    
        function dense = dense_function_3(q_pos,u_pos)
            
            alpha = 1.3;
            k = 0.5;
            % uav pos 
            px = u_pos(1);  py = u_pos(2);
            % particle pos      
            qx = q_pos(1);  qy = q_pos(2);

            dense =(((px-qx)*(px-qx) + (py-qy)*(py-qy)));
        end
   end

end