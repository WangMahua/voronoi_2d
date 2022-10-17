classdef agent
 % 3 UAV property (camera/bearing/range sensor)  

    properties

        agent_number = 3;

        % state of 3 UAV
        pos = zeros(3,3);
        last_pos = zeros(3,3);
        vel = zeros(3,3);
        last_vel = zeros(3,3);

        % param of camera 
        fx = 565.6;
        fy = 565.6;

        % param of range 
        cov_r_x = 0.1;
        cov_r_y = 0.1;
        cov_r_z = 0.1;

        % param of bearing 
        cov_b_alpha = 0.1;
        cov_b_beta = 0.1;

    end
    
   methods (Static)
        % obseverability of 3 UAV, which are also dense function in voronoi
        
        % camera
        function dense = dense_function_1(q_pos,u_pos)

            % uav pos 
            px = u_pos(1);  py = u_pos(2);
            % particle pos      
            qx = q_pos(1);  qy = q_pos(2);

            dense = (((px-qx)*(px-qx) + (py-qy)*(py-qy)));
        end

        % bearing
        function dense = dense_function_2()
            dense = 1/cov_r_x^2 + 1/cov_r_y^2 + 1/cov_r_z^2;
        end

        % range sensor    
        function dense = dense_function_3(q_p,p_p)
            
            % p -> uav pos ; q -> particle pos  
            r = q_p - p_p;
       

            dense = 1/(cov_b_beta*(r(1)^2 + r(2)^2)) + 1/(cov_b_alpha*(r(1)^2 + r(2)^2 + r(3)^2)) + ...
                 1/(cov_b_beta*(r(1)^2 + r(2)^2)) + 1/(cov_b_alpha*(r(1)^2 + r(2)^2 + r(3)^2)) +...
                 1/(cov_b_beta*(r(1)^2 + r(2)^2)) + 1/(cov_b_alpha*(r(1)^2 + r(2)^2 + r(3)^2)) ;
        end
   end

end