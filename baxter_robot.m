classdef baxter_robot
    methods (Static)
        function [links,baxter_left,baxter_right] = create_robot()
            links = [
                    Revolute('d', 0.27,        'a', 0.069, 'alpha', -pi/2)
                    Revolute('d', 0,           'a', 0, 'alpha', pi/2, 'offset', pi/2)
                    Revolute('d', 0.102+0.262, 'a', 0.069, 'alpha', -pi/2)
                    Revolute('d', 0,           'a', 0, 'alpha', pi/2)
                    Revolute('d', 0.103+0.271, 'a', 0.010, 'alpha', -pi/2)
                    Revolute('d', 0,           'a', 0, 'alpha', pi/2)
                    Revolute('d', 0.28,        'a', 0, 'alpha', 0)
            ]; 
                    baxter_left =  SerialLink(links, 'name', 'Baxter LEFT');
                    baxter_right = SerialLink(links, 'name', 'Baxter RIGHT');
        end
        function set_base(baxter_left,baxter_right)
            baxter_left.base = transl(0.064614, 0.25858, 0.119)*rpy2tr(0, 0, pi/4, 'xyz');
            baxter_right.base = transl(0.063534, -0.25966, 0.119)*rpy2tr(0, 0, -pi/4, 'xyz');           
        end
        function teach(left,right)
            figure(1)
            left.teach()
            figure(2)
            right.teach()
        end
        function invkin(baxter_left,baxter_right,end_pose,init_pose)
            init_left = cell2mat(init_pose(1));
            init_right = cell2mat(init_pose(2));
            end_left = cell2mat(end_pose(1));
            end_right = cell2mat(end_pose(2));
            Trans_left = transl(end_left);
            Trans_right = transl(end_right);
            TT_l = SE3.convert(Trans_left);
            TT_r = SE3.convert(Trans_right);
            for i=1:length(TT_l)
                T_l = TT_l(i);
                T_r = TT_r(i);
                q_inv_left = ikine(baxter_left,T_l,'q0',init_left,'rlimit',1000,'ilimit',10000);
                q_inv_right = ikine(baxter_right,T_r,'q0',init_right,'rlimit',1000,'ilimit',10000);
                qtg_l = jtraj(init_left,q_inv_left,5);
                qtg_r = jtraj(init_right,q_inv_right,5);
                [~,n] = size(qtg_l');
                for j=1:n
                    baxter_left.plot(qtg_l(j,:));
                    hold on
                    baxter_right.plot(qtg_r(j,:));
                end
                init_left = q_inv_left;
                init_right = q_inv_right;
            %     left.teach(q_inv)
            end
    
        end
    end
end








%%
% 
% for i=1:length(TT)
%     T = TT(i);
%     e = tr2delta(left.fkine(q), T);
%     J = jacobe(left, q);
%     JtJ = J'*W*J;
%     dq = inv(JtJ + (lambda + lambdamin) * eye(size(JtJ)) ) * J' * W * e;
%     qnew = q + dq';
%     q = qnew;
%     
% end

%,'rlimit',10000,'ilimit',10000,'transpose',1)

% time = 0:0.15:3;
% for i=1:length(time)
%     TT(i)
% % e = tr2delta(left.fkine(q),TT(i))
% % J = jacobe(robot, q);
% % qnew = q + dq';
% % enew = tr2delta(robot.fkine(qnew), T);
% % if enew<0.1
% %     break;
% % else 
% %     continue;
% % end
% 
% end
% qnew



% pos = Trans(1:3,4);
% eul_angles = tr2eul(Trans);
% ve = [pos;eul_angles'];
% J0 = left.jacob0(qr);
% J_dagger = pinv(J0);
% qf = J_dagger*ve;
% qf = J0'*ve;
% left.plot()
% qz(1) = -pi/2;
% qqr = left.ikine(T,'ru');
% qrt = jtraj(qz, qqr, 50);
% 
% plot_sphere(p, 0.05, 'y');
% left.plot3d(qrt, 'view', ae, 'movie', 'move2ball.gif');
