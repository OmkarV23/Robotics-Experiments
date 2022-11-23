clear all;
close all

links = [
        Revolute('d', 0.27,        'a', 0.069, 'alpha', -pi/2)
        Revolute('d', 0,           'a', 0, 'alpha', pi/2, 'offset', pi/2)
        Revolute('d', 0.102+0.262, 'a', 0.069, 'alpha', -pi/2)
        Revolute('d', 0,           'a', 0, 'alpha', pi/2)
        Revolute('d', 0.103+0.271, 'a', 0.010, 'alpha', -pi/2)
        Revolute('d', 0,           'a', 0, 'alpha', pi/2)
        Revolute('d', 0.28,        'a', 0, 'alpha', 0)
];

left =  SerialLink(links, 'name', 'Baxter LEFT');
right = SerialLink(links, 'name', 'Baxter RIGHT');

left.base = transl(0.064614, 0.25858, 0.119)*rpy2tr(0, 0, pi/4, 'xyz');
right.base = transl(0.063534, -0.25966, 0.119)*rpy2tr(0, 0, -pi/4, 'xyz');

%%
% qz = [0 0 0 0 0 0 0]; % zero angles, L shaped pose
% qr = [0 -pi/2 -pi/2 0 0 0 0]; % ready pose, arm up
% qs = [0 0 -pi/2 0 0 0];
% qn = [0 pi/4 pi/2 0 pi/4 0];
% 
% q = qz;
%%
% p = [0.476,0.408,1.168;0.005,0.271,1.227];
p_left = [0.476,0.408,1.168;-0.479,0.727,0.489];
p_right = [0.476,-0.408,1.168;-0.479,-0.727,0.489];
% p_right = [0.746,-0.920,0.139;0.803,-0.254,0.361];
% p=[1.152,0.256,0.310];
Trans_left = transl(p_left); %* troty(pi);
Trans_right = transl(p_right);
TT_l = SE3.convert(Trans_left);
TT_r = SE3.convert(Trans_right);
q0_l = zeros(1, 7);
q0_r = zeros(1, 7);
for i=1:length(TT_l)
    T_l = TT_l(i);
    T_r = TT_r(i);
    q_inv_left = ikine(left,T_l,'q0',q0_l);%,'rlimit',1000,'ilimit',10000);
    q_inv_right = ikine(right,T_r,'q0',q0_r);%,'rlimit',1000,'ilimit',10000);
    qtg_l = jtraj(q0_l,q_inv_left,5);
    qtg_r = jtraj(q0_r,q_inv_right,5);
    [m,n] = size(qtg_l');
    for j=1:n
        left.plot(qtg_l(j,:));
        hold on
        right.plot(qtg_r(j,:));
    end
    q0_l = q_inv_left;
    q0_r = q_inv_right;
%     left.teach(q_inv)
end
% right.teach()








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
