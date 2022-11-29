function puma(final_p,n,traj)
    mdl_puma560
    
%     final_p = [0.8 0 0];
    
    T = transl(final_p) * troty(pi/2);
    qr(1) = -pi/2;
    qqr = p560.ikine6s(T, 'ru');
    x = 1:1:n;
    
    if traj==0

        [qrt,qdrt,qddrt] = jtraj(qr, qqr, n);  % path from qr' to T
        tmp = 0;
        
        figure(1)
        for i=1:6
            title(['position of joint',i]);
            subplot(6,3,tmp+1)
            plot(x,qrt(:,i)')
            title(['velocity of joint',i]);
            subplot(6,3,tmp+2)
            plot(x,qdrt(:,i)')
            title(['Accleration of joint',i]);
            subplot(6,3,tmp+3)
            plot(x,qddrt(:,i)')
            tmp = tmp+3;
        end
        hold on
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if traj==1
        qrt = [];
        qdrt = [];
        qddrt = [];
        tmp = 0;
        figure(1)
        for i=1:6
            [qrt_,qdrt_,qddrt_] = lspb(qr(1,i), qqr(1,i), n);  % path from qr' to T
            title(['position of joint:',i]);
            subplot(6,3,tmp+1)
            plot(x,qrt_)
            title(['velocity of joint:',i]);
            subplot(6,3,tmp+2)
            plot(x,qdrt_)
            title(['Accleration of joint:',i]);
            subplot(6,3,tmp+3)
            plot(x,qddrt_)
            tmp = tmp+3;
            qrt(:,i) = qrt_;
            qdrt(:,i) = qdrt_;
            qddrt(:,i) = qddrt_;
        end
        hold on
    end
    ae = [138 8];
    forward_pts = [];
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    for i=1:n
        forward_pts(i,:) = p560.fkine(qrt(i,:)).t;
    end
    figure(2)
    plot3(forward_pts(:,1),forward_pts(:,2),forward_pts(:,3))
    hold on
    plot_sphere(final_p, 0.05, 'y');%position of the ball
    p560.plot3d(qrt,'nowrist');  % animate moption to the ball  'view', ae,
end





% %% For pick and place
% 
% qr(1) = pi/2;
% 
% qrt = jtraj(qqr, qr, 50);  % path from T to qr'
% 
% clf
% 
% for i=1:length(qrt)
% 
%     T = p560.fkine(qrt(i,:)); % get EE pose
% 
%     Tball = T * SE3(0,0,0.2); %tool transfrom
% 
%     P = Tball.t;
% 
%     clf
% 
%     plot_sphere(P, 0.05, 'y');%plot moving sphere
% 
%     p560.plot3d(qrt(i,:), 'view', ae, 'nowrist');
% 
%     pause(0.1)
% 
% end
% 
% p560.plot3d(qrt(i,:), 'view', ae);