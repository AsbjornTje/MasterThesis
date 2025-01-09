clear all;
clc;
import +ETS3.*




%             th    d       a         alpha    type(0/1 = R/P)
L(1) = Link([ 0     0       134.3        0        0]);
L(2) = Link([ 0     102.35  130       pi/2     0]);
L(3) = Link([ 0     102.55     289   -pi       0]); 
L(4) = Link([ 0    102.55   172.5   -pi/2     0]);
L(5) = Link([ 0     102.55    289    pi/2     0]);


J_5 = SerialLink(L, 'name', '5_joint');
J_5.fkine([0 0 0 0 0])

J_5.teach;


% 
% 
% p = [0.8 0 0];
% T = transl(p) * troty(pi/2);
% qr(1) = -pi/2;
% qqr = p560.ikine6s(T, 'ru');
% qrt = jtraj(qr, qqr, 50);
% 
% plot_sphere(p, 0.05, 'y');
% p560.plot3d(qrt, 'view', ae, 'movie', 'move2ball.gif');

