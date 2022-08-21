close all; clear all;

%% Define system parameters
% Dynamics driven by general energy equation L = T - V
% L = Total system energy; T = kinetic energy; V = potential energy
% States: [x; d_dot; theta; theta_dot]

L = 0.3; % pole length (m)
m = .2; % mass of the pole (kg)
M = .5; % mass of cart (kg)
g = -9.81; % grav (mps2)

Ap = [0 1 0 0;
     0 0 (-m*g/M) 0;
     0 0 0 1;
     0 0 (-((m+M)*g)/(M*L)) 0];

Bp = [0; (1/M); 0; (1/(M*L))];

Cp = eye(4);

Dp = 0*Cp*Bp;

[nx,~] = size(Ap);
%Check Controlability
if nx == rank(ctrb(Ap,Bp))
    disp('Controllable')
else
    disp('Not Controllable')
end

%Check Observability
if nx == rank(obsv(Ap,Cp))
    disp('Observable')
else
    disp('Not Observable')
end

% Create wiggle system
% Lecture 11 - Optimal Control - Slide 36

C = [1 0 0 0; 0 0 1 0];
D = 0.*C*Bp;
zcp = zeros(size(C,1));
% zcp(1,2)= 1; % I would have thought this to be zero since constant command
zap = zeros(size(Ap,1),2);
Aw = [zcp C;
      zap Ap];
Bw = [D; Bp];
F = [-1 ; zeros(size(Aw,1)-1,1)]; % [-1; 0; ...; 0]
Q = eye(size(Aw,1));
Q(1,1) = 10;
Q(2,2) = 100;
Q(3,3) = 10;
R = 1;

[Kx_lqr,~,~]=lqr(Aw,Bw,Q,R);
K_X = Kx_lqr(:,3:6)
K_I = Kx_lqr(:,1:2)

x0 = [0; 0; .174; 0];
%{
%% Define initial point locations
% Point 1
x1 = 1;
y1 = 2;
% Point 2
x2 = 3;
y2 = 4;
% Point 3
x3 = 7;
y3 = 7;
% Line 1
xs1 = [x1 x2];
ys1 = [y1 y2];
% Line 2
xs2 = [x2 x3];
ys2 = [y2 y3];

%% Define movement pattern
figure
plot(x1,y1,'b.')
s1 = plot(x1, y1,'r*','LineWidth',2,'XDataSource','x1','YDataSource','y1');
hold on
s2 = plot(x2, y2,'b*','LineWidth',2,'XDataSource','x2','YDataSource','y2');
s3 = plot(x3, y3,'k*','LineWidth',2,'XDataSource','x3','YDataSource','y3');
line1 = plot(xs1, ys1,'XDataSource','xs1','YDataSource','ys1');
line2 = plot(xs2, ys2,'XDataSource','xs2','YDataSource','ys2');
axis([0 10 0 10])

for x1 = 1:8
    % Arbitrary movement
    x2 = x2- .5;
    x3 = x3 + .2;
    y3 = y3 -.5 ;
    y2 = y2 +.5;
    
    % Refresh lines
    xs1 = [x1 x2];
    ys1 = [y1 y2];
    xs2 = [x2 x3];
    ys2 = [y2 y3];
    
    % Refresh data
    refreshdata(s1,'caller')
    refreshdata(s2,'caller')
    refreshdata(s3,'caller')
    refreshdata(line1,'caller')
    refreshdata(line2,'caller')
    drawnow
    pause(.1)
end
%}