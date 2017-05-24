goalkeeper=[1,3,pi/1.3];
attacker=[10 17];

[ xl, xr, zl,zr,~ ] = cover( goalkeeper,attacker );

fig=figure(1);clf;

% draw the penality space
drawLine([-(16.5+7.32/2) 0],[(16.5+7.32/2) 0],'black');hold all;
drawLine([-(16.5+7.32/2) 0],[-(16.5+7.32/2) 16.5],'black');hold all;
drawLine([-(16.5+7.32/2) 16.5],[(16.5+7.32/2) 16.5],'black');hold all;
drawLine([(16.5+7.32/2) 16.5],[(16.5+7.32/2) 0],'black');hold all;

% draw the borders of the goal
drawLine([-(7.32/2),-1],[-(7.32/2),1],'black');
drawLine([(7.32/2),-1],[(7.32/2),1],'black');

% draw Line between attacker en goalkeeper
drawLine(goalkeeper(1:2),attacker,'red');hold all;

% draw the arms of the keeper
d=1;
delta_x=cos(goalkeeper(3))*d;
delta_y=sin(goalkeeper(3))*d;
drawLine(goalkeeper(1:2),[goalkeeper(1)+delta_x goalkeeper(2)+delta_y],'blue');hold all;
drawLine(goalkeeper(1:2),[goalkeeper(1)-delta_x goalkeeper(2)-delta_y],'blue');hold all;

% draw area covered by goalkeeper
drawLine(attacker,[zl 0],'green');hold all;
drawLine(attacker,[zr 0],'green');hold all;

saveas(fig,'report/img/costFunction/simpleDemo.png');

%% 2D plot
x_coordinates=linspace(-20,20,100);
xls=zeros(size(x_coordinates));
xrs=zeros(size(x_coordinates));
J=zeros(size(x_coordinates));

attacker=[10 17];
for i=1:length(x_coordinates)
    goalkeeper=[x_coordinates(i),10,0];
    [ xls(i), xrs(i), ~ , ~ ] = cover( goalkeeper,attacker );
    J(i) = costFunction(goalkeeper,attacker);
end

fig=figure(2);clf;
subplot(2,1,1);
plot(x_coordinates,xls);hold on;
plot(x_coordinates,xrs);hold on;
legend('xl','xr');
xlabel('x');
subplot(2,1,2);
% plot(x_coordinates,log(xls.^2+xrs.^2));
plot(x_coordinates,log10(J));
ylabel('J');
xlabel('x');
saveas(fig,'report/img/costFunction/simpleDemoJ.png');

%% 3D plot
% x_coordinates=linspace(-50,50,100);
% y_coordinates=linspace(-10,50,100);
theta=0;
steps=100;
[x_coordinates, y_coordinates] = meshgrid(linspace(-30,30,steps),linspace(-10,50,steps));
J=(size(x_coordinates));

attacker=[10 17];
for i_x=1:size(x_coordinates,1)
    for i_y=1:size(x_coordinates,2)
        
        goalkeeper=[x_coordinates(i_x,i_y),y_coordinates(i_x,i_y),theta];
        J(i_x,i_y) = costFunction(goalkeeper,attacker);
        
    end
end

fig=figure(3);clf;
surf(x_coordinates,y_coordinates,log10(J));
xlabel('x');
ylabel('y');
zlabel('J');
saveas(fig,'report/img/costFunction/simpleDemoJSurf.png');