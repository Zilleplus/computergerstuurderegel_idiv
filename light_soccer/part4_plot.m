function part4_plot(x_ref,y,input_data,sim_data,name )
    fig=figure;clf;
    plot(sim_data.Data(:,1),sim_data.Data(:,2),'O'); hold all;
    plot(x_ref(1,:),x_ref(2,:),'.');hold all;
    plot(y(1,:),y(2,:),'black');
    title(name);
    legend('simulation with controller','reference trajectory','theoretical trajectory');
    xlabel('x');ylabel('y');  
    saveas(fig,['./report/img/LQR/',name,'_traject.png']);
    
    fig=figure;clf;
    subplot(2,1,1);
    plot(input_data.Time,input_data.Data(:,1));hold all;
    plot(input_data.Time,input_data.Data(:,2));
    legend('x','y');
    subplot(2,1,2);
    plot(input_data.Time,input_data.Data(:,3));
    xlabel('t(s)');
    legend('\theta');
   saveas(fig,['./report/img/LQR/',name,'_input.png']);
end