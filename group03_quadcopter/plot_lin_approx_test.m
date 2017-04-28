% close all
fig=figure(1);clf;
subplot(1,2,1);
plot(lin_model_stepresponse);
legend('x','y','z','phi','theta','gamma');
subplot(1,2,2);
plot(non_lin_model_stepresponse);

saveas(fig,'./report/img/lin_approx/step_all.eps');