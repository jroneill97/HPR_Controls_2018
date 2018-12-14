load wind;
clear windSpeedAvg;
for i = 1:10
    windSpeedAvg(i) = mean((i-1)*wind.velocity);
end
plot(windSpeedAvg,radius);
    title('Landing Radius vs. Average Wind Speed','interpreter','latex');
    xlabel('Average Wind Speed ($\frac{m}{s}$)','interpreter','latex');
    ylabel('Landing Radius (m)','interpreter','latex');
    grid on;