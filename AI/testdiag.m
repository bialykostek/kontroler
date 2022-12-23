inp = inputs(2:501, :)';
res = net(inp);
out = outputs(1:500, :)';

figure()
plot(res(2, :))
hold on
plot(out(2, :))
xlabel("Frame number");
ylabel("Value");
title("Neural network and pilot output comparation")
legend("Pilot output", "NN output")
ylim([0, 1])