clear all
close all
clc

data = table2array(readtable('pomiary/prawe_przetarte_11_30_2022_11-39-38.csv'));

figure();
yyaxis left
plot(data(1:67, 1), data(1:67, 2));
hold on

xlabel("Napięcie [V]");
ylabel("Natężenie [A]");
ylim([0, 5])
yyaxis right
plot(data(1:67, 1), data(1:67, 3));
ylabel("Moc [W]");
ylim([0, 20])
title("Charakterystyka - prawe skrzydło")

data = table2array(readtable('pomiary/lewe_przetarte_11_30_2022_11-34-10.csv'));

figure();
yyaxis left
plot(data(1:65, 1), data(1:65, 2));
hold on
title("Charakterystyka - lewe skrzydło")
xlabel("Napięcie [V]");
ylabel("Natężenie [A]");
ylim([0, 5])
yyaxis right
plot(data(1:65, 1), data(1:65, 3));
ylabel("Moc [W]");
ylim([0, 20])