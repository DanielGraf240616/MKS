clear all; close all;
data = csvread("ntc.csv");



R5 = 10;
adc_reference = 3.3;
adc_range = 1024;
temperatures = data(:, 1);
resistance = data(:,2);

ad = (resistance./(resistance + R5)) .* adc_range;

figure()
plot(ad, temperatures, "o");
hold on

ad2 = 0:1023;
p = polyfit(ad, temperatures, 10);
ad_fit = linspace(0, 1032, 100);
temp_fit = polyval(p, ad_fit);

%plot(ad_fit, temp_fit);
%hold on


t2 = round(polyval(p,ad2),1);
plot(ad2,t2,"r");

dlmwrite('data.dlm', t2 * 10, ',');
data2 = dlmread("data.dlm");