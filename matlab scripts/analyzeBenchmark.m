close all;
clear all;

%% Parameters

% rel_path = 'benchmarks/2d random sampling/';
% rel_path = 'benchmarks/2d same sampling/';
% exp = 18;
% tries = 10;

rel_path = 'benchmarks/alpha test 2/';
exp = 4;
tries = 10;

fmt_file = [rel_path 'fmt_data.txt'];
fmth_file = [rel_path 'fmth_data.txt'];

fmt = load(fmt_file);
fmth = load(fmth_file);

%% Analyzing unsuccessful plans
fmt_fail = zeros(exp,2);
fmth_fail = zeros(exp,2);
for i = 1 : exp
   idx = (i-1)*tries+1;
   IDX = i*tries;
   fmt_fails = find(fmt(idx:IDX,2)==-1);
   fmth_fails = find(fmth(idx:IDX,2)==-1);
   fmt_fail(i,:) = [fmt(idx,1) size(fmt_fails,1)];
   fmth_fail(i,:) = [fmth(idx,1) size(fmth_fails,1)];
end

fmt_fail(:,2) = fmt_fail(:,2)./tries;
fmth_fail(:,2) = fmth_fail(:,2)./tries;

%% Getting means and std
fmt_mean = zeros(exp,3);
fmth_mean = zeros(exp,3);
fmt_std = zeros(exp,3);
fmth_std = zeros(exp,3);
for i = 1 : exp
   idx = (i-1)*tries+1;
   IDX = i*tries;
   fmt_mean(i,:) = mean(fmt(idx:IDX, :));
   fmth_mean(i,:) = mean(fmth(idx:IDX, :));
   fmt_std(i,:) = std(fmt(idx:IDX, :));
   fmth_std(i,:) = std(fmth(idx:IDX, :));
end

%% Plotting
figure(1)
hold on;
plot(fmt(:,1),fmt(:,3), 'rx');
plot(fmth(:,1),fmth(:,3), 'bs');
legend('FMT', 'FMTH');
title('Samples vs time');
xlabel('Samples');
ylabel('time (s)');
errorbar(fmt_mean(:,1), fmt_mean(:,3), fmt_std(:,3), 'r-')
errorbar(fmth_mean(:,1), fmth_mean(:,3), fmth_std(:,3), 'b-')

figure(2)
hold on;
plot(fmt(:,1),fmt(:,2), 'rx');
plot(fmth(:,1),fmth(:,2), 'bs');
legend('FMT', 'FMTH');
title('Samples vs cost');
xlabel('Samples');
ylabel('Cost');
errorbar(fmt_mean(:,1), fmt_mean(:,2), fmt_std(:,2), 'r-')
errorbar(fmth_mean(:,1), fmth_mean(:,2), fmth_std(:,2), 'b-')

figure(3)
hold on;
plot(fmt(:,3),fmt(:,2), 'rx');
plot(fmth(:,3),fmth(:,2), 'bs');
legend('FMT', 'FMTH');
title('Time vs cost');
xlabel('Time (s)');
ylabel('Cost');
errorbar(fmt_mean(:,3), fmt_mean(:,2), fmt_std(:,2), 'r-')
errorbar(fmth_mean(:,3), fmth_mean(:,2), fmth_std(:,2), 'b-')

figure(4)
hold on;
plot(fmt_fail(:,1), fmt_fail(:,2), 'r-')
plot(fmth_fail(:,1), fmth_fail(:,2), 'b-')
legend('FMT', 'FMTH');
title('Failure in Path computation');
xlabel('Samples');
ylabel('Failure %');