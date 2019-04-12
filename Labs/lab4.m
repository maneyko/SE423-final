[b, a] = butter(7, 50/500, 'low');
tf(b, a, 1/1000);

% freqz(ones(1,8)*0.125)
freqz(b, a);

arraytoCformat(b);
arraytoCformat(a);