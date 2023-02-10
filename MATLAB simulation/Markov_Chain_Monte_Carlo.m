%% Metropoli-Hastings法で解く
%Binomial distribution
clear all

n = 10;
p = 1/3;
cycles = 10000;

X(1) = 0;
for k = 1:cycles
    %1 Assume X(k) = i
    i = X(k);
    %2 Choose j at random
    j = randsample(n+1, 1) - 1;
    
    %3 Draw a realization of uniform random variable U on (0, 1)
    U = rand; %uniform random variable of U on (0, 1)
    
    a = factorial(i)*factorial(n-i)/factorial(j)/factorial(n-j)*(p/(1-p))^(j-i);
    
    %4, 5 
    if U <= a
        X(k+1) = j;
    elseif U > a
        X(k+1) = i;
    end
    
    j_data(k) = j;
    U_data(k) = U;
    a_data(k) = a;
end

X = X(2:end);

%0から10までの頻度
for k = 0:n
    frequency(k+1) = sum(X == k);
end

frequency = 1/cycles * frequency;

%% 解析的に求める
for i = 0:n
    distribution(i+1) = nchoosek(n, i) * (p/(1-p))^i; 
end

distribution = 1/sum(distribution)*distribution;