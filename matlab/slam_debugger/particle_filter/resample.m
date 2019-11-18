function particles = resample(particles)

M = length(particles);
Q = zeros(M);
for i = 1:M
    Q(i) = sum(Q(1:i)) + particles(i).w;
end
Q(M) = 1;

i=1;
indx = zeros(M);
while (i<=M)
    sampl = rand(1,1);  % (0,1]
    j=1;
    while (Q(j)<sampl)
        j=j+1;
    end
    indx(i)=j;
    i=i+1;
end

for i = 1:M
    particles(i).pose    = particles(indx(i)).pose;
    particles(i).w       = particles(indx(i)).w;
    particles(i).r_error = particles(indx(i)).r_error;    
end