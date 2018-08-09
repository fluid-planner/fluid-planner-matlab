function p = prob(z, mu, Sigma)

nz = length(z);

p = 1/sqrt((2*pi)^nz * det(Sigma)) * exp( -0.5 * (z - mu)* inv(Sigma) * ( z - mu)' );


end