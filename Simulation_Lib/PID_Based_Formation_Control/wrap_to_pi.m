function a = wrap_to_pi(a)
    a = mod(a + pi, 2*pi) - pi;
end
