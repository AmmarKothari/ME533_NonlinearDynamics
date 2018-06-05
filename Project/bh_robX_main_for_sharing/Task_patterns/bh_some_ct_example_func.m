function f = bh_some_ct_example_func(y, z, t)

f = (3*(cos(z(t))*diff(z(t), t) + (cos(z(t) + y(t))*(diff(z(t), t) + diff(y(t), t)))/2)^2)/2 - 15*sin(z(t) + y(t)) - 55*sin(z(t)) + diff(z(t), t)^2/2 + (5*cos(z(t))^2*diff(z(t), t)^2)/8 + (3*(sin(z(t))*diff(z(t), t) + (sin(z(t) + y(t))*(diff(z(t), t) + diff(y(t), t)))/2)^2)/2 + (5*sin(z(t))^2*diff(z(t), t)^2)/8 + (diff(z(t), t) + diff(y(t), t))^2/2;

end

