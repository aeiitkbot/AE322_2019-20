function val = ang_wrap(heading)

while heading <= pi
    heading = heading + 2*pi;
end

while heading > pi
    heading = heading - 2*pi;
end

val = heading;