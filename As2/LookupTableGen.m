% invDist = 0.033 - 0.006;
% avOut = 2-0.4;
invDist = 0.033 - 0.0066;
avOut = 417 - 60;
grad = avOut/invDist;

inverseDistanceextended = 0.0066:(invDist/1023):0.033
vOutextended = grad .* inverseDistanceextended

plot(inverseDistanceextended,vOutextended)

distance = (inverseDistanceextended) .^ -1 - 30
plot(distance,vOutextended)

figure(2);
distanceRounded = round(distance,0)
plot(distanceRounded,vOutextended)

dlmwrite('lookupTable.txt',distanceRounded);