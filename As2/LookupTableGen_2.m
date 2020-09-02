ADC = 0:1023;

Distance = nthroot((ADC./18109),-1.09);
Distance(Distance>150)=150;
DistanceRound = round(Distance,0);
plot(DistanceRound,ADC)

dlmwrite('lookupTable_2.txt',DistanceRound);