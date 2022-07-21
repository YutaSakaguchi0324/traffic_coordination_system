function Fml = FuelConsumptionGF(Vel, Acc, dt)
global VTcsv;

Fml=0;
        %For  low velocity range
        b0=0.1569;
        b1=2.450E-2;
        b2=-7.415E-4;
        b3=5.975E-5;

        c0=0.07224;
        c1=9.681E-2;
        c2=1.075E-3;

        Fcruise=b0+Vel*b1+Vel.^2*b2+Vel.^3*b3;
        Facc=Acc.*(c0+Vel*c1+Vel.^2*c2).*(Acc>0);
        Fml=(Fcruise+Facc)*dt;

%% Aimsun Formula
%{
vm=13.9;
k1=0.222999;
k2=0.0033529;
k3=0.000042;
c1=0.42;
c2=0.26;
Fi=0.333;
Fd=0.537;

dFuel=k1*(1+(Vel/vm)^3+k2*Vel); %% Cruising Fuel
if(Vel<1.0 && Acc<0.1)dFuel=Fi; end
if(Vel>1.0 && Acc>0.05)dFuel=(c1+c2*Acc*Vel); end
if(Vel>5.0 && Acc<-0.3)dFuel=Fd; end

%Fml=dFuel*dt;

%I=VTcsv(end,1)+1;
%if(Car.V2V>-1) VTcsv(end+1,:)=[I,Vel*3.6,Acc*3.6];end
%}
end
