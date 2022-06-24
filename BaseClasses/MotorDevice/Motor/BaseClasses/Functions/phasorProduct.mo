within MultiZoneOfficeSimpleAirMotor.BaseClasses.MotorDevice.Motor.BaseClasses.Functions;
function phasorProduct "Product function for a phasor"
  input Real x1[1,2];
  input Real x2[1,2];
  output Real y[1,2];

algorithm
  y[1,1] := x1[1,1]*x2[1,1];
  y[1,2] := x1[1,2]+x2[1,2];

end phasorProduct;
