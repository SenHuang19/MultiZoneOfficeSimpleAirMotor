within MultiZoneOfficeSimpleAirMotor.BaseClasses.MotorDevice.Motor.BaseClasses.Examples;
model TorqueSpeedCharacteristics
  "Test torque speed relationship model for motor"
  import Modelica.Constants.pi;
  extends Modelica.Icons.Example;
  parameter Integer pole=4 "Number of pole pairs";
  parameter Modelica.SIunits.Frequency f = 60 "Fequency";

  MotorDevice.Motor.BaseClasses.TorqueSpeedCharacteristics torSpe(pole=pole)
    "Torque speed relationship"
    annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
  Modelica.Blocks.Sources.Constant Vrms(k=120) "voltage"
    annotation (Placement(transformation(extent={{-80,20},{-60,40}})));
  Modelica.Blocks.Sources.Ramp omega_r(
    duration=4*pi*f/pole,
    startTime=120,
    height=-4*pi*f/pole + 0.01,
    offset=4*pi*f/pole) "Rotational speed of Rotor"
    annotation (Placement(transformation(extent={{-80,-40},{-60,-20}})));

  Modelica.Blocks.Sources.Constant fSou(k=f)
    "Frequency"
    annotation (Placement(transformation(extent={{-80,-10},{-60,10}})));
  Modelica.Blocks.Sources.RealExpression omega_s(y=4*3.14*fSou.y/pole)
    "Rotational speed of the stator"
    annotation (Placement(transformation(extent={{-60,54},{-40,74}})));
  Modelica.Blocks.Sources.RealExpression s(y=(omega_s.y-omega_r.y)/omega_s.y)
    "Slip" annotation (Placement(transformation(extent={{-20,54},{0,74}})));
  Modelica.Blocks.Sources.RealExpression torque(y=torSpe.n*(Vrms.y*torSpe.X_m/(
        torSpe.X_m + torSpe.X_s))^2*torSpe.R_r/s.y/(omega_s.y*((torSpe.R_s*(torSpe.X_m/(
        torSpe.X_m + torSpe.X_s))^2+torSpe.R_r/s.y)^2+(torSpe.X_r + torSpe.X_s)^2)))
    "Slip" annotation (Placement(transformation(extent={{20,54},{40,74}})));
equation
  connect(Vrms.y, torSpe.V_rms) annotation (Line(points={{-59,30},{-24,30},{-24,
          4},{-12,4}}, color={0,0,127}));
  connect(omega_r.y, torSpe.omega_r) annotation (Line(points={{-59,-30},{-24,-30},
          {-24,-4},{-12,-4}}, color={0,0,127}));
  connect(fSou.y, torSpe.f)
    annotation (Line(points={{-59,0},{-12,0}}, color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)),
    experiment(StopTime=480),
    __Dymola_Commands(file=
          "modelica://MultiZoneOfficeSimpleAirMotor/Resources/Scripts/Dymola/BaseClasses/MotorDevice/Motor/BaseClasses/Examples/TorqueSpeedCharacteristics.mos"
        "Simulate and Plot"),
    Documentation(revisions="<html>
<ul>
<li>
June 21, 2022, by Sen Huang:<br/>
Upgrade the model with Building Library Vesison 8.0
</li>
<li>
December 17, 2019, by Yangyang Fu:<br/>
First Implementation
</li>
</ul>
</html>", info="<html>
<p>This example validates the implementation of a motor model which calculates the electromagnetic torque of a induction motor.</p>
</html>"));
end TorqueSpeedCharacteristics;
