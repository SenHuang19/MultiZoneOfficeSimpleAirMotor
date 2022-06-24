within MultiZoneOfficeSimpleAirMotor.BaseClasses.MotorDevice.Motor.Examples;
model SimpleMotor
  extends Modelica.Icons.Example;
  MotorDevice.Motor.SimpleMotor simMot(omegaRat(start=4*3.14*60/4, fixed=true))
    "Motor" annotation (Placement(transformation(extent={{-10,-6},{10,14}})));
  Modelica.Blocks.Sources.Step V(
    height=10,
    startTime=1200,
    offset=100) "Supply voltage"
    annotation (Placement(transformation(extent={{-80,40},{-60,60}})));
  Modelica.Blocks.Sources.Step Tor(
    startTime=2400,
    height=5,
    offset=21)    "Mechanical torque"
    annotation (Placement(transformation(extent={{-80,-40},{-60,-20}})));
  Modelica.Blocks.Sources.Step f(
    height=0,
    startTime=0,
    offset=60)   "Frequency"
    annotation (Placement(transformation(extent={{-80,-6},{-60,14}})));
equation
  connect(V.y, simMot.V) annotation (Line(points={{-59,50},{-36,50},{-36,8},{-12,
          8}}, color={0,0,127}));
  connect(Tor.y, simMot.tauM) annotation (Line(points={{-59,-30},{-34,-30},{-34,
          0},{-12,0}}, color={0,0,127}));
  connect(f.y, simMot.f)
    annotation (Line(points={{-59,4},{-12,4}}, color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)),
    experiment(StopTime=3600),
    __Dymola_Commands(file=
          "modelica://MultiZoneOfficeSimpleAirMotor/Resources/Scripts/Dymola/BaseClasses/MotorDevice/Motor/Examples/SimpleMotor.mos"
        "Simulate and Plot"),
    Documentation(info="<html>
<p>This example validates the implementation of an induction motor</p>
</html>", revisions="<html>
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
</html>"));
end SimpleMotor;
