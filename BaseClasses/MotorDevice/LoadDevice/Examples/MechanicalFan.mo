within MultiZoneOfficeSimpleAirMotor.BaseClasses.MotorDevice.LoadDevice.Examples;
model MechanicalFan
  "Example that demonstrate the use of the mechanical fan"
  extends Modelica.Icons.Example;
  package Medium = Buildings.Media.Air;

  .MultiZoneOfficeSimpleAirMotor.BaseClasses.MotorDevice.LoadDevice.MechanicalFan
    fanWithInputShaft(redeclare package Medium = Medium, redeclare
      Buildings.Fluid.Movers.Data.Pumps.Wilo.Stratos25slash1to4 per(
        use_powerCharacteristic=false)) "fan with input shaft"
    annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
  Buildings.Fluid.Sources.Boundary_pT bouSou2(nPorts=1, redeclare package
      Medium = Medium) "Boundary condition for the source 2"
    annotation (Placement(transformation(extent={{-80,-10},{-60,10}})));
  Buildings.Fluid.Sources.Boundary_pT bouSin2(nPorts=1, redeclare package
      Medium = Medium) "Boundary condition for the sink 2"
    annotation (Placement(transformation(extent={{80,-10},{60,10}})));
  Modelica.Mechanics.Rotational.Components.Inertia loaInt(J=JLoad)
    "Shaft inertia"
    annotation (Placement(transformation(extent={{-40,40},{-20,60}})));
  Modelica.Mechanics.Rotational.Sources.ConstantTorque torSou(tau_constant=
        tauMot) "Boundary condition for the torque"
    annotation (Placement(transformation(extent={{-68,40},{-48,60}})));
  parameter Modelica.SIunits.Torque tauMot = 0.05
    "Constant torque (if negative, torque is acting as load in positive direction of rotation)";
  parameter Modelica.SIunits.Inertia JLoad = 0.01 "Moment of inertia";
  Modelica.Blocks.Sources.RealExpression rotSpe(y=
        Modelica.SIunits.Conversions.to_rpm(loaInt.w)) "Rotation speed"
    annotation (Placement(transformation(extent={{-60,-50},{-40,-30}})));
  Buildings.Fluid.FixedResistances.PressureDrop res2(
    redeclare package Medium = Medium,
    m_flow_nominal=1.2,
    dp_nominal=2000) "Fixed flow resistance 2"
    annotation (Placement(transformation(extent={{-50,-10},{-30,10}})));
  Buildings.Fluid.Sources.Boundary_pT bouSou1(nPorts=1, redeclare package
      Medium = Medium) "Boundary condition for the source 1"
    annotation (Placement(transformation(extent={{-80,-90},{-60,-70}})));
  Buildings.Fluid.FixedResistances.PressureDrop res1(
    redeclare package Medium = Medium,
    m_flow_nominal=1.2,
    dp_nominal=2000) "Fixed flow resistance 1"
    annotation (Placement(transformation(extent={{-48,-90},{-28,-70}})));
  Buildings.Fluid.Movers.SpeedControlled_Nrpm fan(redeclare package Medium =
        Medium, redeclare
      Buildings.Fluid.Movers.Data.Pumps.Wilo.Stratos25slash1to4 per(
        use_powerCharacteristic=false)) "fan"
    annotation (Placement(transformation(extent={{-10,-90},{10,-70}})));
  Buildings.Fluid.Sources.Boundary_pT bouSin1(nPorts=1, redeclare package
      Medium = Medium) "Boundary condition for the sink 1"
    annotation (Placement(transformation(extent={{82,-90},{62,-70}})));
equation
  connect(fanWithInputShaft.port_b, bouSin2.ports[1])
    annotation (Line(points={{10,0},{60,0}}, color={0,127,255}));
  connect(loaInt.flange_b, fanWithInputShaft.shaft)
    annotation (Line(points={{-20,50},{0,50},{0,10}}, color={0,0,0}));
  connect(torSou.flange, loaInt.flange_a)
    annotation (Line(points={{-48,50},{-40,50}}, color={0,0,0}));
  connect(bouSou2.ports[1], res2.port_a)
    annotation (Line(points={{-60,0},{-50,0}}, color={0,127,255}));
  connect(res2.port_b, fanWithInputShaft.port_a)
    annotation (Line(points={{-30,0},{-10,0}}, color={0,127,255}));
  connect(fan.port_b, bouSin1.ports[1])
    annotation (Line(points={{10,-80},{62,-80}}, color={0,127,255}));
  connect(fan.port_a, res1.port_b)
    annotation (Line(points={{-10,-80},{-28,-80}}, color={0,127,255}));
  connect(res1.port_a, bouSou1.ports[1])
    annotation (Line(points={{-48,-80},{-60,-80}}, color={0,127,255}));
  connect(rotSpe.y, fan.Nrpm)
    annotation (Line(points={{-39,-40},{0,-40},{0,-68}}, color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)),
    __Dymola_Commands(file=
          "modelica://MultiZoneOfficeSimpleAirMotor/Resources/Scripts/Dymola/BaseClasses/MotorDevice/LoadDevice/Examples/MechanicalPump.mos"
        "Simulate and Plot"),
    experiment(
      StopTime=100000,
      Tolerance=1e-06,
      __Dymola_Algorithm="Cvode"),
    Documentation(info="<html>
<p>This example demonstrates and tests the use of a flow machine whose inputs are the input rotational flange of a shaft. </p>
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
end MechanicalFan;
