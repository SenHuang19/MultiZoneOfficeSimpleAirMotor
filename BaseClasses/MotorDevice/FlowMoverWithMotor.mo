within MultiZoneOfficeSimpleAirMotor.BaseClasses.MotorDevice;
model FlowMoverWithMotor
  extends Buildings.Fluid.Interfaces.PartialTwoPort(
  port_a(p(start=Medium.p_default)),
  port_b(p(start=Medium.p_default)));

  parameter Modelica.SIunits.Frequency f_base=60 "Frequency of the source";
  parameter Integer pole = 4 "Number of pole pairs";
  parameter Integer n = 3 "Number of phases";
  parameter Modelica.SIunits.Inertia JMotor(min=0) = 2 "Moment of inertia";
  parameter Modelica.SIunits.Resistance R_s=0.013 "Electric resistance of stator";
  parameter Modelica.SIunits.Resistance R_r=0.009 "Electric resistance of rotor";
  parameter Modelica.SIunits.Reactance X_s=0.14 "Complex component of the impedance of stator";
  parameter Modelica.SIunits.Reactance X_r=0.12 "Complex component of the impedance of rotor";
  parameter Modelica.SIunits.Reactance X_m=2.4 "Complex component of the magnetizing reactance";
  parameter Modelica.SIunits.Inertia JLoad=2 "Moment of inertia";
  parameter Boolean addPowerToMedium=true
    "Set to false to avoid any power (=heat and flow work) being added to medium (may give simpler equations)";
  Modelica.Blocks.Interfaces.RealInput V "Voltage signal" annotation (Placement(
        transformation(
        extent={{20,-20},{-20,20}},
        rotation=90,
        origin={4,120})));
  Modelica.Blocks.Interfaces.RealInput f(final quantity="Frequency", final unit=
       "Hz") "Freuqency signal" annotation (Placement(transformation(
        extent={{-20,-20},{20,20}},
        rotation=-90,
        origin={-88,120})));
  Modelica.Blocks.Interfaces.RealOutput P(quantity="Power",unit="W")
    "Real power"
    annotation (Placement(transformation(extent={{100,70},{120,90}})));
  Modelica.Blocks.Interfaces.RealOutput Q "Reactive power"
    annotation (Placement(transformation(extent={{100,50},{120,70}})));
  MultiZoneOfficeSimpleAirMotor.BaseClasses.MotorDevice.Motor.SimpleMotor simMot(
    pole=pole,
    n=n,
    R_s=R_s,
    R_r=R_r,
    X_s=X_s,
    X_r=X_r,
    X_m=X_m,
    J=JMotor)
    annotation (Placement(transformation(extent={{-66,26},{-46,46}})));
  Modelica.Blocks.Sources.RealExpression loaTorExp(y=mover.shaft.tau)
    annotation (Placement(transformation(extent={{-40,-44},{-60,-24}})));
  Modelica.Mechanics.Rotational.Components.Inertia loaInt(J=JLoad)
    "Additional inertia"
    annotation (Placement(transformation(extent={{0,26},{20,46}})));
  Modelica.Mechanics.Rotational.Sources.Speed spe(f_crit=f_base)
    "Converting the angular velocity to a movement"
    annotation (Placement(transformation(extent={{-32,26},{-12,46}})));
  MultiZoneOfficeSimpleAirMotor.BaseClasses.MotorDevice.Load.MechanicalMover
    mover(
    redeclare package Medium = Medium,
    addPowerToMedium=addPowerToMedium,
    per=per) "Mechanical flow mover with a shaft port"
    annotation (Placement(transformation(extent={{40,-10},{60,10}})));

  replaceable parameter Buildings.Fluid.Movers.Data.Generic per
    constrainedby Buildings.Fluid.Movers.Data.Generic
    "Record with performance data"
    annotation (choicesAllMatching=true,
      Placement(transformation(extent={{40,-62},{60,-42}})));

equation

  connect(simMot.V, V) annotation (Line(points={{-68,40},{-78,40},{-78,90},{4,
          90},{4,120}}, color={0,0,127}));

  connect(loaTorExp.y, simMot.tauM) annotation (Line(points={{-61,-34},{-80,-34},
          {-80,32},{-68,32}}, color={0,0,127}));
  connect(simMot.omegaRat, spe.w_ref)
    annotation (Line(points={{-45,36},{-34,36}}, color={0,0,127}));
  connect(spe.flange, loaInt.flange_a)
    annotation (Line(points={{-12,36},{0,36}}, color={0,0,0}));
  connect(simMot.f, f)
    annotation (Line(points={{-68,36},{-88,36},{-88,120}}, color={0,0,127}));
  connect(loaInt.flange_b, mover.shaft)
    annotation (Line(points={{20,36},{50,36},{50,10}}, color={0,0,0}));
  connect(port_a, mover.port_a)
    annotation (Line(points={{-100,0},{40,0}}, color={0,127,255}));
  connect(mover.port_b, port_b)
    annotation (Line(points={{60,0},{100,0}}, color={0,127,255}));
  connect(simMot.P, P) annotation (Line(points={{-45,44},{-40,44},{-40,80},{110,
          80}}, color={0,0,127}));
  connect(simMot.Q, Q) annotation (Line(points={{-45,40},{-38,40},{-38,60},{110,
          60}}, color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
          Rectangle(
          extent={{-100,100},{100,-100}},
          lineColor={0,0,127},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid)}),                      Diagram(
        coordinateSystem(preserveAspectRatio=false)),
    Documentation(info="<html>
<p>
This model utilizes <a href=\"modelica://MultiZoneOfficeSimpleAirMotor.BaseClasses.MotorDevice.FlowMoverWithMotor\">MultiZoneOfficeSimpleAirMotor.BaseClasses.MotorDevice.FlowMoverWithMotor</a> to calulcates the real power and the reactive power of a motor.
</p>
<p>
It also replies on <a href=\"modelica://MultiZoneOfficeSimpleAirMotor.BaseClasses.MotorDevice.Load.MechanicalMover\">MultiZoneOfficeSimpleAirMotor.BaseClasses.MotorDevice.Load.MechanicalMover</a> to couple the real power and the reactive power with shaft power calculated from the fluid system.
</p>
<p>
Defaulted parameters are from the reference listed as follows.
</p>
<h4>
Reference
</h4>
<p>
<uo>
<li>
Price, W. W., C. W. Taylor, and G. J. Rogers. \"Standard load models for power flow and dynamic performance simulation.\" IEEE Transactions on power systems 10, no. CONF-940702- (1995).
</li>
</uo>
</p>
</html>"));
end FlowMoverWithMotor;
