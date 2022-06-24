within MultiZoneOfficeSimpleAirMotor.BaseClasses.MotorDevice.LoadDevice;
model MechanicalFan
  "Fan with a rotational flange as input signal"
  extends Buildings.Fluid.Interfaces.PartialTwoPort(
  port_a(p(start=Medium.p_default)),
  port_b(p(start=Medium.p_default)));

  Modelica.SIunits.Angle phi "Shaft angle";
  Modelica.SIunits.AngularVelocity omega "Shaft angular velocity";
  Real Nrpm "Rational speed";

  Modelica.Mechanics.Rotational.Interfaces.Flange_b shaft
  annotation (Placement(transformation(extent={{-10,90},{10,110}})));
  Buildings.Fluid.Movers.SpeedControlled_Nrpm fan(
    redeclare package Medium = Medium,
    final inputType=Buildings.Fluid.Types.InputType.Continuous,
    final addPowerToMedium=addPowerToMedium,
    per=per,
    final use_inputFilter=false) "Fan device"
    annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
  Modelica.Blocks.Sources.RealExpression rotSpe(y=Nrpm) "Rotation speed"
    annotation (Placement(transformation(extent={{-40,36},{-20,56}})));

  parameter Boolean addPowerToMedium=true
    "Set to false to avoid any power (=heat and flow work) being added to medium (may give simpler equations)";

  replaceable parameter Buildings.Fluid.Movers.Data.Generic per
    constrainedby Buildings.Fluid.Movers.Data.Generic
    "Record with performance data"
    annotation (choicesAllMatching=true,
      Placement(transformation(extent={{52,60},{72,80}})));
  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a heatPort
    "Heat dissipation to environment"
    annotation (Placement(transformation(extent={{-70,-110},{-50,-90}}),
        iconTransformation(extent={{-10,-78},{10,-58}})));
equation
  phi = shaft.phi;
  omega = der(phi);
  Nrpm = Modelica.SIunits.Conversions.to_rpm(omega);
  fan.eff.PEle = shaft.tau*Buildings.Utilities.Math.Functions.smoothMax(omega,1e-6,1e-8);

  connect(rotSpe.y,fan. Nrpm)
    annotation (Line(points={{-19,46},{0,46},{0,12}}, color={0,0,127}));
  connect(port_a,fan. port_a)
    annotation (Line(points={{-100,0},{-10,0}}, color={0,127,255}));
  connect(fan.port_b, port_b)
    annotation (Line(points={{10,0},{100,0}}, color={0,127,255}));
  connect(fan.heatPort, heatPort) annotation (Line(points={{0,-6.8},{0,-70},{
          -60,-70},{-60,-100}}, color={191,0,0}));
  annotation (defaultComponentName="pump",
    Icon(coordinateSystem(preserveAspectRatio=true,  extent={{-100,-100},{100,
            100}}), graphics={
        Rectangle(
          extent={{-100,16},{100,-14}},
          lineColor={0,0,0},
          fillColor={0,127,255},
          fillPattern=FillPattern.HorizontalCylinder),
            Text(
              extent={{26,136},{124,114}},
          textString="Nrpm [rpm]",
          lineColor={0,0,127}),
        Rectangle(
          visible=use_inputFilter,
          extent={{-34,40},{32,100}},
          lineColor={0,0,0},
          fillColor={135,135,135},
          fillPattern=FillPattern.Solid),
        Ellipse(
          visible=use_inputFilter,
          extent={{-34,100},{32,40}},
          lineColor={0,0,0},
          fillColor={135,135,135},
          fillPattern=FillPattern.Solid),
        Text(
          visible=use_inputFilter,
          extent={{-22,92},{20,46}},
          lineColor={0,0,0},
          fillColor={135,135,135},
          fillPattern=FillPattern.Solid,
          textString="M",
          textStyle={TextStyle.Bold}),
        Ellipse(
          extent={{-58,50},{54,-58}},
          lineColor={0,0,0},
          fillPattern=FillPattern.Sphere,
          fillColor={0,100,199}),
        Polygon(
          points={{0,50},{0,-56},{54,2},{0,50}},
          lineColor={0,0,0},
          pattern=LinePattern.None,
          fillPattern=FillPattern.HorizontalCylinder,
          fillColor={255,255,255}),
        Ellipse(
          extent={{4,14},{34,-16}},
          lineColor={0,0,0},
          fillPattern=FillPattern.Sphere,
          visible=energyDynamics <> Modelica.Fluid.Types.Dynamics.SteadyState,
          fillColor={0,100,199})}),
    Documentation(info="<html>
<p><br>This model is extended from <a href=\"modelica://Buildings.Fluid.Movers.SpeedControlled_Nrpm\">Buildings.Fluid.Movers.SpeedControlled_Nrpm</a>.</p>
<p>We include the following equations to associate the fan power with the input rotational flange of a shaft:</p>
<p>&tau; = P/&omega;,</p>
<p>where <i>&omega;</i> is the shaft angular velocity;</p>
<p> <i>&tau;</i> is theshaft torque.</p>
<p>The speed of the fan is calculated by: </p>
<p>s = &omega;*30/&pi;.</p>
</html>",
      revisions="<html>
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
