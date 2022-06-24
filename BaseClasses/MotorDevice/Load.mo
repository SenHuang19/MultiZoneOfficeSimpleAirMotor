within MultiZoneOfficeSimpleAirMotor.BaseClasses.MotorDevice;
package Load "Rotation machine such as fan, pump"

  model MechanicalMover "Fan with a rotational flange as input signal"
    extends Buildings.Fluid.Interfaces.PartialTwoPort(
    port_a(p(start=Medium.p_default)),
    port_b(p(start=Medium.p_default)));

    Modelica.SIunits.Angle phi "Shaft angle";
    Modelica.SIunits.AngularVelocity omega "Shaft angular velocity";
    Real Nrpm "Rational speed";

    Modelica.Mechanics.Rotational.Interfaces.Flange_b shaft
    annotation (Placement(transformation(extent={{-10,90},{10,110}})));
    Buildings.Fluid.Movers.SpeedControlled_Nrpm mover(
      redeclare package Medium = Medium,
      final inputType=Buildings.Fluid.Types.InputType.Continuous,
      final addPowerToMedium=addPowerToMedium,
      per=per,
      final use_inputFilter=false) "Fluid mover"
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
    mover.eff.PEle = shaft.tau*Buildings.Utilities.Math.Functions.smoothMax(
        omega,
        1e-6,
        1e-8);

    connect(rotSpe.y, mover.Nrpm)
      annotation (Line(points={{-19,46},{0,46},{0,12}}, color={0,0,127}));
    connect(port_a, mover.port_a)
      annotation (Line(points={{-100,0},{-10,0}}, color={0,127,255}));
    connect(mover.port_b, port_b)
      annotation (Line(points={{10,0},{100,0}}, color={0,127,255}));
    connect(mover.heatPort, heatPort) annotation (Line(points={{0,-6.8},{0,-70},
            {-60,-70},{-60,-100}}, color={191,0,0}));
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
  end MechanicalMover;

  package Examples "Example package"
    extends Modelica.Icons.ExamplesPackage;

    model MechanicalMover
      "Example that demonstrate the use of the mechanical flow mover"
      extends Modelica.Icons.Example;
      package Medium = Buildings.Media.Air;

      MultiZoneOfficeSimpleAirMotor.BaseClasses.MotorDevice.Load.MechanicalMover
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
              "modelica://MultiZoneOfficeSimpleAirMotor/Resources/Scripts/Dymola/BaseClasses/MotorDevice/Load/Examples/MechanicalMover.mos"
            "Simulate and Plot"),
        experiment(
          StopTime=100000,
          Tolerance=1e-06,
          __Dymola_Algorithm="Cvode"),
        Documentation(info="<html>
<p>This example demonstrates and tests the use of a flow machine whose inputs are the input rotational flange of a shaft. </p>
</html>",     revisions="<html>
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
    end MechanicalMover;
  end Examples;
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end Load;
