within MultiZoneOfficeSimpleAirMotor.BaseClasses.MotorDevice.Examples;
model FlowMoverWithMotor
  "Validate the pump model with a motor"
  extends Modelica.Icons.Example;
  package Medium = Buildings.Media.Water
  "Meidum of the water";
  parameter Modelica.SIunits.MassFlowRate m_flow_nominal = 0.06*1000
  "Design flow rate";
  parameter Modelica.SIunits.Pressure dp_nominal = 10000
  "Design head of the pump";
  MultiZoneOfficeSimpleAirMotor.BaseClasses.MotorDevice.FlowMoverWithMotor pum(redeclare
      package Medium = Medium, per=per,
    simMot(omegaRat(start=0)))
    "A pump with a motor"
    annotation (Placement(transformation(extent={{10,60},{30,80}})));
  Modelica.Blocks.Sources.Constant dpSet(k=6000)
    "Differential pressure setpoint"
    annotation (Placement(transformation(extent={{-120,30},{-100,50}})));
  Modelica.Blocks.Sources.Constant V(k=120)
    "Supply voltage"
    annotation (Placement(transformation(extent={{-100,80},{-80,100}})));
  Buildings.Fluid.FixedResistances.PressureDrop res(
    redeclare package Medium = Medium,
    dp_nominal=2000,
    m_flow_nominal=m_flow_nominal)
    "Resistance in the primary loop"
    annotation (Placement(transformation(extent={{-40,60},{-20,80}})));
  Buildings.Fluid.Sources.Boundary_pT expCol(redeclare package Medium = Medium,
     nPorts=1)
    "Expansion tank"
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=-90,
        origin={0,118})));
  Buildings.Fluid.HeatExchangers.ConstantEffectiveness hex(
    redeclare package Medium1 = Medium,
    redeclare package Medium2 = Medium,
    m1_flow_nominal=m_flow_nominal,
    m2_flow_nominal=m_flow_nominal,
    dp1_nominal=5000,
    dp2_nominal=5000)
    "A simple heat exchanger"
    annotation (Placement(transformation(extent={{-10,-24},{10,-44}})));
  Buildings.Fluid.Sensors.RelativePressure senRelPre(redeclare package Medium =
    Medium)
    "Sensor for the pressure difference"
    annotation (Placement(transformation(extent={{20,10},{0,-10}})));
  Buildings.Fluid.Sensors.TemperatureTwoPort senTemLev1(redeclare package
      Medium = Medium, m_flow_nominal=m_flow_nominal)
    "Sensor for the temperature of the water leaving the heat exchanger in the primary loop"
    annotation (Placement(transformation(extent={{-30,-38},{-50,-18}})));
  Buildings.Fluid.HeatExchangers.SensibleCooler_T coo(
    redeclare package Medium = Medium,
    m_flow_nominal=m_flow_nominal,
    dp_nominal=500) "An ideal cooler"
                    annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=90,
        origin={66,30})));
  Modelica.Blocks.Sources.Constant temEnt(k=273.15 + 7)
    "The temperature of the water entering the heat exchanger in the primary loop"
    annotation (Placement(transformation(extent={{100,40},{80,60}})));
  Modelica.Blocks.Sources.Step TSou(
    height=2,
    offset=273.15 + 18,
    startTime=1000)
    "Temperature of the water entering the heat exchanger in the secondary loop"
    annotation (Placement(transformation(extent={{-100,-74},{-80,-54}})));
  Buildings.Fluid.Sensors.TemperatureTwoPort senTemLea2(redeclare package
      Medium = Medium, m_flow_nominal=m_flow_nominal)
    "Sensor for the temperature of the water leaving the heat exchanger in the secondary loop"
    annotation (Placement(
        transformation(
        extent={{10,10},{-10,-10}},
        rotation=90,
        origin={60,-60})));
  Buildings.Fluid.Actuators.Valves.TwoWayEqualPercentage val(
    redeclare package Medium = Medium,
    m_flow_nominal=m_flow_nominal,
    dpValve_nominal=1000,
    use_inputFilter=false)
    "A valve to modulate the flow rate of the primary loop"
    annotation (Placement(transformation(extent={{40,-38},{20,-18}})));
  Buildings.Controls.Continuous.LimPID conPID(
    controllerType=Modelica.Blocks.Types.SimpleController.PI,
    Ti=60,
    reverseActing=false)
    "A PI controller to maintain the temperature setpoint"
    annotation (Placement(transformation(extent={{110,-40},{90,-20}})));
  Modelica.Blocks.Sources.Constant temSetLev2(k=273.15 + 16)
    "Temperature setpoint of the of the water leaving the heat exchanger in the secondary loop"
    annotation (Placement(transformation(extent={{160,-40},{140,-20}})));
  Buildings.Fluid.Sources.MassFlowSource_T souWat(
    redeclare package Medium = Medium,
    m_flow=m_flow_nominal,
    nPorts=1,
    use_T_in=true) "Souce of the water in the secondary loop"
                   annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=180,
        origin={-50,-60})));
  Buildings.Fluid.Sources.Boundary_pT sinWat(redeclare package Medium = Medium,
    nPorts=1) "Sink of the water in the secondary loop" annotation (Placement(
        transformation(
        extent={{10,-10},{-10,10}},
        rotation=0,
        origin={92,-80})));
  Buildings.Controls.Continuous.LimPID preCon(
    controllerType=Modelica.Blocks.Types.SimpleController.PI,
    Ti=60,
    yMax=60,
    yMin=20,
    reverseActing=true)
    "A PI controller for maintaining the pressure difference"
    annotation (Placement(transformation(extent={{-80,30},{-60,50}})));
  parameter Buildings.Fluid.Movers.Data.Generic per(
    pressure(V_flow=m_flow_nominal/1000*{0,0.41,0.54,0.66,0.77,0.89,1,1.12,1.19},
        dp=dp_nominal*{1.461,1.455,1.407,1.329,1.234,1.126,1.0,0.85,0.731}),
    motorEfficiency(V_flow={0}, eta={1}),
    speed_rpm_nominal=1800)
    "Record with performance data"
    annotation (choicesAllMatching=true,
      Placement(transformation(extent={{120,100},{140,120}})));
equation
  connect(pum.port_a, res.port_b)
    annotation (Line(points={{10,70},{-20,70}}, color={0,127,255}));
  connect(expCol.ports[1], pum.port_a) annotation (Line(points={{-1.77636e-15,108},
          {-1.77636e-15,70},{10,70}}, color={0,127,255}));
  connect(hex.port_b2, senTemLev1.port_a)
    annotation (Line(points={{-10,-28},{-30,-28}}, color={0,127,255}));
  connect(senTemLev1.port_b, res.port_a) annotation (Line(points={{-50,-28},{-52,
          -28},{-52,70},{-40,70}}, color={0,127,255}));
  connect(pum.port_b, coo.port_a)
    annotation (Line(points={{30,70},{66,70},{66,40}}, color={0,127,255}));
  connect(temEnt.y, coo.TSet)
    annotation (Line(points={{79,50},{58,50},{58,42}}, color={0,0,127}));
  connect(hex.port_b1, senTemLea2.port_a)
    annotation (Line(points={{10,-40},{60,-40},{60,-50}}, color={0,127,255}));
  connect(coo.port_b, val.port_a)
    annotation (Line(points={{66,20},{66,-28},{40,-28}}, color={0,127,255}));
  connect(val.port_b, hex.port_a2)
    annotation (Line(points={{20,-28},{10,-28}}, color={0,127,255}));
  connect(senTemLea2.T, conPID.u_m)
    annotation (Line(points={{71,-60},{100,-60},{100,-42}}, color={0,0,127}));
  connect(temSetLev2.y, conPID.u_s)
    annotation (Line(points={{139,-30},{112,-30}}, color={0,0,127}));
  connect(conPID.y, val.y) annotation (Line(points={{89,-30},{80,-30},{80,-8},{30,
          -8},{30,-16}}, color={0,0,127}));
  connect(senTemLea2.port_b, sinWat.ports[1])
    annotation (Line(points={{60,-70},{60,-80},{82,-80}}, color={0,127,255}));
  connect(souWat.ports[1], hex.port_a1) annotation (Line(points={{-40,-60},{-20,
          -60},{-20,-40},{-10,-40}}, color={0,127,255}));
  connect(TSou.y, souWat.T_in)
    annotation (Line(points={{-79,-64},{-62,-64}}, color={0,0,127}));
  connect(dpSet.y, preCon.u_s)
    annotation (Line(points={{-99,40},{-82,40}}, color={0,0,127}));
  connect(senRelPre.p_rel, preCon.u_m) annotation (Line(points={{10,9},{10,14},{
          -70,14},{-70,28}}, color={0,0,127}));
  connect(senRelPre.port_b, hex.port_b2) annotation (Line(points={{0,0},{-20,0},
          {-20,-28},{-10,-28}}, color={0,127,255}));
  connect(senRelPre.port_a, val.port_a) annotation (Line(points={{20,0},{50,0},{
          50,-28},{40,-28}}, color={0,127,255}));
  connect(V.y, pum.V) annotation (Line(points={{-79,90},{20,90},{20,82},{20.4,82}},
        color={0,0,127}));
  connect(preCon.y, pum.f) annotation (Line(points={{-59,40},{-6,40},{-6,88},{12,
          88},{12,82},{11.2,82}}, color={0,0,127}));
  annotation (Diagram(
        coordinateSystem(preserveAspectRatio=false, extent={{-140,-100},{180,140}})),
    experiment(StopTime=500, __Dymola_Algorithm="Dassl"),
    __Dymola_Commands(file=
          "modelica://MultiZoneOfficeSimpleAirMotor/Resources/Scripts/Dymola/BaseClasses/MotorDevice/Examples/FlowMoverWithMotor.mos"
        "Simulate and Plot"),
    Documentation(info="<html>
<p>This example demonstrates the usage of <a href=\"modelica://MultiZoneOfficeSimpleAirMotor.BaseClasses.MotorDevice.FlowMoverWithMotor\">MultiZoneOfficeSimpleAirMotor.BaseClasses.MotorDevice.FlowMoverWithMotor</a></p>
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
end FlowMoverWithMotor;
