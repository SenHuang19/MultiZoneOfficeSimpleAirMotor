within MultiZoneOfficeSimpleAirMotor.BaseClasses.MotorDevice.Motor;
model SimpleMotor "A model for an induction motor"
  extends Modelica.Icons.MotorIcon;

  parameter Integer pole = 4 "Number of pole pairs";
  parameter Integer n = 3 "Number of phases";
  parameter Modelica.SIunits.Inertia J(min=0.001) = 2 "Moment of inertia";
  parameter Modelica.SIunits.Resistance R_s=0.013 "Electric resistance of stator";
  parameter Modelica.SIunits.Resistance R_r=0.009 "Electric resistance of rotor";
  parameter Modelica.SIunits.Reactance X_s=0.14 "Complex component of the impedance of stator";
  parameter Modelica.SIunits.Reactance X_r=0.12 "Complex component of the impedance of rotor";
  parameter Modelica.SIunits.Reactance X_m=2.4 "Complex component of the magnetizing reactance";
  Modelica.Blocks.Interfaces.RealInput V(unit="V") "Voltage signal" annotation (
     Placement(transformation(
        extent={{-20,-20},{20,20}},
        rotation=0,
        origin={-120,40}), iconTransformation(
        extent={{-20,-20},{20,20}},
        rotation=0,
        origin={-120,40})));
  Modelica.Blocks.Interfaces.RealInput tauM(unit="N.m") "Mechanical torque"
    annotation (Placement(transformation(
        extent={{-20,-20},{20,20}},
        rotation=0,
        origin={-120,-40}), iconTransformation(
        extent={{-20,-20},{20,20}},
        rotation=0,
        origin={-120,-40})));
  Modelica.Blocks.Interfaces.RealInput f(final quantity="Frequency", final unit=
       "Hz") "Freuqency signal"
    annotation (Placement(transformation(extent={{-140,-20},{-100,20}})));
  Modelica.Blocks.Interfaces.RealOutput omegaRat(start=0)
    "Angular velocity of a rator"
    annotation (Placement(transformation(extent={{100,-10},{120,10}})));
  Modelica.SIunits.Torque tau_e "Electromagenetic torque of rotor";
  Modelica.SIunits.Power pow_gap "Air gap power";
  BaseClasses.TorqueSpeedCharacteristics torSpe(
    pole=pole,
    n=n,
    R_s=R_s,
    R_r=R_r,
    X_s=X_s,
    X_r=X_r,
    X_m=X_m)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
  Modelica.Blocks.Sources.RealExpression w_r(y=omegaRat)
    annotation (Placement(transformation(extent={{-60,-30},{-40,-10}})));
  Modelica.SIunits.Resistance Req "Equivelant resistance";
  Modelica.SIunits.Reactance Xeq "Equivelant reactance";
  Real s(min=0,max=1) "Motor slip";
  Modelica.Blocks.Interfaces.RealOutput P(
    quantity = "Power",
    unit = "W")
    "Real power"
    annotation (Placement(transformation(extent={{100,70},{120,90}})));
  Modelica.Blocks.Interfaces.RealOutput Q(
    quantity = "Power",
    unit = "W")
    "Reactive power"
    annotation (Placement(transformation(extent={{100,30},{120,50}})));
equation
  s = torSpe.s;
  tau_e = torSpe.tau_e;
  der(omegaRat) = (tau_e - tauM)/J;
  // constraint the solution of the speed of rotor
  pow_gap = torSpe.omega_s*tau_e;
  // add equations for calculate power
  Req = R_s + R_r*s*X_m^2/(R_r^2+(s^2)*(X_r+X_m)^2);
  Xeq = X_s + X_m*(R_r^2+(s*X_r)^2+(s^2)*X_r*X_m)/(R_r^2+(s^2)*(X_r+X_m)^2);
  P =if noEvent(torSpe.omega_s > 0) then n*V^2*Req/(Req^2 + Xeq^2) else 0;
  Q =if noEvent(torSpe.omega_s > 0) then n*V^2*Xeq/(Req^2 + Xeq^2) else 0;

  connect(V, torSpe.V_rms) annotation (Line(points={{-120,40},{-44,40},{-44,4},
          {-12,4}}, color={0,0,127}));
  connect(w_r.y, torSpe.omega_r) annotation (Line(points={{-39,-20},{-26,-20},{-26,
          -4},{-12,-4}}, color={0,0,127}));

  connect(torSpe.f, f)
    annotation (Line(points={{-12,0},{-120,0}}, color={0,0,127}));
  annotation(defaultComponentName="simMot", Icon(graphics={
                                          Text(
          extent={{-118,160},{46,114}},
          lineColor={0,0,255},
          fillColor={255,255,255},
          fillPattern=FillPattern.None,
          textString="%name")}),
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
<p>This block consists of two parts:</p>
<p>The first part calcualtes the rotational speed of the rotor, <i>&omega;<sub>r</sub></i> by</p>
<p>d(&omega;<sub>r</sub>)/dt = (&tau;<sub>e</sub> -&tau;<sub>l</sub>)/J,<sub>l</sub></p>
<p>where <i>&tau;<sub>e</sub></i> and <i>&tau;<sub>l</sub></i> are the electromagnetic torque and the load torque, respectively.</p>
<p><i>J<sub>l</sub></i> is the load moment inertia.</p>
<p>The second part calculates the real power and the reactive power of an induction motor by</p>
<p>P=nV<sup>2</sup>R<sub>eq</sub>/(R<sub>eq</sub><sup>2</sup>+X<sub>eq</sub><sup>2</sup>),</p>
<p>Q=nV<sup>2</sup>R<sub>eq</sub>/(X<sub>eq</sub><sup>2</sup>+X<sub>eq</sub><sup>2</sup>),</p>
<p>where <i>n</i> and <i>V</i> are the number of phases and the supply voltage, respectively,</p>
<p>where <i>R<sub>eq</sub></i> and <i>X<sub>eq</sub></i> are the equivelant resistance and the equivelant reactance, respectively, and are calculated by</p>
<p>R<sub>eq</sub>=R<sub>s</sub> + R<sub>r</sub>sX<sub>m</sub><sup>2</sup>/(R<sub>r</sub><sup>2</sup>+(s<sup>2</sup>)(X<sub>r</sub>+X<sub>m</sub>)<sup>2</sup>),</p>
<p>X<sub>eq</sub>=X<sub>s</sub> + X<sub>m</sub>(R<sub>r</sub><sup>2</sup>+(s*X<sub>r</sub>)<sup>2</sup>+(s<sup>2</sup>)X<sub>r</sub>X<sub>m</sub>)/(R<sub>r</sub><sup>2</sup>+(s<sup>2</sup>)(X<sub>r</sub>+X<sub>m</sub>)<sup>2</sup>),</p>
<p>where <i>X<sub>m</sub></i>, <i>X<sub>s</sub></i>, <i>X<sub>r</sub></i> are the complex component of the impedance of stator, the complex component of the impedance of rotor, and the complex component of the magnetizing reactance, respctively;</p>
<p><i>R<sub>s</sub></i> and <i>R<sub>r</sub></i> are the electric resistance of the rotor and the electric resistance of the stator, respectively;</p>
<p><i>s</i> reflects the internal slip.</p>
</html>"));
end SimpleMotor;
