within MultiZoneOfficeSimpleAirMotor.BaseClasses.MotorDevice.Motor.BaseClasses;
model TorqueSpeedCharacteristics
  "A model for calculating the electromagnetic torque"
  parameter Integer pole = 2 "Number of pole pairs";
  parameter Integer n = 3 "Number of phases";
  parameter Modelica.SIunits.Resistance R_s=0.013 "Electric resistance of stator";
  parameter Modelica.SIunits.Resistance R_r=0.009 "Electric resistance of rotor";
  parameter Modelica.SIunits.Reactance X_s=0.14 "Complex component of the impedance of stator";
  parameter Modelica.SIunits.Reactance X_r=0.12 "Complex component of the impedance of rotor";
  parameter Modelica.SIunits.Reactance X_m=2.4 "Complex component of the magnetizing reactance";
  Modelica.Blocks.Interfaces.RealInput V_rms(unit="V") "Prescribed rms voltage"
    annotation (Placement(transformation(
        extent={{-20,-20},{20,20}},
        rotation=0,
        origin={-120,40}),
                         iconTransformation(
        extent={{-20,-20},{20,20}},
        rotation=0,
        origin={-120,40})));
  Modelica.Blocks.Interfaces.RealInput f(
    final quantity="Frequency",
    final unit="Hz") "Controllale freuqency to the motor"
    annotation (Placement(transformation(
        extent={{-20,-20},{20,20}},
        rotation=0,
        origin={-120,0}), iconTransformation(
        extent={{-20,-20},{20,20}},
        rotation=0,
        origin={-120,0})));
  Modelica.Blocks.Interfaces.RealInput omega_r(
    final quantity="AngularVelocity",
    final unit="rad/s")
    "Prescribed rotational speed of rotor"
    annotation (Placement(transformation(
        extent={{-20,-20},{20,20}},
        rotation=0,
        origin={-120,-40}), iconTransformation(
        extent={{-20,-20},{20,20}},
        rotation=0,
        origin={-120,-40})));
  Modelica.Blocks.Interfaces.RealOutput tau_e(
    final quantity="Torque",
    final unit="N.m")
    "Electromagenetic torque of rotor"
    annotation (Placement(transformation(extent={{100,-10},{120,10}})));
  Real s(min=0,max=1) "Motor slip";
  Modelica.SIunits.AngularVelocity omega_s "Synchronous angular velocity";

protected
  Real ratio "Intermediate value";
  //Real s_intern "Internal slip for numerical efficiency";
equation

  omega_s = 4*Modelica.Constants.pi*f/pole;
  s = (omega_s-omega_r)/Buildings.Utilities.Math.Functions.smoothMax(1e-05,omega_s,1e-06);
  ratio = X_m/(X_m+X_s);
  tau_e = if noEvent(omega_s>0) then n*R_r*s*(V_rms*ratio)^2/(omega_s*((s*R_s*ratio^2+R_r)^2+s^2*(X_s+X_r)^2)) else 0;

  annotation (defaultComponentName="torSpe",
  Icon(coordinateSystem(preserveAspectRatio=false), graphics={
          Rectangle(
          extent={{-100,100},{100,-100}},
          lineColor={0,0,127},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid), Text(
          extent={{-82,162},{82,116}},
          lineColor={0,0,255},
          fillColor={255,255,255},
          fillPattern=FillPattern.None,
          textString="%name")}),                                 Diagram(
        coordinateSystem(preserveAspectRatio=false)),
    Documentation(info="<html>
<p>This block calculates the electromagnetic torque of a induction motor based on the input voltage (<i>v</i>), the frequency (<i>f</i>), and the rotational speed of the rotor (<i>&omega;<sub>r</sub></i>) by</p>
<p>&tau;<sub>e</sub> = n(vX<sub>m</sub>/(X<sub>m</sub>+X<sub>s</sub>))<sup>2</sup>R<sub>s</sub>/s)/((<i>&omega;<sub>s</sub>(R<sub>s</sub>(X<sub>m</sub>/(X<sub>r</sub>+X<sub>s</sub>))<sup>2</sup>)+R<sub>s</sub>/s)<sup>2</sup>+(X<sub>m</sub>+X<sub>s</sub>)<sup>2</sup>)</p>
<p>where <i>n</i> is the number of phases,</p>
<p><i>X<sub>m</sub></i>, <i>X<sub>s</sub></i>, <i>X<sub>r</sub></i> are the complex component of the impedance of stator, the complex component of the impedance of rotor\", and the complex component of the magnetizing reactance, respctively</p>
<p><i>R<sub>s</sub></i> and <i>R<sub>r</sub></i> are the electric resistance of the rotor and the electric resistance of the stator, respectively;</p>
<p><i>&omega;<sub>s</sub></i> the rotational speed of the stator and is calculalted by: </p>
<p>&omega;<sub>s</sub>=4&pi;f/n<sub>p<sub></p>
<p><i>n<sub>p<sub></i> is the number of pole pairs.</p>
<p><i>s</i> reflects the internal slip and is calculated by:</p>
<p>s=(&omega;<sub>s</sub>-&omega;<sub>r</sub>)/&omega;<sub>s</sub>.<p>
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
end TorqueSpeedCharacteristics;
