package DragPackage
  model DragSimple
    inner Modelica.Mechanics.MultiBody.World world annotation(
      Placement(transformation(origin = {-64, 0}, extent = {{-10, -10}, {10, 10}})));
    Modelica.Blocks.Sources.Constant const[3](k = {-100, 0, 0}) annotation(
      Placement(transformation(origin = {166, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
    Modelica.Mechanics.MultiBody.Parts.PointMass pointMass(m = 1) annotation(
      Placement(transformation(origin = {74, 0}, extent = {{-10, -10}, {10, 10}})));
    Modelica.Mechanics.MultiBody.Parts.FixedTranslation fixedTranslation(r = {1, 0, 0}) annotation(
      Placement(transformation(origin = {48, 0}, extent = {{-10, -10}, {10, 10}})));
    Modelica.Mechanics.MultiBody.Forces.WorldForce force annotation(
      Placement(transformation(origin = {112, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
    Modelica.Blocks.Sources.Constant const2[3](k = {100, 0, 0}) annotation(
      Placement(transformation(origin = {-30, 28}, extent = {{-10, -10}, {10, 10}})));
    Modelica.Mechanics.MultiBody.Forces.WorldForce force1 annotation(
      Placement(transformation(origin = {12, 28}, extent = {{-10, -10}, {10, 10}})));
    Modelica.Mechanics.MultiBody.Joints.Prismatic prismatic(useAxisFlange = false) annotation(
      Placement(transformation(origin = {18, 0}, extent = {{-10, -10}, {10, 10}})));
  equation
    connect(const.y, force.force) annotation(
      Line(points = {{155, 0}, {124, 0}}, color = {0, 0, 127}, thickness = 0.5));
    connect(prismatic.frame_b, fixedTranslation.frame_a) annotation(
      Line(points = {{28, 0}, {38, 0}}, color = {95, 95, 95}));
    connect(fixedTranslation.frame_b, pointMass.frame_a) annotation(
      Line(points = {{58, 0}, {74, 0}}, color = {95, 95, 95}));
    connect(force.frame_b, pointMass.frame_a) annotation(
      Line(points = {{102, 0}, {74, 0}}, color = {95, 95, 95}));
    connect(const2.y, force1.force) annotation(
      Line(points = {{-19, 28}, {-1, 28}}, color = {0, 0, 127}, thickness = 0.5));
    connect(force1.frame_b, prismatic.frame_b) annotation(
      Line(points = {{22, 28}, {28, 28}, {28, 0}}, color = {95, 95, 95}));
    connect(world.frame_b, prismatic.frame_a) annotation(
      Line(points = {{-54, 0}, {8, 0}}, color = {95, 95, 95}));
    annotation(
      Diagram(coordinateSystem(extent = {{-80, 40}, {180, -20}})));
  end DragSimple;

  model DragSystem
    inner Modelica.Mechanics.MultiBody.World world(gravityType = Modelica.Mechanics.MultiBody.Types.GravityTypes.NoGravity) annotation(
      Placement(transformation(origin = {-82, -82}, extent = {{-10, -10}, {10, 10}})));
    Modelica.Mechanics.MultiBody.Parts.Fixed fixed annotation(
      Placement(transformation(origin = {-2, 74}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
    Modelica.Mechanics.MultiBody.Joints.Prismatic prismatic(n = {0, 1, 0}) annotation(
      Placement(transformation(origin = {-2, 44}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
    Modelica.Mechanics.MultiBody.Parts.PointMass pointMass(m = 10) annotation(
      Placement(transformation(origin = {-2, 8}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
    Modelica.Blocks.Sources.Constant const1[3](k = {0, -100, 0}) annotation(
      Placement(transformation(origin = {64, 34}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
    Modelica.Mechanics.MultiBody.Sensors.AbsoluteVelocity absoluteVelocity annotation(
      Placement(transformation(origin = {30, 8}, extent = {{-10, -10}, {10, 10}})));
    Modelica.Mechanics.MultiBody.Forces.WorldForce force annotation(
      Placement(transformation(origin = {-2, -22}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
    Modelica.Mechanics.MultiBody.Forces.WorldForce force1 annotation(
      Placement(transformation(origin = {28, 34}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
    DragForce dragForce annotation(
      Placement(transformation(origin = {26, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
  equation
    connect(prismatic.frame_a, fixed.frame_b) annotation(
      Line(points = {{-2, 54}, {-2, 64}}, color = {95, 95, 95}));
    connect(prismatic.frame_b, pointMass.frame_a) annotation(
      Line(points = {{-2, 34}, {-2, 8}}, color = {95, 95, 95}));
    connect(force.frame_b, pointMass.frame_a) annotation(
      Line(points = {{-2, -12}, {-2, 8}}, color = {95, 95, 95}));
    connect(const1.y, force1.force) annotation(
      Line(points = {{53, 34}, {40, 34}}, color = {0, 0, 127}));
    connect(force1.frame_b, prismatic.frame_b) annotation(
      Line(points = {{18, 34}, {-2, 34}}, color = {95, 95, 95}));
    connect(absoluteVelocity.frame_a, pointMass.frame_a) annotation(
      Line(points = {{20, 8}, {-2, 8}}, color = {95, 95, 95}));
    connect(dragForce.y, force.force) annotation(
      Line(points = {{16, -40}, {-1.8, -40}, {-1.8, -33.8}}, color = {0, 0, 127}, thickness = 0.5));
    connect(absoluteVelocity.v, dragForce.u) annotation(
      Line(points = {{42, 8}, {44, 8}, {44, -40}, {37, -40}}, color = {0, 0, 127}, thickness = 0.5));
    annotation(
      Diagram);
  end DragSystem;

  model Buoyancy
    inner Modelica.Mechanics.MultiBody.World world(gravityType = Modelica.Mechanics.MultiBody.Types.GravityTypes.NoGravity, n = {0, 0, -1}) annotation(
      Placement(transformation(origin = {-64, -14}, extent = {{-10, -10}, {10, 10}})));
    Modelica.Mechanics.MultiBody.Sensors.AbsolutePosition absolutePosition annotation(
      Placement(transformation(origin = {58, 34}, extent = {{-10, -10}, {10, 10}})));
    Modelica.Mechanics.MultiBody.Sensors.AbsoluteVelocity absoluteVelocity annotation(
      Placement(transformation(origin = {58, 56}, extent = {{-10, -10}, {10, 10}})));
    Modelica.Mechanics.MultiBody.Sensors.AbsoluteAngularVelocity absoluteAngularVelocity annotation(
      Placement(transformation(origin = {58, 10}, extent = {{-10, -10}, {10, 10}})));
    Modelica.Mechanics.MultiBody.Sensors.AbsoluteAngles absoluteAngles annotation(
      Placement(transformation(origin = {58, -14}, extent = {{-10, -10}, {10, 10}})));
    HD6D hd6d annotation(
      Placement(transformation(origin = {6, -14}, extent = {{-10, -10}, {10, 10}})));
    ExcitationForce excitationForce annotation(
      Placement(transformation(origin = {-24, 30}, extent = {{-10, -10}, {10, 10}})));
    Modelica.Mechanics.MultiBody.Forces.WorldForce force annotation(
      Placement(transformation(origin = {6, 30}, extent = {{-10, -10}, {10, 10}})));
    Modelica.Mechanics.MultiBody.Joints.Prismatic prismatic(n = {0, 0, 1}) annotation(
      Placement(transformation(origin = {-32, -14}, extent = {{-10, -10}, {10, 10}})));
  equation
    connect(absoluteVelocity.frame_a, hd6d.frame_b) annotation(
      Line(points = {{48, 56}, {27, 56}, {27, -14}, {16, -14}}, color = {95, 95, 95}));
    connect(hd6d.frame_b, absolutePosition.frame_a) annotation(
      Line(points = {{16, -14}, {48, -14}, {48, 34}}, color = {95, 95, 95}));
    connect(hd6d.frame_b, absoluteAngularVelocity.frame_a) annotation(
      Line(points = {{16, -14}, {32, -14}, {32, 10}, {48, 10}}, color = {95, 95, 95}));
    connect(hd6d.frame_b, absoluteAngles.frame_a) annotation(
      Line(points = {{16, -14}, {48, -14}}, color = {95, 95, 95}));
    connect(excitationForce.y, force.force) annotation(
      Line(points = {{-13.2, 30}, {-5.2, 30}}, color = {0, 0, 127}, thickness = 0.5));
    connect(force.frame_b, hd6d.frame_b) annotation(
      Line(points = {{16, 30}, {16, -14}}, color = {95, 95, 95}));
    connect(prismatic.frame_b, hd6d.frame_a) annotation(
      Line(points = {{-22, -14}, {-4, -14}}, color = {95, 95, 95}));
    connect(world.frame_b, prismatic.frame_a) annotation(
      Line(points = {{-54, -14}, {-42, -14}}, color = {95, 95, 95}));
    annotation(
      Diagram(coordinateSystem(extent = {{-80, 80}, {80, -40}})));
  end Buoyancy;

  block DragForce
    extends Modelica.Blocks.Icons.Block;
    Modelica.Blocks.Interfaces.RealInput u[3] "Input Signals (velocity vector)" annotation(
      Placement(transformation(origin = {-106, 0}, extent = {{-20, -20}, {20, 20}}), iconTransformation(origin = {-106, 0}, extent = {{-20, -20}, {20, 20}})));
    Modelica.Blocks.Interfaces.RealOutput y[3] "Output Signals (drag force vector)" annotation(
      Placement(transformation(origin = {108, 0}, extent = {{-10, -10}, {10, 10}}), iconTransformation(origin = {102, 2}, extent = {{-10, -10}, {10, 10}})));
    parameter Real rho_w = 1.25 "Density of sea water [kg/m^3]";
    parameter Real Cd = 1 "Normal drag coefficient";
    parameter Real A = 1 "Cross-sectional Area of object [m^2]";
    Real Fd[3];
  equation
    Fd = -0.5*rho_w*Cd*A*(u.*abs(u));
    y = Fd;
  end DragForce;

  block DragForce3D
    extends Modelica.Blocks.Icons.Block;
    Modelica.Blocks.Interfaces.RealInput u[3] "Input Signals (velocity vector)" annotation(
      Placement(transformation(origin = {-106, 0}, extent = {{-20, -20}, {20, 20}})));
    Modelica.Blocks.Interfaces.RealOutput y[3] "Output Signals (3D drag force vector)" annotation(
      Placement(transformation(origin = {108, 0}, extent = {{-10, -10}, {10, 10}})));
    parameter Real rho = 1.25 "Density of fluid [kg/m^3]";
    parameter Real A = 1 "Reference area [m^2]";
    parameter Real Cdx = 1 "Drag coefficient for x-axis";
    parameter Real Cdy = 1 "Drag coefficient for y-axis";
    parameter Real Cdz = 1 "Drag coefficient for z-axis";
    parameter Real Cd[3, 3] = diagonal({Cdx, Cdy, Cdz});
    Real c "Combined constant term";
    Real Fd[3] "3D drag force vector";
  equation
    c = 0.5*rho*A;
    Fd = -c*Cd*(u.*abs(u));
    y = Fd;
  end DragForce3D;

  block PTO3D
    extends Modelica.Blocks.Icons.Block;
    Modelica.Blocks.Interfaces.RealInput u[3] annotation(
      Placement(transformation(origin = {-106, 50}, extent = {{-20, -20}, {20, 20}}), iconTransformation(origin = {-98, 50}, extent = {{-20, -20}, {20, 20}})));
    Modelica.Blocks.Interfaces.RealInput s[3] annotation(
      Placement(transformation(origin = {-106, -50}, extent = {{-20, -20}, {20, 20}}), iconTransformation(origin = {-98, -50}, extent = {{-20, -20}, {20, 20}})));
    Modelica.Blocks.Interfaces.RealOutput y[3] "Output Signals (3D drag force vector)" annotation(
      Placement(transformation(origin = {108, 0}, extent = {{-10, -10}, {10, 10}})));
    parameter Real Kpx = 0.1 "Proportional gain for x-axis";
    parameter Real Kpy = 0.1 "Proportional gain for y-axis";
    parameter Real Kpz = 0.1 "Proportional gain for z-axis";
    parameter Real Kp[3, 3] = diagonal({Kpx, Kpy, Kpz});
    parameter Real Kix = 0.1 "Integral gain for x-axis";
    parameter Real Kiy = 0.1 "Integral gain for y-axis";
    parameter Real Kiz = 0.1 "Integral gain for z-axis";
    parameter Real Ki[3, 3] = diagonal({Kix, Kiy, Kiz});
    Real Fp[3] "PTO";
  equation
    Fp = Kp*u + Ki*s;
    y = -Fp;
  end PTO3D;

  block HydrostaticForce3D
    extends Modelica.Blocks.Icons.Block;
    Modelica.Blocks.Interfaces.RealInput u[3] annotation(
      Placement(transformation(origin = {-106, 0}, extent = {{-20, -20}, {20, 20}}), iconTransformation(origin = {-96, 0}, extent = {{-20, -20}, {20, 20}})));
    Modelica.Blocks.Interfaces.RealOutput y[3] "Output Signals (drag force vector)" annotation(
      Placement(transformation(origin = {108, 0}, extent = {{-10, -10}, {10, 10}}), iconTransformation(origin = {102, 2}, extent = {{-10, -10}, {10, 10}})));
    parameter Real G1 = 0 "hydrostatic restoring coefficients for x-axis";
    parameter Real G2 = 0 "hydrostatic restoring coefficients for y-axis";
    parameter Real G3 = 1 "hydrostatic restoring coefficients for z-axis";
    parameter Real G[3, 3] = diagonal({G1, G2, G3});
  equation
    y = -G*u;
  end HydrostaticForce3D;

  model HydrodynamicBlock3D
    extends Modelica.Blocks.Icons.Block;
    Modelica.Mechanics.MultiBody.Sensors.AbsolutePosition absolutePosition annotation(
      Placement(transformation(origin = {-50, 0}, extent = {{-10, -10}, {10, 10}})));
    Modelica.Mechanics.MultiBody.Sensors.AbsoluteVelocity absoluteVelocity annotation(
      Placement(transformation(origin = {-50, 30}, extent = {{-10, -10}, {10, 10}})));
    DragForce3D dragForce3D(Cdx = 0.01, Cdy = 0.01, Cdz = 0.01, rho = 10) annotation(
      Placement(transformation(origin = {-8, 30}, extent = {{-10, -10}, {10, 10}})));
    Modelica.Mechanics.MultiBody.Forces.WorldForce force annotation(
      Placement(transformation(origin = {36, 30}, extent = {{-10, -10}, {10, 10}})));
    Modelica.Mechanics.MultiBody.Forces.WorldForce force1 annotation(
      Placement(transformation(origin = {36, 0}, extent = {{-10, -10}, {10, 10}})));
    Modelica.Mechanics.MultiBody.Forces.WorldForce force2 annotation(
      Placement(transformation(origin = {36, -30}, extent = {{-10, -10}, {10, 10}})));
    Modelica.Mechanics.MultiBody.Interfaces.Frame_a frame_a annotation(
      Placement(transformation(origin = {-102, 0}, extent = {{-16, -16}, {16, 16}}), iconTransformation(origin = {-102, 0}, extent = {{-16, -16}, {16, 16}})));
    Modelica.Mechanics.MultiBody.Interfaces.Frame_b frame_b annotation(
      Placement(transformation(origin = {102, 0}, extent = {{-16, -16}, {16, 16}}), iconTransformation(origin = {102, 0}, extent = {{-16, -16}, {16, 16}})));
    PTO3D pto3d annotation(
      Placement(transformation(origin = {-8, -30}, extent = {{-10, -10}, {10, 10}})));
    HydrostaticForce3D hydrostaticForce3D annotation(
      Placement(transformation(origin = {-8, 0}, extent = {{-10, -10}, {10, 10}})));
  equation
    connect(absoluteVelocity.v, dragForce3D.u) annotation(
      Line(points = {{-39, 30}, {-19, 30}}, color = {0, 0, 127}, thickness = 0.5));
    connect(dragForce3D.y, force.force) annotation(
      Line(points = {{2.8, 30}, {24.8, 30}}, color = {0, 0, 127}, thickness = 0.5));
    connect(frame_a, absolutePosition.frame_a) annotation(
      Line(points = {{-102, 0}, {-60, 0}}));
    connect(frame_a, absoluteVelocity.frame_a) annotation(
      Line(points = {{-102, 0}, {-60, 0}, {-60, 30}}));
    connect(force1.frame_b, frame_b) annotation(
      Line(points = {{46, 0}, {102, 0}}, color = {95, 95, 95}));
    connect(force.frame_b, frame_b) annotation(
      Line(points = {{46, 30}, {80, 30}, {80, 0}, {102, 0}}, color = {95, 95, 95}));
    connect(force2.frame_b, frame_b) annotation(
      Line(points = {{46, -30}, {80, -30}, {80, 0}, {102, 0}}, color = {95, 95, 95}));
    connect(pto3d.y, force2.force) annotation(
      Line(points = {{2, -30}, {24, -30}}, color = {0, 0, 127}, thickness = 0.5));
    connect(hydrostaticForce3D.y, force1.force) annotation(
      Line(points = {{2, 0}, {24, 0}}, color = {0, 0, 127}, thickness = 0.5));
    connect(absolutePosition.r, hydrostaticForce3D.u) annotation(
      Line(points = {{-38, 0}, {-18, 0}}, color = {0, 0, 127}, thickness = 0.5));
    connect(absoluteVelocity.v, pto3d.u) annotation(
      Line(points = {{-38, 30}, {-30, 30}, {-30, -24}, {-18, -24}}, color = {0, 0, 127}, thickness = 0.5));
    connect(absolutePosition.r, pto3d.s) annotation(
      Line(points = {{-38, 0}, {-34, 0}, {-34, -34}, {-18, -34}}, color = {0, 0, 127}, thickness = 0.5));
  end HydrodynamicBlock3D;

  block DragForce6D
    extends Modelica.Blocks.Icons.Block;
    Modelica.Blocks.Interfaces.RealInput u[3] "Linear velocity" annotation(
      Placement(transformation(origin = {-108, 50}, extent = {{-20, -20}, {20, 20}}), iconTransformation(origin = {-98, 50}, extent = {{-20, -20}, {20, 20}})));
    Modelica.Blocks.Interfaces.RealInput a[3] "Angular velocity" annotation(
      Placement(transformation(origin = {-108, -50}, extent = {{-20, -20}, {20, 20}}), iconTransformation(origin = {-102, -52}, extent = {{-20, -20}, {20, 20}})));
    Modelica.Blocks.Interfaces.RealOutput y[3] "Translational drag force" annotation(
      Placement(transformation(origin = {108, 50}, extent = {{-10, -10}, {10, 10}}), iconTransformation(origin = {110, 50}, extent = {{-10, -10}, {10, 10}})));
    Modelica.Blocks.Interfaces.RealOutput y1[3] "Rotational drag torque" annotation(
      Placement(transformation(origin = {108, -50}, extent = {{-10, -10}, {10, 10}}), iconTransformation(origin = {108, -50}, extent = {{-10, -10}, {10, 10}})));
    parameter Real rho = 1.25 "Density of fluid [kg/m^3]";
    parameter Real A = 1 "Reference area [m^2]";
    parameter Real Cdx = 1 "Drag coefficient for x-axis";
    parameter Real Cdy = 1 "Drag coefficient for y-axis";
    parameter Real Cdz = 1 "Drag coefficient for z-axis";
    parameter Real Crx = 1 "Rotational Drag coefficient for x-axis";
    parameter Real Cry = 1 "Rotational Drag coefficient for y-axis";
    parameter Real Crz = 1 "Rotational Drag coefficient for z-axis";
    parameter Real Cd[6, 6] = diagonal({Cdx, Cdy, Cdz, Crx, Cry, Crz});
    // Add a switch parameter
    parameter Boolean enableDragForce = true "Switch to enable/disable drag force";
    Real c "Combined constant term";
    Real Fd[6] "6D drag force/torque vector";
    Real v[6];
  equation
    c = 0.5*rho*A;
    v = cat(1, u, a);
    Fd = -c*Cd*v.*abs(v);
// Use the switch to conditionally output the force
    if enableDragForce then
      y = Fd[1:3];
      y1 = Fd[4:6];
    else
      y = zeros(3);
      y1 = zeros(3);
    end if;
  end DragForce6D;

  block PTO6D
    extends Modelica.Blocks.Icons.Block;
    Modelica.Blocks.Interfaces.RealInput u[3] "Linear velocity" annotation(
      Placement(transformation(origin = {-106, 60}, extent = {{-20, -20}, {20, 20}}), iconTransformation(origin = {-98, 60}, extent = {{-20, -20}, {20, 20}})));
    Modelica.Blocks.Interfaces.RealInput s[3] "Linear displacement" annotation(
      Placement(transformation(origin = {-106, 20}, extent = {{-20, -20}, {20, 20}}), iconTransformation(origin = {-98, 20}, extent = {{-20, -20}, {20, 20}})));
    Modelica.Blocks.Interfaces.RealInput omega[3] "Angular velocity" annotation(
      Placement(transformation(origin = {-106, -20}, extent = {{-20, -20}, {20, 20}}), iconTransformation(origin = {-98, -20}, extent = {{-20, -20}, {20, 20}})));
    Modelica.Blocks.Interfaces.RealInput theta[3] "Angular displacement" annotation(
      Placement(transformation(origin = {-106, -60}, extent = {{-20, -20}, {20, 20}}), iconTransformation(origin = {-98, -60}, extent = {{-20, -20}, {20, 20}})));
    Modelica.Blocks.Interfaces.RealOutput y[3] "Translational PTO force" annotation(
      Placement(transformation(origin = {108, 30}, extent = {{-10, -10}, {10, 10}}), iconTransformation(origin = {102, 30}, extent = {{-10, -10}, {10, 10}})));
    Modelica.Blocks.Interfaces.RealOutput y1[3] "Rotational PTO torque" annotation(
      Placement(transformation(origin = {108, -30}, extent = {{-10, -10}, {10, 10}}), iconTransformation(origin = {102, -30}, extent = {{-10, -10}, {10, 10}})));
    parameter Real Kpx = 0.1 "Proportional gain for x-axis translation";
    parameter Real Kpy = 0.1 "Proportional gain for y-axis translation";
    parameter Real Kpz = 0.1 "Proportional gain for z-axis translation";
    parameter Real Kprx = 0.1 "Proportional gain for x-axis rotation";
    parameter Real Kpry = 0.1 "Proportional gain for y-axis rotation";
    parameter Real Kprz = 0.1 "Proportional gain for z-axis rotation";
    parameter Real Kp[6, 6] = diagonal({Kpx, Kpy, Kpz, Kprx, Kpry, Kprz});
    parameter Real Kix = 0.1 "Integral gain for x-axis translation";
    parameter Real Kiy = 0.1 "Integral gain for y-axis translation";
    parameter Real Kiz = 0.1 "Integral gain for z-axis translation";
    parameter Real Kirx = 0.1 "Integral gain for x-axis rotation";
    parameter Real Kiry = 0.1 "Integral gain for y-axis rotation";
    parameter Real Kirz = 0.1 "Integral gain for z-axis rotation";
    parameter Real Ki[6, 6] = diagonal({Kix, Kiy, Kiz, Kirx, Kiry, Kirz});
    // Add a switch parameter
    parameter Boolean enablePTOForce = true "Switch to enable/disable PTO force";
    Real Fp[6] "PTO force/torque vector";
  equation
    Fp = Kp*cat(1, u, omega) + Ki*cat(1, s, theta);
// Use the switch to conditionally output the force
    if enablePTOForce then
      y = -Fp[1:3];
      y1 = -Fp[4:6];
    else
      y = zeros(3);
      y1 = zeros(3);
    end if;
  end PTO6D;

  block HydrostaticForce6D
    extends Modelica.Blocks.Icons.Block;
  
    Modelica.Blocks.Interfaces.RealInput u[3] "Linear displacement" annotation(
      Placement(transformation(origin = {-106, 50}, extent = {{-20, -20}, {20, 20}}), iconTransformation(origin = {-96, 50}, extent = {{-20, -20}, {20, 20}})));
    Modelica.Blocks.Interfaces.RealInput theta[3] "Angular displacement" annotation(
      Placement(transformation(origin = {-106, -50}, extent = {{-20, -20}, {20, 20}}), iconTransformation(origin = {-96, -50}, extent = {{-20, -20}, {20, 20}})));
  
    Modelica.Blocks.Interfaces.RealOutput y[3] "Translational hydrostatic force" annotation(
      Placement(transformation(origin = {108, 50}, extent = {{-10, -10}, {10, 10}}), iconTransformation(origin = {102, 50}, extent = {{-10, -10}, {10, 10}})));
    Modelica.Blocks.Interfaces.RealOutput y1[3] "Rotational hydrostatic torque" annotation(
      Placement(transformation(origin = {108, -50}, extent = {{-10, -10}, {10, 10}}), iconTransformation(origin = {102, -50}, extent = {{-10, -10}, {10, 10}})));
    
    parameter Real G1 = 0 "Hydrostatic restoring coefficient for x-axis translation";
    parameter Real G2 = 0 "Hydrostatic restoring coefficient for y-axis translation";
    parameter Real G3 = 2800951.20000000 "Hydrostatic restoring coefficient for z-axis translation";
    parameter Real G4 = 0 "Hydrostatic restoring coefficient for x-axis rotation";
    parameter Real G5 = 0 "Hydrostatic restoring coefficient for y-axis rotation";
    parameter Real G6 = 0 "Hydrostatic restoring coefficient for z-axis rotation";
    parameter Real G[6, 6] = diagonal({G1, G2, G3, G4, G5, G6});
  
    parameter Boolean enableHydrostaticForce = true "Switch to enable/disable hydrostatic force";
    Real u_theta[6] "Combined displacement vector";
    Real F[6] "Hydrostatic force/torque vector";
  initial equation
  equation
    u_theta[1:3] = u;
    u_theta[4:6] = theta;
    
    F = -G * u_theta;
    
  
    if enableHydrostaticForce then
      y = F[1:3];
      y1 = F[4:6];
    else
      y = zeros(3);
      y1 = zeros(3);
    end if;
  
  end HydrostaticForce6D;

  model HydrodynamicBlock6D
    extends Modelica.Blocks.Icons.Block;
    Modelica.Mechanics.MultiBody.Sensors.AbsolutePosition absolutePosition annotation(
      Placement(transformation(origin = {-50, 0}, extent = {{-10, -10}, {10, 10}})));
    Modelica.Mechanics.MultiBody.Sensors.AbsoluteVelocity absoluteVelocity annotation(
      Placement(transformation(origin = {-50, 70}, extent = {{-10, -10}, {10, 10}})));
    Modelica.Mechanics.MultiBody.Interfaces.Frame_a frame_a annotation(
      Placement(transformation(origin = {-102, 0}, extent = {{-16, -16}, {16, 16}}), iconTransformation(origin = {-102, 0}, extent = {{-16, -16}, {16, 16}})));
    Modelica.Mechanics.MultiBody.Interfaces.Frame_b frame_b annotation(
      Placement(transformation(origin = {102, 0}, extent = {{-16, -16}, {16, 16}}), iconTransformation(origin = {102, 0}, extent = {{-16, -16}, {16, 16}})));
    DragForce6D dragForce6D(Cdy = 0.01, rho = 1000, Cdx = 100, Cdz = 100, Crx = 100, Cry = 0.01, Crz = 100, enableDragForce = false) annotation(
      Placement(transformation(origin = {18, 34}, extent = {{-10, -10}, {10, 10}})));
    PTO6D pto6d(enablePTOForce = false) annotation(
      Placement(transformation(origin = {16, -46}, extent = {{-10, -10}, {10, 10}})));
    HydrostaticForce6D hydrostaticForce6D(enableHydrostaticForce = true) annotation(
      Placement(transformation(origin = {16, -6}, extent = {{-10, -10}, {10, 10}})));
    Modelica.Mechanics.MultiBody.Forces.WorldForceAndTorque forceAndTorque annotation(
      Placement(transformation(origin = {58, 34}, extent = {{-10, 10}, {10, -10}})));
    Modelica.Mechanics.MultiBody.Forces.WorldForceAndTorque forceAndTorque1 annotation(
      Placement(transformation(origin = {60, -42}, extent = {{-10, 10}, {10, -10}})));
    Modelica.Mechanics.MultiBody.Forces.WorldForceAndTorque forceAndTorque2 annotation(
      Placement(transformation(origin = {62, -6}, extent = {{-10, 10}, {10, -10}})));
    Modelica.Mechanics.MultiBody.Sensors.AbsoluteAngularVelocity absoluteAngularVelocity annotation(
      Placement(transformation(origin = {-50, -48}, extent = {{-10, -10}, {10, 10}})));
    Modelica.Mechanics.MultiBody.Sensors.AbsoluteAngles absoluteAngles annotation(
      Placement(transformation(origin = {-50, -76}, extent = {{-10, -10}, {10, 10}})));
    RadiationF radiationF annotation(
      Placement(transformation(origin = {18, 66}, extent = {{-10, -10}, {10, 10}})));
    Modelica.Mechanics.MultiBody.Forces.WorldForce force annotation(
      Placement(transformation(origin = {56, 66}, extent = {{-10, -10}, {10, 10}})));
  equation
    connect(absoluteVelocity.v, pto6d.u) annotation(
      Line(points = {{-39, 70}, {-34, 70}, {-34, -40}, {6, -40}}, color = {0, 0, 127}, thickness = 0.5));
    connect(hydrostaticForce6D.y, forceAndTorque2.force) annotation(
      Line(points = {{26, -1}, {26, 0}, {50, 0}}, color = {0, 0, 127}, thickness = 0.5));
    connect(hydrostaticForce6D.y1, forceAndTorque2.torque) annotation(
      Line(points = {{26, -11}, {26, -12}, {50, -12}}, color = {0, 0, 127}, thickness = 0.5));
    connect(pto6d.y1, forceAndTorque1.torque) annotation(
      Line(points = {{26.2, -49}, {48.2, -49}, {48.2, -48}}, color = {0, 0, 127}, thickness = 0.5));
    connect(pto6d.y, forceAndTorque1.force) annotation(
      Line(points = {{26.2, -43}, {48.2, -43}, {48.2, -36}}, color = {0, 0, 127}, thickness = 0.5));
    connect(dragForce6D.y1, forceAndTorque.torque) annotation(
      Line(points = {{28.8, 29}, {46.8, 29}, {46.8, 27}}, color = {0, 0, 127}, thickness = 0.5));
    connect(dragForce6D.y, forceAndTorque.force) annotation(
      Line(points = {{29, 39}, {45, 39}}, color = {0, 0, 127}, thickness = 0.5));
    connect(forceAndTorque2.frame_b, frame_b) annotation(
      Line(points = {{72, -6}, {78, -6}, {78, 0}, {102, 0}}, color = {95, 95, 95}));
    connect(forceAndTorque1.frame_b, frame_b) annotation(
      Line(points = {{70, -42}, {86, -42}, {86, 0}, {102, 0}}, color = {95, 95, 95}));
    connect(forceAndTorque.frame_b, frame_b) annotation(
      Line(points = {{68, 34}, {78, 34}, {78, 0}, {102, 0}}, color = {95, 95, 95}));
    connect(absoluteAngles.angles, pto6d.theta) annotation(
      Line(points = {{-39, -76}, {-15.5, -76}, {-15.5, -52}, {6, -52}}, color = {0, 0, 127}, thickness = 0.5));
    connect(absoluteAngularVelocity.w, pto6d.omega) annotation(
      Line(points = {{-39, -48}, {6, -48}}, color = {0, 0, 127}, thickness = 0.5));
    connect(absoluteVelocity.v, radiationF.v) annotation(
      Line(points = {{-39, 70}, {6, 70}}, color = {0, 0, 127}, thickness = 0.5));
    connect(radiationF.y, force.force) annotation(
      Line(points = {{29, 66}, {44, 66}}, color = {0, 0, 127}, thickness = 0.5));
    connect(force.frame_b, frame_b) annotation(
      Line(points = {{66, 66}, {78, 66}, {78, 0}, {102, 0}}, color = {95, 95, 95}));
    connect(frame_a, absolutePosition.frame_a) annotation(
      Line(points = {{-102, 0}, {-60, 0}}));
    connect(frame_a, absoluteAngularVelocity.frame_a) annotation(
      Line(points = {{-102, 0}, {-60, 0}, {-60, -48}}));
    connect(frame_a, absoluteAngles.frame_a) annotation(
      Line(points = {{-102, 0}, {-60, 0}, {-60, -76}}));
    connect(frame_a, absoluteVelocity.frame_a) annotation(
      Line(points = {{-102, 0}, {-60, 0}, {-60, 70}}));
    connect(absolutePosition.r, hydrostaticForce6D.u) annotation(
      Line(points = {{-38, 0}, {6, 0}}, color = {0, 0, 127}, thickness = 0.5));
    connect(absoluteVelocity.v, dragForce6D.u) annotation(
      Line(points = {{-38, 70}, {-34, 70}, {-34, 40}, {8, 40}}, color = {0, 0, 127}, thickness = 0.5));
    connect(absoluteAngularVelocity.w, hydrostaticForce6D.theta) annotation(
      Line(points = {{-38, -48}, {-34, -48}, {-34, -10}, {6, -10}}, color = {0, 0, 127}, thickness = 0.5));
    connect(absolutePosition.r, pto6d.s) annotation(
      Line(points = {{-38, 0}, {-34, 0}, {-34, -44}, {6, -44}}, color = {0, 0, 127}, thickness = 0.5));
    connect(absoluteAngularVelocity.w, dragForce6D.a) annotation(
      Line(points = {{-38, -48}, {-34, -48}, {-34, 28}, {8, 28}}, color = {0, 0, 127}, thickness = 0.5));
    connect(absolutePosition.r, radiationF.z) annotation(
      Line(points = {{-38, 0}, {-34, 0}, {-34, 62}, {6, 62}}, color = {0, 0, 127}, thickness = 0.5));
    annotation(
      Diagram);
  end HydrodynamicBlock6D;

  model HD6D
    extends Modelica.Blocks.Icons.Block;
    Modelica.Mechanics.MultiBody.Interfaces.Frame_a frame_a annotation(
      Placement(transformation(origin = {-102, 0}, extent = {{-16, -16}, {16, 16}}), iconTransformation(origin = {-102, 0}, extent = {{-16, -16}, {16, 16}})));
    Modelica.Mechanics.MultiBody.Interfaces.Frame_b frame_b annotation(
      Placement(transformation(origin = {102, 0}, extent = {{-16, -16}, {16, 16}}), iconTransformation(origin = {102, 0}, extent = {{-16, -16}, {16, 16}})));
    Modelica.Mechanics.MultiBody.Parts.BodyShape bodyShape(height = 0.1, length = 0.1, m = 1958672, r = {0, 0, 1}, width = 0.1) annotation(
      Placement(transformation(origin = {-50, 0}, extent = {{-10, -10}, {10, 10}})));
    HydrodynamicBlock6DUPD hydrodynamicBlock6DUPD annotation(
      Placement(transformation(origin = {32, 42}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation fixedTranslation(r = {0, 0, -1})  annotation(
      Placement(transformation(origin = {-20, 0}, extent = {{-10, -10}, {10, 10}})));
  equation
  connect(frame_a, bodyShape.frame_a) annotation(
      Line(points = {{-102, 0}, {-60, 0}}));
  connect(bodyShape.frame_b, fixedTranslation.frame_a) annotation(
      Line(points = {{-40, 0}, {-30, 0}}, color = {95, 95, 95}));
  connect(fixedTranslation.frame_b, frame_b) annotation(
      Line(points = {{-10, 0}, {102, 0}}, color = {95, 95, 95}));
  connect(hydrodynamicBlock6DUPD.frame_a, fixedTranslation.frame_b) annotation(
      Line(points = {{22, 42}, {-10, 42}, {-10, 0}}, color = {95, 95, 95}));
  connect(hydrodynamicBlock6DUPD.frame_b, fixedTranslation.frame_b) annotation(
      Line(points = {{42, 42}, {54, 42}, {54, 0}, {-10, 0}}, color = {95, 95, 95}));
  end HD6D;

  model RadiationF
    extends Modelica.Blocks.Icons.Block;
    
    // Input connectors
    Modelica.Blocks.Interfaces.RealInput z[3] "Position vector" annotation(
      Placement(transformation(extent = {{-140, -60}, {-100, -20}})));
    Modelica.Blocks.Interfaces.RealInput v[3] "Velocity vector" annotation(
      Placement(transformation(extent = {{-140, 20}, {-100, 60}})));
    
    // Output connector
    Modelica.Blocks.Interfaces.RealOutput y[3] "Radiation force vector" annotation(
      Placement(transformation(extent = {{100, -10}, {120, 10}})));
    
    // Parameters
    parameter Real A[2, 2] = [0, 1; -1.01116567551434, -0.936555983964093] "State matrix";
    parameter Real B[2] = {683236.706073938, -585411.342188539} "Input vector";
    parameter Real C[2] = {1, 0} "Output vector";
    parameter Real D = 0 "Feed-through scalar";
    
    // Switch parameter
    parameter Boolean enableRadiationForce = true "Switch to enable/disable radiation force";
    
    // State variables
    Real x[2];
    Real F_rad;
   
  initial equation
    x = {0, 0};
   
  equation
    // Radiation force state-space model using the third element of v
    der(x) = A * x + B * v[3];
    F_rad = C * x + D * v[3];
   
    // Output: radiation force only in the third element, with switch
    if enableRadiationForce then
      y = {0, 0, -F_rad};
    else
      y = {0, 0, 0};
    end if;
  end RadiationF;

  model ExcitationForce
    extends Modelica.Blocks.Icons.Block;
    Modelica.Blocks.Interfaces.RealOutput y[3] annotation(
      Placement(transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}), iconTransformation(origin = {108, 0}, extent = {{-10, -10}, {10, 10}})));
    import Modelica.Blocks.Sources.CombiTimeTable;
    CombiTimeTable excitationData(tableOnFile = true, fileName = "C:/Users/Duke/SysModel2024/DragSystem/ExcF6.csv", tableName = "excitation", columns = {2}, extrapolation = Modelica.Blocks.Types.Extrapolation.HoldLastPoint);
  equation
    y = {0, 0, excitationData.y[1]};
  end ExcitationForce;

  model HydrodynamicBlock6DUPD
    extends Modelica.Blocks.Icons.Block;
    Modelica.Mechanics.MultiBody.Sensors.AbsolutePosition absolutePosition annotation(
      Placement(transformation(origin = {-50, 0}, extent = {{-10, -10}, {10, 10}})));
    Modelica.Mechanics.MultiBody.Sensors.AbsoluteVelocity absoluteVelocity annotation(
      Placement(transformation(origin = {-50, 40}, extent = {{-10, -10}, {10, 10}})));
    Modelica.Mechanics.MultiBody.Interfaces.Frame_a frame_a annotation(
      Placement(transformation(origin = {-102, 0}, extent = {{-16, -16}, {16, 16}}), iconTransformation(origin = {-102, 0}, extent = {{-16, -16}, {16, 16}})));
    Modelica.Mechanics.MultiBody.Interfaces.Frame_b frame_b annotation(
      Placement(transformation(origin = {102, 0}, extent = {{-16, -16}, {16, 16}}), iconTransformation(origin = {102, 0}, extent = {{-16, -16}, {16, 16}})));
    HydrostaticForce6D hydrostaticForce6D(enableHydrostaticForce = true) annotation(
      Placement(transformation(origin = {18, -38}, extent = {{-10, -10}, {10, 10}})));
    Modelica.Mechanics.MultiBody.Forces.WorldForceAndTorque forceAndTorque2 annotation(
      Placement(transformation(origin = {60, -40}, extent = {{-10, 10}, {10, -10}}, rotation = -0)));
    Modelica.Mechanics.MultiBody.Sensors.AbsoluteAngularVelocity absoluteAngularVelocity annotation(
      Placement(transformation(origin = {-50, -32}, extent = {{-10, -10}, {10, 10}})));
    Modelica.Mechanics.MultiBody.Sensors.AbsoluteAngles absoluteAngles annotation(
      Placement(transformation(origin = {-50, -60}, extent = {{-10, -10}, {10, 10}})));
    RadiationF radiationF(enableRadiationForce = true) annotation(
      Placement(transformation(origin = {18, 68}, extent = {{-10, -10}, {10, 10}})));
    Modelica.Mechanics.MultiBody.Forces.WorldForce force annotation(
      Placement(transformation(origin = {56, 68}, extent = {{-10, -10}, {10, 10}})));
  equation
    connect(frame_a, absolutePosition.frame_a) annotation(
      Line(points = {{-102, 0}, {-60, 0}}));
    connect(absolutePosition.r, hydrostaticForce6D.u) annotation(
      Line(points = {{-38, 0}, {-32, 0}, {-32, -33}, {8, -33}}, color = {0, 0, 127}, thickness = 0.5));
    connect(hydrostaticForce6D.y, forceAndTorque2.force) annotation(
      Line(points = {{28, -32}, {48, -32}, {48, -34}}, color = {0, 0, 127}, thickness = 0.5));
    connect(hydrostaticForce6D.y1, forceAndTorque2.torque) annotation(
      Line(points = {{28, -42}, {48, -42}, {48, -46}}, color = {0, 0, 127}, thickness = 0.5));
    connect(forceAndTorque2.frame_b, frame_b) annotation(
      Line(points = {{70, -40}, {78, -40}, {78, 0}, {102, 0}}, color = {95, 95, 95}));
    connect(frame_a, absoluteVelocity.frame_a) annotation(
      Line(points = {{-102, 0}, {-60, 0}, {-60, 40}}));
    connect(frame_a, absoluteAngularVelocity.frame_a) annotation(
      Line(points = {{-102, 0}, {-60, 0}, {-60, -32}}));
    connect(absoluteAngularVelocity.w, hydrostaticForce6D.theta) annotation(
      Line(points = {{-38, -32}, {-32, -32}, {-32, -42}, {8, -42}}, color = {0, 0, 127}, thickness = 0.5));
    connect(frame_a, absoluteAngles.frame_a) annotation(
      Line(points = {{-102, 0}, {-60, 0}, {-60, -60}}));
    connect(absoluteVelocity.v, radiationF.v) annotation(
      Line(points = {{-38, 40}, {-38, 72}, {6, 72}}, color = {0, 0, 127}, thickness = 0.5));
    connect(absolutePosition.r, radiationF.z) annotation(
      Line(points = {{-38, 0}, {-10, 0}, {-10, 64}, {6, 64}}, color = {0, 0, 127}, thickness = 0.5));
    connect(radiationF.y, force.force) annotation(
      Line(points = {{30, 68}, {44, 68}}, color = {0, 0, 127}, thickness = 0.5));
    connect(force.frame_b, frame_b) annotation(
      Line(points = {{66, 68}, {78, 68}, {78, 0}, {102, 0}}, color = {95, 95, 95}));
  end HydrodynamicBlock6DUPD;

  block HydrostaticForce6DUPD
    extends Modelica.Blocks.Icons.Block;
    Modelica.Blocks.Interfaces.RealInput u[3] "Linear displacement" annotation(
      Placement(transformation(origin = {-106, 50}, extent = {{-20, -20}, {20, 20}}), iconTransformation(origin = {-96, 50}, extent = {{-20, -20}, {20, 20}})));
    Modelica.Blocks.Interfaces.RealInput theta[3] "Angular displacement" annotation(
      Placement(transformation(origin = {-106, -50}, extent = {{-20, -20}, {20, 20}}), iconTransformation(origin = {-96, -50}, extent = {{-20, -20}, {20, 20}})));
    Modelica.Blocks.Interfaces.RealOutput y[3] "Translational hydrostatic force" annotation(
      Placement(transformation(origin = {108, 50}, extent = {{-10, -10}, {10, 10}}), iconTransformation(origin = {102, 50}, extent = {{-10, -10}, {10, 10}})));
    Modelica.Blocks.Interfaces.RealOutput y1[3] "Rotational hydrostatic torque" annotation(
      Placement(transformation(origin = {108, -50}, extent = {{-10, -10}, {10, 10}}), iconTransformation(origin = {102, -50}, extent = {{-10, -10}, {10, 10}})));
    parameter Real G1 = 0 "Hydrostatic restoring coefficient for x-axis translation";
    parameter Real G2 = 0 "Hydrostatic restoring coefficient for y-axis translation";
    parameter Real G3 = 1958672 "Hydrostatic restoring coefficient for z-axis translation";
    parameter Real G4 = 0 "Hydrostatic restoring coefficient for x-axis rotation";
    parameter Real G5 = 0 "Hydrostatic restoring coefficient for y-axis rotation";
    parameter Real G6 = 0 "Hydrostatic restoring coefficient for z-axis rotation";
    parameter Real G[6, 6] = diagonal({G1, G2, G3, G4, G5, G6});
    // Add a switch parameter
    parameter Boolean enableHydrostaticForce = true "Switch to enable/disable hydrostatic force";
    Real F[6] "Hydrostatic force/torque vector";
    Real u_theta[6] "Combined linear and angular displacements";
  equation
// Combine u and theta into a 6x1 matrix
    u_theta[1:3] = u;
    u_theta[4:6] = theta;
    F = -G*u_theta;
// Use the switch to conditionally output the force
    if enableHydrostaticForce then
      y = F[1:3];
      y1 = F[4:6];
    else
      y = zeros(3);
      y1 = zeros(3);
    end if;
  end HydrostaticForce6DUPD;
  annotation(
    uses(Modelica(version = "4.0.0")));
end DragPackage;
