package DragPackage
  model DragSys
    inner Modelica.Mechanics.MultiBody.World world(gravityType = Modelica.Mechanics.MultiBody.Types.GravityTypes.NoGravity) annotation(
      Placement(transformation(origin = {-82, -82}, extent = {{-10, -10}, {10, 10}})));
    Modelica.Mechanics.MultiBody.Parts.Fixed fixed annotation(
      Placement(transformation(origin = {-2, 74}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  Modelica.Mechanics.MultiBody.Joints.Prismatic prismatic(n = {0, 1, 0}) annotation(
      Placement(transformation(origin = {-2, 44}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  Modelica.Mechanics.MultiBody.Parts.PointMass pointMass(m = 10)  annotation(
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
      Diagram);end DragSys;

  model DragSystem
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
  end DragSystem;

  block DragForce
    extends Modelica.Blocks.Icons.Block;
  
    Modelica.Blocks.Interfaces.RealInput u[3] "Input Signals (velocity vector)"
      annotation(Placement(transformation(origin = {-106, 0}, extent = {{-20, -20}, {20, 20}}), 
                 iconTransformation(origin = {-106, 0}, extent = {{-20, -20}, {20, 20}})));
    
    Modelica.Blocks.Interfaces.RealOutput y[3] "Output Signals (drag force vector)"
      annotation(Placement(transformation(origin = {108, 0}, extent = {{-10, -10}, {10, 10}}), 
                 iconTransformation(origin = {102, 2}, extent = {{-10, -10}, {10, 10}})));
  
    parameter Real rho_w = 1.25 "Density of sea water [kg/m^3]";
    parameter Real Cd = 1 "Normal drag coefficient";
    parameter Real A = 1 "Cross-sectional Area of object [m^2]";
  
    Real Fd[3];
    Real v_mag;
  
  equation
    v_mag = sqrt(u[1]^2 + u[2]^2 + u[3]^2);
    Fd = -0.5 * rho_w * Cd * A * v_mag * u * abs(v_mag);
    y = Fd;
  end DragForce;

  model Buoyancy
    inner Modelica.Mechanics.MultiBody.World world(gravityType = Modelica.Mechanics.MultiBody.Types.GravityTypes.NoGravity) annotation(
      Placement(transformation(origin = {-46, -48}, extent = {{-10, -10}, {10, 10}})));
    Modelica.Mechanics.MultiBody.Parts.FixedTranslation fixedTranslation(r = {0, 2, 0}, width = 0.1, height = 0.1) annotation(
      Placement(transformation(origin = {6, 8}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
    Modelica.Mechanics.MultiBody.Parts.BodyShape bodyShape(r = {0, 1, 0}, m = 1, length = 0.1, width = 0.1, height = 0.1, r_CM = {0, 0, 0}) annotation(
      Placement(transformation(origin = {6, 46}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
    Modelica.Mechanics.MultiBody.Sensors.AbsoluteVelocity absoluteVelocity annotation(
      Placement(transformation(origin = {46, 56}, extent = {{-10, -10}, {10, 10}})));
    Modelica.Mechanics.MultiBody.Sensors.AbsolutePosition absolutePosition annotation(
      Placement(transformation(origin = {46, 32}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Sources.Cosine cosine[3] (f = {0, 1, 0}, amplitude = {0, 100., 0})  annotation(
      Placement(transformation(origin = {-50, 82}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Forces.WorldForce force annotation(
      Placement(transformation(origin = {-10, 82}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Forces.Spring spring(c = 1e2, s_unstretched = 2)  annotation(
      Placement(transformation(origin = {6, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
    equation
    connect(fixedTranslation.frame_b, bodyShape.frame_a) annotation(
      Line(points = {{6, 18}, {6, 36}}, color = {95, 95, 95}));
    connect(absoluteVelocity.frame_a, bodyShape.frame_b) annotation(
      Line(points = {{36, 56}, {6, 56}}, color = {95, 95, 95}));
    connect(absolutePosition.frame_a, bodyShape.frame_b) annotation(
      Line(points = {{36, 32}, {20, 32}, {20, 56}, {6, 56}}, color = {95, 95, 95}));
    connect(cosine.y, force.force) annotation(
      Line(points = {{-38, 82}, {-22, 82}}, color = {0, 0, 127}));
    connect(force.frame_b, bodyShape.frame_b) annotation(
      Line(points = {{0, 82}, {6, 82}, {6, 56}}, color = {95, 95, 95}));
  connect(spring.frame_b, fixedTranslation.frame_a) annotation(
      Line(points = {{6, -20}, {6, -2}}, color = {95, 95, 95}));
  connect(world.frame_b, spring.frame_a) annotation(
      Line(points = {{-36, -48}, {6, -48}, {6, -40}}, color = {95, 95, 95}));
    annotation(
      Diagram);
  end Buoyancy;
  annotation(
    uses(Modelica(version = "4.0.0")));
end DragPackage;
