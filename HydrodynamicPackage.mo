package Hydrodynamic
  package Example
    model SingleBodyWEC1D "1D Single-Body Wave Energy Converter Model"
      extends Modelica.Icons.Example;
      // World component (no gravity, Z-axis pointing downwards)
      inner Modelica.Mechanics.MultiBody.World world(gravityType = Modelica.Mechanics.MultiBody.Types.GravityTypes.NoGravity, n = {0, 0, -1}) "World coordinate system without gravity" annotation(
        Placement(transformation(origin = {-58, -16}, extent = {{-10, -10}, {10, 10}})));
      // Sensors for position, velocity, angular velocity, and angles
      // Excitation force component
      // Force application component
      Modelica.Mechanics.MultiBody.Forces.WorldForce force "Applies the excitation force to the body" annotation(
        Placement(transformation(origin = {38, -16}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
      // Prismatic joint for vertical motion
      Modelica.Mechanics.MultiBody.Joints.Prismatic prismatic(n = {0, 0, 1}) "Prismatic joint allowing vertical motion" annotation(
        Placement(transformation(origin = {-26, -16}, extent = {{-10, -10}, {10, 10}})));
      // Hydrodynamic body component
      Forces.BodyHD6D bodyHD6D annotation(
        Placement(transformation(origin = {8, -16}, extent = {{-10, -10}, {10, 10}})));
  WaveProfile.IrregularWave.PiersonMoskowitzWave piersonMoskowitzWave annotation(
        Placement(transformation(origin = {36, 12}, extent = {{-10, -10}, {10, 10}})));
    equation
// Connections between components
      connect(world.frame_b, prismatic.frame_a) "Connect world to prismatic joint" annotation(
        Line(points = {{-48, -16}, {-36, -16}}, color = {95, 95, 95}));
      connect(force.frame_b, bodyHD6D.frame_b) annotation(
        Line(points = {{28, -16}, {18, -16}}, color = {95, 95, 95}));
      connect(prismatic.frame_b, bodyHD6D.frame_a) annotation(
        Line(points = {{-16, -16}, {-2, -16}}, color = {95, 95, 95}));
  connect(piersonMoskowitzWave.y, force.force) annotation(
        Line(points = {{46, 12}, {68, 12}, {68, -16}, {50, -16}}, color = {0, 0, 127}, thickness = 0.5));
      annotation(
        Documentation(info = "<html>
          <h4>1D Single-Body Wave Energy Converter (WEC) Model</h4>
          <p>This model represents a simplified 1D single-body wave energy converter system, 
          focusing on the vertical motion of the body in response to wave excitation forces.</p>
          
          <p>Model Description:</p>
          <p>The WEC consists of a hydrodynamic body constrained to move vertically using a prismatic joint. 
          The body is subjected to wave excitation forces, and its motion is measured using various sensors.</p>
          
          <p>Key Components:</p>
          <ul>
            <li><code>world</code>: Defines the world coordinate system without gravity</li>
            <li><code>bodyHD6D</code>: Represents the hydrodynamic body of the WEC</li>
            <li><code>prismatic</code>: Allows vertical motion of the body</li>
            <li><code>excitationForce</code>: Provides the wave excitation force</li>
            <li><code>force</code>: Applies the excitation force to the body</li>
            <li>Sensors: Measure position, velocity, angular velocity, and angles of the body</li>
          </ul>
          
          <p>Assumptions and Simplifications:</p>
          <ul>
            <li>The model considers only vertical motion (1D) of the WEC</li>
            <li>Gravity is not included in the world model</li>
            <li>The excitation force is applied as an external input</li>
          </ul>
          
          <p>Notes:</p>
          <ul>
            <li>This model serves as a basic framework for WEC simulations and can be extended for more complex analyses</li>
            <li>Additional forces like radiation damping or PTO forces can be added to enhance the model's realism</li>
            <li>Ensure that the ExcitationForce and BodyHD6D components are properly configured for accurate results</li>
          </ul>
        </html>"),
        Diagram(coordinateSystem(extent = {{-80, 40}, {100, -40}})),
        experiment(StartTime = 0, StopTime = 500, Tolerance = 1e-08, Interval = 0.05));
    end SingleBodyWEC1D;
  end Example;

  package Forces
    block DragForce6D "6-Dimensional Drag Force and Torque Calculation"
      extends Modelica.Blocks.Icons.Block;
      // Input ports
      Modelica.Blocks.Interfaces.RealInput u[3] "Linear velocity vector [m/s]" annotation(
        Placement(transformation(origin = {-108, 50}, extent = {{-20, -20}, {20, 20}}), iconTransformation(origin = {-98, 50}, extent = {{-20, -20}, {20, 20}})));
      Modelica.Blocks.Interfaces.RealInput a[3] "Angular velocity vector [rad/s]" annotation(
        Placement(transformation(origin = {-108, -50}, extent = {{-20, -20}, {20, 20}}), iconTransformation(origin = {-102, -52}, extent = {{-20, -20}, {20, 20}})));
      // Output ports
      Modelica.Blocks.Interfaces.RealOutput y[3] "Translational drag force vector [N]" annotation(
        Placement(transformation(origin = {108, 50}, extent = {{-10, -10}, {10, 10}}), iconTransformation(origin = {110, 50}, extent = {{-10, -10}, {10, 10}})));
      Modelica.Blocks.Interfaces.RealOutput y1[3] "Rotational drag torque vector [N*m]" annotation(
        Placement(transformation(origin = {108, -50}, extent = {{-10, -10}, {10, 10}}), iconTransformation(origin = {108, -50}, extent = {{-10, -10}, {10, 10}})));
      // Fluid and reference parameters
      parameter Real rho = 1.25 "Density of fluid [kg/m^3]";
      parameter Real A = 1 "Reference area [m^2]";
      // Drag coefficients
      parameter Real Cdx = 1 "Translational drag coefficient for x-axis [-]";
      parameter Real Cdy = 1 "Translational drag coefficient for y-axis [-]";
      parameter Real Cdz = 1 "Translational drag coefficient for z-axis [-]";
      parameter Real Crx = 1 "Rotational drag coefficient for x-axis [-]";
      parameter Real Cry = 1 "Rotational drag coefficient for y-axis [-]";
      parameter Real Crz = 1 "Rotational drag coefficient for z-axis [-]";
      parameter Real Cd[6, 6] = diagonal({Cdx, Cdy, Cdz, Crx, Cry, Crz}) "Combined drag coefficient matrix";
      // Control parameter
      parameter Boolean enableDragForce = true "Switch to enable/disable drag force calculation";
      // Internal variables
      Real c "Combined constant term for drag calculation";
      Real Fd[6] "6D drag force/torque vector [N, N*m]";
      Real v[6] "Combined linear and angular velocity vector [m/s, rad/s]";
    equation
// Calculate the combined constant term
      c = 0.5*rho*A;
// Combine linear and angular velocities into a single vector
      v = cat(1, u, a);
// Calculate the 6D drag force/torque vector
      Fd = -c*Cd*v.*abs(v);
// Use the switch to conditionally output the force and torque
      if enableDragForce then
        y = Fd[1:3];
// Translational drag force
        y1 = Fd[4:6];
// Rotational drag torque
      else
        y = zeros(3);
// Zero translational drag force when disabled
        y1 = zeros(3);
// Zero rotational drag torque when disabled
      end if;
      annotation(
        Documentation(info = "<html>
        <p>This block calculates the 6-dimensional drag force and torque for both translational and rotational motion in a fluid medium.</p>
        <p>The drag force/torque is calculated using a quadratic drag model, where the force is proportional to the square of the velocity.</p>
        <p>The block can be enabled or disabled using the <code>enableDragForce</code> parameter.</p>
        <p>Inputs:</p>
        <ul>
          <li><code>u</code>: Linear velocity vector [m/s]</li>
          <li><code>a</code>: Angular velocity vector [rad/s]</li>
        </ul>
        <p>Outputs:</p>
        <ul>
          <li><code>y</code>: Translational drag force vector [N]</li>
          <li><code>y1</code>: Rotational drag torque vector [N*m]</li>
        </ul>
        <p>Key Parameters:</p>
        <ul>
          <li><code>rho</code>: Fluid density [kg/m^3]</li>
          <li><code>A</code>: Reference area [m^2]</li>
          <li><code>Cdx</code>, <code>Cdy</code>, <code>Cdz</code>: Translational drag coefficients [-]</li>
          <li><code>Crx</code>, <code>Cry</code>, <code>Crz</code>: Rotational drag coefficients [-]</li>
        </ul>
        <p>The drag coefficients are combined into a diagonal matrix to allow for different coefficients in each dimension.</p>
      </html>"));
    end DragForce6D;

    block PTO6D "6-Dimensional Power Take-Off (PTO) System"
      extends Modelica.Blocks.Icons.Block;
      // Input ports
      Modelica.Blocks.Interfaces.RealInput u[3] "Linear velocity vector [m/s]" annotation(
        Placement(transformation(origin = {-106, 60}, extent = {{-20, -20}, {20, 20}}), iconTransformation(origin = {-98, 60}, extent = {{-20, -20}, {20, 20}})));
      Modelica.Blocks.Interfaces.RealInput s[3] "Linear displacement vector [m]" annotation(
        Placement(transformation(origin = {-106, 20}, extent = {{-20, -20}, {20, 20}}), iconTransformation(origin = {-98, 20}, extent = {{-20, -20}, {20, 20}})));
      Modelica.Blocks.Interfaces.RealInput omega[3] "Angular velocity vector [rad/s]" annotation(
        Placement(transformation(origin = {-106, -20}, extent = {{-20, -20}, {20, 20}}), iconTransformation(origin = {-98, -20}, extent = {{-20, -20}, {20, 20}})));
      Modelica.Blocks.Interfaces.RealInput theta[3] "Angular displacement vector [rad]" annotation(
        Placement(transformation(origin = {-106, -60}, extent = {{-20, -20}, {20, 20}}), iconTransformation(origin = {-98, -60}, extent = {{-20, -20}, {20, 20}})));
      // Output ports
      Modelica.Blocks.Interfaces.RealOutput y[3] "Translational PTO force vector [N]" annotation(
        Placement(transformation(origin = {108, 30}, extent = {{-10, -10}, {10, 10}}), iconTransformation(origin = {102, 30}, extent = {{-10, -10}, {10, 10}})));
      Modelica.Blocks.Interfaces.RealOutput y1[3] "Rotational PTO torque vector [N*m]" annotation(
        Placement(transformation(origin = {108, -30}, extent = {{-10, -10}, {10, 10}}), iconTransformation(origin = {102, -30}, extent = {{-10, -10}, {10, 10}})));
      // Proportional gain parameters
      parameter Real Kpx = 0.1 "Proportional gain for x-axis translation [N/(m/s)]";
      parameter Real Kpy = 0.1 "Proportional gain for y-axis translation [N/(m/s)]";
      parameter Real Kpz = 0.1 "Proportional gain for z-axis translation [N/(m/s)]";
      parameter Real Kprx = 0.1 "Proportional gain for x-axis rotation [N*m/(rad/s)]";
      parameter Real Kpry = 0.1 "Proportional gain for y-axis rotation [N*m/(rad/s)]";
      parameter Real Kprz = 0.1 "Proportional gain for z-axis rotation [N*m/(rad/s)]";
      parameter Real Kp[6, 6] = diagonal({Kpx, Kpy, Kpz, Kprx, Kpry, Kprz}) "Combined proportional gain matrix";
      // Integral gain parameters
      parameter Real Kix = 0.1 "Integral gain for x-axis translation [N/m]";
      parameter Real Kiy = 0.1 "Integral gain for y-axis translation [N/m]";
      parameter Real Kiz = 0.1 "Integral gain for z-axis translation [N/m]";
      parameter Real Kirx = 0.1 "Integral gain for x-axis rotation [N*m/rad]";
      parameter Real Kiry = 0.1 "Integral gain for y-axis rotation [N*m/rad]";
      parameter Real Kirz = 0.1 "Integral gain for z-axis rotation [N*m/rad]";
      parameter Real Ki[6, 6] = diagonal({Kix, Kiy, Kiz, Kirx, Kiry, Kirz}) "Combined integral gain matrix";
      // Control parameter
      parameter Boolean enablePTOForce = true "Switch to enable/disable PTO force calculation";
      // Internal variable
      Real Fp[6] "Combined PTO force/torque vector [N, N*m]";
    equation
// Calculate the combined PTO force/torque vector
      Fp = Kp*cat(1, u, omega) + Ki*cat(1, s, theta);
// Use the switch to conditionally output the force and torque
      if enablePTOForce then
        y = -Fp[1:3];
// Translational PTO force (negative for reactive force)
        y1 = -Fp[4:6];
// Rotational PTO torque (negative for reactive torque)
      else
        y = zeros(3);
// Zero translational PTO force when disabled
        y1 = zeros(3);
// Zero rotational PTO torque when disabled
      end if;
      annotation(
        Documentation(info = "<html>
        <p>This block models a 6-dimensional Power Take-Off (PTO) system for both translational and rotational motion.</p>
        <p>The PTO force/torque is calculated using a combination of proportional and integral control based on the input velocities and displacements.</p>
        <p>The block can be enabled or disabled using the <code>enablePTOForce</code> parameter.</p>
        <p>Inputs:</p>
        <ul>
          <li><code>u</code>: Linear velocity vector [m/s]</li>
          <li><code>s</code>: Linear displacement vector [m]</li>
          <li><code>omega</code>: Angular velocity vector [rad/s]</li>
          <li><code>theta</code>: Angular displacement vector [rad]</li>
        </ul>
        <p>Outputs:</p>
        <ul>
          <li><code>y</code>: Translational PTO force vector [N]</li>
          <li><code>y1</code>: Rotational PTO torque vector [N*m]</li>
        </ul>
      </html>"));
    end PTO6D;

    block HydrostaticForce6D "6-Dimensional Hydrostatic Force and Torque Calculation"
      extends Modelica.Blocks.Icons.Block;
      // Input ports
      Modelica.Blocks.Interfaces.RealInput u[3] "Linear displacement vector [m]" annotation(
        Placement(transformation(origin = {-106, 50}, extent = {{-20, -20}, {20, 20}}), iconTransformation(origin = {-96, 50}, extent = {{-20, -20}, {20, 20}})));
      Modelica.Blocks.Interfaces.RealInput theta[3] "Angular displacement vector [rad]" annotation(
        Placement(transformation(origin = {-106, -50}, extent = {{-20, -20}, {20, 20}}), iconTransformation(origin = {-96, -50}, extent = {{-20, -20}, {20, 20}})));
      // Output ports
      Modelica.Blocks.Interfaces.RealOutput y[3] "Translational hydrostatic force vector [N]" annotation(
        Placement(transformation(origin = {108, 50}, extent = {{-10, -10}, {10, 10}}), iconTransformation(origin = {102, 50}, extent = {{-10, -10}, {10, 10}})));
      Modelica.Blocks.Interfaces.RealOutput y1[3] "Rotational hydrostatic torque vector [N*m]" annotation(
        Placement(transformation(origin = {108, -50}, extent = {{-10, -10}, {10, 10}}), iconTransformation(origin = {102, -50}, extent = {{-10, -10}, {10, 10}})));
      // Hydrostatic restoring coefficients
      parameter Real G1 = 0 "Hydrostatic restoring coefficient for x-axis translation [N/m]";
      parameter Real G2 = 0 "Hydrostatic restoring coefficient for y-axis translation [N/m]";
      parameter Real G3 = 2800951.20000000 "Hydrostatic restoring coefficient for z-axis translation [N/m]";
      parameter Real G4 = 0 "Hydrostatic restoring coefficient for x-axis rotation [N*m/rad]";
      parameter Real G5 = 0 "Hydrostatic restoring coefficient for y-axis rotation [N*m/rad]";
      parameter Real G6 = 0 "Hydrostatic restoring coefficient for z-axis rotation [N*m/rad]";
      parameter Real G[6, 6] = diagonal({G1, G2, G3, G4, G5, G6}) "Combined hydrostatic restoring coefficient matrix";
      // Control parameter
      parameter Boolean enableHydrostaticForce = true "Switch to enable/disable hydrostatic force calculation";
      // Internal variables
      Real u_theta[6] "Combined displacement vector [m, rad]";
      Real F[6] "Hydrostatic force/torque vector [N, N*m]";
    equation
// Combine linear and angular displacements into a single vector
      u_theta[1:3] = u;
      u_theta[4:6] = theta;
// Calculate the 6D hydrostatic force/torque vector
      F = -G*u_theta;
// Use the switch to conditionally output the force and torque
      if enableHydrostaticForce then
        y = F[1:3];
// Translational hydrostatic force
        y1 = F[4:6];
// Rotational hydrostatic torque
      else
        y = zeros(3);
// Zero translational hydrostatic force when disabled
        y1 = zeros(3);
// Zero rotational hydrostatic torque when disabled
      end if;
      annotation(
        Documentation(info = "<html>
        <p>This block calculates the 6-dimensional hydrostatic force and torque for both translational and rotational motion in a fluid medium.</p>
        <p>The hydrostatic force/torque is calculated using linear restoring coefficients, where the force is proportional to the displacement from the equilibrium position.</p>
        <p>The block can be enabled or disabled using the <code>enableHydrostaticForce</code> parameter.</p>
        <p>Inputs:</p>
        <ul>
          <li><code>u</code>: Linear displacement vector [m]</li>
          <li><code>theta</code>: Angular displacement vector [rad]</li>
        </ul>
        <p>Outputs:</p>
        <ul>
          <li><code>y</code>: Translational hydrostatic force vector [N]</li>
          <li><code>y1</code>: Rotational hydrostatic torque vector [N*m]</li>
        </ul>
        <p>Key Parameters:</p>
        <ul>
          <li><code>G1</code>, <code>G2</code>, <code>G3</code>: Translational hydrostatic restoring coefficients [N/m]</li>
          <li><code>G4</code>, <code>G5</code>, <code>G6</code>: Rotational hydrostatic restoring coefficients [N*m/rad]</li>
        </ul>
        <p>The hydrostatic restoring coefficients are combined into a diagonal matrix to allow for different coefficients in each dimension.</p>
        <p>Note: By default, only the z-axis translation (heave) has a non-zero restoring coefficient, which is typical for floating bodies.</p>
      </html>"));
    end HydrostaticForce6D;

    model RadiationF "1D Radiation Force Calculation for Hydrodynamic Systems"
      extends Modelica.Blocks.Icons.Block;
      // Input connectors
      Modelica.Blocks.Interfaces.RealInput z[3] "Position vector [m] (not used in calculations)" annotation(
        Placement(transformation(extent = {{-140, -60}, {-100, -20}})));
      Modelica.Blocks.Interfaces.RealInput v[3] "Velocity vector [m/s] (only vertical component used)" annotation(
        Placement(transformation(extent = {{-140, 20}, {-100, 60}})));
      // Output connector
      Modelica.Blocks.Interfaces.RealOutput y[3] "Radiation force vector [N] (only vertical component non-zero)" annotation(
        Placement(transformation(extent = {{100, -10}, {120, 10}})));
      // State-space model parameters for 1D radiation force
      parameter Real A[2, 2] = [0, 1; -1.01116567551434, -0.936555983964093] "State matrix for 1D model";
      parameter Real B[2] = {683236.706073938, -585411.342188539} "Input vector for 1D model";
      parameter Real C[2] = {1, 0} "Output vector for 1D model";
      parameter Real D = 0 "Feed-through scalar for 1D model";
      // Control parameter
      parameter Boolean enableRadiationForce = true "Switch to enable/disable 1D radiation force calculation";
      // State variables
      Real x[2] "State vector for 1D radiation force model";
      Real F_rad "Calculated 1D radiation force [N]";
    initial equation
      x = {0, 0} "Initialize state vector to zero";
    equation
// 1D Radiation force state-space model
// Note: Only the third element of the velocity vector (vertical motion) is used
      der(x) = A*x + B*v[3];
      F_rad = C*x + D*v[3];
// Output: 1D radiation force only in the third element (vertical direction), with enable/disable switch
      if enableRadiationForce then
        y = {0, 0, -F_rad};
      else
        y = {0, 0, 0};
      end if;
      annotation(
        Documentation(info = "<html>
        <h4>1D Radiation Force Model for Hydrodynamic Systems</h4>
        <p>This model calculates the radiation force for hydrodynamic systems using a state-space representation, 
        considering only the vertical direction (1D model).</p>
        
        <p>Model Description:</p>
        <p>The radiation force is computed solely for the vertical direction (third element of the vectors) 
        based on the vertical velocity input. This simplification allows for efficient modeling of systems 
        where the primary concern is the vertical motion, such as in wave energy converters or floating structures.</p>
        
        <p>Inputs:</p>
        <ul>
          <li><code>z[3]</code>: Position vector [m] (currently not used in calculations, included for future extensions)</li>
          <li><code>v[3]</code>: Velocity vector [m/s] (only the third element, representing vertical velocity, is used)</li>
        </ul>
        
        <p>Outputs:</p>
        <ul>
          <li><code>y[3]</code>: Radiation force vector [N] (force applied only in the vertical direction, third element)</li>
        </ul>
        
        <p>Key Parameters:</p>
        <ul>
          <li><code>A</code>, <code>B</code>, <code>C</code>, <code>D</code>: State-space model matrices and vectors for the 1D system</li>
          <li><code>enableRadiationForce</code>: Boolean switch to enable/disable the radiation force calculation</li>
        </ul>
        
        <p>Notes:</p>
        <ul>
          <li>The state-space model parameters (A, B, C, D) should be adjusted based on the specific 1D hydrodynamic system being modeled.</li>
          <li>This model assumes that the radiation force acts only in the vertical direction, simplifying the calculations for many marine applications.</li>
          <li>The position input (z) is currently not used in the calculations but is included for potential future enhancements or compatibility with other models.</li>
        </ul>
      </html>"),
        Icon(graphics = {Text(extent = {{-100, 100}, {100, -100}}, textString = "1D Rad F", fontName = "Arial")}));
    end RadiationF;

    model HydrodynamicBlock6D "6-Dimensional Hydrodynamic Forces and Moments Calculation"
      extends Modelica.Blocks.Icons.Block;
      // Sensors
      Modelica.Mechanics.MultiBody.Sensors.AbsolutePosition absolutePosition annotation(
        Placement(transformation(origin = {-50, 0}, extent = {{-10, -10}, {10, 10}})));
      Modelica.Mechanics.MultiBody.Sensors.AbsoluteVelocity absoluteVelocity annotation(
        Placement(transformation(origin = {-50, 70}, extent = {{-10, -10}, {10, 10}})));
      Modelica.Mechanics.MultiBody.Sensors.AbsoluteAngularVelocity absoluteAngularVelocity annotation(
        Placement(transformation(origin = {-50, -48}, extent = {{-10, -10}, {10, 10}})));
      Modelica.Mechanics.MultiBody.Sensors.AbsoluteAngles absoluteAngles annotation(
        Placement(transformation(origin = {-50, -76}, extent = {{-10, -10}, {10, 10}})));
      // MultiBody connectors
      Modelica.Mechanics.MultiBody.Interfaces.Frame_a frame_a "Input frame" annotation(
        Placement(transformation(origin = {-102, 0}, extent = {{-16, -16}, {16, 16}}), iconTransformation(origin = {-102, 0}, extent = {{-16, -16}, {16, 16}})));
      Modelica.Mechanics.MultiBody.Interfaces.Frame_b frame_b "Output frame" annotation(
        Placement(transformation(origin = {102, 0}, extent = {{-16, -16}, {16, 16}}), iconTransformation(origin = {102, 0}, extent = {{-16, -16}, {16, 16}})));
      // Hydrodynamic force components
      DragForce6D dragForce6D(Cdy = 0.01, rho = 1000, Cdx = 100, Cdz = 100, Crx = 100, Cry = 0.01, Crz = 100, enableDragForce = false) "Drag force calculation" annotation(
        Placement(transformation(origin = {18, 34}, extent = {{-10, -10}, {10, 10}})));
      PTO6D pto6d(enablePTOForce = false) "Power Take-Off force calculation" annotation(
        Placement(transformation(origin = {18, -46}, extent = {{-10, -10}, {10, 10}})));
      HydrostaticForce6D hydrostaticForce6D(enableHydrostaticForce = true) "Hydrostatic force calculation" annotation(
        Placement(transformation(origin = {16, -6}, extent = {{-10, -10}, {10, 10}})));
      RadiationF radiationF "Radiation force calculation" annotation(
        Placement(transformation(origin = {18, 66}, extent = {{-10, -10}, {10, 10}})));
      // Force and torque application components
      Modelica.Mechanics.MultiBody.Forces.WorldForceAndTorque forceAndTorque "Apply drag forces and torques" annotation(
        Placement(transformation(origin = {58, 34}, extent = {{-10, 10}, {10, -10}})));
      Modelica.Mechanics.MultiBody.Forces.WorldForceAndTorque forceAndTorque1 "Apply PTO forces and torques" annotation(
        Placement(transformation(origin = {60, -42}, extent = {{-10, 10}, {10, -10}})));
      Modelica.Mechanics.MultiBody.Forces.WorldForceAndTorque forceAndTorque2 "Apply hydrostatic forces and torques" annotation(
        Placement(transformation(origin = {62, -6}, extent = {{-10, 10}, {10, -10}})));
      Modelica.Mechanics.MultiBody.Forces.WorldForce force "Apply radiation forces" annotation(
        Placement(transformation(origin = {56, 66}, extent = {{-10, -10}, {10, 10}})));
    equation
      connect(absoluteVelocity.v, pto6d.u) annotation(
        Line(points = {{-39, 70}, {-34, 70}, {-34, -40}, {8, -40}}, color = {0, 0, 127}, thickness = 0.5));
      connect(hydrostaticForce6D.y, forceAndTorque2.force) annotation(
        Line(points = {{26, -1}, {26, 0}, {50, 0}}, color = {0, 0, 127}, thickness = 0.5));
      connect(hydrostaticForce6D.y1, forceAndTorque2.torque) annotation(
        Line(points = {{26, -11}, {26, -12}, {50, -12}}, color = {0, 0, 127}, thickness = 0.5));
      connect(pto6d.y1, forceAndTorque1.torque) annotation(
        Line(points = {{28, -49}, {50, -49}, {50, -48}, {48.2, -48}}, color = {0, 0, 127}, thickness = 0.5));
      connect(pto6d.y, forceAndTorque1.force) annotation(
        Line(points = {{28, -43}, {50, -43}, {50, -36}, {48.2, -36}}, color = {0, 0, 127}, thickness = 0.5));
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
        Line(points = {{-39, -76}, {-15.5, -76}, {-15.5, -52}, {8, -52}}, color = {0, 0, 127}, thickness = 0.5));
      connect(absoluteAngularVelocity.w, pto6d.omega) annotation(
        Line(points = {{-39, -48}, {8, -48}}, color = {0, 0, 127}, thickness = 0.5));
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
        Line(points = {{-38, 0}, {-34, 0}, {-34, -44}, {8, -44}}, color = {0, 0, 127}, thickness = 0.5));
      connect(absoluteAngularVelocity.w, dragForce6D.a) annotation(
        Line(points = {{-38, -48}, {-34, -48}, {-34, 28}, {8, 28}}, color = {0, 0, 127}, thickness = 0.5));
      connect(absolutePosition.r, radiationF.z) annotation(
        Line(points = {{-38, 0}, {-34, 0}, {-34, 62}, {6, 62}}, color = {0, 0, 127}, thickness = 0.5));
      annotation(
        Diagram);
      annotation(
        Documentation(info = "<html>
        <p>This model calculates and applies 6-dimensional hydrodynamic forces and moments to a multibody system.</p>
        <p>It incorporates the following hydrodynamic effects:</p>
        <ul>
          <li>Drag forces and torques</li>
          <li>Power Take-Off (PTO) forces and torques</li>
          <li>Hydrostatic forces and torques</li>
          <li>Radiation forces</li>
        </ul>
        <p>The model uses absolute position, velocity, and angular sensors to determine the body's state, 
        calculates the various hydrodynamic forces, and applies them to the body.</p>
        <p>Inputs:</p>
        <ul>
          <li><code>frame_a</code>: Input frame for body state</li>
        </ul>
        <p>Outputs:</p>
        <ul>
          <li><code>frame_b</code>: Output frame with applied hydrodynamic forces and torques</li>
        </ul>
        <p>Note: Some force components (e.g., drag and PTO) are disabled by default and can be enabled by modifying the respective parameters.</p>
      </html>"),
        Diagram);
    end HydrodynamicBlock6D;

    model BodyHD6D "6-Dimensional Hydrodynamic Body Model"
      extends Modelica.Blocks.Icons.Block;
      // MultiBody connectors
      Modelica.Mechanics.MultiBody.Interfaces.Frame_a frame_a "Input frame" annotation(
        Placement(transformation(origin = {-102, 0}, extent = {{-16, -16}, {16, 16}}), iconTransformation(origin = {-102, 0}, extent = {{-16, -16}, {16, 16}})));
      Modelica.Mechanics.MultiBody.Interfaces.Frame_b frame_b "Output frame" annotation(
        Placement(transformation(origin = {102, 0}, extent = {{-16, -16}, {16, 16}}), iconTransformation(origin = {102, 0}, extent = {{-16, -16}, {16, 16}})));
      // Body components
      Modelica.Mechanics.MultiBody.Parts.BodyShape bodyShape(height = 0.1, length = 0.1, m = 1958671, r = {0, 0, 1}, width = 0.1) "Main body with mass and inertia" annotation(
        Placement(transformation(origin = {-50, 0}, extent = {{-10, -10}, {10, 10}})));
      Modelica.Mechanics.MultiBody.Parts.FixedTranslation fixedTranslation(r = {0, 0, -1}, animation = false) "Fixed translation to adjust body position" annotation(
        Placement(transformation(origin = {-20, 0}, extent = {{-10, -10}, {10, 10}})));
      // Hydrodynamic forces component
      HydrodynamicBlock6D hydrodynamicBlock6D "Calculation of hydrodynamic forces and moments" annotation(
        Placement(transformation(origin = {24, 34}, extent = {{-10, -10}, {10, 10}})));
    equation
// Connect input frame to body
      connect(frame_a, bodyShape.frame_a) annotation(
        Line(points = {{-102, 0}, {-60, 0}}));
// Connect body to fixed translation
      connect(bodyShape.frame_b, fixedTranslation.frame_a) annotation(
        Line(points = {{-40, 0}, {-30, 0}}, color = {95, 95, 95}));
// Connect fixed translation to output frame
      connect(fixedTranslation.frame_b, frame_b) annotation(
        Line(points = {{-10, 0}, {102, 0}}, color = {95, 95, 95}));
// Connect hydrodynamic block to body
      connect(hydrodynamicBlock6D.frame_a, fixedTranslation.frame_b) annotation(
        Line(points = {{14, 34}, {-10, 34}, {-10, 0}}, color = {95, 95, 95}));
      connect(hydrodynamicBlock6D.frame_b, fixedTranslation.frame_b) annotation(
        Line(points = {{34, 34}, {46, 34}, {46, 0}, {-10, 0}}, color = {95, 95, 95}));
      annotation(
        Documentation(info = "<html>
        <p>This model represents a 6-dimensional hydrodynamic body, incorporating both rigid body dynamics and hydrodynamic forces.</p>
        <p>The model consists of:</p>
        <ul>
          <li>A main body with specified mass and inertia</li>
          <li>A fixed translation component to adjust the body's position</li>
          <li>A hydrodynamic block that calculates and applies various hydrodynamic forces and moments</li>
        </ul>
        <p>Inputs:</p>
        <ul>
          <li><code>frame_a</code>: Input frame for external connections</li>
        </ul>
        <p>Outputs:</p>
        <ul>
          <li><code>frame_b</code>: Output frame for external connections</li>
        </ul>
        <p>The hydrodynamic forces are applied to the body through the HydrodynamicBlock6D component, 
        which calculates drag, power take-off, hydrostatic, and radiation forces based on the body's motion.</p>
        <p>Note: The body's shape parameters (height, length, width) and mass can be adjusted as needed for specific applications.</p>
      </html>"),
        Icon(graphics = {Text(origin = {-2, -95}, extent = {{-58, 15}, {62, -5}}, textString = "Rigid Body 6D"), Rectangle(origin = {3, -10}, fillColor = {211, 215, 207}, fillPattern = FillPattern.Solid, extent = {{-95, 50}, {89, -50}}), Text(origin = {0, 54}, extent = {{-100, 16}, {100, -24}}, textString = "%name")}));
    end BodyHD6D;

    model ExcitationForce "1D Excitation Force Model for Hydrodynamic Systems"
      extends Modelica.Blocks.Icons.Block;
      // Output connector
      Modelica.Blocks.Interfaces.RealOutput y[3] "Excitation force vector [N] (only vertical component non-zero)" annotation(
        Placement(transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}), iconTransformation(origin = {108, 0}, extent = {{-10, -10}, {10, 10}})));
      // Import the CombiTimeTable for data input
      import Modelica.Blocks.Sources.CombiTimeTable;
      // Excitation force data input
      CombiTimeTable excitationData(tableOnFile = true, fileName = "C:/Users/Duke/SysModel2024/DragSystem/ExcF9.csv", tableName = "excitation", columns = {2}, extrapolation = Modelica.Blocks.Types.Extrapolation.HoldLastPoint) "Time series data for vertical excitation force";
    equation
// Apply excitation force only in the vertical direction (third element)
      y = {0, 0, excitationData.y[1]};
      annotation(
        Documentation(info = "<html>
        <h4>1D Excitation Force Model for Hydrodynamic Systems</h4>
        <p>This model represents the excitation force for hydrodynamic systems, 
        considering only the vertical direction (1D model).</p>
        
        <p>Model Description:</p>
        <p>The excitation force is applied solely in the vertical direction (third element of the output vector). 
        The force magnitude is obtained from a time series data file, allowing for the representation of 
        time-varying excitation forces such as those caused by waves in marine applications.</p>
        
        <p>Inputs:</p>
        <ul>
          <li>No direct inputs. The excitation force is read from an external CSV file.</li>
        </ul>
        
        <p>Outputs:</p>
        <ul>
          <li><code>y[3]</code>: Excitation force vector [N] (force applied only in the vertical direction, third element)</li>
        </ul>
        
        <p>Key Components:</p>
        <ul>
          <li><code>excitationData</code>: CombiTimeTable that reads the excitation force data from a CSV file</li>
        </ul>
        
        <p>File Input:</p>
        <ul>
          <li>File path: C:/Users/Duke/SysModel2024/DragSystem/ExcF6.csv</li>
          <li>Table name: 'excitation'</li>
          <li>Only the second column of the CSV file is used (assumed to contain force values)</li>
        </ul>
        
        <p>Notes:</p>
        <ul>
          <li>The model assumes that the excitation force acts only in the vertical direction, simplifying the calculations for many marine applications.</li>
          <li>The excitation force data is held at its last value if the simulation time exceeds the data in the input file.</li>
          <li>Ensure that the CSV file path is correct and the file is accessible for successful simulation.</li>
        </ul>
      </html>"),
        Icon(graphics = {Text(extent = {{-100, 100}, {100, -100}}, textString = "1D Exc F", fontName = "Arial")}));
    end ExcitationForce;
  end Forces;

  package WaveProfile
    package RegularWave
      /* Package for regular wave elevation profile and excitation force calculations */

      model LinearWave
  /*  Implementation of linear Airy wave model.
                    Excitation force transferred through 'wconn'.
                    Elevation profile is local and not transferred.
                */
        Modelica.Blocks.Interfaces.RealOutput y[3] annotation(
          Placement(transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}), iconTransformation(origin = {108, 0}, extent = {{-10, -10}, {10, 10}})));
      
        extends Modelica.Blocks.Icons.Block;
        import Modelica.Math.Vectors;
        Hydrodynamic.Internal.Connectors.WaveOutConn wconn;
        /* Environmental constants */
        constant Real pi = Modelica.Constants.pi "Mathematical constant pi";
        constant Real g = Modelica.Constants.g_n "Acceleration due to gravity";
        /*  'wDims' is 2-element vector of size of hydroCoeff frequency vector [1, size].
                    Convert to scalar 'parameter Integer wSize' to pass as argument to readRealMatrix().
                    The first matrix dimension is size=1 since w, F_excRe, and F_excIm are vectors.
                    Size of w (frequency vector) is passed as argument for F_excRe and F_excIm also.
                */
        parameter String fileName = "C:/Users/Duke/SysModel2024/OET_Sys-MoDEL/tutorial/hydroCoeff.mat" "Path to the hydroCoeff.mat file";
        parameter Integer wDims[:] = Modelica.Utilities.Streams.readMatrixSize(fileName, "hydroCoeff.w");
        parameter Integer wSize = wDims[2];
        parameter Real F_excRe[:] = vector(Modelica.Utilities.Streams.readRealMatrix(fileName, "hydroCoeff.FexcRe", 1, wSize));
        parameter Real F_excIm[:] = vector(Modelica.Utilities.Streams.readRealMatrix(fileName, "hydroCoeff.FexcIm", 1, wSize));
        parameter Real w[:] = vector(Modelica.Utilities.Streams.readRealMatrix(fileName, "hydroCoeff.w", 1, wSize));
        /* Variable declarations */
        parameter Modelica.Units.SI.Length d = 100 "Water depth";
        parameter Modelica.Units.SI.Density rho = 1025 "Density of seawater";
        parameter Modelica.Units.SI.Length Hs = 2.5 "Significant Wave Height";
        parameter Modelica.Units.SI.AngularFrequency omega = 0.9423 "Wave frequency";
        parameter Real Trmp = 50 "Interval for ramping up of waves during start phase";
        parameter Modelica.Units.SI.Length zeta = Hs/2 "Wave amplitude";
        parameter Real Tp = 2*pi/omega "Wave period";
        parameter Real k = 2*pi/(1.56*(Tp^2)) "Wave number";
        Real ExcCoeffRe "Real component of excitation coefficient";
        Real ExcCoeffIm "Imaginary component of excitation coefficient";
        Modelica.Units.SI.Length SSE "Sea surface elevation";
      equation
/* Interpolate excitation coefficients (Re & Im) for wave frequency */
        ExcCoeffRe = Modelica.Math.Vectors.interpolate(w, F_excRe, omega);
        ExcCoeffIm = Modelica.Math.Vectors.interpolate(w, F_excIm, omega);
/* Define wave elevation profile (SSE) and excitation force */
        SSE = zeta*cos(omega*time);
/*  Ramp the excitation force for any time < ramp time (Trmp) */
        if time < Trmp then
          wconn.F_exc = 0.5*(1 + cos(pi + (pi*time/Trmp)))*((ExcCoeffRe*zeta*cos(omega*time)) - (ExcCoeffIm*zeta*sin(omega*time)))*rho*g;
        else
          wconn.F_exc = ((ExcCoeffRe*zeta*cos(omega*time)) - (ExcCoeffIm*zeta*sin(omega*time)))*rho*g;
        end if;
        y = {0,0, wconn.F_exc};
        annotation(
          Icon(graphics = {Line(origin = {-50.91, 48.08}, points = {{-33.2809, -22.5599}, {-21.2809, -20.5599}, {-13.2809, 27.4401}, {6.71907, -20.5599}, {24.7191, -24.5599}, {42.7191, -24.5599}, {44.7191, -24.5599}}, color = {255, 0, 0}, smooth = Smooth.Bezier), Line(origin = {-37, 51}, points = {{-51, 29}, {-51, -29}, {37, -29}}), Text(origin = {6, 55}, extent = {{-40, 17}, {40, -17}}, textString = "Hs"), Line(origin = {22, 4}, points = {{0, 22}, {0, -22}}, thickness = 1, arrow = {Arrow.None, Arrow.Filled}), Line(origin = {-7.57, -61.12}, points = {{-82.4341, -12.8774}, {-76.4341, -2.87735}, {-72.4341, -6.87735}, {-62.4341, 13.1226}, {-50.4341, -26.8774}, {-46.4341, -20.8774}, {-38.4341, -26.8774}, {-34.4341, -18.8774}, {-34.4341, 3.12265}, {-26.4341, 1.12265}, {-20.4341, 7.12265}, {-12.4341, 9.12265}, {-8.43408, 19.1226}, {1.56592, -4.87735}, {7.56592, -24.8774}, {19.5659, -6.87735}, {21.5659, 9.12265}, {31.5659, 13.1226}, {39.5659, -0.87735}, {43.5659, 11.1226}, {55.5659, 15.1226}, {63.5659, 27.1226}, {79.5659, -22.8774}}, color = {0, 0, 255}, smooth = Smooth.Bezier), Rectangle(origin = {100, 0}, fillColor = {85, 255, 127}, fillPattern = FillPattern.Solid, extent = {{-20, 20}, {20, -20}})}, coordinateSystem(initialScale = 0.1)),
          experiment(StartTime = 0, StopTime = 500, Tolerance = 1e-08, Interval = 0.05));
      end LinearWave;
    end RegularWave;

    package IrregularWave
      /* Package for irregular wave elevation profile and excitation force calculations */

      model PiersonMoskowitzWave
        /*  Implements Pierson Moskowitz (PM) energy spectrum.
                    Excitation force transferred through 'wconn'.
                    Elevation profile is local and not transferred.
                */
        extends Modelica.Blocks.Icons.Block;
        import Modelica.Math.Vectors;
        OceanEngineeringToolbox.Internal.Connectors.WaveOutConn wconn;
        
        Modelica.Blocks.Interfaces.RealOutput y[3] annotation(
          Placement(transformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}}), iconTransformation(origin = {108, 0}, extent = {{-10, -10}, {10, 10}})));
      
        /* Environmental constants */
        constant Real pi = Modelica.Constants.pi "Mathematical constant pi";
        constant Real g = Modelica.Constants.g_n "Acceleration due to gravity";
        /*  'wDims' is 2-element vector of size of hydroCoeff frequency vector [1, size].
                    Convert to scalar 'parameter Integer wSize' to pass as argument to readRealMatrix().
                    The first matrix dimension is size=1 since w, F_excRe, and F_excIm are vectors.
                    Size of w (frequency vector) is passed as argument for F_excRe and F_excIm also.
                */
        parameter String fileName = "C:/Users/Duke/SysModel2024/OET_Sys-MoDEL/tutorial/hydroCoeff.mat" "Path to the hydroCoeff.mat file";
        parameter Integer wDims[:] = Modelica.Utilities.Streams.readMatrixSize(fileName, "hydroCoeff.w");
        parameter Integer wSize = wDims[2];
        parameter Real F_excRe[:] = vector(Modelica.Utilities.Streams.readRealMatrix(fileName, "hydroCoeff.FexcRe", 1, wSize));
        parameter Real F_excIm[:] = vector(Modelica.Utilities.Streams.readRealMatrix(fileName, "hydroCoeff.FexcIm", 1, wSize));
        parameter Real w[:] = vector(Modelica.Utilities.Streams.readRealMatrix(fileName, "hydroCoeff.w", 1, wSize));
        /* Variable declarations */
        parameter Modelica.Units.SI.Length d = 100 "Water depth";
        parameter Modelica.Units.SI.Density rho = 1025 "Density of seawater";
        parameter Modelica.Units.SI.Length Hs = 2.5 "Significant Wave Height";
        parameter Modelica.Units.SI.AngularFrequency omega_min = 0.03141 "Lowest frequency component/frequency interval";
        parameter Modelica.Units.SI.AngularFrequency omega_max = 3.141 "Highest frequency component";
        parameter Modelica.Units.SI.AngularFrequency omega_peak = 0.9423 "Peak spectral frequency";
        parameter Real spectralWidth_min = 0.07 "Lower spectral bound for JONSWAP";
        parameter Real spectralWidth_max = 0.09 "Upper spectral bound for JONSWAP";
        parameter Integer n_omega = 100 "Number of frequency components";
        parameter Integer localSeed = 614657 "Local random seed";
        parameter Integer globalSeed = 30020 "Global random seed";
        parameter Real rnd_shft[n_omega] = OceanEngineeringToolbox.Internal.Functions.randomNumberGen(localSeed, globalSeed, n_omega);
        parameter Integer localSeed1 = 614757 "Local rand seed";
        parameter Integer globalSeed1 = 40020 "Global rand seed";
        parameter Real epsilon[n_omega] = OceanEngineeringToolbox.Internal.Functions.randomNumberGen(localSeed1, globalSeed1, n_omega) "Wave components phase shift";
        parameter Real Trmp = 100 "Interval for ramping up of waves during start phase";
        parameter Real omega[n_omega] = OceanEngineeringToolbox.Internal.Functions.frequencySelector(omega_min, omega_max, rnd_shft);
        parameter Real S[n_omega] = OceanEngineeringToolbox.Internal.Functions.spectrumGenerator_PM(Hs, omega) "Spectral values for frequency components";
        parameter Modelica.Units.SI.Length zeta[n_omega] = sqrt(2*S*omega_min) "Wave amplitude component";
        parameter Real Tp[n_omega] = 2*pi./omega "Wave period components";
        parameter Real k[n_omega] = OceanEngineeringToolbox.Internal.Functions.waveNumber(d, omega) "Wave number component";
        Real ExcCoeffRe[n_omega] "Real component of excitation coefficient for frequency components";
        Real ExcCoeffIm[n_omega] "Imaginary component of excitation coefficient for frequency components";
        Modelica.Units.SI.Length SSE "Sea surface elevation";
      equation
      /* Interpolate excitation coefficients (Re & Im) for each frequency component */
        for i in 1:n_omega loop
          ExcCoeffRe[i] = Modelica.Math.Vectors.interpolate(w, F_excRe, omega[i])*rho*g;
          ExcCoeffIm[i] = Modelica.Math.Vectors.interpolate(w, F_excIm, omega[i])*rho*g;
        end for;
      /* Define wave elevation profile (SSE) and excitation force */
        SSE = sum(zeta.*cos(omega*time - 2*pi*epsilon));
      /*  Ramp the excitation force for any time < ramp time (Trmp) */
        if time < Trmp then
          wconn.F_exc = 0.5*(1 + cos(pi + (pi*time/Trmp)))*sum((ExcCoeffRe.*zeta.*cos(omega*time - 2*pi*epsilon)) - (ExcCoeffIm.*zeta.*sin(omega*time - 2*pi*epsilon)));
        else
          wconn.F_exc = sum((ExcCoeffRe.*zeta.*cos(omega*time - 2*pi*epsilon)) - (ExcCoeffIm.*zeta.*sin(omega*time - 2*pi*epsilon)));
        end if;
        
        y = {0, 0, wconn.F_exc};
      
        annotation(
          Icon(graphics = {Line(origin = {-50.91, 48.08}, points = {{-33.2809, -22.5599}, {-21.2809, -20.5599}, {-13.2809, 27.4401}, {6.71907, -20.5599}, {24.7191, -24.5599}, {42.7191, -24.5599}, {44.7191, -24.5599}}, color = {255, 0, 0}, smooth = Smooth.Bezier), Line(origin = {-37, 51}, points = {{-51, 29}, {-51, -29}, {37, -29}}), Text(origin = {6, 55}, extent = {{-40, 17}, {40, -17}}, textString = "Hs"), Line(origin = {22, 4}, points = {{0, 22}, {0, -22}}, thickness = 1, arrow = {Arrow.None, Arrow.Filled}), Line(origin = {-7.57, -61.12}, points = {{-82.4341, -12.8774}, {-76.4341, -2.87735}, {-72.4341, -6.87735}, {-62.4341, 13.1226}, {-50.4341, -26.8774}, {-46.4341, -20.8774}, {-38.4341, -26.8774}, {-34.4341, -18.8774}, {-34.4341, 3.12265}, {-26.4341, 1.12265}, {-20.4341, 7.12265}, {-12.4341, 9.12265}, {-8.43408, 19.1226}, {1.56592, -4.87735}, {7.56592, -24.8774}, {19.5659, -6.87735}, {21.5659, 9.12265}, {31.5659, 13.1226}, {39.5659, -0.87735}, {43.5659, 11.1226}, {55.5659, 15.1226}, {63.5659, 27.1226}, {79.5659, -22.8774}}, color = {0, 0, 255}, smooth = Smooth.Bezier), Rectangle(origin = {100, 0}, fillColor = {85, 255, 127}, fillPattern = FillPattern.Solid, extent = {{-20, 20}, {20, -20}})}, coordinateSystem(initialScale = 0.1),
          experiment(StartTime = 0, StopTime = 500, Tolerance = 1e-08, Interval = 0.05)));
      end PiersonMoskowitzWave;

      model BretschneiderWave
        /*  Implements Bretschneider energy spectrum.
                    Excitation force transferred through 'wconn'.
                    Elevation profile is local and not transferred.
                */
        extends Modelica.Blocks.Icons.Block;
        import Modelica.Math.Vectors;
        OceanEngineeringToolbox.Internal.Connectors.WaveOutConn wconn;
        /* Environmental constants */
        constant Real pi = Modelica.Constants.pi "Mathematical constant pi";
        constant Real g = Modelica.Constants.g_n "Acceleration due to gravity";
        /*  'wDims' is 2-element vector of size of hydroCoeff frequency vector [1, size].
                    Convert to scalar 'parameter Integer wSize' to pass as argument to readRealMatrix().
                    The first matrix dimension is size=1 since w, F_excRe, and F_excIm are vectors.
                    Size of w (frequency vector) is passed as argument for F_excRe and F_excIm also.
                */
        parameter String fileName "Address of hydroCoeff.mat file";
        parameter Integer wDims[:] = Modelica.Utilities.Streams.readMatrixSize(fileName, "hydroCoeff.w");
        parameter Integer wSize = wDims[2];
        parameter Real F_excRe[:] = vector(Modelica.Utilities.Streams.readRealMatrix(fileName, "hydroCoeff.FexcRe", 1, wSize));
        parameter Real F_excIm[:] = vector(Modelica.Utilities.Streams.readRealMatrix(fileName, "hydroCoeff.FexcIm", 1, wSize));
        parameter Real w[:] = vector(Modelica.Utilities.Streams.readRealMatrix(fileName, "hydroCoeff.w", 1, wSize));
        /*  Variable declarations */
        parameter Modelica.Units.SI.Length d = 100 "Water depth";
        parameter Modelica.Units.SI.Density rho = 1025 "Density of seawater";
        parameter Modelica.Units.SI.Length Hs = 2.5 "Significant Wave Height";
        parameter Modelica.Units.SI.AngularFrequency omega_min = 0.03141 "Lowest frequency component/frequency interval";
        parameter Modelica.Units.SI.AngularFrequency omega_max = 3.141 "Highest frequency component";
        parameter Modelica.Units.SI.AngularFrequency omega_peak = 0.9423 "Peak spectral frequency";
        parameter Integer n_omega = 100 "Number of frequency components";
        parameter Integer localSeed = 614657 "Local random seed";
        parameter Integer globalSeed = 30020 "Global random seed";
        parameter Real rnd_shft[n_omega] = OceanEngineeringToolbox.Internal.Functions.randomNumberGen(localSeed, globalSeed, n_omega);
        parameter Integer localSeed1 = 614757 "Local rand seed";
        parameter Integer globalSeed1 = 40020 "Global rand seed";
        parameter Real epsilon[n_omega] = OceanEngineeringToolbox.Internal.Functions.randomNumberGen(localSeed1, globalSeed1, n_omega) "Wave components phase shift";
        parameter Real Trmp = 200 "Interval for ramping up of waves during start phase";
        parameter Real omega[n_omega] = OceanEngineeringToolbox.Internal.Functions.frequencySelector(omega_min, omega_max, rnd_shft);
        parameter Real S[n_omega] = OceanEngineeringToolbox.Internal.Functions.spectrumGenerator_BRT(Hs, omega, omega_peak) "Spectral values for frequency components";
        parameter Modelica.Units.SI.Length zeta[n_omega] = sqrt(2*S*omega_min) "Wave amplitude component";
        parameter Real Tp[n_omega] = 2*pi./omega "Wave period components";
        parameter Real k[n_omega] = OceanEngineeringToolbox.Internal.Functions.waveNumber(d, omega) "Wave number component";
        Real ExcCoeffRe[n_omega] "Real component of excitation coefficient for frequency components";
        Real ExcCoeffIm[n_omega] "Imaginary component of excitation coefficient for frequency components";
        Modelica.Units.SI.Length SSE "Sea surface elevation";
      equation
/* Interpolate excitation coefficients (Re & Im) for each frequency component */
        for i in 1:n_omega loop
          ExcCoeffRe[i] = Modelica.Math.Vectors.interpolate(w, F_excRe, omega[i])*rho*g;
          ExcCoeffIm[i] = Modelica.Math.Vectors.interpolate(w, F_excIm, omega[i])*rho*g;
        end for;
/* Define wave elevation profile (SSE) and excitation force */
        SSE = sum(zeta.*cos(omega*time - 2*pi*epsilon));
/*  Ramp the excitation force for any time < ramp time (Trmp) */
        if time < Trmp then
          wconn.F_exc = 0.5*(1 + cos(pi + (pi*time/Trmp)))*sum((ExcCoeffRe.*zeta.*cos(omega*time - 2*pi*epsilon)) - (ExcCoeffIm.*zeta.*sin(omega*time - 2*pi*epsilon)));
        else
          wconn.F_exc = sum((ExcCoeffRe.*zeta.*cos(omega*time - 2*pi*epsilon)) - (ExcCoeffIm.*zeta.*sin(omega*time - 2*pi*epsilon)));
        end if;
        annotation(
          Icon(graphics = {Line(origin = {-50.91, 48.08}, points = {{-33.2809, -22.5599}, {-21.2809, -20.5599}, {-13.2809, 27.4401}, {6.71907, -20.5599}, {24.7191, -24.5599}, {42.7191, -24.5599}, {44.7191, -24.5599}}, color = {255, 0, 0}, smooth = Smooth.Bezier), Line(origin = {-37, 51}, points = {{-51, 29}, {-51, -29}, {37, -29}}), Text(origin = {6, 55}, extent = {{-40, 17}, {40, -17}}, textString = "Hs"), Line(origin = {22, 4}, points = {{0, 22}, {0, -22}}, thickness = 1, arrow = {Arrow.None, Arrow.Filled}), Line(origin = {-7.57, -61.12}, points = {{-82.4341, -12.8774}, {-76.4341, -2.87735}, {-72.4341, -6.87735}, {-62.4341, 13.1226}, {-50.4341, -26.8774}, {-46.4341, -20.8774}, {-38.4341, -26.8774}, {-34.4341, -18.8774}, {-34.4341, 3.12265}, {-26.4341, 1.12265}, {-20.4341, 7.12265}, {-12.4341, 9.12265}, {-8.43408, 19.1226}, {1.56592, -4.87735}, {7.56592, -24.8774}, {19.5659, -6.87735}, {21.5659, 9.12265}, {31.5659, 13.1226}, {39.5659, -0.87735}, {43.5659, 11.1226}, {55.5659, 15.1226}, {63.5659, 27.1226}, {79.5659, -22.8774}}, color = {0, 0, 255}, smooth = Smooth.Bezier), Rectangle(origin = {100, 0}, fillColor = {85, 255, 127}, fillPattern = FillPattern.Solid, extent = {{-20, 20}, {20, -20}})}, coordinateSystem(initialScale = 0.1)),
          experiment(StartTime = 0, StopTime = 400, Tolerance = 1e-06, Interval = 0.05));
      end BretschneiderWave;

      model JonswapWave
        /*  Implements JONSWAP energy spectrum.
                    Excitation force transferred through 'wconn'.
                    Elevation profile is local and not transferred.
                */
        extends Modelica.Blocks.Icons.Block;
        import Modelica.Math.Vectors;
        OceanEngineeringToolbox.Internal.Connectors.WaveOutConn wconn;
        /* Environmental constants */
        constant Real pi = Modelica.Constants.pi "Mathematical constant pi";
        constant Real g = Modelica.Constants.g_n "Acceleration due to gravity";
        /*  'wDims' is 2-element vector of size of hydroCoeff frequency vector [1, size].
                    Convert to scalar 'parameter Integer wSize' to pass as argument to readRealMatrix().
                    The first matrix dimension is size=1 since w, F_excRe, and F_excIm are vectors.
                    Size of w (frequency vector) is passed as argument for F_excRe and F_excIm also.
                */
        parameter String fileName "Address of hydroCoeff.mat file";
        parameter Integer wDims[:] = Modelica.Utilities.Streams.readMatrixSize(fileName, "hydroCoeff.w");
        parameter Integer wSize = wDims[2];
        parameter Real F_excRe[:] = vector(Modelica.Utilities.Streams.readRealMatrix(fileName, "hydroCoeff.FexcRe", 1, wSize));
        parameter Real F_excIm[:] = vector(Modelica.Utilities.Streams.readRealMatrix(fileName, "hydroCoeff.FexcIm", 1, wSize));
        parameter Real w[:] = vector(Modelica.Utilities.Streams.readRealMatrix(fileName, "hydroCoeff.w", 1, wSize));
        /* Variable declarations */
        parameter Modelica.Units.SI.Length d = 100 "Water depth";
        parameter Modelica.Units.SI.Density rho = 1025 "Density of seawater";
        parameter Modelica.Units.SI.Length Hs = 2.5 "Significant Wave Height";
        parameter Modelica.Units.SI.AngularFrequency omega_min = 0.03141 "Lowest frequency component/frequency interval";
        parameter Modelica.Units.SI.AngularFrequency omega_max = 3.141 "Highest frequency component";
        parameter Modelica.Units.SI.AngularFrequency omega_peak = 0.9423 "Peak spectral frequency";
        parameter Real spectralWidth_min = 0.07 "Lower spectral bound for JONSWAP";
        parameter Real spectralWidth_max = 0.09 "Upper spectral bound for JONSWAP";
        parameter Integer n_omega = 100 "Number of frequency components";
        parameter Integer localSeed = 614657 "Local random seed";
        parameter Integer globalSeed = 30020 "Global random seed";
        parameter Real rnd_shft[n_omega] = OceanEngineeringToolbox.Internal.Functions.randomNumberGen(localSeed, globalSeed, n_omega);
        parameter Integer localSeed1 = 614757 "Local rand seed";
        parameter Integer globalSeed1 = 40020 "Global rand seed";
        parameter Real epsilon[n_omega] = OceanEngineeringToolbox.Internal.Functions.randomNumberGen(localSeed1, globalSeed1, n_omega) "Wave components phase shift";
        parameter Real Trmp = 200 "Interval for ramping up of waves during start phase";
        parameter Real omega[n_omega] = OceanEngineeringToolbox.Internal.Functions.frequencySelector(omega_min, omega_max, rnd_shft);
        parameter Real S[n_omega] = OceanEngineeringToolbox.Internal.Functions.spectrumGenerator_JONSWAP(Hs, omega, omega_peak, spectralWidth_min, spectralWidth_max) "Spectral values for frequency components";
        parameter Modelica.Units.SI.Length zeta[n_omega] = sqrt(2*S*omega_min) "Wave amplitude component";
        parameter Real Tp[n_omega] = 2*pi./omega "Wave period components";
        parameter Real k[n_omega] = OceanEngineeringToolbox.Internal.Functions.waveNumber(d, omega) "Wave number component";
        Real ExcCoeffRe[n_omega] "Real component of excitation coefficient for frequency components";
        Real ExcCoeffIm[n_omega] "Imaginary component of excitation coefficient for frequency components";
        Modelica.Units.SI.Length SSE "Sea surface elevation";
      equation
/* Interpolate excitation coefficients (Re & Im) for each frequency component */
        for i in 1:n_omega loop
          ExcCoeffRe[i] = Modelica.Math.Vectors.interpolate(w, F_excRe, omega[i])*rho*g;
          ExcCoeffIm[i] = Modelica.Math.Vectors.interpolate(w, F_excIm, omega[i])*rho*g;
        end for;
/* Define wave elevation profile (SSE) and excitation force */
        SSE = sum(zeta.*cos(omega*time - 2*pi*epsilon));
/*  Ramp the excitation force for any time < ramp time (Trmp) */
        if time < Trmp then
          wconn.F_exc = 0.5*(1 + cos(pi + (pi*time/Trmp)))*sum((ExcCoeffRe.*zeta.*cos(omega*time - 2*pi*epsilon)) - (ExcCoeffIm.*zeta.*sin(omega*time - 2*pi*epsilon)));
        else
          wconn.F_exc = sum((ExcCoeffRe.*zeta.*cos(omega*time - 2*pi*epsilon)) - (ExcCoeffIm.*zeta.*sin(omega*time - 2*pi*epsilon)));
        end if;
        annotation(
          Icon(graphics = {Line(origin = {-50.91, 48.08}, points = {{-33.2809, -22.5599}, {-21.2809, -20.5599}, {-13.2809, 27.4401}, {6.71907, -20.5599}, {24.7191, -24.5599}, {42.7191, -24.5599}, {44.7191, -24.5599}}, color = {255, 0, 0}, smooth = Smooth.Bezier), Line(origin = {-37, 51}, points = {{-51, 29}, {-51, -29}, {37, -29}}), Text(origin = {6, 55}, extent = {{-40, 17}, {40, -17}}, textString = "Hs"), Line(origin = {22, 4}, points = {{0, 22}, {0, -22}}, thickness = 1, arrow = {Arrow.None, Arrow.Filled}), Line(origin = {-7.57, -61.12}, points = {{-82.4341, -12.8774}, {-76.4341, -2.87735}, {-72.4341, -6.87735}, {-62.4341, 13.1226}, {-50.4341, -26.8774}, {-46.4341, -20.8774}, {-38.4341, -26.8774}, {-34.4341, -18.8774}, {-34.4341, 3.12265}, {-26.4341, 1.12265}, {-20.4341, 7.12265}, {-12.4341, 9.12265}, {-8.43408, 19.1226}, {1.56592, -4.87735}, {7.56592, -24.8774}, {19.5659, -6.87735}, {21.5659, 9.12265}, {31.5659, 13.1226}, {39.5659, -0.87735}, {43.5659, 11.1226}, {55.5659, 15.1226}, {63.5659, 27.1226}, {79.5659, -22.8774}}, color = {0, 0, 255}, smooth = Smooth.Bezier), Rectangle(origin = {100, 0}, fillColor = {85, 255, 127}, fillPattern = FillPattern.Solid, extent = {{-20, 20}, {20, -20}})}, coordinateSystem(initialScale = 0.1)),
          experiment(StartTime = 0, StopTime = 400, Tolerance = 1e-06, Interval = 0.05));
      end JonswapWave;
    end IrregularWave;
  end WaveProfile;

  package Internal
    /*  Internal library of core functions and connectors */

    package Functions
      /* Package defining explicit library functions */

      function waveNumber
        /* Function to iteratively compute the wave number from the frequency components */
        input Real d "Water depth";
        input Real omega[:] "Wave frequency components";
        output Real k[size(omega, 1)] "Wave number components";
      protected
        constant Real g = Modelica.Constants.g_n;
        constant Real pi = Modelica.Constants.pi;
        parameter Integer n = size(omega, 1);
        Real T[size(omega, 1)] "Wave period components";
        Real L0[size(omega, 1)] "Deepwater wave length";
        Real L1(start = 0, fixed = true) "Temporary variable";
        Real L1c(start = 0, fixed = true) "Temporary variable";
        Real L[size(omega, 1)] "Iterated wave length";
      algorithm
        T := 2*pi./omega;
        L0 := g*T.^2/(2*pi);
        for i in 1:size(omega, 1) loop
          L1 := L0[i];
          L1c := 0;
          while abs(L1c - L1) > 0.001 loop
            L1c := L1;
            L[i] := g*T[i]^2/(2*pi)*tanh(2*pi/L1*d);
            L1 := L[i];
          end while;
        end for;
        k := 2*pi./L;
      end waveNumber;

      function randomNumberGen
        /* Function to generate random numbers from local and global seeds using XOR shift */
        input Integer ls = 614657 "Local seed";
        input Integer gs = 30020 "Global seed";
        input Integer n = 100 "Number of frequency components";
        output Real r64[n] "Random number vector";
      protected
        Integer state64[2](each start = 0, each fixed = true);
      algorithm
        state64[1] := 0;
        state64[2] := 0;
        for i in 1:n loop
          if i == 1 then
            state64 := Modelica.Math.Random.Generators.Xorshift64star.initialState(ls, gs);
            r64[i] := 0;
          else
            (r64[i], state64) := Modelica.Math.Random.Generators.Xorshift64star.random((state64));
          end if;
        end for;
      end randomNumberGen;

      function frequencySelector
        /* Function to randomly select frequency components */
        input Real omega_min "Frequency minima";
        input Real omega_max "Frequency maxima";
        input Real epsilon[:] "Random phase vector";
        output Real omega[size(epsilon, 1)] "Output vector of frequency components";
      protected
        parameter Real ref_omega[size(epsilon, 1)] = omega_min:(omega_max - omega_min)/(size(epsilon, 1) - 1):omega_max;
      algorithm
        omega[1] := omega_min;
        for i in 2:size(epsilon, 1) - 1 loop
          omega[i] := ref_omega[i] + epsilon[i]*omega_min;
        end for;
        omega[size(epsilon, 1)] := omega_max;
      end frequencySelector;

      function spectrumGenerator_PM
        /* Function to generate Pierson Moskowitz spectrum */
        input Real Hs = 1 "Significant wave height";
        input Real omega[:] "Frequency components";
        output Real spec[size(omega, 1)] "Spectral values for input frequencies";
      protected
        constant Real pi = Modelica.Constants.pi;
        constant Real g = Modelica.Constants.g_n;
      algorithm
        for i in 1:size(omega, 1) loop
          spec[i] := 0.0081*g^2/omega[i]^5*exp(-0.0358*(g/(Hs*omega[i]^2))^2);
        end for;
      end spectrumGenerator_PM;

      function spectrumGenerator_BRT
        /* Function to generate Bretschneider spectrum */
        input Real Hs = 1 "Significant wave height";
        input Real omega[:] "Frequency components";
        input Real omega_peak = 0.9423 "Peak spectral frequency";
        output Real spec[size(omega, 1)] "Spectral values for input frequencies";
      protected
        constant Real pi = Modelica.Constants.pi;
        constant Real g = Modelica.Constants.g_n;
      algorithm
        for i in 1:size(omega, 1) loop
          spec[i] := 1.9635*Hs^2*omega_peak^4/omega[i]^5*exp(-1.25*((omega_peak/omega[i])^4));
        end for;
      end spectrumGenerator_BRT;

      function spectrumGenerator_JONSWAP
        /* Function to generate JONSWAP spectrum */
        input Real Hs = 1 "Significant wave height";
        input Real omega[:] "Frequency components";
        input Real omega_peak = 0.9423 "Peak spectral frequency";
        input Real spectralWidth_min "Spectral width lower bound";
        input Real spectralWidth_max "Spectral width upper bound";
        output Real spec[size(omega, 1)] "Spectral values for input frequencies";
      protected
        constant Real pi = Modelica.Constants.pi;
        constant Real g = Modelica.Constants.g_n;
        constant Real gamma = 3.3;
        Real sigma;
        Real b;
      algorithm
        for i in 1:size(omega, 1) loop
          if omega[i] > omega_peak then
            sigma := spectralWidth_max;
          else
            sigma := spectralWidth_min;
          end if;
          b := exp(-0.5*(((omega[i] - omega_peak)/(sigma*omega_peak))^2));
          spec[i] := 0.0081*g^2/omega[i]^5*exp(-1.25*((omega_peak/omega[i])^4))*gamma^b;
        end for;
      end spectrumGenerator_JONSWAP;
    end Functions;

    package Connectors
      /* Package defining library connectors */

      connector WaveOutConn
        /* Output datastream - wave elevation & excitation force */
        Modelica.Blocks.Interfaces.RealOutput F_exc;
      end WaveOutConn;

      connector WaveInConn
        /* Input datastream - wave elevation & excitation force */
        Modelica.Blocks.Interfaces.RealInput F_exc;
      end WaveInConn;

      connector DataCollector
        /* Output datastream - velocity and radiation force */
        Modelica.Blocks.Interfaces.RealOutput F_rad;
        Modelica.Blocks.Interfaces.RealOutput v_z;
      end DataCollector;
    end Connectors;

    model TestDevelopment
      /* Model to test all wave components and WEC rigid body */
      parameter String filePath = "F:/.../hydroCoeff.mat";
      OceanEngineeringToolbox.WaveProfile.RegularWave.LinearWave Reg1(fileName = filePath, Hs = 2.5, Trmp = 50);
      OceanEngineeringToolbox.WaveProfile.IrregularWave.PiersonMoskowitzWave Irr1(fileName = filePath, Hs = 2.5, n_omega = 100, Trmp = 50);
      OceanEngineeringToolbox.WaveProfile.IrregularWave.BretschneiderWave Irr2(fileName = filePath, Hs = 2.5, n_omega = 100, Trmp = 50);
      OceanEngineeringToolbox.WaveProfile.IrregularWave.JonswapWave Irr3(fileName = filePath, Hs = 2.5, n_omega = 100, Trmp = 50);
      OceanEngineeringToolbox.Structures.RigidBody Body1(fileName = filePath);
      OceanEngineeringToolbox.Structures.RigidBody Body2(fileName = filePath);
      OceanEngineeringToolbox.Structures.RigidBody Body3(fileName = filePath);
      OceanEngineeringToolbox.Structures.RigidBody Body4(fileName = filePath);
    equation
      connect(Reg1.wconn.F_exc, Body1.wconn.F_exc);
      connect(Irr1.wconn.F_exc, Body2.wconn.F_exc);
      connect(Irr2.wconn.F_exc, Body3.wconn.F_exc);
      connect(Irr3.wconn.F_exc, Body4.wconn.F_exc);
      annotation(
        experiment(StartTime = 0, StopTime = 200, Tolerance = 1e-06, Interval = 0.1));
    end TestDevelopment;
  end Internal;

  package Testing
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
  end Testing;
  /*  Modelica Ocean Engineering Toolbox v0.3
          Copyright (C) 2024  Ajay Menon, Ali Haider, Kush Bubbar
      
          This program is free software: you can redistribute it and/or modify
          it under the terms of the GNU General Public License as published by
          the Free Software Foundation, either version 3 of the License, or
          (at your option) any later version.
      
          This program is distributed in the hope that it will be useful,
          but WITHOUT ANY WARRANTY; without even the implied warranty of
          MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
          GNU General Public License for more details.
      
          Your copy of the GNU General Public License can be viewed
          here <https://www.gnu.org/licenses/>.
      */
  /*  Library structure:
           Ocean Engineering Toolbox (LIBRARY)
           |  
           |->  Wave Profile (PACKAGE)
           |    |-> Regular Wave (PACKAGE)
           |    |   |-> LinearWave (MODEL)                         [!- Monochromatic regular wave]
           |    |
           |    |-> Irregular Wave (PACKAGE)
           |    |   |-> Pierson Moskowitz Spectrum (MODEL)          [!- Fully-developed sea state]
           |    |   |-> Bretschneider Spectrum (MODEL)             [!- Modified PM spectrum for developing sea state]
           |    |   |-> JONSWAP Spectrum (MODEL)                    [!- Developing sea state with limited fetch]
           |    |
           |->  Structures (PACKAGE)
           |    |-> RigidBody (MODEL)                               [!- Solves motion using the Cummins equation]
           |
           |->  Internal (PACKAGE)
           |    |-> Functions (PACKAGE)
           |    |   |-> waveNumber (FUNCTION)                       [!- Wave number iterations from frequency and depth]
           |    |   |-> randomNumberGen (FUNCTION)                  [!- Random numbers through XOR shift generator]
           |    |   |-> frequencySelector (FUNCTION)                [!- Select wave frequencies from a range]
           |    |   |-> spectrumGenerator_PM (FUNCTION)             [!- Generate Pierson Moskowitz spectrum for frequency components]
           |    |   |-> spectrumGenerator_BRT (FUNCTION)            [!- Generate Bretschneider spectrum for frequency components]
           |    |   |-> spectrumGenerator_JONSWAP (FUNCTION)        [!- Generate JONSWAP spectrum for frequency components]
           |    |
           |    |-> Connectors (PACKAGE)
           |    |   |-> WaveOutConn (CONNECTOR)                     [!- Output transfer wave elevation and excitation force]
           |    |   |-> WaveInConn (CONNECTOR)                      [!- Input transfer wave elevation and excitation force]
           |    |   |-> DataCollector (CONNECTOR)                   [!- Transfer 'Rigid Body' dynamics and forces]
           |    |
           |    |-> TestDevelopment (MODEL)                         [!- Developer component to test all models, functions, connectors]
           |
           |->  Tutorial (PACKAGE)
           |    |-> Sample1 (MODEL)                                 [!- Example model to simulate a rigid body in regular waves]
           |    |-> Sample2 (MODEL)                                 [!- Example model to simulate a rigid body in irregular waves]
           |
           |->  Simulations (PACKAGE)
                |-> * Directory for users to build custom simulation models *
      */
  /*  Modelica Ocean Engineering Toolbox (OET) v0.3
        Developed at:
              Sys-MoDEL, 
              University of New Brunswick, Fredericton
              New Brunswick, E3B 5A3, Canada
        Copyright under the terms of the GNU General Public License
    */
  annotation(
    Documentation(info = "<html>
    <p><strong>Hydrodynamic Package</strong></p>
    
    <p>This package contains models and components for simulating hydrodynamic systems, 
    particularly focused on Wave Energy Converters (WECs). The current implementation 
    includes a single-body WEC model with various force components in 6 degrees of freedom (6D).</p>
    
    <p><strong>Key Components:</strong></p>
    <ul>
      <li><strong>SingleBodyWEC1D:</strong> A 1D single-body Wave Energy Converter model.</li>
      <li><strong>BodyHD6D:</strong> A 6D hydrodynamic body model that incorporates various force components.</li>
      <li><strong>HydrodynamicBlock6D:</strong> Calculates and applies 6D hydrodynamic forces and moments.</li>
      <li><strong>DragForce6D:</strong> Computes 6D drag forces and torques.</li>
      <li><strong>PTO6D:</strong> Models a 6D Power Take-Off system.</li>
      <li><strong>HydrostaticForce6D:</strong> Calculates 6D hydrostatic forces and torques.</li>
      <li><strong>RadiationF:</strong> Computes radiation forces (currently 1D, vertical direction only).</li>
      <li><strong>ExcitationForce:</strong> Provides excitation forces from external data (currently 1D).</li>
    </ul>
    
    <p><strong>Current Capabilities:</strong></p>
    <p>The package allows for modeling of a single-body WEC with the following features:</p>
    <ul>
      <li>6D modeling of drag, PTO, and hydrostatic forces</li>
      <li>1D modeling of radiation forces (vertical direction only)</li>
      <li>External excitation force input</li>
      <li>Rigid body dynamics in 6D</li>
    </ul>
    
    <p><strong>Limitations and Future Work:</strong></p>
    <ul>
      <li>Radiation forces are currently limited to 1D and should be extended to 6D in future versions</li>
      <li>The excitation force model could be expanded to include full 6D capabilities</li>
      <li>Additional WEC configurations (e.g., multi-body systems) could be implemented</li>
    </ul>
    
    <p>This package provides a foundation for hydrodynamic simulations in Modelica, 
    particularly suited for Wave Energy Converter applications. Users can leverage 
    these components to build and simulate various marine energy devices.</p>
  </html>"));
end Hydrodynamic;
