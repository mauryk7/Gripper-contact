# ==============================================================================
# PyMechanical Script for Gripper Simulation
# ==============================================================================
#
# GOAL:
# Simulate a gripper mechanism. Two arms, driven by moments on revolute joints,
# close in to grab a fixed cylinder.
#
# PROBLEM:
# The model fails to solve ("silent failure") without any clear error message
# in the Python console. The same setup built manually in the GUI works.
#
# ==============================================================================

# --- Cell 1 & 2: Imports and Application Initialization ---
import os
from PIL import Image
from ansys.mechanical.core import App
from ansys.mechanical.core.examples import delete_downloads, download_file
from matplotlib import image as mpimg
from matplotlib import pyplot as plt
from matplotlib.animation import FuncAnimation
import Ansys.ACT.Math # Required for defining vectors

# Launch Mechanical and connect to the app
app = App()
app.update_globals(globals())
print(app)

# Set the current working directory for output files
cwd = os.path.join(os.getcwd(), "out")

# --- Cell 3 & 4: Graphics and Helper Function ---
# Helper function to display images in the notebook
def display_image(image_name):
    plt.figure(figsize=(16, 9))
    plt.imshow(mpimg.imread(os.path.join(cwd, image_name)))
    plt.xticks([])
    plt.yticks([])
    plt.axis("off")
    plt.show()

# Set up graphics settings for image export
Graphics.Camera.SetSpecificViewOrientation(ViewOrientationType.Front)
image_export_format = GraphicsImageExportFormat.PNG
settings_720p = Ansys.Mechanical.Graphics.GraphicsImageExportSettings()
settings_720p.Resolution = GraphicsResolutionType.EnhancedResolution
settings_720p.Background = GraphicsBackgroundType.White
settings_720p.Width = 1280
settings_720p.Height = 720
settings_720p.CurrentGraphicsDisplay = False
Graphics.Camera.Rotate(180, CameraAxisType.ScreenY)

# --- Cell 5 & 6: Material and Geometry Import ---
# Define file paths
gripper_file = r"C:\\prueba3\\gripper_amp_0.14_off_2.00.step"
steel_path = "Structural_Steel.xml"

# Import material
MAT = Model.Materials
MAT.Import(steel_path)

# Import geometry
geometry_import = Model.GeometryImportGroup.AddGeometryImport()
geometry_import_format = (
    Ansys.Mechanical.DataModel.Enums.GeometryImportPreference.Format.Automatic
)
geometry_import_preferences = Ansys.ACT.Mechanical.Utilities.GeometryImportPreferences()
geometry_import_preferences.ProcessNamedSelections = True
geometry_import_preferences.ProcessCoordinateSystems = True
geometry_import.Import(
    gripper_file, geometry_import_format, geometry_import_preferences
)

# --- Cell 7: Material Assignment ---
print("Assigning materials to bodies...")
try:
    all_bodies = Model.GetChildren(DataModelObjectCategory.Body, True)
    
    # Assign 'Structural Steel' to all 5 bodies in the model
    # The script assumes the import order is:
    # [0] Eje_1 (Axis_1)
    # [1] Eje_2 (Axis_2)
    # [2] Cilindro_Prueba (Cylinder)
    # [3] Gripper_1
    # [4] Gripper_2
    
    for body in all_bodies:
        body.Material = "Structural Steel"
    print("Material 'Structural Steel' assigned to all bodies.")
    
except Exception as e:
    print(f"An error occurred during material assignment: {e}")

# --- Cell 8, 9, 10, 11, 12: Named Selections (NS) Creation ---
# These cells create all necessary Named Selections for scoping
# joints, contacts, loads, and supports.

# Get all bodies as a Python list
all_bodies = Model.GetChildren(DataModelObjectCategory.Body, True)
python_bodies = list(all_bodies)

# Create NS for the 5 main bodies
ns_eje1 = Model.AddNamedSelection()
ns_eje1.Name = "Eje_1"
selection_e1 = ExtAPI.SelectionManager.CreateSelectionInfo(SelectionTypeEnum.GeometryEntities)
selection_e1.Ids = [python_bodies[0].GetGeoBody().Id]
ns_eje1.Location = selection_e1

ns_eje2 = Model.AddNamedSelection()
ns_eje2.Name = "Eje_2"
selection_e2 = ExtAPI.SelectionManager.CreateSelectionInfo(SelectionTypeEnum.GeometryEntities)
selection_e2.Ids = [python_bodies[1].GetGeoBody().Id]
ns_eje2.Location = selection_e2

ns_cilindro = Model.AddNamedSelection()
ns_cilindro.Name = "Cilindro_Prueba"
selection_c = ExtAPI.SelectionManager.CreateSelectionInfo(SelectionTypeEnum.GeometryEntities)
selection_c.Ids = [python_bodies[2].GetGeoBody().Id]
ns_cilindro.Location = selection_c

ns_gripper1 = Model.AddNamedSelection()
ns_gripper1.Name = "Gripper_1"
selection_g1 = ExtAPI.SelectionManager.CreateSelectionInfo(SelectionTypeEnum.GeometryEntities)
selection_g1.Ids = [python_bodies[3].GetGeoBody().Id]
ns_gripper1.Location = selection_g1

ns_gripper2 = Model.AddNamedSelection()
ns_gripper2.Name = "Gripper_2"
selection_g2 = ExtAPI.SelectionManager.CreateSelectionInfo(SelectionTypeEnum.GeometryEntities)
selection_g2.Ids = [python_bodies[4].GetGeoBody().Id]
ns_gripper2.Location = selection_g2
print("Body Named Selections created.")

# Helper function to create NS by face radius
def create_ns_by_radius(body_index, ns_name, radius):
    print(f"--- Processing body at index {body_index} for radius {radius}...")
    cuerpo = python_bodies[body_index]
    faces_found = []
    geo_body = cuerpo.GetGeoBody()
    for face in geo_body.Faces:
        try:
            if math.isclose(face.Radius, radius, rel_tol=1e-5):
                faces_found.append(face)
        except:
            pass # Face is not cylindrical
    
    if not faces_found:
        print(f"WARNING: No faces found with radius {radius} on body '{cuerpo.Name}'.")
        return
        
    ns = Model.AddNamedSelection()
    ns.Name = ns_name
    ids = [f.Id for f in faces_found]
    selection = ExtAPI.SelectionManager.CreateSelectionInfo(SelectionTypeEnum.GeometryEntities)
    selection.Ids = ids
    ns.Location = selection
    print(f"Named Selection '{ns_name}' created with {len(faces_found)} face(s).")

# Helper function to create NS by proximity (for contact faces)
def create_ns_by_proximity(search_body_idx, target_body_idx, ns_name):
    print(f"--- [Proximity Search] For NS '{ns_name}' ---")
    search_body = python_bodies[search_body_idx]
    target_body = python_bodies[target_body_idx]
    
    target_centroid = target_body.GetGeoBody().Centroid
    print(f"  > Searching for face on '{search_body.Name}' closest to '{target_body.Name}'.")
    
    closest_face = None
    min_dist = float('inf')
    
    for face in search_body.GetGeoBody().Faces:
        face_centroid = face.Centroid
        dist = math.sqrt(
            (face_centroid[0] - target_centroid[0])**2 +
            (face_centroid[1] - target_centroid[1])**2 +
            (face_centroid[2] - target_centroid[2])**2
        )
        if dist < min_dist:
            min_dist = dist
            closest_face = face
            
    if closest_face is None:
        print(f"WARNING: Could not find a candidate face on '{search_body.Name}'.")
        return

    ns = Model.AddNamedSelection()
    ns.Name = ns_name
    selection = ExtAPI.SelectionManager.CreateSelectionInfo(SelectionTypeEnum.GeometryEntities)
    selection.Ids = [closest_face.Id]
    ns.Location = selection
    print(f"  > Contact face found (ID: {closest_face.Id}). NS '{ns_name}' created.")

# Helper function to find the *first* flat face (for fixed support)
def create_ns_first_flat_face(body_index, ns_name):
    print(f"--- Searching for first flat face on body index {body_index} ---")
    cuerpo = python_bodies[body_index]
    flat_faces = []
    for face in cuerpo.GetGeoBody().Faces:
        if math.isclose(face.Radius, 0.0, abs_tol=1e-9):
            print(f"  > Flat face found with ID {face.Id}. Stopping search.")
            flat_faces.append(face)
            break # Stop after finding the first one
            
    if not flat_faces:
        print(f"WARNING: No flat faces found on '{cuerpo.Name}'.")
        return
        
    ns = Model.AddNamedSelection()
    ns.Name = ns_name
    selection = ExtAPI.SelectionManager.CreateSelectionInfo(SelectionTypeEnum.GeometryEntities)
    selection.Ids = [f.Id for f in flat_faces]
    ns.Location = selection
    print(f"Named Selection '{ns_name}' created.")

# --- Call NS helper functions ---

# NS for Gripper Holes (for Fixed Joints)
RADIUS_HOLE = 2.0
create_ns_by_radius(body_index=3, ns_name="NS_Agujero_Gripper1", radius=RADIUS_HOLE)
create_ns_by_radius(body_index=4, ns_name="NS_Agujero_Gripper2", radius=RADIUS_HOLE)

# NS for Contact Faces
RADIUS_CYLINDER = 12.0
create_ns_by_proximity(search_body_idx=3, target_body_idx=2, ns_name="NS_Contacto_Gripper1_Interna")
create_ns_by_proximity(search_body_idx=4, target_body_idx=2, ns_name="NS_Contacto_Gripper2_Interna")
create_ns_by_radius(body_index=2, ns_name="NS_Contacto_Cilindro_CaraCilindrica", radius=RADIUS_CYLINDER)

# NS for Axis Faces (for Revolute Joints)
RADIUS_AXIS = 2.0
create_ns_by_radius(body_index=0, ns_name="NS_Cara_Eje1", radius=RADIUS_AXIS)
create_ns_by_radius(body_index=1, ns_name="NS_Cara_Eje2", radius=RADIUS_AXIS)

# NS for Fixed Support (on Cylinder)
create_ns_first_flat_face(body_index=2, ns_name="NS_Cara_Plana_Cilindro")

# --- Cell 13: Set Up Connections ---
ExtAPI.Application.ActiveUnitSystem = MechanicalUnitSystem.StandardNMM
MODEL = Model
GEOM = Model.Geometry
CS_GRP = Model.CoordinateSystems
CONN_GRP = Model.Connections
MSH = Model.Mesh
NS_GRP = Model.NamedSelections

# Delete any existing connections
for connection in CONN_GRP.Children:
    if connection.DataModelObjectCategory==DataModelObjectCategory.ConnectionGroup:
        connection.Delete()

# --- Cell 14: Revolute Joint 1 (Axis 1 to Ground) ---
joint_group_1 = CONN_GRP.AddConnectionGroup()
joint_group_1.Name = "Junta_Revolute_Eje_1"
ns_eje1_faces = [ns for ns in Model.NamedSelections.Children if ns.Name == "NS_Cara_Eje1"][0]

revolute_joint_1 = joint_group_1.AddJoint()
revolute_joint_1.ConnectionType = Ansys.Mechanical.DataModel.Enums.JointScopingType.BodyToGround
revolute_joint_1.Type = Ansys.Mechanical.DataModel.Enums.JointType.Revolute
revolute_joint_1.MobileLocation = ns_eje1_faces
revolute_joint_1.MobileRelaxationMethod = True
revolute_joint_1.ReferenceBehavior = Ansys.Mechanical.DataModel.Enums.LoadBehavior.Deformable

# *** CRITICAL ***
# Force the joint's axis of rotation to be the GLOBAL Z-AXIS (0, 0, 1)
# This overrides the geometry's natural axis.
revolute_joint_1.CoordinateSystem = Model.CoordinateSystems.Children[0] # Global CS
revolute_joint_1.PrincipalAxisX = 0
revolute_joint_1.PrincipalAxisY = 0
revolute_joint_1.PrincipalAxisZ = 1
revolute_joint_1.RenameBasedOnDefinition()
print(f"Joint '{revolute_joint_1.Name}' created and FORCED to rotate on Global Z.")

# --- Cell 15: Revolute Joint 2 (Axis 2 to Ground) ---
joint_group_2 = CONN_GRP.AddConnectionGroup()
joint_group_2.Name = "Junta_Revolute_Eje_2"
ns_eje2_faces = [ns for ns in Model.NamedSelections.Children if ns.Name == "NS_Cara_Eje2"][0]

revolute_joint_2 = joint_group_2.AddJoint()
revolute_joint_2.ConnectionType = Ansys.Mechanical.DataModel.Enums.JointScopingType.BodyToGround
revolute_joint_2.Type = Ansys.Mechanical.DataModel.Enums.JointType.Revolute
revolute_joint_2.MobileLocation = ns_eje2_faces
revolute_joint_2.MobileRelaxationMethod = True
revolute_joint_2.ReferenceBehavior = Ansys.Mechanical.DataModel.Enums.LoadBehavior.Deformable

# *** CRITICAL ***
# Force the joint's axis of rotation to be the GLOBAL Z-AXIS (0, 0, 1)
revolute_joint_2.CoordinateSystem = Model.CoordinateSystems.Children[0] # Global CS
revolute_joint_2.PrincipalAxisX = 0
revolute_joint_2.PrincipalAxisY = 0
revolute_joint_2.PrincipalAxisZ = 1
revolute_joint_2.RenameBasedOnDefinition()
print(f"Joint '{revolute_joint_2.Name}' created and FORCED to rotate on Global Z.")

# --- Cell 16: Fixed Joints (Body-to-Body) ---
# This function creates the Fixed joints connecting the Axes to the Grippers
def create_fixed_joint(ref_ns_name, mob_ns_name, parent_group):
    ns_ref = [ns for ns in Model.NamedSelections.Children if ns.Name == ref_ns_name][0]
    ns_mob = [ns for ns in Model.NamedSelections.Children if ns.Name == mob_ns_name][0]
    
    joint = parent_group.AddJoint()
    joint.Type = Ansys.Mechanical.DataModel.Enums.JointType.Fixed
    joint.ReferenceLocation = ns_ref
    joint.MobileLocation = ns_mob
    
    # Set Formulation to MPC (Multi-Point Constraint)
    joint.ReferenceFormulation = Ansys.Mechanical.DataModel.Enums.RemotePointFormulation.MPC
    joint.MobileFormulation = Ansys.Mechanical.DataModel.Enums.RemotePointFormulation.MPC
    
    # Set Behavior to Deformable
    joint.ReferenceRelaxationMethod = True
    joint.MobileRelaxationMethod = True
    joint.ReferenceBehavior = Ansys.Mechanical.DataModel.Enums.LoadBehavior.Deformable
    joint.MobileBehavior = Ansys.Mechanical.DataModel.Enums.LoadBehavior.Deformable
    
    joint.RenameBasedOnDefinition()
    print(f"Created Fixed Joint: '{joint.Name}'")

# Create the two fixed joints, adding them to the FIRST joint group
create_fixed_joint("NS_Cara_Eje1", "NS_Agujero_Gripper1", joint_group_1)
create_fixed_joint("NS_Cara_Eje2", "NS_Agujero_Gripper2", joint_group_1) # Changed to group 1 for tidiness

# --- Cell 17: Contact Regions ---
contact_group = CONN_GRP.AddConnectionGroup()
contact_group.Name = "Contactos_Gripper_Cilindro"

# Get the contact face NS
ns_contact_gripper1 = [ns for ns in Model.NamedSelections.Children if ns.Name == "NS_Contacto_Gripper1_Interna"][0]
ns_contact_gripper2 = [ns for ns in Model.NamedSelections.Children if ns.Name == "NS_Contacto_Gripper2_Interna"][0]
ns_contact_cilindro = [ns for ns in Model.NamedSelections.Children if ns.Name == "NS_Contacto_Cilindro_CaraCilindrica"][0]

# Contact 1: Gripper 1 (Contact) -> Cylinder (Target)
contact_1 = contact_group.AddContactRegion()
contact_1.SourceLocation = ns_contact_gripper1.Location # Contact
contact_1.TargetLocation = ns_contact_cilindro.Location # Target
contact_1.ContactType = Ansys.Mechanical.DataModel.Enums.ContactType.Frictionless
contact_1.RenameBasedOnDefinition()
print(f"Created contact: '{contact_1.Name}'")

# Contact 2: Gripper 2 (Contact) -> Cylinder (Target)
contact_2 = contact_group.AddContactRegion()
contact_2.SourceLocation = ns_contact_gripper2.Location # Contact
contact_2.TargetLocation = ns_contact_cilindro.Location # Target
contact_2.ContactType = Ansys.Mechanical.DataModel.Enums.ContactType.Frictionless
contact_2.RenameBasedOnDefinition()
print(f"Created contact: '{contact_2.Name}'")

# --- Cell 18 & 19: Contact Tool & Initial Results ---
contact_tool_setup = CONN_GRP.AddContactTool()
contact_tool_setup.Name = "Herramienta de Contacto (Setup)"
print("Added 'Contact Tool (Setup)' to Connections.")

# Generate initial contact results to check status
CONN_GRP.GenerateInitialContactResults()
print("Generated Initial Contact Results.")

# --- Cell 20 & 21: Add Analysis and Set Settings ---
Model.AddTransientStructuralAnalysis()
TRA_STRUC = Model.Analyses[0]
TRA_STRUC_SOLN = TRA_STRUC.Solution
TRA_STRUC_ANA_SETTING = TRA_STRUC.Children[0] # This is the Analysis Settings object

# Set robust Analysis Settings for this contact problem
TRA_STRUC_ANA_SETTING.NumberOfSteps = 1
TRA_STRUC_ANA_SETTING.StepEndTime = Quantity("1 [s]")
TRA_STRUC_ANA_SETTING.LargeDeflection = True # CRITICAL: Must be True for rotation/contact
TRA_STRUC_ANA_SETTING.AutomaticTimeStepping = Ansys.Mechanical.DataModel.Enums.AutomaticTimeStepping.On
TRA_STRUC_ANA_SETTING.InitialTimeStep = Quantity("0.001 [s]")
TRA_STRUC_ANA_SETTING.MinimumTimeStep = Quantity("1e-5 [s]")
TRA_STRUC_ANA_SETTING.MaximumTimeStep = Quantity("0.005 [s]")
TRA_STRUC_ANA_SETTING.NodalForces = Ansys.Mechanical.DataModel.Enums.OutputControlsNodalForcesType.Yes
print("Robust Transient Analysis Settings applied.")

# --- Cell 22: Apply Fixed Support ---
# This function applies the fixed support to the flat face of the cylinder
def apply_fixed_support(ns_name):
    print(f"--- Applying Fixed Support to '{ns_name}' ---")
    ns_obj = [ns for ns in Model.NamedSelections.Children if ns.Name == ns_name][0]
    fixed_support = TRA_STRUC.AddFixedSupport()
    selection = ExtAPI.SelectionManager.CreateSelectionInfo(SelectionTypeEnum.GeometryEntities)
    selection.Ids = ns_obj.Location.Ids
    fixed_support.Location = selection
    fixed_support.Suppressed = False
    print(f"Fixed Support applied successfully to '{ns_name}'.")

apply_fixed_support(ns_name="NS_Cara_Plana_Cilindro")

# --- Cell 23: Apply Moments ---
# This function applies the moments to the axis faces
def apply_moment(ns_name, magnitude, axis_vector):
    print(f"--- Applying moment to '{ns_name}' ---")
    ns_obj = [ns for ns in Model.NamedSelections.Children if ns.Name == ns_name][0]
    
    moment_load = TRA_STRUC.AddMoment()
    
    # Scope to the geometry faces
    selection_loc = ExtAPI.SelectionManager.CreateSelectionInfo(SelectionTypeEnum.GeometryEntities)
    selection_loc.Ids = ns_obj.Location.Ids
    moment_load.Location = selection_loc
    
    # Define by Vector
    moment_load.DefineBy = LoadDefineBy.Vector
    moment_load.Magnitude.Output.SetDiscreteValue(0, Quantity(magnitude, "N m"))
    
    # Set the direction vector
    vector = Ansys.ACT.Math.Vector3D(axis_vector[0], axis_vector[1], axis_vector[2])
    moment_load.Direction = vector
    
    moment_load.Suppressed = False
    print(f"Moment of {magnitude} N·m applied successfully.")

# Apply moments around the Global Z-axis (matches the joints)
vector_z_axis = [0, 0, 1]
apply_moment(ns_name="NS_Cara_Eje1", magnitude=1, axis_vector=vector_z_axis)
apply_moment(ns_name="NS_Cara_Eje2", magnitude=-1, axis_vector=vector_z_axis)

# --- Cell 24: Add Solution Results ---
# Add Total Deformation
total_deformation = TRA_STRUC_SOLN.AddTotalDeformation()
total_deformation.Name = "Total Deformation"

# Add Contact Tool to Solution
contact_tool = TRA_STRUC_SOLN.AddContactTool()
contact_tool.Name = "Contact Tool (Solution)"

# Add Contact Pressure results at 0s and 1s
pressure_0s = contact_tool.AddPressure()
pressure_0s.DisplayTime = Quantity("0 [s]")
pressure_0s.Name = "Contact Pressure at 0s"

pressure_1s = contact_tool.AddPressure()
pressure_1s.DisplayTime = Quantity("1 [s]")
pressure_1s.Name = "Contact Pressure at 1s"

# Add Force Reaction probes for each contact
for contact_region in contact_group.Children:
    if "ContactRegion" in contact_region.GetType().ToString():
        force_probe = TRA_STRUC_SOLN.AddForceReaction()
        force_probe.Location = contact_region
        force_probe.Name = f"Force Reaction - {contact_region.Name}"
print("Added Deformation, Contact, and Probe results to Solution.")

# --- Cell 25: Generate Mesh ---
MSH.ElementOrder = ElementOrder.Linear
MSH.ElementSize = Quantity("2 [mm]")
MSH.GenerateMesh()
print("Mesh generated.")

# --- Cell 26: Solve & Error Handling ---
# This is the final step. The script includes diagnostics to read
# the 'file.err' if the solver fails silently.

print("\n==============================\n  ATTEMPTING TO SOLVE\n==============================")
try:
    TRA_STRUC.Solve(block=True)
    print("The solution process has finished.")
    
    # If solve succeeds, generate result images
    print("\nGenerating result images...")
    
    # Activate and capture Total Deformation
    Tree.Activate([total_deformation])
    Graphics.Camera.SetFit()
    Graphics.ExportImage(os.path.join(cwd, "total_deformation.png"), GraphicsImageExportFormat.PNG, settings_720p)
    display_image("total_deformation.png")
    
    # Activate and capture Contact Pressure at 1s
    Tree.Activate([pressure_1s])
    Graphics.Camera.SetFit()
    Graphics.ExportImage(os.path.join(cwd, "contact_pressure.png"), GraphicsImageExportFormat.PNG, settings_720p)
    display_image("contact_pressure.png")

except Exception as e:
    print(f"\nPYTHON ERROR: The 'Solve' command failed with an exception: {e}")
    print("This may indicate a license, installation, or pre-solve setup issue.")

# --- Diagnostic: Check for Solver Error Files (file.err) ---
# This section runs after a solve attempt to find the *real* solver error.
print("\n" + "="*40)
print("  READING PRE-SOLVER ERROR FILES")
print("="*40)

try:
    working_dir = TRA_STRUC.Solution.WorkingDir
    print(f"Searching for error files in: {working_dir}\n")
    time.sleep(1) # Give filesystem time to update

    if not os.path.exists(working_dir):
        print("CRITICAL ERROR: The working directory does not exist.")
    else:
        all_files = os.listdir(working_dir)
        if not all_files:
            print("ERROR: The working directory is empty.")
            print("The failure occurred before any solver files were written.")
        
        # Try to read the most common error file
        target_error_file = 'file.err'
        file_path = os.path.join(working_dir, target_error_file)
        
        if os.path.exists(file_path):
            print(f"\n*** Found error file: '{target_error_file}'! ***")
            print("--- BEGIN ERROR MESSAGE ---")
            with open(file_path, 'r') as f:
                print(f.read())
            print("--- END ERROR MESSAGE ---")
            print("\nRECOMMENDATION: This is the real error. It is often a license issue or a material with no properties.")
        else:
            print(f"\nWARNING: Standard '{target_error_file}' not found.")
            print("If the solve failed, check other .err or .log files in the directory.")

except Exception as e:
    print(f"An error occurred while trying to read the error files: {e}")

# --- Cell 27: Final Tree Print ---
app.print_tree()