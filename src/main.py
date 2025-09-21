import numpy as np
import open3d as o3d

from OCC.Core.STEPControl import (
    STEPControl_Reader,
    STEPControl_Writer,
    STEPControl_AsIs,
)
from OCC.Core.TopExp import TopExp_Explorer
from OCC.Core.TopAbs import TopAbs_FACE, TopAbs_EDGE
from OCC.Core.BRepBndLib import brepbndlib_Add
from OCC.Core.Bnd import Bnd_Box
from OCC.Core.BRep import BRep_Tool
from OCC.Core.GeomAPI import GeomAPI_ProjectPointOnSurf

# The original code had an ImportError here.
# BRepGProp is the module, not a class/function to be imported from it.
# We import the module directly to use its methods, e.g., BRepGProp.SurfaceProperties().
import OCC.Core.BRepGProp as BRepGProp
from OCC.Core.GProp import GProp_GProps
from OCC.Core.BRepAdaptor import BRepAdaptor_Surface
from OCC.Core.GeomAbs import GeomAbs_Plane
from OCC.Core.gp import gp_Pnt, gp_Dir, gp_Ax1, gp_Trsf
from OCC.Core.BRepBuilderAPI import BRepBuilderAPI_Transform
from OCC.Core.Interface import Interface_Static
from OCC.Core.Quantity import Quantity_Color
import math
import tkinter as tk
from tkinter import filedialog, messagebox
import os


class STEPColoringProcessor:
    def __init__(self):
        self.shape = None
        self.faces = []
        self.edges = []
        self.bounding_box = None
        self.z0_face = None
        self.stock_dimensions = {
            "width": 48.0,  # inches
            "length": 167.26,  # inches
            "thickness_options": [2.0, 3.0, 4.0],  # inches
        }
        self.aux_spindle_offset = 16.0  # inches along X axis

        # Color definitions
        self.colors = {
            "pink": Quantity_Color(1.0, 0.8, 0.8, 0),  # Entire body
            "tan": Quantity_Color(0.8, 0.7, 0.5, 0),  # Z0 plane face
            "cyan": Quantity_Color(0.0, 1.0, 1.0, 0),  # Slot cuts
            "lime": Quantity_Color(0.5, 1.0, 0.0, 0),  # Primary cutter slots
            "yellow": Quantity_Color(1.0, 1.0, 0.0, 0),  # Aux spindles up
            "blue": Quantity_Color(0.0, 0.0, 1.0, 0),  # Closed loops
            "purple": Quantity_Color(0.5, 0.0, 0.5, 0),  # Open loops
        }

        # Depth-based colors for pockets
        self.depth_colors = {
            0.5: Quantity_Color(0.9, 0.9, 0.9, 0),  # 0.5" deep
            1.0: Quantity_Color(0.8, 0.8, 0.8, 0),  # 1.0" deep
            1.5: Quantity_Color(0.7, 0.7, 0.7, 0),  # 1.5" deep
            2.0: Quantity_Color(0.6, 0.6, 0.6, 0),  # 2.0" deep
            2.5: Quantity_Color(0.5, 0.5, 0.5, 0),  # 2.5" deep
            3.0: Quantity_Color(0.4, 0.4, 0.4, 0),  # 3.0" deep
            3.5: Quantity_Color(0.3, 0.3, 0.3, 0),  # 3.5" deep
        }

    def load_step_file(self, filepath):
        """Load STEP file and extract shape"""
        try:
            reader = STEPControl_Reader()
            reader.ReadFile(filepath)
            reader.TransferRoots()
            self.shape = reader.OneShape()

            if self.shape.IsNull():
                raise ValueError("Failed to load STEP file or empty shape")

            self._extract_faces_and_edges()
            print(f"Loaded STEP file: {filepath}")
            print(f"Found {len(self.faces)} faces and {len(self.edges)} edges")
            return True

        except Exception as e:
            print(f"Error loading STEP file: {e}")
            return False

    def _extract_faces_and_edges(self):
        """Extract all faces and edges from the shape"""
        self.faces = []
        self.edges = []

        # Extract faces
        face_explorer = TopExp_Explorer(self.shape, TopAbs_FACE)
        while face_explorer.More():
            self.faces.append(face_explorer.Current())
            face_explorer.Next()

        # Extract edges
        edge_explorer = TopExp_Explorer(self.shape, TopAbs_EDGE)
        while edge_explorer.More():
            self.edges.append(edge_explorer.Current())
            edge_explorer.Next()

    def calculate_bounding_box(self):
        """Calculate bounding box of the shape"""
        bbox = Bnd_Box()
        brepbndlib_Add(self.shape, bbox)

        xmin, ymin, zmin, xmax, ymax, zmax = bbox.Get()
        self.bounding_box = {
            "xmin": xmin,
            "ymin": ymin,
            "zmin": zmin,
            "xmax": xmax,
            "ymax": ymax,
            "zmax": zmax,
            "width": xmax - xmin,
            "height": ymax - ymin,
            "depth": zmax - zmin,
        }

        print(f"Bounding box: {self.bounding_box}")
        return self.bounding_box

    def get_face_area(self, face):
        """Calculate surface area of a face"""
        props = GProp_GProps()
        BRepGProp.SurfaceProperties(face, props)
        return props.Mass()

    def get_face_normal(self, face):
        """Get normal vector of a face"""
        surface = BRepAdaptor_Surface(face)
        if surface.GetType() == GeomAbs_Plane:
            plane = surface.Plane()
            normal = plane.Axis().Direction()
            return [normal.X(), normal.Y(), normal.Z()]
        return None

    def find_largest_face(self):
        """Find the face with the largest surface area"""
        largest_area = 0
        largest_face = None

        for face in self.faces:
            area = self.get_face_area(face)
            if area > largest_area:
                largest_area = area
                largest_face = face

        return largest_face, largest_area

    def orient_shape(self):
        """Orient the shape according to the specifications"""
        if not self.shape or not self.bounding_box:
            return False

        # Step 1: Find largest face and orient it perpendicular to Z axis at Z0
        largest_face, _ = self.find_largest_face()
        if not largest_face:
            return False

        # Get face normal
        normal = self.get_face_normal(largest_face)
        if not normal:
            return False

        # Create transformation to align face with XY plane
        transform = gp_Trsf()

        # If face normal is not aligned with Z axis, rotate it
        if abs(normal[2]) < 0.99:  # Not already aligned with Z
            # Calculate rotation axis and angle
            z_axis = gp_Dir(0, 0, 1)
            face_normal = gp_Dir(normal[0], normal[1], normal[2])

            if not face_normal.IsParallel(z_axis, 0.01):
                rotation_axis = face_normal.Crossed(z_axis)
                angle = face_normal.Angle(z_axis)

                axis = gp_Ax1(gp_Pnt(0, 0, 0), rotation_axis)
                transform.SetRotation(axis, angle)

        # Apply transformation
        transform_op = BRepBuilderAPI_Transform(self.shape, transform)
        self.shape = transform_op.Shape()

        # Recalculate bounding box after rotation
        self.calculate_bounding_box()

        # Step 2: Rotate about Z axis to align longest side with Y axis
        if self.bounding_box["width"] > self.bounding_box["height"]:
            # Need to rotate 90 degrees
            z_rotation = gp_Trsf()
            z_rotation.SetRotation(
                gp_Ax1(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1)), math.pi / 2
            )

            transform_op = BRepBuilderAPI_Transform(self.shape, z_rotation)
            self.shape = transform_op.Shape()
            self.calculate_bounding_box()

        # Step 3: Move origin to match machining origin
        # Move X+, Y-, Z- corner to origin, then offset X-24, Y+48
        translation = gp_Trsf()
        dx = -self.bounding_box["xmax"] - 24.0
        dy = -self.bounding_box["ymin"] + 48.0
        dz = -self.bounding_box["zmin"]

        translation.SetTranslation(gp_Pnt(0, 0, 0), gp_Pnt(dx, dy, dz))
        transform_op = BRepBuilderAPI_Transform(self.shape, translation)
        self.shape = transform_op.Shape()

        # Final bounding box calculation
        self.calculate_bounding_box()

        print("Shape oriented successfully")
        return True

    def identify_z0_face(self):
        """Identify the face at Z=0 (bottom face)"""
        tolerance = 0.001

        for face in self.faces:
            # Check if face is at Z=0
            bbox = Bnd_Box()
            brepbndlib_Add(face, bbox)
            _, _, zmin, _, _, zmax = bbox.Get()

            if abs(zmin) < tolerance and abs(zmax) < tolerance:
                # Check if face is horizontal (normal parallel to Z axis)
                normal = self.get_face_normal(face)
                if normal and abs(abs(normal[2]) - 1.0) < 0.01:
                    self.z0_face = face
                    print("Z0 face identified")
                    return face

        return None

    def find_slot_faces(self):
        """Find faces that represent slot cuts based on edge lengths"""
        if not self.z0_face:
            return []

        slot_faces = []
        target_lengths = [0.5, 3.187]  # inches ± 0.015
        tolerance = 0.015

        # Get edges of Z0 face
        z0_edges = []
        edge_explorer = TopExp_Explorer(self.z0_face, TopAbs_EDGE)
        while edge_explorer.More():
            z0_edges.append(edge_explorer.Current())
            edge_explorer.Next()

        # For each edge, find adjacent non-horizontal faces
        for edge in z0_edges:
            edge_length = self._get_edge_length(edge)

            # Check if edge length matches slot criteria
            for target_length in target_lengths:
                if abs(edge_length - target_length) <= tolerance:
                    # Find faces adjacent to this edge
                    adjacent_faces = self._find_faces_adjacent_to_edge(edge)

                    for face in adjacent_faces:
                        if face != self.z0_face and not self._is_face_horizontal(face):
                            slot_faces.append(face)
                            break  # Found a face for this edge, move to the next edge.

        print(f"Found {len(slot_faces)} slot faces")
        return slot_faces

    def _get_edge_length(self, edge):
        """Calculate length of an edge"""
        props = GProp_GProps()
        BRepGProp.LinearProperties(edge, props)
        return props.Mass()

    def _find_faces_adjacent_to_edge(self, edge):
        """Find all faces that share the given edge"""
        adjacent_faces = []

        for face in self.faces:
            edge_explorer = TopExp_Explorer(face, TopAbs_EDGE)
            while edge_explorer.More():
                if edge_explorer.Current().IsSame(edge):
                    adjacent_faces.append(face)
                    break
                edge_explorer.Next()

        return adjacent_faces

    def _is_face_horizontal(self, face):
        """Check if a face is horizontal (normal parallel to Z axis)"""
        normal = self.get_face_normal(face)
        if normal:
            return abs(abs(normal[2]) - 1.0) < 0.1
        return False

    def categorize_slots_for_spindles(self, slot_faces):
        """Categorize slots based on aux spindle positioning"""
        # TODO: Implement the logic to group slots based on the 16"
        # X-axis spacing requirement from the project PDF.
        # This will involve:
        # 1. Finding a representative point for each slot face.
        # 2. Grouping slots with 16" spacing along the X-axis.
        # 3. Applying the coloring rules (lime, cyan, yellow) based on the groupings
        #    and whether the aux spindle would cut outside the stock.

        lime_faces = []  # Primary cutter with aux down
        cyan_faces = []  # Aux spindles active
        yellow_faces = []  # Aux spindles up

        # Placeholder - assign all slot faces to cyan
        cyan_faces = slot_faces

        print(
            f"Slot categorization: {len(lime_faces)} lime, {len(cyan_faces)} cyan, {len(yellow_faces)} yellow"
        )
        return lime_faces, cyan_faces, yellow_faces

    def find_closed_loop_faces(self):
        """Find faces adjacent to closed loops on Z0 plane"""
        # TODO: Implement logic to find faces adjacent to closed loops
        # that are not already identified as slots.
        return []

    def find_open_loop_faces(self):
        """Find faces adjacent to open loops intersecting bounding box"""
        # TODO: Implement logic to find faces adjacent to open loops
        # that intersect the bounding box.
        return []

    def categorize_pocket_faces_by_depth(self):
        """Categorize horizontal pocket faces by depth from max Z"""
        if not self.bounding_box:
            return {}

        pocket_faces = {}
        max_z = self.bounding_box["zmax"]

        for face in self.faces:
            if self._is_face_horizontal(face):
                # Get face Z coordinate
                bbox = Bnd_Box()
                brepbndlib_Add(face, bbox)
                _, _, zmin, _, _, zmax = bbox.Get()
                face_z = (zmin + zmax) / 2

                depth_from_top = max_z - face_z

                # Round to nearest 0.5" increment
                depth_increment = round(depth_from_top * 2) / 2

                if depth_increment > 0 and depth_increment <= 3.5:
                    if depth_increment not in pocket_faces:
                        pocket_faces[depth_increment] = []
                    pocket_faces[depth_increment].append(face)

        return pocket_faces

    def apply_colors(self):
        """Apply colors to faces according to the specification"""
        if not self.shape:
            return False

        print("Starting coloring process...")

        # Step 1: Color entire body pink (base color)
        # This would be applied to the shape as a whole.
        # You'll need to use a library like `open3d` or `pyvista` for visualization
        # or find an equivalent coloring method in pythonocc.

        # Step 2: Identify and color Z0 face tan
        z0_face = self.identify_z0_face()
        if z0_face:
            # TODO: Apply tan color to Z0 face.
            pass

        # Step 3: Find and color slot faces cyan
        slot_faces = self.find_slot_faces()
        lime_faces, cyan_faces, yellow_faces = self.categorize_slots_for_spindles(
            slot_faces
        )
        # TODO: Apply lime, cyan, and yellow colors to the respective faces.

        # Step 4: Color closed loop faces blue
        closed_loop_faces = self.find_closed_loop_faces()
        # TODO: Apply blue color to closed loop faces.

        # Step 5: Color open loop faces purple
        open_loop_faces = self.find_open_loop_faces()
        # TODO: Apply purple color to open loop faces.

        # Step 6: Color pocket faces by depth
        pocket_faces = self.categorize_pocket_faces_by_depth()
        # TODO: Apply the grayscale depth colors to the pocket faces.

        print("Coloring process completed")
        return True

    def save_colored_step(self, output_path):
        """Save the colored STEP file"""
        # This function seems to be missing the code to actually apply the colors
        # to the shape before writing it. You will need to use a library to
        # set the colors of the faces. This is a complex task with OCC.
        if not self.shape:
            return False

        try:
            writer = STEPControl_Writer()
            writer.Transfer(self.shape, STEPControl_AsIs)
            status = writer.Write(output_path)

            if status == 1:  # Success
                print(f"Colored STEP file saved: {output_path}")
                return True
            else:
                print("Failed to save STEP file")
                return False

        except Exception as e:
            print(f"Error saving STEP file: {e}")
            return False

    def process_step_file(self, input_path, output_path):
        """Complete processing pipeline"""
        print(f"Processing STEP file: {input_path}")

        # Load file
        if not self.load_step_file(input_path):
            return False

        # Calculate bounding box
        self.calculate_bounding_box()

        # Orient shape
        if not self.orient_shape():
            print("Failed to orient shape")
            return False

        # Apply colors
        if not self.apply_colors():
            print("Failed to apply colors")
            return False

        # Save result
        if not self.save_colored_step(output_path):
            print("Failed to save colored file")
            return False

        print("Processing completed successfully!")
        return True


class STEPColoringGUI:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("STEP File Coloring Tool")
        self.root.geometry("600x400")

        self.processor = STEPColoringProcessor()

        self.create_widgets()

    def create_widgets(self):
        # Input file selection
        input_frame = tk.Frame(self.root)
        input_frame.pack(pady=10, padx=20, fill=tk.X)

        tk.Label(input_frame, text="Input STEP File:").pack(anchor=tk.W)

        input_select_frame = tk.Frame(input_frame)
        input_select_frame.pack(fill=tk.X, pady=5)

        self.input_path_var = tk.StringVar()
        self.input_entry = tk.Entry(
            input_select_frame, textvariable=self.input_path_var, width=50
        )
        self.input_entry.pack(side=tk.LEFT, fill=tk.X, expand=True)

        tk.Button(
            input_select_frame, text="Browse", command=self.select_input_file
        ).pack(side=tk.RIGHT, padx=(5, 0))

        # Output file selection
        output_frame = tk.Frame(self.root)
        output_frame.pack(pady=10, padx=20, fill=tk.X)

        tk.Label(output_frame, text="Output STEP File:").pack(anchor=tk.W)

        output_select_frame = tk.Frame(output_frame)
        output_select_frame.pack(fill=tk.X, pady=5)

        self.output_path_var = tk.StringVar()
        self.output_entry = tk.Entry(
            output_select_frame, textvariable=self.output_path_var, width=50
        )
        self.output_entry.pack(side=tk.LEFT, fill=tk.X, expand=True)

        tk.Button(
            output_select_frame, text="Browse", command=self.select_output_file
        ).pack(side=tk.RIGHT, padx=(5, 0))

        # Process button
        process_frame = tk.Frame(self.root)
        process_frame.pack(pady=20)

        self.process_button = tk.Button(
            process_frame,
            text="Process STEP File",
            command=self.process_file,
            bg="#4CAF50",
            fg="white",
            font=("Arial", 12, "bold"),
            padx=20,
            pady=10,
        )
        self.process_button.pack()

        # Progress/status area
        status_frame = tk.Frame(self.root)
        status_frame.pack(pady=10, padx=20, fill=tk.BOTH, expand=True)

        tk.Label(status_frame, text="Status:").pack(anchor=tk.W)

        self.status_text = tk.Text(status_frame, height=15, wrap=tk.WORD)
        self.status_text.pack(fill=tk.BOTH, expand=True, pady=5)

        scrollbar = tk.Scrollbar(
            status_frame, orient=tk.VERTICAL, command=self.status_text.yview
        )
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        self.status_text.config(yscrollcommand=scrollbar.set)

    def select_input_file(self):
        filename = filedialog.askopenfilename(
            title="Select Input STEP File",
            filetypes=[("STEP files", "*.step *.stp"), ("All files", "*.*")],
        )
        if filename:
            self.input_path_var.set(filename)

            # Auto-generate output filename
            base_name = os.path.splitext(filename)[0]
            output_name = f"{base_name}_colored.step"
            self.output_path_var.set(output_name)

    def select_output_file(self):
        filename = filedialog.asksaveasfilename(
            title="Save Colored STEP File As",
            defaultextension=".step",
            filetypes=[("STEP files", "*.step"), ("All files", "*.*")],
        )
        if filename:
            self.output_path_var.set(filename)

    def log_status(self, message):
        """Add message to status text area"""
        self.status_text.insert(tk.END, message + "\n")
        self.status_text.see(tk.END)
        self.root.update()

    def process_file(self):
        input_path = self.input_path_var.get()
        output_path = self.output_path_var.get()

        if not input_path or not output_path:
            messagebox.showerror("Error", "Please select both input and output files")
            return

        if not os.path.exists(input_path):
            messagebox.showerror("Error", "Input file does not exist")
            return

        # Clear status
        self.status_text.delete(1.0, tk.END)

        # Disable button during processing
        self.process_button.config(state=tk.DISABLED)

        try:
            # Redirect print to status area (simplified)
            import sys

            original_stdout = sys.stdout

            class StatusRedirect:
                def __init__(self, gui):
                    self.gui = gui

                def write(self, text):
                    if text.strip():
                        self.gui.log_status(text.strip())

                def flush(self):
                    pass

            sys.stdout = StatusRedirect(self)

            # Process file
            success = self.processor.process_step_file(input_path, output_path)

            # Restore stdout
            sys.stdout = original_stdout

            if success:
                messagebox.showinfo("Success", "STEP file processed successfully!")
            else:
                messagebox.showerror("Error", "Failed to process STEP file")

        except Exception as e:
            sys.stdout = original_stdout
            messagebox.showerror("Error", f"An error occurred: {str(e)}")

        finally:
            # Re-enable button
            self.process_button.config(state=tk.NORMAL)

    def run(self):
        self.root.mainloop()


def main():
    """Main entry point"""
    # You can run either GUI or command line version

    # For GUI version:
    gui = STEPColoringGUI()
    gui.run()

    # For command line version (uncomment to use):
    # if len(sys.argv) != 3:
    #     print("Usage: python main.py <input_step_file> <output_step_file>")
    #     sys.exit(1)
    #
    # processor = STEPColoringProcessor()
    # success = processor.process_step_file(sys.argv[1], sys.argv[2])
    # sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()
