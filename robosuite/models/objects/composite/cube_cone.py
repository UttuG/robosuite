import numpy as np

from robosuite.models.objects import CompositeObject
from robosuite.utils.mjcf_utils import RED, CustomMaterial, add_to_dict


class CubeConeObject(CompositeObject):
    """
    Generates a composite object with a cube base and a cone top.
    
    Args:
        name (str): Name of this CubeCone object
        cube_size (float or 3-tuple): Size of the cube base. If float, creates a cube.
            If 3-tuple, specifies (half-x, half-y, half-z) dimensions.
        cone_base_radius (float): Radius of the cone base (should roughly match cube top)
        cone_tip_radius (float): Radius of the cone tip
        cone_height (float): Height of the cone
        cone_ngeoms (int): Number of cylinder geoms used to approximate the cone.
            More geoms = better approximation.
        rgba (4-tuple): RGBA color for the object
        material (CustomMaterial): Optional material for the object
        density (float): Density of the object
        solref (2-tuple): MuJoCo solver reference parameters
        solimp (3-tuple): MuJoCo solver impedance parameters
        friction (3-tuple): Friction parameters (sliding, torsional, rolling)
    """

    def __init__(
        self,
        name,
        cube_size=0.04,
        cone_base_radius=0.04,
        cone_tip_radius=0.02,
        cone_height=0.04,
        cone_ngeoms=7,
        rgba=None,
        material=None,
        density=1000.0,
        solref=(0.02, 1.0),
        solimp=(0.9, 0.95, 0.001),
        friction=None,
    ):
        # Set object attributes
        self._name = name
        self.rgba = rgba if rgba is not None else [0.0, 0.5, 0.8, 1.0]
        self.density = density
        self.friction = friction if friction is None else np.array(friction)
        self.solref = solref
        self.solimp = solimp

        self.has_material = material is not None
        if self.has_material:
            assert isinstance(material, CustomMaterial)
            self.material = material

        # Cube parameters
        if isinstance(cube_size, (int, float)):
            self.cube_half_size = np.array([cube_size, cube_size, cube_size])
        else:
            self.cube_half_size = np.array(cube_size)

        # Cone parameters
        self.cone_base_radius = cone_base_radius
        self.cone_tip_radius = cone_tip_radius
        self.cone_height = cone_height
        
        # Number of geoms for cone approximation (use odd number for easier computation)
        if cone_ngeoms % 2 == 0:
            cone_ngeoms += 1
        self.cone_ngeoms = cone_ngeoms
        
        # Unit measurements for cone construction
        self.cone_unit_height = (cone_height / cone_ngeoms) / 2.0
        self.cone_unit_radius = (cone_base_radius - cone_tip_radius) / (cone_ngeoms - 1)

        # Calculate total bounding box
        max_radius = max(np.max(self.cube_half_size[:2]), cone_base_radius)
        total_height = self.cube_half_size[2] + cone_height
        
        # Other private attributes
        self._important_sites = {}

        # Create dictionary of values to create geoms for composite object and run super init
        super().__init__(
            total_size=[max_radius, max_radius, total_height / 2.0],
            **self._get_geom_attrs()
        )

        # Optionally add material
        if self.has_material:
            self.append_material(self.material)

    def _get_geom_attrs(self):
        """
        Creates geom elements for both cube base and cone top
        
        Returns:
            dict: args to be used by CompositeObject to generate geoms
        """
        # Initialize base args
        base_args = {
            "name": self.name,
            "locations_relative_to_center": True,
            "obj_types": "all",
            "density": self.density,
            "solref": self.solref,
            "solimp": self.solimp,
        }
        
        # Initialize empty dict for geom attributes (to be populated by add_to_dict)
        obj_args = {}

        # Cube base position (centered at origin in x-y, positioned below center in z)
        cube_z_pos = -self.cone_height / 2.0

        # Add cube base geom
        add_to_dict(
            dic=obj_args,
            geom_types="box",
            geom_locations=(0.0, 0.0, cube_z_pos),
            geom_quats=None,
            geom_sizes=tuple(self.cube_half_size),
            geom_names="cube_base",
            geom_rgbas=self.rgba,
            geom_materials=self.material.mat_attrib["name"] if self.has_material else None,
            geom_frictions=self.friction,
            geom_condims=4,
        )

        # Add cone geoms on top of cube
        # Stack cylinders in z-direction to approximate cone
        ngeoms_each_side = (self.cone_ngeoms - 1) // 2
        cone_start_z = cube_z_pos + self.cube_half_size[2]
        
        # Cone geom locations
        geom_locations = [
            (0.0, 0.0, cone_start_z + i * self.cone_unit_height * 2.0) 
            for i in range(-ngeoms_each_side, ngeoms_each_side + 1)
        ]

        # Cone geom sizes (cylinders with decreasing radius)
        geom_sizes = [
            (
                self.cone_tip_radius + i * self.cone_unit_radius,
                self.cone_unit_height,
            )
            for i in range(self.cone_ngeoms)
        ][::-1]  # Reverse to go from large to small

        # Add each cone cylinder geom
        for i in range(self.cone_ngeoms):
            add_to_dict(
                dic=obj_args,
                geom_types="cylinder",
                geom_locations=geom_locations[i],
                geom_quats=None,
                geom_sizes=geom_sizes[i],
                geom_names=f"cone_{i}",
                geom_rgbas=self.rgba,
                geom_materials=self.material.mat_attrib["name"] if self.has_material else None,
                geom_frictions=self.friction,
                geom_condims=4,
            )

        # Add sites for reference points
        obj_args["sites"] = [
            {
                "name": "center",
                "pos": (0, 0, 0),
                "size": "0.002",
                "rgba": RED,
                "type": "sphere",
            },
            {
                "name": "top",
                "pos": (0, 0, cone_start_z + self.cone_height),
                "size": "0.002",
                "rgba": RED,
                "type": "sphere",
            },
        ]

        # Merge base_args into obj_args
        obj_args.update(base_args)

        return obj_args

    @property
    def important_sites(self):
        """
        Returns:
            dict: important sites for this object
        """
        return {
            "center": self.naming_prefix + "center",
            "top": self.naming_prefix + "top",
        }
