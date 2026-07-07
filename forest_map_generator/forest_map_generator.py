#!/usr/bin/env python3
import os
import cv2
import math
import rclpy
import random
import numpy as np
import xml.etree.ElementTree as ET

from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory


# Terrain helper class
class TerrainHelper:
    def __init__(self, node):
        self.node = node
        self.get_logger = node.get_logger

        package_share_dir = get_package_share_directory("forest_map_generator")
        sdf_file_path = os.path.join(
            package_share_dir, "models", "terrain", "materials", "model.sdf"
        )
        if not os.path.exists(sdf_file_path):
            self.get_logger().warn(f"SDF file {sdf_file_path} does not exist.")
            sdf_file_path = os.path.join(
                package_share_dir, "models", "terrain", "model.sdf"
            )

        self.terrain_sdf_metadata = self.load_terrain_sdf_metadata(sdf_file_path)
        if self.terrain_sdf_metadata is None:
            self.get_logger().error("Failed to load terrain_sdf_metadata.")
            return

        (
            terrain_world_size_x,
            terrain_world_size_y,
            terrain_world_size_z,
        ), (
            terrain_world_pos_x,
            terrain_world_pos_y,
            terrain_world_pos_z,
        ), min_height_second_layer, min_height_third_layer, heightmap_file = self.terrain_sdf_metadata

        self.heightmap_file = heightmap_file
        self.package_path = node.package_path
        self.terrain_size_x = terrain_world_size_x
        self.terrain_size_y = terrain_world_size_y
        self.terrain_size_z = terrain_world_size_z
        self.terrain_world_pos_x = terrain_world_pos_x
        self.terrain_world_pos_y = terrain_world_pos_y
        self.terrain_world_pos_z = terrain_world_pos_z
        self.min_height_second_layer = min_height_second_layer
        self.min_height_third_layer = min_height_third_layer
        self.max_slope = node.max_slope

        self.heightmap_data = self.load_heightmap()

    def load_heightmap(self):
        heightmap_path = os.path.join(self.package_path, "models", *self.heightmap_file)
        if not os.path.exists(heightmap_path):
            self.get_logger().error(f"Heightmap file {heightmap_path} does not exist.")
            return None

        try:
            heightmap_data = cv2.imread(heightmap_path, cv2.IMREAD_GRAYSCALE)
            if heightmap_data is None:
                self.get_logger().error(
                    f"Failed to load heightmap image from {heightmap_path}."
                )
                return None
            heightmap_data = np.array(heightmap_data, dtype=np.float32)
            self.heightmap_data = heightmap_data
            return self.heightmap_data
        except Exception as e:
            self.get_logger().error(f"Error loading heightmap: {e}")
            return None

    @staticmethod
    def parse_sdf_vector3(value, tag_name):
        if value is None:
            raise ValueError(f"Missing <{tag_name}> tag")

        vector = [float(component) for component in value.split()]
        if len(vector) < 3:
            raise ValueError(f"<{tag_name}> must contain at least 3 values")

        return vector[:3]

    @staticmethod
    def find_sdf_text(root, paths):
        for path in paths:
            value = root.findtext(path)
            if value is not None:
                return value
        return None

    def load_terrain_sdf_metadata(self, sdf_file_path):
        try:
            root = ET.parse(sdf_file_path).getroot()
            size_text = self.find_sdf_text(
                root,
                [
                    ".//link[@name='link']/visual/size",
                    ".//link[@name='link']/visual/geometry/heightmap/size",
                    ".//visual/geometry/heightmap/size",
                ],
            )
            pos_text = self.find_sdf_text(
                root,
                [
                    ".//link[@name='link']/visual/pos",
                    ".//link[@name='link']/visual/pose",
                    ".//link[@name='link']/visual/geometry/heightmap/pos",
                    ".//link[@name='link']/visual/geometry/heightmap/pose",
                    ".//visual/geometry/heightmap/pos",
                    ".//visual/geometry/heightmap/pose",
                ],
            )
            terrain_world_size = self.parse_sdf_vector3(size_text, "size")
            terrain_world_pos = self.parse_sdf_vector3(pos_text, "pos")

            # terrain design (in model.sdf) has three blend layers: 1st Dirt, 2nd Grass, 3rd Fungi
            # getting the minimum height for the second and third layer
            min_height = root.findtext(
                ".//link[@name='link']/visual/geometry/heightmap/blend[1]/min_height",
            )
            fade_dist = root.findtext(".//link[@name='link']/visual/geometry/heightmap/blend[1]/fade_dist")
            min_height_second_layer = float(min_height) + float(fade_dist) + np.floor(0.1 * terrain_world_size[-1])
            min_height = root.findtext(
                ".//link[@name='link']/visual/geometry/heightmap/blend[2]/min_height",
            )
            fade_dist = root.findtext(".//link[@name='link']/visual/geometry/heightmap/blend[2]/fade_dist")
            min_height_third_layer = float(min_height) + float(fade_dist) + np.floor(0.1 * terrain_world_size[-1])

            heightmap_file = root.findtext(".//link[@name='link']/visual/geometry/heightmap/uri").split("/")[2:]

            self.terrain_sdf_metadata = (
                terrain_world_size, terrain_world_pos,
                min_height_second_layer, min_height_third_layer,
                heightmap_file
            )
            return self.terrain_sdf_metadata
        except (OSError, ET.ParseError, ValueError) as e:
            self.get_logger().error(f"Error loading terrain SDF metadata: {e}")
            return None

    def calculate_scope(self, px, py, radius=2):
        if self.heightmap_data is None:
            return self.max_slope

        if (
            px < radius
            or px >= self.heightmap_data.shape[1] - radius
            or py < radius
            or py >= self.heightmap_data.shape[0] - radius
        ):
            return self.max_slope

        dz_dx = (self.heightmap_data[py, px + radius] - self.heightmap_data[py, px - radius])
        dz_dy = (self.heightmap_data[py + radius, px] - self.heightmap_data[py - radius, px])
        slope_angle = math.degrees(math.atan2(dz_dy, dz_dx))
        return slope_angle

    def pixel_to_world(self, px, py):
        if self.heightmap_data is None:
            self.get_logger().error("No heightmap data for coordinate conversion.")
            return 0.0, 0.0, 0.0

        height, width = self.heightmap_data.shape

        world_x = (
            self.terrain_world_pos_x
            - self.terrain_size_x / 2.0
            + px * self.terrain_size_x / (width - 1)
        )
        world_y = (
            self.terrain_world_pos_y
            + self.terrain_size_y / 2.0
            - py * self.terrain_size_y / (height - 1)
        )
        height_value = self.heightmap_data[py, px]
        world_z = (
            self.terrain_world_pos_z
            + (height_value / np.max(self.heightmap_data)) * self.terrain_size_z
        )
        return world_x, world_y, world_z

    def world_to_pixel(self, world_x, world_y):
        if self.heightmap_data is None:
            self.get_logger().error("No heightmap data for coordinate conversion.")
            return 0, 0

        height, width = self.heightmap_data.shape

        px = round(
            (world_x - (self.terrain_world_pos_x - self.terrain_size_x / 2.0))
            * (width - 1)
            / self.terrain_size_x
        )
        py = round(
            ((self.terrain_world_pos_y + self.terrain_size_y / 2.0) - world_y)
            * (height - 1)
            / self.terrain_size_y
        )

        px = max(0, min(width - 1, px))
        py = max(0, min(height - 1, py))
        return px, py


# Tree generation class
class TreeGenerator(TerrainHelper):
    def __init__(self, node):
        super().__init__(node)

        self.num_trees = node.num_trees
        self.tree_types = node.tree_types
        self.min_tree_distance = node.min_tree_distance
        self.plant_tree_above_dirt = node.plant_tree_above_dirt

    def is_valid_tree_position(self, px, py, trees):
        if (
            px < 10
            or px >= self.heightmap_data.shape[1] - 10
            or py < 10
            or py >= self.heightmap_data.shape[0] - 10
        ):
            return False

        slope = self.calculate_scope(px, py)
        if slope >= self.max_slope:
            return False

        if self.plant_tree_above_dirt and self.heightmap_data[py, px] < self.min_height_second_layer:
            self.get_logger().info(
                f"""Minimum allowed height to plant is above dirt level at {self.min_height_second_layer}, 
                    the current height at {px}, {py} is {self.heightmap_data[py, px]}"""
            )
            return False

        for tree_x, tree_y, _ in trees:
            dist = math.sqrt((px - tree_x) ** 2 + (py - tree_y) ** 2)
            if dist < self.min_tree_distance:
                return False

        return True

    @staticmethod
    def create_tree_include_xml(tree_type, world_x, world_y, world_z, tree_id):
        yaw = random.uniform(0, 2 * math.pi)
        tree_xml = f"""
        <include>
            <name>{tree_type}_{tree_id}</name>
            <uri>model://{tree_type}</uri>
            <pose>{world_x} {world_y} {world_z} 0 0 {yaw}</pose>
        </include>
        """
        return tree_xml

    def generate_trees(self):
        self.get_logger().info(
            f"Generating {self.num_trees} trees on heightmap {self.heightmap_file[-1]}..."
        )

        if self.heightmap_data is None:
            self.get_logger().error(
                "Heightmap data could not be loaded. Aborting tree generation."
            )
            return []

        self.get_logger().info(f"Heightmap dimensions: {self.heightmap_data.shape}")
        self.get_logger().info(
            f"Heightmap value range: {np.min(self.heightmap_data)} to {np.max(self.heightmap_data)}"
        )

        trees = []
        attempts = 0
        max_attempts = self.num_trees * 10

        while len(trees) < self.num_trees and attempts < max_attempts:
            attempts += 1
            px = random.randint(0, self.heightmap_data.shape[1] - 1)
            py = random.randint(0, self.heightmap_data.shape[0] - 1)

            if self.is_valid_tree_position(px, py, trees):
                tree_type = random.choice(self.tree_types)
                trees.append((px, py, tree_type))
                world_x, world_y, world_z = self.pixel_to_world(px, py)
                self.get_logger().info(
                    f"Placed tree {len(trees)}/{self.num_trees} at ({px}, {py}), World: ({world_x:.2f}, {world_y:.2f}, {world_z:.2f})"
                )

        self.get_logger().info(f"Tree placement completed. {len(trees)} trees placed.")
        return trees

    def generate_trees_xml(self, trees):
        trees_xml = "\n    <!-- Auto-generated trees -->\n"
        for i, (px, py, tree_type) in enumerate(trees):
            world_x, world_y, world_z = self.pixel_to_world(px, py)
            trees_xml += self.create_tree_include_xml(
                tree_type, world_x, world_y, world_z, i
            )
        trees_xml += "    <!-- End auto-generated trees -->\n"
        return trees_xml


# Fire generation class
class FireGenerator(TerrainHelper):
    MIN_FIRE_SIZE_LIMIT = 1.0
    MAX_FIRE_SIZE_LIMIT = 50.0
    FIRE_MODEL_PLACEMENT_PADDING = 10.0
    FIRE_MODEL_SIZE_STEP = 10.0
    FIRE_MODEL_Z_OFFSET = 0.25
    MIN_PARTICLE_PARAMETERS = {
        "lifetime": 5.0,
        "min_velocity": 0.1,
        "max_velocity": 0.2,
        "scale_rate": 0.3,
        "rate": 10.0,
        "particle_scatter_ratio": 0.2,
    }
    MAX_PARTICLE_PARAMETERS = {
        "lifetime": 20.0,
        "min_velocity": 1.0,
        "max_velocity": 3.0,
        "scale_rate": 1.0,
        "rate": 100.0,
        "particle_scatter_ratio": 1.0,
    }

    def __init__(self, node):
        super().__init__(node)

        self.num_fires = node.num_fires
        self.min_fire_distance = node.min_fire_distance
        self.plant_fire_above_dirt = node.plant_fire_above_dirt
        self.min_fire_size = node.min_fire_size
        self.max_fire_size = node.max_fire_size

    def is_valid_fire_position(self, px, py, fires):
        if (
            px < 10
            or px >= self.heightmap_data.shape[1] - 10
            or py < 10
            or py >= self.heightmap_data.shape[0] - 10
        ):
            return False

        if (
            self.plant_fire_above_dirt
            and self.heightmap_data[py, px] < self.min_height_second_layer
        ):
            self.get_logger().info(
                f"""Minimum allowed height for fire is above dirt level at {self.min_height_second_layer}, 
                    the current height at {px}, {py} is {self.heightmap_data[py, px]}"""
            )
            return False

        for fire_x, fire_y in fires:
            dist = math.sqrt((px - fire_x) ** 2 + (py - fire_y) ** 2)
            if dist < self.min_fire_distance:
                return False

        return True

    @staticmethod
    def create_particle_emitter_plugin_xml():
        return (
            """
                <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"></plugin>
                <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"></plugin>
                <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"></plugin>
                <plugin filename="gz-sim-particle-emitter-system" name="gz::sim::systems::ParticleEmitter"></plugin>
            """
        )

    @classmethod
    def interpolate_particle_parameter(cls, fire_size, parameter_name):
        fire_size = max(
            cls.MIN_FIRE_SIZE_LIMIT,
            min(cls.MAX_FIRE_SIZE_LIMIT, fire_size),
        )
        size_ratio = (
            (fire_size - cls.MIN_FIRE_SIZE_LIMIT)
            / (cls.MAX_FIRE_SIZE_LIMIT - cls.MIN_FIRE_SIZE_LIMIT)
        )
        min_value = cls.MIN_PARTICLE_PARAMETERS[parameter_name]
        max_value = cls.MAX_PARTICLE_PARAMETERS[parameter_name]
        return min_value + size_ratio * (max_value - min_value)

    @classmethod
    def calculate_particle_parameters(cls, fire_size):
        return {
            parameter_name: cls.interpolate_particle_parameter(
                fire_size, parameter_name
            )
            for parameter_name in cls.MIN_PARTICLE_PARAMETERS
        }

    @classmethod
    def calculate_fire_model_count(cls, fire_size):
        return max(1, math.ceil(fire_size / cls.FIRE_MODEL_SIZE_STEP))

    @classmethod
    def calculate_fire_model_padding(cls, fire_size):
        # A 10 m margin cannot fit inside small smoke squares, so keep those random.
        if fire_size <= 2 * cls.FIRE_MODEL_PLACEMENT_PADDING:
            return fire_size * 0.1
        return cls.FIRE_MODEL_PLACEMENT_PADDING

    def create_fire_model_includes_xml(
        self, world_x, world_y, fire_id, fire_size
    ):
        fire_model_count = self.calculate_fire_model_count(fire_size)
        placement_padding = self.calculate_fire_model_padding(fire_size)
        max_offset = max(0.0, fire_size / 2.0 - placement_padding)

        fire_models_xml = ""
        for model_id in range(fire_model_count):
            model_x = world_x + random.uniform(-max_offset, max_offset)
            model_y = world_y + random.uniform(-max_offset, max_offset)
            model_px, model_py = self.world_to_pixel(model_x, model_y)
            _, _, model_z = self.pixel_to_world(model_px, model_py)
            model_z += self.FIRE_MODEL_Z_OFFSET
            yaw = random.uniform(0, 2 * math.pi)
            fire_models_xml += f"""
        <include>
            <name>fire_model_{fire_id}_{model_id}</name>
            <uri>model://fire_model</uri>
            <pose>{model_x} {model_y} {model_z} 0 0 {yaw}</pose>
        </include>
"""
        return fire_models_xml

    def create_fire_particle_emitter_xml(
        self, world_x, world_y, world_z, fire_id, fire_size, color_range_image
    ):
        smoke_z = world_z + self.FIRE_MODEL_Z_OFFSET
        particle_parameters = self.calculate_particle_parameters(fire_size)
        fire_models_xml = self.create_fire_model_includes_xml(
            world_x, world_y, fire_id, fire_size
        )
        fire_xml = f"""
{fire_models_xml}
        <model name="fire_smoke_{fire_id}">
            <pose>{world_x} {world_y} {smoke_z} 0 -1.5707 0</pose>
            <static>true</static>
            <link name="smoke_link">
                <particle_emitter name="emitter" type="box">
                    <emitting>true</emitting>
                    <size>{fire_size} {fire_size} 0</size>
                    <particle_size>0.6 0.6 0.6</particle_size>
                    <lifetime>{particle_parameters["lifetime"]}</lifetime>
                    <min_velocity>{particle_parameters["min_velocity"]}</min_velocity>
                    <max_velocity>{particle_parameters["max_velocity"]}</max_velocity>
                    <scale_rate>{particle_parameters["scale_rate"]}</scale_rate>
                    <rate>{particle_parameters["rate"]}</rate>
                    <particle_scatter_ratio>{particle_parameters["particle_scatter_ratio"]}</particle_scatter_ratio>
                    <material>
                        <diffuse>0.7 0.7 0.7</diffuse>
                        <specular>0.2 0.2 0.2</specular>
                        <pbr>
                            <metal>
                                <albedo_map>model://fog_generator/materials/textures/fog.png</albedo_map>
                            </metal>
                        </pbr>
                    </material>
                    <color_range_image>{color_range_image}</color_range_image>
                </particle_emitter>
            </link>
        </model>
        """
        return fire_xml

    def get_fire_size_range(self):
        min_fire_size, max_fire_size = sorted(
            [self.min_fire_size, self.max_fire_size]
        )
        min_fire_size = max(
            self.MIN_FIRE_SIZE_LIMIT,
            min(self.MAX_FIRE_SIZE_LIMIT, min_fire_size),
        )
        max_fire_size = max(
            self.MIN_FIRE_SIZE_LIMIT,
            min(self.MAX_FIRE_SIZE_LIMIT, max_fire_size),
        )
        return min_fire_size, max_fire_size

    def generate_fires(self):
        self.get_logger().info(
            f"Generating {self.num_fires} fires on heightmap {self.heightmap_file[-1]}..."
        )

        if self.heightmap_data is None:
            self.get_logger().error(
                "Heightmap data could not be loaded. Aborting fire generation."
            )
            return []

        fires = []
        attempts = 0
        max_attempts = self.num_fires * 10

        while len(fires) < self.num_fires and attempts < max_attempts:
            attempts += 1
            px = random.randint(0, self.heightmap_data.shape[1] - 1)
            py = random.randint(0, self.heightmap_data.shape[0] - 1)

            if self.is_valid_fire_position(px, py, fires):
                fires.append((px, py))
                world_x, world_y, world_z = self.pixel_to_world(px, py)
                self.get_logger().info(
                    f"Placed fire {len(fires)}/{self.num_fires} at ({px}, {py}), World: ({world_x:.2f}, {world_y:.2f}, {world_z:.2f})"
                )

        self.get_logger().info(f"Fire placement completed. {len(fires)} fires placed.")
        return fires

    def generate_fires_xml(self, fires):
        fires_xml = "\n    <!-- Auto-generated fire smoke particle emitters -->\n"
        color_range_images = [
            os.path.join(
                self.package_path,
                "models",
                "fog_generator",
                "materials",
                "textures",
                color_range_image,
            )
            for color_range_image in ["fogcolors.png", "smokecolors.png"]
        ]
        min_fire_size, max_fire_size = self.get_fire_size_range()
        for i, (px, py) in enumerate(fires):
            world_x, world_y, world_z = self.pixel_to_world(px, py)
            fire_size = random.uniform(min_fire_size, max_fire_size)
            color_range_image = random.choice(color_range_images)
            fires_xml += self.create_fire_particle_emitter_xml(
                world_x, world_y, world_z, i, fire_size, color_range_image
            )
        fires_xml += "    <!-- End auto-generated fire smoke particle emitters -->\n"
        return fires_xml


# Main node
class ForestMapGenerator(Node):
    def __init__(self):
        super().__init__("forest_map_generator")
        self.get_logger().info("Forest Map Generator Node started.")

        self.declare_parameter("num_trees", 50)
        self.declare_parameter("tree_types", ["oak_tree", "pine_tree"])
        self.declare_parameter("min_tree_distance", 5.0)
        self.declare_parameter("max_slope", 30.0)
        self.declare_parameter("output_world_file", "world_with_trees.world")
        self.declare_parameter("plant_tree_above_dirt", False)
        self.declare_parameter("num_fires", 5)
        self.declare_parameter("min_fire_distance", 5.0)
        self.declare_parameter("plant_fire_above_dirt", False)
        self.declare_parameter("min_fire_size", 2.0)
        self.declare_parameter("max_fire_size", 10.0)

        self.num_trees = self.get_parameter("num_trees").value
        self.tree_types = self.get_parameter("tree_types").value
        self.min_tree_distance = self.get_parameter("min_tree_distance").value
        self.max_slope = self.get_parameter("max_slope").value
        self.output_world_file = self.get_parameter("output_world_file").value
        self.plant_tree_above_dirt = self.get_parameter("plant_tree_above_dirt").value
        self.num_fires = self.get_parameter("num_fires").value
        self.min_fire_distance = self.get_parameter("min_fire_distance").value
        self.plant_fire_above_dirt = self.get_parameter("plant_fire_above_dirt").value
        self.min_fire_size = self.get_parameter("min_fire_size").value
        self.max_fire_size = self.get_parameter("max_fire_size").value

        self.package_path = get_package_share_directory("forest_map_generator")

        self.tree_generator = TreeGenerator(self)
        self.fire_generator = FireGenerator(self)

        self.run_generation()

    def get_source_worlds_path(self):
        package_path_parts = os.path.abspath(self.output_world_file).split(os.sep)
        if "install" in package_path_parts and os.path.basename(self.package_path) not in package_path_parts:
            output_world_file = os.path.basename(self.output_world_file)
            install_index = (
                    len(package_path_parts)
                    - 1
                    - package_path_parts[::-1].index("install")
            )
            package_name = package_path_parts[install_index+1]
            workspace_path = os.sep.join(package_path_parts[:install_index])
            if not workspace_path:
                workspace_path = os.sep
            install_worlds_path = os.path.join('/', *package_path_parts[:-2], 'worlds', output_world_file)
            source_worlds_path = os.path.join(workspace_path, "src", package_name, "worlds", output_world_file)
            return install_worlds_path, source_worlds_path
        elif os.path.basename(self.package_path) in package_path_parts:
            if "install" in os.path.abspath(self.package_path).split(os.sep):
                package_name = os.path.basename(self.package_path)
                package_path_parts = os.path.abspath(self.package_path).split(os.sep)
                output_world_file = os.path.basename(self.output_world_file)
                install_index = (
                        len(package_path_parts)
                        - 1
                        - package_path_parts[::-1].index("install")
                )
                workspace_path = os.sep.join(package_path_parts[:install_index])
                if not workspace_path:
                    workspace_path = os.sep
                install_worlds_path = os.path.join('/', *package_path_parts, 'worlds', output_world_file)
                source_worlds_path = os.path.join(workspace_path, "src", package_name, "worlds", output_world_file)
                return install_worlds_path, source_worlds_path

        worlds_path = os.path.join(
            os.path.abspath(os.path.join(os.path.dirname(__file__), "..")),
            "worlds",
        )
        return worlds_path, worlds_path

    def generate_final_world_file(self, trees_xml, fires_xml):
        original_world_path = os.path.join(self.package_path, "worlds", "world.world")
        install_worlds_path, source_worlds_path = self.get_source_worlds_path()

        try:
            with open(original_world_path, "r") as f:
                world_content = f.read()
        except Exception as e:
            self.get_logger().error(f"Failed to read world file: {e}")
            return False

        if fires_xml and "gz-sim-particle-emitter-system" not in world_content:
            world_tag_start = world_content.find("<world")
            world_tag_end = world_content.find(">", world_tag_start)

            if world_tag_start == -1 or world_tag_end == -1:
                self.get_logger().error("Invalid world file: missing <world> tag.")
                return False

            world_content = (
                world_content[: world_tag_end + 1]
                + "\n"
                + FireGenerator.create_particle_emitter_plugin_xml()
                + world_content[world_tag_end + 1 :]
            )

        combined_xml = trees_xml + fires_xml

        if "</world>" in world_content:
            new_world_content = world_content.replace(
                "</world>", combined_xml + "  </world>"
            )
        else:
            self.get_logger().error("Invalid world file: missing </world> tag.")
            return False

        try:
            with open(install_worlds_path, "w") as f:
                f.write(new_world_content)
            self.get_logger().info(f"World file saved to: {install_worlds_path}")

            with open(source_worlds_path, "w") as f:
                f.write(new_world_content)
            self.get_logger().info(
                f"World file saved to: {source_worlds_path}"
            )
            return True
        except Exception as e:
            self.get_logger().error(f"Failed to write world file: {e}")
            return False

    def run_generation(self):
        trees = self.tree_generator.generate_trees()
        trees_xml = self.tree_generator.generate_trees_xml(trees) if trees else ""
        fires = self.fire_generator.generate_fires()
        fires_xml = self.fire_generator.generate_fires_xml(fires) if fires else ""

        if self.generate_final_world_file(trees_xml, fires_xml):
            self.get_logger().info("Generation completed successfully!")
        else:
            self.get_logger().error("Generation failed!")


def main():
    rclpy.init()
    node = ForestMapGenerator()
    rclpy.spin_once(node, timeout_sec=1)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
