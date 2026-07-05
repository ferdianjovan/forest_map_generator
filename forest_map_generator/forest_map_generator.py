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

        self.num_trees = self.get_parameter("num_trees").value
        self.tree_types = self.get_parameter("tree_types").value
        self.min_tree_distance = self.get_parameter("min_tree_distance").value
        self.max_slope = self.get_parameter("max_slope").value
        self.output_world_file = self.get_parameter("output_world_file").value
        self.plant_tree_above_dirt = self.get_parameter("plant_tree_above_dirt").value

        self.package_path = get_package_share_directory("forest_map_generator")

        self.tree_generator = TreeGenerator(self)

        self.run_generation()

    def generate_final_world_file(self, trees_xml):
        original_world_path = os.path.join(self.package_path, "worlds", "world.world")
        output_world_path = os.path.join(
            self.package_path, "worlds", self.output_world_file
        )
        try:
            with open(original_world_path, "r") as f:
                world_content = f.read()
        except Exception as e:
            self.get_logger().error(f"Failed to read world file: {e}")
            return False

        combined_xml = trees_xml

        if "</world>" in world_content:
            new_world_content = world_content.replace(
                "</world>", combined_xml + "  </world>"
            )
        else:
            self.get_logger().error("Invalid world file: missing </world> tag.")
            return False

        try:
            with open(output_world_path, "w") as f:
                f.write(new_world_content)
            self.get_logger().info(f"World file saved to: {output_world_path}")
            return True
        except Exception as e:
            self.get_logger().error(f"Failed to write world file: {e}")
            return False

    def run_generation(self):
        trees = self.tree_generator.generate_trees()
        trees_xml = self.tree_generator.generate_trees_xml(trees) if trees else ""

        if self.generate_final_world_file(trees_xml):
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
