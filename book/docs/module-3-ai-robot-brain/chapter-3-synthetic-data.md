# Chapter 3: Synthetic Data Generation for Humanoid Robot AI

## Introduction to Synthetic Data in Robotics

Synthetic data generation is a transformative approach in robotics and AI development that uses simulation environments to create large volumes of labeled training data. For humanoid robots, this is particularly important because:

1. **Safety**: Training complex behaviors in simulation avoids risking expensive hardware
2. **Cost-effectiveness**: Generating data in simulation is significantly cheaper than real-world data collection
3. **Variety**: Simulation allows for countless environmental variations and edge cases
4. **Ground truth**: Perfect labels for training perception and control systems
5. **Speed**: Thousands of hours of robot experience can be generated rapidly

For humanoid robots operating in human environments, synthetic data enables training on diverse scenarios like different lighting conditions, varied terrains, and complex human interactions that would be difficult and time-consuming to collect in the real world.

## Why Synthetic Data Matters for Humanoid Robots

Humanoid robots face unique challenges that make synthetic data particularly valuable:

- **Complex kinematics**: Many degrees of freedom require extensive training data
- **Balance requirements**: Training sophisticated balance controllers needs diverse scenarios
- **Human interaction**: Safe training of human-robot interaction requires simulation
- **Environment diversity**: Human environments vary greatly (homes, offices, public spaces)

### The Domain Randomization Approach

Domain randomization is a technique that systematically varies environmental parameters to increase the robustness of AI models:

- **Lighting**: Varying intensities, colors, and directions
- **Textures**: Changing surface materials and patterns
- **Objects**: Varying appearances, positions, and properties
- **Weather**: Simulating different atmospheric conditions (for outdoor scenarios)
- **Camera properties**: Adjusting focal length, noise, and distortion

## Isaac Sim's Synthetic Data Generation Capabilities

Isaac Sim provides a comprehensive toolkit for synthetic data generation specifically designed for robotics:

### 1. Photorealistic Rendering

Isaac Sim's RTX-powered rendering creates images indistinguishable from real photographs:

```python
import omni
from omni.isaac.synthetic_utils import SyntheticDataHelper
import numpy as np

class IsaacSyntheticDataGenerator:
    def __init__(self):
        self.sd_helper = SyntheticDataHelper()
        
    def setup_render_products(self, camera_paths, resolution):
        """
        Set up render products for synthetic data generation
        """
        render_products = []
        
        for cam_path in camera_paths:
            render_product = self.sd_helper.create_render_product(
                cam_path, 
                resolution,
                semantic_labels=False,
                instance_masks=False,
                depth_maps=True,
                normals=True
            )
            render_products.append(render_product)
        
        return render_products
    
    def generate_synthetic_dataset(self, 
                                 num_samples=10000,
                                 output_dir="synthetic_dataset",
                                 domain_rand_params=None):
        """
        Generate a synthetic dataset with domain randomization
        """
        # Create output directory structure
        os.makedirs(output_dir, exist_ok=True)
        os.makedirs(os.path.join(output_dir, "images"), exist_ok=True)
        os.makedirs(os.path.join(output_dir, "labels"), exist_ok=True)
        os.makedirs(os.path.join(output_dir, "depth"), exist_ok=True)
        os.makedirs(os.path.join(output_dir, "normals"), exist_ok=True)
        
        # Get render products from helper
        camera_paths = ["/World/Camera"]  # Define your camera paths
        resolution = (640, 480)
        render_products = self.setup_render_products(camera_paths, resolution)
        
        # Generate samples
        for i in range(num_samples):
            # Apply domain randomization
            if domain_rand_params:
                self.randomize_domain(domain_rand_params)
            
            # Wait for simulation to settle
            self.wait_for_settle()
            
            # Capture data from all cameras
            for j, render_prod in enumerate(render_products):
                # Capture RGB
                rgb = self.sd_helper.get_rgb(render_prod)
                self.save_image(rgb, f"{output_dir}/images/{i:06d}_{j}.png")
                
                # Capture depth
                depth = self.sd_helper.get_depth(render_prod)
                self.save_depth(depth, f"{output_dir}/depth/{i:06d}_{j}.exr")
                
                # Capture segmentation (if enabled)
                if self.semantic_labels_enabled:
                    seg = self.sd_helper.get_segmentation(render_prod)
                    self.save_segmentation(seg, f"{output_dir}/labels/{i:06d}_{j}.png")
                
                # Capture normals
                normals = self.sd_helper.get_normals(render_prod)
                self.save_normals(normals, f"{output_dir}/normals/{i:06d}_{j}.exr")
            
            # Log progress
            if i % 1000 == 0:
                print(f"Generated {i}/{num_samples} samples")
    
    def randomize_domain(self, params):
        """
        Apply domain randomization parameters to the scene
        """
        # Lighting randomization
        if 'lighting' in params:
            light_range = params['lighting']['intensity_range']
            color_range = params['lighting']['color_range']
            
            # Randomize light intensity and color
            new_intensity = np.random.uniform(*light_range)
            new_color = np.random.uniform(*color_range, size=3)
            
            # Apply to all lights in scene
            self.apply_light_randomization(new_intensity, new_color)
        
        # Material randomization
        if 'materials' in params:
            # Randomize surface properties like albedo, roughness, metallic
            self.apply_material_randomization(params['materials'])
        
        # Object placement randomization
        if 'objects' in params:
            # Randomize positions, rotations, and scales of objects
            self.apply_object_randomization(params['objects'])
    
    def apply_light_randomization(self, intensity, color):
        """
        Randomize lighting in the scene
        """
        # Get all light prims in the stage
        light_prims = self.get_light_prims()
        
        for light_prim in light_prims:
            # Set new light properties
            # Implementation would interact with Omniverse Kit APIs
            pass
    
    def apply_material_randomization(self, material_params):
        """
        Randomize material properties in the scene
        """
        # Get all material prims in the scene
        material_prims = self.get_material_prims()
        
        for material_prim in material_prims:
            # Randomize material properties based on provided parameters
            # This could affect albedo, roughness, metallic, etc.
            pass
    
    def apply_object_randomization(self, object_params):
        """
        Randomize object placements in the scene
        """
        # Get all object prims in the scene
        object_prims = self.get_object_prims()
        
        for obj_prim in object_prims:
            # Apply random transformations based on parameters
            # Position, rotation, scale variations
            pass

# Integration with ROS 2 for humanoid robot training
class IsaacSyntheticDataManager:
    def __init__(self, robot_namespace="/humanoid_robot"):
        self.robot_namespace = robot_namespace
        self.generator = IsaacSyntheticDataGenerator()
        self.dataset_stats = {}
    
    def setup_humanoid_training_dataset(self,
                                      scenario="object_manipulation",
                                      num_samples=50000,
                                      domain_rand_params=None):
        """
        Set up a synthetic dataset for humanoid robot training
        """
        # Define scenarios based on common humanoid tasks
        scenarios = {
            "object_manipulation": self.setup_manipulation_scenario,
            "walking_balance": self.setup_walking_scenario,
            "human_interaction": self.setup_interaction_scenario,
            "navigation": self.setup_navigation_scenario
        }
        
        if scenario not in scenarios:
            raise ValueError(f"Unknown scenario: {scenario}")
        
        # Set up the specific scenario
        scenario_setup = scenarios[scenario]()
        
        # Generate the dataset
        self.generator.generate_synthetic_dataset(
            num_samples=num_samples,
            output_dir=f"datasets/humanoid_{scenario}",
            domain_rand_params=domain_rand_params or self.get_default_domain_rand_params(scenario)
        )
        
        # Register the dataset
        self.register_dataset(f"humanoid_{scenario}", num_samples)
    
    def setup_manipulation_scenario(self):
        """
        Set up scenario for object manipulation training
        """
        # Place manipulable objects in scene
        # Set up humanoid robot in manipulation pose
        # Configure cameras for hand-object interaction
        pass
    
    def setup_walking_scenario(self):
        """
        Set up scenario for bipedal walking training
        """
        # Create varied terrain (stairs, slopes, obstacles)
        # Configure humanoid robot in walking pose
        # Set up cameras for whole-body observation
        pass
    
    def setup_interaction_scenario(self):
        """
        Set up scenario for human-robot interaction
        """
        # Add human avatars/models to scene
        # Configure humanoid robot in interaction pose
        # Set up cameras for both human and robot observation
        pass
    
    def setup_navigation_scenario(self):
        """
        Set up scenario for navigation training
        """
        # Create diverse indoor environments
        # Place humanoid robot with navigation goals
        # Configure cameras for environment observation
        pass
    
    def get_default_domain_rand_params(self, scenario):
        """
        Get default domain randomization parameters for a scenario
        """
        defaults = {
            "object_manipulation": {
                "lighting": {
                    "intensity_range": (500, 5000),
                    "color_range": (0.8, 1.2)  # Multiplier for each RGB channel
                },
                "materials": {
                    "albedo_range": (0.1, 1.0),
                    "roughness_range": (0.05, 0.95),
                    "metallic_range": (0.0, 0.1)
                },
                "objects": {
                    "position_jitter": 0.05,  # 5cm jitter
                    "rotation_jitter": 0.1,   # 0.1 radian jitter
                    "scale_range": (0.8, 1.2)  # Scale between 0.8 and 1.2
                }
            },
            "walking_balance": {
                "lighting": {
                    "intensity_range": (1000, 10000),
                    "color_range": (0.9, 1.1)
                },
                "materials": {
                    "albedo_range": (0.2, 1.0),
                    "roughness_range": (0.1, 0.9),
                    "metallic_range": (0.0, 0.2)
                },
                "objects": {
                    "position_jitter": 0.1,  # 10cm for terrain variations
                    "rotation_jitter": 0.05,   # 0.05 radian for terrain
                    "scale_range": (0.95, 1.05)  # Small scale variations
                }
            }
        }
        
        return defaults.get(scenario, defaults["object_manipulation"])
    
    def register_dataset(self, name, size):
        """
        Register the dataset for tracking
        """
        self.dataset_stats[name] = {
            "size": size,
            "creation_date": datetime.now(),
            "path": f"datasets/{name}",
            "status": "completed"
        }
    
    def validate_dataset_quality(self, dataset_name):
        """
        Validate the quality of generated synthetic data
        """
        # Check for common issues in synthetic datasets:
        # 1. Image quality (blur, exposure, etc.)
        # 2. Label accuracy (proper segmentation)
        # 3. Diversity (variance across samples)
        # 4. Consistency (between modalities)
        pass
```

### 2. USD-Based Scene Composition

Isaac Sim uses Universal Scene Description (USD) for scene composition, enabling complex, hierarchical environments:

```python
from pxr import Usd, UsdGeom, Sdf, Gf
import omni

class USDSceneComposer:
    def __init__(self):
        self.stage = omni.usd.get_context().get_stage()
    
    def create_diverse_training_environments(self, base_environment_path, variations_count=100):
        """
        Create diverse training environments based on a base environment
        """
        # Load the base environment
        base_env_stage = Usd.Stage.Open(base_environment_path)
        
        # Create variations
        for i in range(variations_count):
            variation_path = f"/World/EnvVariation_{i:03d}"
            
            # Instance the base environment
            self.stage.DefinePrim(variation_path, "Xform")
            env_instance = self.stage.GetPrimAtPath(variation_path)
            
            # Apply variations to the environment
            self.apply_environment_variations(env_instance, i)
    
    def apply_environment_variations(self, env_prim, variation_index):
        """
        Apply variations to an environment prim
        """
        # Randomize floor materials
        floor_material = self.get_random_floor_material(variation_index)
        self.assign_material_to_objects(env_prim, "floor", floor_material)
        
        # Randomize wall materials
        wall_materials = self.get_random_wall_materials(variation_index)
        self.assign_material_to_objects(env_prim, "wall", wall_materials)
        
        # Add random furniture/props
        self.add_random_objects(env_prim, variation_index)
        
        # Randomize lighting
        self.randomize_lighting(env_prim, variation_index)
    
    def get_random_floor_material(self, index):
        """
        Get a random floor material based on the variation index
        """
        floor_types = [
            "wood_planks", "tile", "carpet", "concrete", "marble"
        ]
        surface_roughnesses = [0.1, 0.3, 0.5, 0.7, 0.9]
        
        # Pseudo-random selection based on index to ensure reproducibility
        import hashlib
        seed = hashlib.md5(str(index).encode()).hexdigest()
        random.seed(seed)
        
        floor_type = random.choice(floor_types)
        roughness = random.choice(surface_roughnesses)
        
        return {
            "type": floor_type,
            "roughness": roughness,
            "texture_scale": random.uniform(0.5, 2.0)
        }
    
    def assign_material_to_objects(self, parent_prim, object_class, material_spec):
        """
        Assign a material to objects matching a class in the scene
        """
        # Find all objects of specified class in the scene
        for prim in parent_prim.GetChildren():
            if self.has_class_tag(prim, object_class):
                # Assign the material to this object
                self.set_material(prim, material_spec)
    
    def add_random_objects(self, env_prim, index):
        """
        Add random objects to the environment
        """
        # Use a seeded RNG to ensure reproducibility
        import hashlib
        seed = hashlib.md5(f"{index}_objects".encode()).hexdigest()
        random.seed(seed)
        
        # Define possible object categories for humanoid training
        object_categories = {
            "kitchen": ["cup", "plate", "bowl", "utensil"],
            "office": ["pen", "paper", "cup", "phone"],
            "living_room": ["book", "pillow", "remote", "plant"]
        }
        
        # Randomly select a category based on index
        categories = list(object_categories.keys())
        chosen_category = categories[index % len(categories)]
        objects_to_place = object_categories[chosen_category]
        
        # Place objects in the environment
        for i in range(random.randint(3, 8)):  # 3-8 objects per scene
            if i < len(objects_to_place):
                obj_name = objects_to_place[i]
                obj_position = [
                    random.uniform(-3, 3),  # Random x position
                    random.uniform(-2, 2),  # Random y position
                    random.uniform(0.1, 1.0)  # Random z position
                ]
                
                # Add the object to the scene
                self.place_object(env_prim, obj_name, obj_position)
    
    def place_object(self, env_prim, obj_name, position):
        """
        Place an object in the environment
        """
        # Import object from asset library
        asset_path = self.get_asset_path_for_object(obj_name)
        
        # Create a new prim for the object
        obj_prim_path = f"{env_prim.GetPath()}/{obj_name}_{len(list(env_prim.GetChildren()))}"
        self.stage.DefinePrim(obj_prim_path, "Xform")
        
        # Set position
        obj_xform = UsdGeom.Xformable(self.stage.GetPrimAtPath(obj_prim_path))
        obj_xform.AddTranslateOp().Set(Gf.Vec3d(*position))
        
        # Add the object geometry
        self.add_geometry_to_object(obj_prim_path, asset_path)
    
    def randomize_lighting(self, env_prim, index):
        """
        Randomize lighting conditions in the environment
        """
        # Get existing lights in the environment
        lights = self.get_lights_in_env(env_prim)
        
        # Modify light properties randomly
        for i, light in enumerate(lights):
            # Use seeded randomness based on environment index and light index
            import hashlib
            seed = hashlib.md5(f"{index}_light_{i}".encode()).hexdigest()
            random.seed(seed)
            
            # Randomize intensity and color
            intensity = random.uniform(200, 5000)
            color = [random.uniform(0.8, 1.2), random.uniform(0.8, 1.2), random.uniform(0.8, 1.2)]
            
            self.modify_light_properties(light, intensity, color)
```

## Synthetic Data Pipelines for Humanoid Robot AI

### 1. Object Detection Dataset Generation

Generating synthetic data for object detection in humanoid environments:

```python
import numpy as np
from omni.isaac.synthetic_utils import BoundingBoxNumpyWriter
from PIL import Image
import json
import os

class HumanoidObjectDetectionDataset:
    def __init__(self, output_dir="synthetic_datasets/object_detection"):
        self.output_dir = output_dir
        self.bbox_writer = BoundingBoxNumpyWriter()
        self.stats = {
            "total_samples": 0,
            "avg_objects_per_scene": 0,
            "class_distribution": {}
        }
        
    def generate_object_detection_dataset(
        self,
        scene_configs,
        object_library,
        num_samples=10000,
        image_size=(640, 480)
    ):
        """
        Generate a synthetic object detection dataset for humanoid robot
        """
        # Create output directories
        os.makedirs(os.path.join(self.output_dir, "images"), exist_ok=True)
        os.makedirs(os.path.join(self.output_dir, "labels"), exist_ok=True)
        
        total_objects = 0
        
        for i in range(num_samples):
            # Randomize scene configuration
            scene_config = self.randomize_scene_config(scene_configs, i)
            
            # Place objects in scene
            objects_in_scene = self.place_objects_in_scene(
                object_library, 
                scene_config["num_objects"],
                scene_config["placement_area"]
            )
            
            # Render the scene
            rgb_image = self.render_rgb_image()
            depth_image = self.render_depth_image()
            
            # Get bounding boxes
            bboxes = self.get_bounding_boxes(objects_in_scene)
            
            # Save image
            img_path = os.path.join(self.output_dir, "images", f"{i:06d}.png")
            Image.fromarray((rgb_image * 255).astype(np.uint8)).save(img_path)
            
            # Save bounding box annotations
            label_path = os.path.join(self.output_dir, "labels", f"{i:06d}.json")
            self.save_bounding_box_annotations(label_path, bboxes, image_size)
            
            # Update statistics
            total_objects += len(bboxes)
            for bbox in bboxes:
                cls = bbox["class"]
                self.stats["class_distribution"][cls] = self.stats["class_distribution"].get(cls, 0) + 1
            
            # Log progress
            if i % 1000 == 0:
                print(f"Generated {i}/{num_samples} samples, "
                      f"Avg objects per scene: {total_objects/max(1, i):.1f}")
        
        # Update final statistics
        self.stats["total_samples"] = num_samples
        self.stats["avg_objects_per_scene"] = total_objects / num_samples
        
        # Save dataset statistics
        self.save_statistics()
    
    def randomize_scene_config(self, base_configs, sample_index):
        """
        Randomize scene configuration based on sample index for reproducibility
        """
        import random
        seed_value = hash(f"scene_{sample_index}") % (2**32)
        random.seed(seed_value)
        
        # Select a random base configuration
        config = random.choice(base_configs)
        
        # Randomize number of objects in scene
        num_objects = random.randint(config["min_objects"], config["max_objects"])
        
        # Randomize placement area
        placement_area = {
            "center": [
                random.uniform(-2.0, 2.0),  # x
                random.uniform(-1.5, 1.5),  # y (narrower for humanoid workspace)
                random.uniform(0.1, 2.0)    # z (ground to table height)
            ],
            "dimensions": [
                random.uniform(1.0, 4.0),   # x extent
                random.uniform(0.5, 3.0),   # y extent
                random.uniform(0.2, 1.5)    # z extent
            ]
        }
        
        return {
            "num_objects": num_objects,
            "placement_area": placement_area,
            "lighting_condition": random.choice(["bright", "dim", "mixed"]),
            "background": random.choice(["indoor", "cluttered", "clean"])
        }
    
    def place_objects_in_scene(self, object_library, num_objects, placement_area):
        """
        Place specified number of objects in the scene area
        """
        import random
        
        objects_placed = []
        center = placement_area["center"]
        dims = placement_area["dimensions"]
        
        for i in range(num_objects):
            # Randomly select an object from the library
            obj_info = random.choice(object_library)
            
            # Generate random position within placement area
            pos = [
                center[0] + random.uniform(-dims[0]/2, dims[0]/2),
                center[1] + random.uniform(-dims[1]/2, dims[1]/2),
                center[2] + random.uniform(0, dims[2])
            ]
            
            # Generate random rotation
            rot = [0, 0, random.uniform(0, 2*np.pi)]  # Only random yaw
            
            obj_instance = {
                "type": obj_info["type"],
                "name": f"{obj_info['type']}_{len(objects_placed)}",
                "position": pos,
                "rotation": rot,
                "scale": obj_info.get("default_scale", [1.0, 1.0, 1.0]),
                "class_id": obj_info["class_id"]
            }
            
            objects_placed.append(obj_instance)
            
            # Actually place the object in the simulation
            self.add_object_to_simulation(obj_instance)
        
        return objects_placed
    
    def get_bounding_boxes(self, objects_in_scene):
        """
        Get bounding boxes for all objects in the current camera view
        """
        bboxes = []
        
        for obj in objects_in_scene:
            # Get the 2D bounding box of the object as viewed by the camera
            bbox_2d = self.get_rendered_bbox_2d(obj["name"])
            
            if bbox_2d is not None:
                bboxes.append({
                    "class": obj["type"],
                    "class_id": obj["class_id"],
                    "bbox": [bbox_2d[0], bbox_2d[1], bbox_2d[2], bbox_2d[3]],  # [x, y, width, height]
                    "confidence": 1.0,  # Perfect for synthetic data
                    "occluded": self.is_object_occluded(obj["name"])
                })
        
        return bboxes
    
    def save_bounding_box_annotations(self, path, bboxes, image_size):
        """
        Save bounding box annotations in COCO format
        """
        annotation = {
            "image_size": image_size,
            "objects": []
        }
        
        for bbox in bboxes:
            annotation["objects"].append({
                "category": bbox["class"],
                "category_id": bbox["class_id"],
                "bbox": bbox["bbox"],  # [x, y, width, height]
                "area": bbox["bbox"][2] * bbox["bbox"][3],
                "iscrowd": 0,
                "occluded": bbox["occluded"]
            })
        
        with open(path, 'w') as f:
            json.dump(annotation, f, indent=2)
    
    def save_statistics(self):
        """
        Save dataset statistics
        """
        stats_path = os.path.join(self.output_dir, "dataset_stats.json")
        with open(stats_path, 'w') as f:
            json.dump(self.stats, f, indent=2)

# Example object library for humanoid environments
HUMANOID_OBJECT_LIBRARY = [
    {"type": "cup", "class_id": 1, "default_scale": [0.05, 0.05, 0.1]},
    {"type": "plate", "class_id": 2, "default_scale": [0.2, 0.2, 0.02]},
    {"type": "bottle", "class_id": 3, "default_scale": [0.06, 0.06, 0.25]},
    {"type": "book", "class_id": 4, "default_scale": [0.15, 0.1, 0.03]},
    {"type": "phone", "class_id": 5, "default_scale": [0.06, 0.12, 0.01]},
    {"type": "apple", "class_id": 6, "default_scale": [0.06, 0.06, 0.06]},
    {"type": "banana", "class_id": 7, "default_scale": [0.18, 0.03, 0.03]},
    {"type": "toy_block", "class_id": 8, "default_scale": [0.04, 0.04, 0.04]},
    {"type": "fork", "class_id": 9, "default_scale": [0.05, 0.2, 0.01]},
    {"type": "spoon", "class_id": 10, "default_scale": [0.04, 0.18, 0.01]}
]

# Example scene configurations for object detection
HUMANOID_SCENE_CONFIGS = [
    {
        "name": "kitchen_counter",
        "min_objects": 3,
        "max_objects": 8,
        "placement_area": "counter",
        "environment_type": "indoor"
    },
    {
        "name": "table_setting",
        "min_objects": 4,
        "max_objects": 10,
        "placement_area": "table",
        "environment_type": "dining"
    },
    {
        "name": "office_desk",
        "min_objects": 2,
        "max_objects": 6,
        "placement_area": "desk",
        "environment_type": "office"
    }
]
```

### 2. Depth Estimation Dataset Generation

Creating synthetic datasets for depth estimation tasks:

```python
class DepthEstimationDataset:
    def __init__(self, output_dir="synthetic_datasets/depth_estimation"):
        self.output_dir = output_dir
        self.stats = {
            "total_samples": 0,
            "depth_range_stats": {"min": float('inf'), "max": float('-inf'), "avg": 0.0}
        }
        
    def generate_depth_dataset(self, 
                             scene_configs, 
                             num_samples=5000, 
                             image_size=(640, 480)):
        """
        Generate a synthetic depth estimation dataset
        """
        # Create output directories
        os.makedirs(os.path.join(self.output_dir, "rgb"), exist_ok=True)
        os.makedirs(os.path.join(self.output_dir, "depth"), exist_ok=True)
        os.makedirs(os.path.join(self.output_dir, "metadata"), exist_ok=True)
        
        depth_sum = 0.0
        sample_count = 0
        
        for i in range(num_samples):
            # Randomize scene
            self.randomize_scene(scene_configs[i % len(scene_configs)], i)
            
            # Render RGB and depth images
            rgb_img = self.render_rgb_image()
            depth_img = self.render_depth_image()
            
            # Save images
            rgb_path = os.path.join(self.output_dir, "rgb", f"{i:06d}.png")
            depth_path = os.path.join(self.output_dir, "depth", f"{i:06d}.exr")  # Use EXR for high precision
            
            # Save RGB
            Image.fromarray((rgb_img * 255).astype(np.uint8)).save(rgb_path)
            
            # Save depth (preserve precision)
            self.save_depth_exr(depth_img, depth_path)
            
            # Calculate depth statistics
            valid_depths = depth_img[np.isfinite(depth_img) & (depth_img > 0)]
            if len(valid_depths) > 0:
                depth_min = np.min(valid_depths)
                depth_max = np.max(valid_depths)
                depth_avg = np.mean(valid_depths)
                
                # Update stats
                self.stats["depth_range_stats"]["min"] = min(
                    self.stats["depth_range_stats"]["min"], depth_min
                )
                self.stats["depth_range_stats"]["max"] = max(
                    self.stats["depth_range_stats"]["max"], depth_max
                )
                depth_sum += depth_avg
                sample_count += 1
            
            # Save metadata
            metadata = {
                "sample_id": i,
                "scene_config": scene_configs[i % len(scene_configs)],
                "timestamp": i / 30.0,  # Assuming 30Hz camera
                "camera_pose": self.get_camera_pose(),
                "depth_range": {"min": float(depth_min) if len(valid_depths) > 0 else None,
                               "max": float(depth_max) if len(valid_depths) > 0 else None,
                               "mean": float(depth_avg) if len(valid_depths) > 0 else None}
            }
            
            meta_path = os.path.join(self.output_dir, "metadata", f"{i:06d}.json")
            with open(meta_path, 'w') as f:
                json.dump(metadata, f, indent=2)
            
            # Log progress
            if i % 500 == 0:
                print(f"Generated {i}/{num_samples} depth samples")
        
        # Finalize statistics
        self.stats["total_samples"] = num_samples
        self.stats["depth_range_stats"]["avg"] = depth_sum / sample_count if sample_count > 0 else 0.0
        
        # Save statistics
        self.save_statistics()
    
    def save_depth_exr(self, depth_array, path):
        """
        Save depth data in EXR format with high precision
        """
        try:
            import OpenEXR
            import Imath
            
            # Get image dimensions
            height, width = depth_array.shape
            
            # Create EXR header
            header = OpenEXR.Header(width, height)
            header['channels'] = {'R': Imath.Channel(Imath.PixelType(Imath.FLOAT))}
            
            # Create EXR file
            exr = OpenEXR.OutputFile(path, header)
            
            # Convert to bytes
            depth_bytes = depth_array.astype(np.float32).tobytes()
            
            # Write to EXR file
            exr.writePixels({'R': depth_bytes})
            exr.close()
        except ImportError:
            # Fallback: save as numpy binary
            np.save(path.replace('.exr', '.npy'), depth_array)
            print(f"EXR not available, saved as numpy binary: {path.replace('.exr', '.npy')}")
    
    def randomize_scene(self, config, sample_index):
        """
        Randomize scene based on configuration
        """
        # This would implement scene randomization logic
        # such as placing objects, adjusting lighting, etc.
        pass
    
    def render_rgb_image(self):
        """
        Render RGB image from current camera view
        """
        # This would interface with Isaac Sim rendering system
        # For now, return a dummy image
        return np.random.rand(480, 640, 3).astype(np.float32)
    
    def render_depth_image(self):
        """
        Render depth image from current camera view
        """
        # This would interface with Isaac Sim depth rendering
        # For now, return a dummy depth map
        return np.random.rand(480, 640).astype(np.float32) * 10.0  # 0-10m range
    
    def get_camera_pose(self):
        """
        Get the current camera pose in the scene
        """
        # This would return actual camera pose from simulation
        return {"position": [0, 0, 0], "rotation": [0, 0, 0, 1]}  # xyz position, wxyz quaternion
```

## Quality Assurance for Synthetic Data

### Validation Techniques

Ensuring synthetic data quality requires several validation techniques:

```python
class SyntheticDataValidator:
    def __init__(self):
        self.validation_results = []
    
    def validate_synthetic_dataset(self, dataset_path, validation_config):
        """
        Validate synthetic dataset quality
        """
        results = {
            "image_quality": self.validate_image_quality(dataset_path),
            "label_accuracy": self.validate_label_accuracy(dataset_path),
            "diversity_metrics": self.calculate_diversity(dataset_path),
            "consistency_check": self.validate_modality_consistency(dataset_path),
            "realism_score": self.assess_realism(dataset_path)
        }
        
        self.validation_results.append({
            "dataset_path": dataset_path,
            "timestamp": datetime.now(),
            "results": results
        })
        
        return results
    
    def validate_image_quality(self, dataset_path):
        """
        Check image quality metrics
        """
        import cv2
        
        rgb_dir = os.path.join(dataset_path, "rgb")
        quality_metrics = {
            "blur_score": 0,
            "exposure_score": 0,
            "noise_level": 0,
            "total_images": 0,
            "invalid_images": 0
        }
        
        for img_file in os.listdir(rgb_dir)[:100]:  # Sample first 100 images
            img_path = os.path.join(rgb_dir, img_file)
            
            try:
                img = cv2.imread(img_path)
                img_float = img.astype(np.float32)
                
                # Blur detection using Laplacian variance
                gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
                blur_score = cv2.Laplacian(gray, cv2.CV_64F).var()
                
                # Exposure assessment
                mean_brightness = np.mean(gray)
                exposure_score = abs(mean_brightness - 128) / 128  # Normalize to [0,1], 0 is perfect
    
                quality_metrics["blur_score"] += blur_score
                quality_metrics["exposure_score"] += exposure_score
                quality_metrics["total_images"] += 1
                
            except Exception as e:
                quality_metrics["invalid_images"] += 1
        
        if quality_metrics["total_images"] > 0:
            quality_metrics["avg_blur_score"] = quality_metrics["blur_score"] / quality_metrics["total_images"]
            quality_metrics["avg_exposure_score"] = quality_metrics["exposure_score"] / quality_metrics["total_images"]
        
        return quality_metrics
    
    def validate_label_accuracy(self, dataset_path):
        """
        Validate that labels accurately correspond to images
        """
        # For object detection data, verify bounding boxes match visible objects
        # For depth data, verify depth values are physically plausible
        # For segmentation, verify pixel-level accuracy
        pass
    
    def calculate_diversity(self, dataset_path):
        """
        Calculate diversity metrics for the dataset
        """
        # Measure diversity in:
        # - Object arrangements
        # - Lighting conditions
        # - Camera viewpoints
        # - Environmental contexts
        pass
    
    def validate_modality_consistency(self, dataset_path):
        """
        Validate consistency between different modalities (RGB, depth, segmentation)
        """
        # Ensure RGB images align with corresponding depth and segmentation
        pass
    
    def assess_realism(self, dataset_path):
        """
        Assess how realistic the synthetic data appears
        """
        # Could use ML models trained to distinguish real vs synthetic
        # Or human evaluation studies
        pass
```

## Integration with Training Pipelines

Synthetic data needs to be properly integrated with machine learning training pipelines:

```python
import torch
from torch.utils.data import Dataset, DataLoader
import torchvision.transforms as transforms

class IsaacSyntheticDataset(Dataset):
    """
    PyTorch Dataset for Isaac Sim synthetic data
    """
    def __init__(self, data_dir, transform=None, task="object_detection"):
        self.data_dir = data_dir
        self.task = task
        self.transform = transform
        
        # Load data paths
        self.rgb_paths, self.label_paths = self.load_data_paths()
        
    def load_data_paths(self):
        """
        Load paths to RGB images and corresponding labels
        """
        if self.task == "object_detection":
            rgb_dir = os.path.join(self.data_dir, "images")
            label_dir = os.path.join(self.data_dir, "labels")
            
            rgb_files = sorted([f for f in os.listdir(rgb_dir) if f.endswith(('.png', '.jpg'))])
            label_files = sorted([f for f in os.listdir(label_dir) if f.endswith('.json')])
            
            # Match RGB images with labels
            pairs = []
            for rgb_file in rgb_files:
                base_name = os.path.splitext(rgb_file)[0]
                label_file = f"{base_name}.json"
                
                if label_file in label_files:
                    pairs.append((
                        os.path.join(rgb_dir, rgb_file),
                        os.path.join(label_dir, label_file)
                    ))
            
            rgb_paths, label_paths = zip(*pairs) if pairs else ([], [])
            return list(rgb_paths), list(label_paths)
        
        elif self.task == "depth_estimation":
            rgb_dir = os.path.join(self.data_dir, "rgb")
            depth_dir = os.path.join(self.data_dir, "depth")
            
            rgb_files = sorted([f for f in os.listdir(rgb_dir) if f.endswith(('.png', '.jpg'))])
            depth_files = sorted([f for f in os.listdir(depth_dir) if f.endswith(('.exr', '.npy'))])
            
            # Match RGB images with depth maps
            pairs = []
            for rgb_file in rgb_files:
                base_name = os.path.splitext(rgb_file)[0]
                
                # Look for corresponding depth file (same base name)
                depth_file = None
                for df in depth_files:
                    if os.path.splitext(df)[0] == base_name:
                        depth_file = df
                        break
                
                if depth_file:
                    pairs.append((
                        os.path.join(rgb_dir, rgb_file),
                        os.path.join(depth_dir, depth_file)
                    ))
            
            rgb_paths, depth_paths = zip(*pairs) if pairs else ([], [])
            return list(rgb_paths), list(depth_paths)
    
    def __len__(self):
        return len(self.rgb_paths)
    
    def __getitem__(self, idx):
        rgb_path = self.rgb_paths[idx]
        
        if self.task == "object_detection":
            label_path = self.label_paths[idx]
            
            # Load image
            image = Image.open(rgb_path).convert('RGB')
            
            # Load annotations
            with open(label_path, 'r') as f:
                annotations = json.load(f)
            
            # Extract bounding boxes and labels
            boxes = []
            labels = []
            
            for obj in annotations.get("objects", []):
                bbox = obj["bbox"]  # Format: [x, y, width, height]
                boxes.append([bbox[0], bbox[1], bbox[0]+bbox[2], bbox[1]+bbox[3]])  # Convert to [x1, y1, x2, y2]
                labels.append(obj["category_id"])
            
            # Convert to tensors
            boxes = torch.tensor(boxes, dtype=torch.float32)
            labels = torch.tensor(labels, dtype=torch.int64)
            
            # Apply transforms
            if self.transform:
                image = self.transform(image)
            
            return {
                "image": image,
                "boxes": boxes,
                "labels": labels
            }
        
        elif self.task == "depth_estimation":
            depth_path = self.label_paths[idx]
            
            # Load image
            image = Image.open(rgb_path).convert('RGB')
            
            # Load depth map
            if depth_path.endswith('.exr'):
                depth = self.load_exr_depth(depth_path)
            else:
                depth = np.load(depth_path)  # Load from .npy file
            
            # Apply transforms
            if self.transform:
                image = self.transform(image)
            
            # Convert depth to tensor
            depth_tensor = torch.from_numpy(depth).unsqueeze(0).float()  # Add channel dimension
            
            return {
                "image": image,
                "depth": depth_tensor
            }

def create_training_dataloader(data_dir, batch_size=8, task="object_detection"):
    """
    Create a DataLoader for Isaac synthetic data
    """
    # Define transforms
    transform = transforms.Compose([
        transforms.Resize((224, 224)),  # Resize to standard size
        transforms.ToTensor(),
        transforms.Normalize(mean=[0.485, 0.456, 0.406], 
                            std=[0.229, 0.224, 0.225])  # ImageNet normalization
    ])
    
    # Create dataset
    dataset = IsaacSyntheticDataset(data_dir, transform=transform, task=task)
    
    # Create data loader
    dataloader = DataLoader(
        dataset,
        batch_size=batch_size,
        shuffle=True,
        num_workers=4,
        pin_memory=True
    )
    
    return dataloader

# Example usage in training
def train_with_synthetic_data():
    """
    Example of using synthetic data in training
    """
    # Create data loader
    dataloader = create_training_dataloader(
        "synthetic_datasets/object_detection",
        batch_size=16,
        task="object_detection"
    )
    
    # Load model (example with torchvision)
    import torchvision
    from torchvision.models.detection import fasterrcnn_resnet50_fpn
    
    model = fasterrcnn_resnet50_fpn(pretrained=False, num_classes=11)  # 10 object classes + background
    model.train()
    
    optimizer = torch.optim.SGD(model.parameters(), lr=0.001, momentum=0.9)
    
    # Training loop
    for epoch in range(10):
        for batch_idx, batch in enumerate(dataloader):
            optimizer.zero_grad()
            
            # Forward pass
            images = batch["image"]
            targets = [{"boxes": b, "labels": l} for b, l in zip(batch["boxes"], batch["labels"])]
            
            loss_dict = model(images, targets)
            losses = sum(loss for loss in loss_dict.values())
            
            # Backward pass
            losses.backward()
            optimizer.step()
            
            if batch_idx % 100 == 0:
                print(f"Epoch {epoch}, Batch {batch_idx}, Loss: {losses.item():.4f}")
```

## Transfer Learning from Synthetic to Real Data

One of the key goals of synthetic data generation is to enable transfer learning to real-world robotics applications:

```python
def validate_transfer_learning_performance(
    synthetic_model_path,
    real_world_test_data,
    metrics=["accuracy", "mAP", "IoU"]
):
    """
    Validate how well synthetic-trained models perform on real data
    """
    # Load the model trained on synthetic data
    model = torch.load(synthetic_model_path)
    model.eval()
    
    # Test on real-world data
    real_dataloader = create_real_world_dataloader(real_world_test_data)
    
    results = {}
    
    with torch.no_grad():
        total_iou = 0
        total_samples = 0
        
        for batch in real_dataloader:
            images = batch["image"]
            real_targets = batch["targets"]  # Real ground truth
            
            # Get predictions from model
            predictions = model(images)
            
            # Calculate IoU between predictions and real targets
            for pred, target in zip(predictions, real_targets):
                iou = calculate_prediction_iou(pred, target)
                total_iou += iou
                total_samples += 1
    
    results["average_iou"] = total_iou / total_samples if total_samples > 0 else 0
    results["transfer_success_rate"] = results["average_iou"] > 0.5  # Threshold for success
    
    return results

def domain_adaptation_finetuning(
    synthetic_trained_model,
    small_real_dataset,
    epochs=5
):
    """
    Fine-tune synthetic-trained model on small amount of real data
    """
    model = synthetic_trained_model
    model.train()
    
    # Use a lower learning rate for fine-tuning
    optimizer = torch.optim.Adam(model.parameters(), lr=0.0001)
    
    dataloader = create_real_world_dataloader(small_real_dataset, batch_size=4)  # Smaller batches for real data
    
    for epoch in range(epochs):
        for batch_idx, batch in enumerate(dataloader):
            optimizer.zero_grad()
            
            images = batch["image"]
            targets = batch["targets"]
            
            loss_dict = model(images, targets)
            losses = sum(loss for loss in loss_dict.values())
            
            losses.backward()
            optimizer.step()
            
            if batch_idx % 10 == 0:
                print(f"Fine-tuning - Epoch {epoch}, Batch {batch_idx}, Loss: {losses.item():.4f}")
    
    return model
```

## Chapter Summary

Synthetic data generation with Isaac Sim provides a powerful capability for training humanoid robot AI systems. The combination of photorealistic rendering, accurate physics simulation, and domain randomization techniques enables the creation of diverse, labeled datasets that can significantly reduce the need for expensive real-world data collection while improving model robustness.

Key benefits include:
- Safe, rapid data generation for dangerous scenarios
- Perfect ground truth for training perception systems
- Infinite variation for testing edge cases
- Reduced real-world data collection costs
- Improved model transferability through domain randomization

Proper validation is essential to ensure synthetic models perform well on real-world tasks, and techniques like domain adaptation can further improve transfer performance.

## Learning Objectives

After completing this chapter, students should be able to:
- Explain the importance of synthetic data for humanoid robot AI development
- Configure Isaac Sim for realistic sensor simulation
- Implement domain randomization for improved model robustness
- Create synthetic datasets for specific robotics tasks (object detection, depth estimation, etc.)
- Validate synthetic data quality and model transfer performance
- Integrate synthetic datasets with ML training pipelines
- Apply domain adaptation techniques to bridge synthetic-to-real gaps