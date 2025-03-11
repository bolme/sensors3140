import cv2
import numpy as np
import json
import os
from typing import Dict, List, Tuple, Any

GLOBAL_MAP_STORAGE = {}

class FieldMap:
    def __init__(self, field_width: float, field_length: float):
        self.field_width = field_width
        self.field_length = field_length
        self.tag_data: Dict[int, Dict[str, np.ndarray]] = {}

    def add_tag(self, id: int, x: float, y: float, z: float, qw: float, qx: float, qy: float, qz: float) -> None:
        location = np.array([x, y, z])
        quaternion = np.array([qw, qx, qy, qz])
        rotation_matrix = np.array([
            [1 - 2*qy**2 - 2*qz**2, 2*qx*qy - 2*qz*qw, 2*qx*qz + 2*qy*qw],
            [2*qx*qy + 2*qz*qw, 1 - 2*qx**2 - 2*qz**2, 2*qy*qz - 2*qx*qw],
            [2*qx*qz - 2*qy*qw, 2*qy*qz + 2*qx*qw, 1 - 2*qx**2 - 2*qy**2]
        ])
        translation_matrix = np.array([x, y, z])
        field_transform = np.eye(4)
        field_transform[:3, :3] = rotation_matrix
        field_transform[:3, 3] = translation_matrix
        correction = np.eye(4)
        correction[:3, :3] = np.array([[0, 0, -1], [1, 0, 0], [0, -1, 0]], dtype=np.float32)
        tag_transform = np.dot(field_transform, correction)
        self.tag_data[id] = {
            'location': location,
            'quaternion': quaternion,
            'field_transform': field_transform,
            'tag_transform': tag_transform
        }

    def get_tag_location(self, id: int) -> np.ndarray:
        return self.tag_data[id]['location']

    def get_tag_transform(self, id: int) -> np.ndarray:
        return self.tag_data[id]['tag_transform']

    def get_field_size(self) -> Tuple[float, float]:
        return self.field_width, self.field_length

    def get_all_tags(self) -> List[int]:
        return list(self.tag_data.keys())
    
    def get_tag_corners_in_field_coordinates(self, id: int, tag_size=0.1651) -> np.ndarray:
        corners_homogeneous = np.array([
            [-tag_size/2, -tag_size/2, 0, 1],
            [tag_size/2, -tag_size/2, 0, 1],
            [tag_size/2, tag_size/2, 0, 1],
            [-tag_size/2, tag_size/2, 0, 1]
        ]).T
        tag_transform = self.get_tag_transform(id)
        corners = np.dot(tag_transform, corners_homogeneous)
        return corners[:3, :].T


def get_map(game_id: str) -> FieldMap:
    if game_id not in GLOBAL_MAP_STORAGE:
        tags = _load_apriltags(game_id)
        GLOBAL_MAP_STORAGE[game_id] = tags
    return GLOBAL_MAP_STORAGE[game_id]

def _load_apriltags(game_id: str) -> FieldMap:
    json_path = os.path.join(os.path.dirname(__file__), 'data', f"{game_id}.json")
    with open(json_path) as f:
        data = json.load(f)
        field_map = FieldMap(data['field']['width'], data['field']['length'])
        for tag in data['tags']:
            id = tag['ID']
            pose = tag['pose']
            translation = pose['translation']
            rotation = pose['rotation']['quaternion']
            field_map.add_tag(id, translation['x'], translation['y'], translation['z'],
                              rotation['W'], rotation['X'], rotation['Y'], rotation['Z'])
        return field_map

