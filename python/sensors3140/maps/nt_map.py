from sensors3140.maps.base_map_display import BaseMapDisplay
import sensors3140.tables.network_tables as nt
import cv2

class NTMapDisplay(BaseMapDisplay):
    """
    NTMapDisplay class that updates and displays the map using NetworkTables data.
    """
    def __init__(self, game_id):
        """
        Initialize the NTMapDisplay
        
        Args:
            game_id: ID of the game field map to use
        """
        super().__init__(game_id)
        self.timestamp = -1.0
        self.tables = nt.NetworkTablesManager()

    def set_current_detections(self, tag_ids, distances, bearings):
        """
        Set the current detections
        
        Args:
            tag_ids: List of detected tag IDs
            distances: List of distances to the detected tags
            bearings: List of bearings to the detected tags
        """
        try:
            assert len(tag_ids) == len(distances) == len(bearings), "Tag, distance, and bearing arrays must be the same length"
            self.tag_ids = tag_ids
            self.distances = distances
            self.bearings = bearings
        except:
            print("Warning: Could not set AprilTag Detections")

    def update(self):
        """
        Update the map display from NetworkTables data
        """
        camera_ids = self.tables.getCameraIds()
        for camera_id in camera_ids:
            tag_ids = self.tables.getIntegerArray(f"sensors3140/apriltags/{camera_id}/ids", [])
            distances = self.tables.getDoubleArray(f"sensors3140/apriltags/{camera_id}/distances", [])
            bearings = self.tables.getDoubleArray(f"sensors3140/apriltags/{camera_id}/bearings", [])
            self.set_current_detections(tag_ids, distances, bearings)

            best_camera_translation = self.tables.getDoubleArray(f"sensors3140/apriltags/{camera_id}/camera_position")
            best_camera_direction = self.tables.getDoubleArray(f"sensors3140/apriltags/{camera_id}/camera_direction")
            best_camera_pose_tag = self.tables.getInteger(f"sensors3140/apriltags/{camera_id}/camera_position_tag", -1)
            best_camera_timestamp = self.tables.getDouble(f"sensors3140/apriltags/{camera_id}/camera_position_timestamp", -1.0)
            camera_timestamp = self.tables.getDouble(f"sensors3140/apriltags/{camera_id}/timestamp", -1.0)
            elapsed_time = camera_timestamp - best_camera_timestamp

            best_camera_distance = None
            if best_camera_pose_tag in tag_ids:
                # Get the index of the tag in the list of tags
                tag_index = tag_ids.index(best_camera_pose_tag)
                # Get the distance and bearing to the tag
                best_camera_distance = distances[tag_index]

            if best_camera_translation is not None and len(best_camera_translation) == 3:
                self.update_camera_position(camera_id, best_camera_translation, best_camera_direction, best_camera_pose_tag, tag_distance=best_camera_distance, elapsed_time=elapsed_time)

if __name__ == "__main__":
    import argparse

    def parse_arguments():
        """
        Parse command line arguments
        
        Returns:
            argparse.Namespace: Parsed arguments
        """
        parser = argparse.ArgumentParser(description='AprilTag Field Map Display')
        parser.add_argument('--server', default='127.0.0.1', help='NetworkTables server address')
        parser.add_argument('--game', default='2025-reefscape', help='Game ID for field map')
        return parser.parse_args()

    print('Creating Map...')
    args = parse_arguments()

    tables = nt.NetworkTablesManager(args.server)

    while not tables.is_connected():
        print("Looking for connection.")

    display = NTMapDisplay(args.game)
    display.load()
    print(f'Loaded {args.game} map...')
    print(f'Connected to NetworkTables server: {args.server}')

    while True:
        display.update()
        display.display()
        key = cv2.waitKey(1)

        if key == ord('q'):
            break

    print("Closing Map")
    cv2.destroyAllWindows()