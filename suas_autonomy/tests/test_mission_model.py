import unittest

from suas_autonomy.mission.model import GeoPoint, MissionPoint, distance_m, offset_point, point_in_polygon, validate_mission


class MissionGeometryTests(unittest.TestCase):
    def test_offset_distance(self):
        origin = GeoPoint(-35.36, 149.16)
        moved = offset_point(origin, 300, 400)
        self.assertAlmostEqual(distance_m(origin, moved), 500, delta=0.1)

    def test_polygon(self):
        polygon = [GeoPoint(0, 0), GeoPoint(0, 1), GeoPoint(1, 1), GeoPoint(1, 0)]
        self.assertTrue(point_in_polygon(GeoPoint(0.5, 0.5), polygon))
        self.assertFalse(point_in_polygon(GeoPoint(2, 2), polygon))

    def test_valid_mission(self):
        points = [
            MissionPoint("TAKEOFF", 0, 0, 60),
            MissionPoint("WAYPOINT", 300, 0, 60),
            MissionPoint("LAND", 0, 0, 0),
        ]
        self.assertEqual(
            validate_mission(
                points,
                min_cruise_altitude_m=45.72,
                max_altitude_m=121.92,
                runway_exception_radius_m=152.4,
            ),
            [],
        )

    def test_rejects_low_remote_waypoint(self):
        points = [
            MissionPoint("TAKEOFF", 0, 0, 60),
            MissionPoint("WAYPOINT", 300, 0, 30),
            MissionPoint("LAND", 0, 0, 0),
        ]
        errors = validate_mission(
            points,
            min_cruise_altitude_m=45.72,
            max_altitude_m=121.92,
            runway_exception_radius_m=152.4,
        )
        self.assertTrue(any("below" in error for error in errors))


if __name__ == "__main__":
    unittest.main()
