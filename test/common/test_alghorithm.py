import unittest
from algoritms import multilateration

class TestAlgorithms(unittest.TestCase):
    def setup_method(self, anchors, position=(0.5, 0.5, 0.5)):
        """
        Setup method for trilateration tests.
        :param anchors: Dictionary of anchor positions {id: (x, y, z)}
        :param position: Expected position (x, y, z)
        """
        raw_data = {}
        print(position)
        print(anchors)
        for i in anchors.keys():
            raw_data[i] = 0
            for j in range(len(position)):
                raw_data[i] += (position[j] - anchors[i][j])**2
            raw_data[i] = raw_data[i]**0.5
        x, y, z = multilateration(raw_data, anchors)
        print(f"Calculated position: x={x}, y={y}, z={z}")
        print(f"Expected position: x={position[0]}, y={position[1]}, z={position[2]}")
        self.assertAlmostEqual(x, position[0], delta=0.1)
        self.assertAlmostEqual(y, position[1], delta=0.1)
        self.assertAlmostEqual(z, position[2], delta=0.1)

    def test_3_point(self):
        anchors = {
            1: (0, 0, 0),
            2: (1, 0, 0),
            3: (0, 1, 0),
            4: (0, 0, 1),
        }
        self.setup_method(anchors)

    def test_4_point(self):
        anchors = {
            1: (0, 0, 0),
            2: (1, 0, 0),
            3: (0, 1, 0),
            4: (0, 0, 1),
        }
        self.setup_method(anchors)

    def test_trilateration(self):
        anchor_positions = {
            1: (0, 0, 0),
            2: (1, 0, 0),
            3: (0, 1, 0),
            4: (1, 1, 0),
            5: (0, 0, 1),
            6: (1, 0, 1),
            7: (0, 1, 1),
            8: (1, 1, 1),
            9: (0, 0, 2),
            10: (1, 0, 2),
        }
        self.setup_method(anchor_positions)

    def test_with_one_anchor(self):
        anchor_positions = {
            1: (1, 1, 1),
        }
        self.assertRaises(ValueError, self.setup_method, anchor_positions)

    def test_with_no_anchors(self):
        anchor_positions = {}
        self.assertRaises(ValueError, self.setup_method, anchor_positions)
    
    def test_with_two_anchors(self):
        anchor_positions = {
            1: (0, 0, 0),
            2: (1, 0, 0),
        }
        self.assertRaises(ValueError, self.setup_method, anchor_positions)
        # self.setup_method(anchor_positions)

if __name__ == '__main__':
    unittest.main()
