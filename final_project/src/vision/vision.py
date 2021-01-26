import json


class Vision:
    """
    """

    def __init__(self):
        pass
    
    def get_ball(self, img):
        """
        Find bounding box for ball

        Args:
            img(Image)

        Returns:
            list[tuple(int, int, int, int)]: return list of all founded balls(cx, cy - center of bounding box; w, h - width, height of bounding box)
        """
        cx, cy, w, h = (0, 0, 0, 0)
        return [(cx, cy, w, h)]
    def get(self, img):
        result = {}
        result['ball'] = self.get_ball(img)
        return result
