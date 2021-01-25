import math

class Model:
    def __init__(self):
        self.camera_pan = 0
        self.camera_tilt = 0
        self.active_servos = {}

    def update_camera_pan_tilt(self, camera_pan, camera_tilt):
        self.camera_pan = camera_pan
        self.camera_tilt = camera_tilt

    def setParams(self, cam_matrix, h, k_coefs, p_coefs):
        # cam_matrix - camera matrix
        # h - camera height
        # k_coefs, p_coefs - coefficients of lense distortion
        self.cam_matrix = cam_matrix
        self.h = h
        self.k_coefs = k_coefs
        self.p_coefs = p_coefs
        self.head_to_base_matrix = [[0.99936382, 0.034978, -0.00696373],
                                    [-0.03510436, 0.99920425, -0.01893518],
                                    [0.00629587, 0.01916759, 0.99979646]]

    # function calculating dot product of matrix and vector
    def matrix_dot_vec(self, matr, vect):
        n = len(vect)
        res_vect = []
        for i in range(n):
            res_vect.append(0.0)
            for j in range(n):
                res_vect[i] += matr[i][j] * vect[j]

        return res_vect

    # transformation from robot coords to camera coords
    def r2cam(self, xb, yb):
        r_coords = [xb, yb, -self.h]
        a = self.camera_pan
        b = self.camera_tilt

        r2cam_rot_matrix = [[math.cos(a) * math.cos(b),
                             math.sin(a),
                             -math.cos(a) * math.sin(b)],
                            [-math.sin(a) * math.cos(b),
                             math.cos(a),
                             math.sin(a) * math.sin(b)],
                            [math.sin(b),
                             0.0,
                             math.cos(b)]]

        return self.matrix_dot_vec(r2cam_rot_matrix, r_coords)

    # transformation from camera coords to robot coords
    def cam2r(self, cam_coords):
        a = self.camera_pan
        b = self.camera_tilt

        cam2r_rot_matrix = [[math.cos(a) * math.cos(b),
                             -math.sin(a),
                             math.cos(a) * math.sin(b)],
                            [math.sin(a) * math.cos(b),
                             math.cos(a),
                             math.sin(a) * math.sin(b)],
                            [-math.sin(b),
                             0.0,
                             math.cos(b)]]

        return self.matrix_dot_vec(cam2r_rot_matrix, cam_coords)

    # transformation from robot coords to pixel picture coords
    def r2pic(self, xb, yb):
        # xb, yb - coords of the ball in the robot system

        # transformation to camera coords and back to apply head-to-base matrix
        cam_coords = self.r2cam(xb, yb)
        fixed_cam_coords = self.matrix_dot_vec(self.head_to_base_matrix,
                                               cam_coords)
        xb, yb, zb = self.cam2r(fixed_cam_coords)

        # rotating the coordinate system by the cameraPan
        # xbi,ybi - coords of the ball in the same system after rotation
        xbi = xb * math.cos(self.camera_pan) + yb * math.sin(self.camera_pan)
        ybi = yb * math.cos(self.camera_pan) - xb * math.sin(self.camera_pan)

        # taking into account the case when the ball is behind the camera
        if xbi <= 0:
            print("Ball behind the camera")
            return (-1, -1)

        # alp - artificially created angle, used for simplification
        # x,y - coords of the ball in system, parallel to the camera screen
        # u,v - pixel coords of the ball in the screen system
        alp = math.atan(self.h / xbi) + self.camera_tilt
        x = ybi / math.cos(alp) / math.sqrt(xbi**2 + self.h ** 2)
        y = math.tan(alp)

        # applying the lense distortion-fix formula
        r_sq = x ** 2 + y ** 2
        coef_numerator = (1 +
                          self.k_coefs[0] * r_sq +
                          self.k_coefs[1] * (r_sq ** 2) +
                          self.k_coefs[2] * (r_sq ** 3))

        coef_denominator = (1 +
                            self.k_coefs[3] * r_sq +
                            self.k_coefs[4] * (r_sq ** 2) +
                            self.k_coefs[5] * (r_sq ** 3))

        coef = coef_numerator / coef_denominator

        # x_cor, y_cor - ball cords in artificially created coord system
        x_cor_parts = [x * coef,
                       2 * self.p_coefs[0] * x * y,
                       self.p_coefs[1] * (r_sq + 2 * x ** 2)]
        x_cor = sum(x_cor_parts)

        y_cor_parts = [y * coef,
                       2 * self.p_coefs[1] * x * y,
                       self.p_coefs[0] * (r_sq + 2 * y ** 2)]
        y_cor = sum(y_cor_parts)

        # u, v - pixel coords of the ball
        u = -x_cor * self.cam_matrix[0][0] + self.cam_matrix[0][2]
        v = y_cor * self.cam_matrix[1][1] + self.cam_matrix[1][2]
        return (int(u), int(v))

    # transformation from pixel coords to robot coords
    def pic2r(self, u, v):
        # u,v - pixel coords of the ball in the screen system

        # x,y - coords of the ball in the coordinate system,
        # parallel to the camera screen
        x = (float(u) - self.cam_matrix[0][2]) / self.cam_matrix[0][0]
        y = (float(v) - self.cam_matrix[1][2]) / self.cam_matrix[1][1]

        # alp,bet - vertical and horizontal angles
        # of the ball radius-vector relative to the camera 0,0 direction
        alp = math.atan(y)
        bet = math.atan(x)

        robot_alp = alp - self.camera_tilt

        # xb, yb - coordinates of the ball in the system turned by cameraPan
        xb = self.h / math.tan(robot_alp)
        yb = math.tan(bet) * math.sqrt(self.h ** 2 + xb ** 2)

        # xb_r, yb_r - coordinates of the ball in the robot system
        xb_r = xb * math.cos(self.camera_pan) + yb * math.sin(self.camera_pan)
        yb_r = yb * math.cos(self.camera_pan) - xb * math.sin(self.camera_pan)

        # transformation to camera coords and back to apply head-to-base matrix
        cam_coords = self.r2cam(xb_r, -yb_r)
        fixed_cam_coords = self.matrix_dot_vec(self.head_to_base_matrix,
                                               cam_coords)

        # final coords in robot system
        xb_r, yb_r, zb_r = self.cam2r(fixed_cam_coords)

        return (xb_r, yb_r)
