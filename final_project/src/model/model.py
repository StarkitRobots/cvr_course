import math


class Model:
    def __init__(self):
        self.camera_pan = 0
        self.camera_tilt = 0
        self.active_servos = {}

    def update_camera_pan_tilt(self, camera_pan, camera_tilt):
        self.camera_pan = camera_pan
        self.camera_tilt = camera_tilt

    def set_parameters(self, cam_matrix, h, k_coefs, p_coefs):
        # cam_matrix - camera matrix
        # h - camera height
        # k_coefs, p_coefs - coefficients of lense distortion
        self.cam_matrix = cam_matrix
        self.h = h
        self.k_coefs = k_coefs
        self.p_coefs = p_coefs
        self.head_to_base_matrix = [[1., 0., 0.],
                                    [0., 1., 0.],
                                    [0., 0., 1.]]

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
                             math.sin(a) * math.cos(b),
                             math.sin(b)],
                            [-math.sin(a),
                             math.cos(a),
                             0.0],
                            [-math.sin(b) * math.cos(a),
                             -math.sin(a) * math.sin(b),
                             math.cos(b)]]

        return self.matrix_dot_vec(r2cam_rot_matrix, r_coords)

    # transformation from camera coords to robot coords
    def cam2r(self, cam_coords):
        a = self.camera_pan
        b = self.camera_tilt

        cam2r_rot_matrix = [[math.cos(a) * math.cos(b),
                             -math.sin(a),
                             -math.cos(a) * math.sin(b)],
                            [math.sin(a) * math.cos(b),
                             math.cos(a),
                             -math.sin(a) * math.sin(b)],
                            [math.sin(b),
                             0.0,
                             math.cos(b)]]

        return self.matrix_dot_vec(cam2r_rot_matrix, cam_coords)


    def distortion_straight(self, x, y):
        r_sq = x ** 2 + y ** 2
        coef_numerator = (1 +
                          self.k_coefs[0] * r_sq +
                          self.k_coefs[1] * (r_sq ** 2) +
                          self.k_coefs[2] * (r_sq ** 3))

        coef_denominator = (1 +
                            self.k_coefs[3] * r_sq +
                            self.k_coefs[4] * (r_sq ** 2) +
                            self.k_coefs[5] * (r_sq ** 3))

        g = coef_numerator / coef_denominator

        # x_cor, y_cor - ball cords in artificially created coord system
        x_cor_parts = [x * g,
                       2 * self.p_coefs[0] * x * y,
                       self.p_coefs[1] * (r_sq + 2 * x ** 2)]
        x_cor = sum(x_cor_parts)

        y_cor_parts = [y * g,
                       2 * self.p_coefs[1] * x * y,
                       self.p_coefs[0] * (r_sq + 2 * y ** 2)]
        y_cor = sum(y_cor_parts)

        return [x_cor, y_cor]

    def dist_error(self, x, y, c1, c2):
        r1, r2 = self.distortion_straight(x, y)
        return [r1 - c1, r2 - c2]

    def dist_er_jac_inv(self, x, y):
        r_sq = x ** 2 + y ** 2
        coef_numerator = (1 +
                          self.k_coefs[0] * r_sq +
                          self.k_coefs[1] * (r_sq ** 2) +
                          self.k_coefs[2] * (r_sq ** 3))

        coef_denominator = (1 +
                            self.k_coefs[3] * r_sq +
                            self.k_coefs[4] * (r_sq ** 2) +
                            self.k_coefs[5] * (r_sq ** 3))

        g = coef_numerator / coef_denominator
        gdx = ((2 * self.k_coefs[0] * x + 
            4 * self.k_coefs[1] * x * r_sq +
            6 * self.k_coefs[2] * x * r_sq**2) / 
            coef_denominator - 
            (coef_numerator * 
            (2 * self.k_coefs[3] * x + 
                4 * self.k_coefs[4] * x * r_sq +
                6 * self.k_coefs[5] * x * r_sq**2)) /
            coef_denominator**2)

        gdy = ((2 * self.k_coefs[0] * y + 
            4 * self.k_coefs[1] * y * r_sq +
            6 * self.k_coefs[2] * y * r_sq**2) / 
            coef_denominator - 
            (coef_numerator * 
            (2 * self.k_coefs[3] * y + 
                4 * self.k_coefs[4] * y * r_sq +
                6 * self.k_coefs[5] * y * r_sq**2)) /
            coef_denominator**2)

        a11 = g + x * gdx + 2 * self.p_coefs[0] * y + 6 * self.p_coefs[1] * x
        a12 = y * gdx + 2 * self.p_coefs[0] * x + 2 * self.p_coefs[1] * y
        a21 = x * gdy + 2 * self.p_coefs[0] * x + 2 * self.p_coefs[1] * y
        a22 = g + y * gdy + 2 * self.p_coefs[1] * x + 6 * self.p_coefs[0] * y

        det = a11 * a22 - a12 * a21
        return [[a22 / det, -a12 / det], [-a21 / det, a11 / det]]


    def distortion_inverse(self, c1, c2):
        x = c1
        y = c2
        for i in range(100):
            x_new = (x - 
                    (self.dist_er_jac_inv(x, y)[0][0] * self.dist_error(x, y, c1, c2)[0] +
                     self.dist_er_jac_inv(x, y)[0][1] * self.dist_error(x, y, c1, c2)[1]))
            y_new = (y -
                    (self.dist_er_jac_inv(x, y)[1][0] * self.dist_error(x, y, c1, c2)[0] +
                     self.dist_er_jac_inv(x, y)[1][1] * self.dist_error(x, y, c1, c2)[1]))

            if abs(x_new - x) + abs(y_new - y) < 0.00001:
                break
            else:
                x = x_new
                y = y_new
        return [x, y]


    def r2pic(self, xb, yb):
        # xb, yb - coords of the ball in the robot system

        # transformation to camera coords and back to apply head-to-base matrix
        cam_coords = self.r2cam(xb, yb)
        x_c, y_c, z_c = self.matrix_dot_vec(self.head_to_base_matrix,
                                               cam_coords)
        x = y_c / x_c
        y = z_c / x_c

        # applying the lense distortion-fix formula
        x_cor, y_cor = self.distortion_straight(x, y)

        # u, v - pixel coords of the ball
        u = -x_cor * self.cam_matrix[0][0] + self.cam_matrix[0][2]
        v = -y_cor * self.cam_matrix[1][1] + self.cam_matrix[1][2]
        return (int(u), int(v))


    def pic2r(self, u, v):
        # u,v - pixel coords of the ball in the screen system

        # x,y - coords of the ball in the coordinate system,
        # parallel to the camera screen
        x = -(float(u) - self.cam_matrix[0][2]) / self.cam_matrix[0][0]
        y = -(float(v) - self.cam_matrix[1][2]) / self.cam_matrix[1][1]

        # transformation of x and y using the distortion coefficients
        x, y = self.distortion_inverse(x, y)

        x_cam = -self.h / (math.sin(self.camera_tilt) + y * math.cos(self.camera_tilt))
        x_cam = abs(x_cam)
        y_cam = x_cam * x
        z_cam = x_cam * y

        x_r, y_r, _ = self.cam2r([x_cam, y_cam, z_cam])

        return (x_r, y_r)