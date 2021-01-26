import cv2
from pathlib import Path
import numpy as np



def get_available_cameras(upper_bound=10, lower_bound=0):
    available = []

    for i in range(lower_bound, upper_bound):
        cap = cv2.VideoCapture(i)

        if cap.isOpened():
            available.append(i)

        cap.release()

    return available


def folder_files(path):
    files_png = sorted(Path(path).glob('*.png'))
    files_jpg = sorted(Path(path).glob('*.jpg'))
    files_bmp = sorted(Path(path).glob('*.bmp'))

    files = files_png + files_jpg + files_bmp

    return files

# Incapsulates reading frames from the following types of sources:
# photo (single image), photos series, video, camera, ROS node (?)

# If an input file or camera number is given, desired input type
# can be omitted. Numeric value given in initialization is considered
# as a number of camera desired to be opened. Value "-1" opens camera
# with minimal available id, "-2" - with maximal.

# TODO: semi-automatic source type detection
# TODO: implementation of reading from all the sources (except for ROS) [later]
# TODO: output [later]


class Source:
    #type = ""
    #path = ""

    # Taking sample image requires reading, in case of non-constant
    # sources like camera or video it can lead to a loss of a single
    # frame. These is the fix :)
    sample_image_obtained = False
    sample_image_incoherency_fixed = False

    def __init__(self, path_, type_="", instant_init=True):
        if (not isinstance(path_, str) and not isinstance(path_, unicode)):
            self.path = path_["data_path"]

        else:
            self.path = path_

        if type_ == "":
            if (self.path.endswith(".jpg") or
                self.path.endswith(".png") or
                    self.path.endswith(".bmp")):
                self.type = "photo"

            elif (self.path.endswith(".webm") or
                  self.path.endswith(".mp4") or
                  self.path.endswith(".MTS") or
                  self.path.endswith(".mkv") or
                  self.path.endswith(".mov") or
                  self.path.endswith(".avi")):
                self.type = "video"

            elif self.path.endswith("/"):
                self.type = "photo series"

            elif (self.path.isnumeric() or
                  self.path[1:].isnumeric()):
                self.type = "camera"

                num = int(self.path)

                if num < 0:
                    cameras = get_available_cameras()

                    if num == -1:
                        self.cam_num = min(cameras)

                    else:
                        self.cam_num = max(cameras)

                else:
                    self.cam_num = num

            else:
                self.type = "ros flex"

        else:
            self.type = type_

        #print (self.type)

        if instant_init:
            self.init_source()

    def shape(self):
        return self.sample_image().shape

    def sample_image(self):
        if not self.sample_image_obtained:
            self.sample_image = self.get_frame()

            self.sample_image_obtained = True

        return self.sample_image

    def init_source(self):
        self.sources = {}

        self.sources.update(
            {"photo": (self.init_photo,        self.get_frame_photo)})
        self.sources.update(
            {"photo series": (self.init_photo_series, self.get_frame_photo_series)})
        self.sources.update(
            {"video": (self.init_video,        self.get_frame_video)})
        self.sources.update(
            {"camera": (self.init_camera,       self.get_frame_camera)})
        #self.sources.update ({"ros flex"     : (self.init_ros_flex,     self.get_frame_ros_flex)})

        self.sources[self.type][0]()

    def init_photo(self):
        self.img = cv2.imread(self.path)
        if self.img is None:
            print("Failed to load img{}".format(self.path))

    def init_photo_series(self):
        self.file_num = 0
        self.files = folder_files(self.path)

        #print (self.files)
        #print (len (self.files), " files")

    def init_video(self):
        self.video = cv2.VideoCapture(self.path)

    def init_camera(self):
        self.camera = cv2.VideoCapture(self.cam_num)

    # def init_photo (self):
    #    self.img = cv2.imread (self.path)

    def get_frame(self):
        if (self.sample_image_obtained and not
            self.sample_image_incoherency_fixed):
            self.sample_image_incoherency_fixed = True

            return True, self.sample_image

        return self.sources[self.type][1]()

    def get_frame_photo(self):
        return True, self.img.copy()

    def get_frame_photo_series(self):
        filename = str(self.files[self.file_num])

        #print (filename)

        img = cv2.imread(filename)

        self.file_num += 1

        if (self.file_num == len(self.files)):
            self.file_num = 0

        #print ("big shap ", img.shape)

        return img

    def get_frame_video(self):
        reading_success, frame = self.video.read()

        if not reading_success:
            self.video.release()
            self.init_video()

            reading_success, frame = self.video.read()

        return reading_success, frame

    def get_frame_camera(self):
        reading_success, frame = self.camera.read()

        return reading_success, frame

    def get_frame_photo(self):
        return self.img.copy()

    def release(self):
        if self.type == "camera":
            self.camera.release()

# output (stream to video file)

# generalize to the desired a by b cells grid (?)
# generalize to the desired acpect ratio (?)


def form_grid(images_, window_x_sz=-1, one_img_x_sz=-1, names=[]):
    images = []
    forms = {
        # (number of images x, number of images y, number of empty images)
        1: [1, 1, 0],
        2: [2, 1, 0],
        3: [3, 1, 0],
        4: [2, 2, 0],
        5: [3, 2, 1],
        6: [3, 2, 0],
        7: [4, 2, 1],
        8: [4, 2, 0],
        9: [3, 3, 0],
        10: [4, 3, 2],
        11: [4, 3, 1],
        12: [4, 3, 0]
    }
    if len(images) <= 12:
        form = forms[len(images_)]
    else:
        print("Can process only 12 images")
        return 0

    #print ("images0 shape", images_)
    shape = images_[0].shape
    if one_img_x_sz != -1:
        rescale_factor = one_img_x_sz/shape[1]
        shape = [int(x*rescale_factor) for x in shape]

    if window_x_sz != -1:
        rescale_factor = window_x_sz/shape[1]/form[0]
        shape = [int(x*rescale_factor) for x in shape]

    #print ("len", len (images_))

    #print ("0", images_ [0].shape)
    #print ("1", images_ [1].shape)
    #print ("2", images_ [2].shape)
    #print ("3", images_ [3].shape)

    for img, i in zip(images_, range(len(images_))):
        #print ("before resize", img.shape)

        img = cv2.resize(img, (shape[1], shape[0]))
        if len(img.shape) == 2:  # gray_scale
            img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        if img.shape[2] == 4:  # rgba
            img = img[:, :, :3]

        if len(names) != 0:
            cv2.putText(img, names[i], (30, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (20, 250, 231), 1, cv2.LINE_AA)

        images.append(img)

    for j in range(form[2]):
        images.append(np.zeros_like(images[0]))

    rows = []
    for i in range(form[1]):
        rows.append(np.concatenate(images[i*form[0]:(i+1)*form[0]], axis=1))
    return np.concatenate(rows)


class Writer:
    def __init__(self, name_, xsz_, ysz_, fps_=30):
        self.name = name_
        self.xsz = xsz_
        self.ysz = ysz_
        self.fps = fps_

        #self.fourcc = cv2.VideoWriter_fourcc(*'MP4V')
        #self.out    = cv2.VideoWriter(self.name, self.fourcc, self.fps, (self.xsz, self.ysz))
        self.fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        self.out = cv2.VideoWriter(
            self.name, self.fourcc, self.fps, (self.xsz, self.ysz))

    def write(self, frame):
        self.out.write(frame)

    def __del__(self):
        self.release()

    def release(self):
        self.out.release()
