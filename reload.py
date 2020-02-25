import cv2
import math
import time
import numpy as np

class Blob:
    def __init__ (self, x_, y_, w_, h_):
        self.x_ = x_
        self.y_ = y_
        self.w_ = w_
        self.h_ = h_

    def x (self):
        return self.x_

    def y (self):
        return self.y_

    def w (self):
        return self.w_

    def h (self):
        return self.h_

    def cx (self):
        return int (self.x_ + self.w_ / 2)

    def cy (self):
        return int (self.y_ + self.h_ / 2)

    def rect (self):
        return (self.x_, self.y_, self.w_, self.h_)

class Image:
    def __init__ (self, img_):
        self.img = img_.copy ()

    def find_blobs (self, th, pixels_threshold, area_threshold, merge):
        low_th  = (int (th [0] * 2.55), th[2]+128, th [4]+128)
        high_th = (int (th [1] * 2.55), th[3]+128, th [5]+128)

        labimg = cv2.cvtColor (self.img, cv2.COLOR_BGR2LAB)
        #labimg = self.img

        mask = cv2.inRange (labimg, low_th, high_th)

        output = cv2.connectedComponentsWithStats (mask, 8, cv2.CV_32S)

        labels_count = output      [0]
        labels       = output      [1]
        stats        = output      [2]
        sz           = stats.shape [0]

        blobs = []

        for label_num in range (1, sz):
            area = stats [label_num, cv2.CC_STAT_AREA]
            
            if (area >= pixels_threshold):
                new_blob = Blob (stats [label_num, cv2.CC_STAT_LEFT],
                                 stats [label_num, cv2.CC_STAT_TOP],
                                 stats [label_num, cv2.CC_STAT_WIDTH],
                                 stats [label_num, cv2.CC_STAT_HEIGHT])

                #print ("append", area)
                blobs.append (new_blob)

        return blobs

    def draw_rectangle (self, rect):
        (x, y, w, h) = rect
        #cont_img = np.ascontiguousarray (self.img, dtype=np.uint8)
        cv2.rectangle (self.img, (x, y), (x+w, y+h), (255, 0, 0), 3)
        #return cont_img

class Sensor:
    def __init__ (self, filename_):
        self.filename = filename_
        self.img = cv2.imread (self.filename)

    def snapshot (self):
        return Image (self.img)

def main ():
    sensor = Sensor ("rgb_basket.jpg")

    while (True):
        img = sensor.snapshot ()

        blobs = img.find_blobs ((35, 50, 15, 75, 50, 135), 200, 20, True, 10)

        for blob in blobs:
            img.draw_rectangle (blob.rect ())

        cv2.imshow ("objects", img.img)

        time.sleep (0.02)
        
        keyb = cv2.waitKey (1) & 0xFF
        
        if (keyb == ord('q')):
            break

    cv2.destroyAllWindows()

if __name__ == "__main__":
    main ()