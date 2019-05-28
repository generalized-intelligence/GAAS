from __future__ import print_function
import pyzbar.pyzbar as pyzbar
import numpy as np
import cv2


class QRdetect:

    def __init__(self, QRcode_image, k=np.array([[376.0, 0, 376], [0, 376.0, 240.0], [0, 0, 1]])):

        if QRcode_image is None:
            return

        self.query_image = QRcode_image
        self.K = k

        # points order is:
        # A-------D
        # |       |
        # |       |
        # B-------C
        self.query_image_position = self.QRcode_Position(self.query_image)
        print("query QRcode position: ", self.query_image_position)

        query_image = self.drawPoints(self.query_image.copy(), self.query_image_position)
        cv2.imwrite("query_detected.png", query_image)

        self.train_qrcode_position = None
        self.found_QRcode = False


    def decode(self, image):

        decodedObjects = pyzbar.decode(image)
        if not decodedObjects:
            return None

        # There could be multiple qr codes in the same image
        # each detected qr code has a type and corresponding polygon
        for obj in decodedObjects:
            print('Type : ', obj.type)
            print('Polygon : ', obj.polygon)
            if obj.type is not "QRCODE":
                return None

        return decodedObjects

    # Display barcode and QR code location
    def display(self, image):

        decodedObjects = self.decode(image)

        # Loop over all decoded objects
        for decodedObject in decodedObjects:
            points = decodedObject.polygon

            # If the points do not form a quad, find convex hull
            if len(points) > 4:
                hull = cv2.convexHull(np.array([point for point in points], dtype=np.float32))
                hull = list(map(tuple, np.squeeze(hull)))
            else:
                hull = points

            # Number of points in the convex hull
            n = len(hull)

            # Draw the convext hull
            for j in range(0, n):
                cv2.line(image, hull[j], hull[(j + 1) % n], (255, 0, 0), 3)

        # Display results
        # image = cv2.resize(image, (np.shape(image)[1] / 3, np.shape(image)[0] / 3), interpolation=cv2.INTER_CUBIC)
        # cv2.imshow("Results", image)
        # cv2.waitKey(5000)

    def QRcode_Position(self, image):

        qrcode_results = self.decode(image)
        if not qrcode_results:
            return None

        if len(qrcode_results) > 0:
            # assume there is only one qr code in the FOV
            qrcode_result = qrcode_results[0]
            polygon = qrcode_result.polygon
            print("polygon: ", polygon)
            qrcode_position = np.array([[polygon[0][0], polygon[0][1]],
                                        [polygon[1][0], polygon[1][1]],
                                        [polygon[2][0], polygon[2][1]],
                                        [polygon[3][0], polygon[3][1]]], dtype="float32")

            print(qrcode_position)
            return qrcode_position

        else:

            return None


    def drawPoints(self, image, target_qrcode_position):

        if target_qrcode_position is None:
            return

        for p in target_qrcode_position:
            image = cv2.circle(image, (p[0], p[1]), 10, (0, 255, 0))
            image = cv2.putText(image,
                                str(p[0]) + ", " + str(p[1]),
                                (p[0], p[1]),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                                (0, 0, 255), 1, cv2.LINE_AA)
        return image


    def getPerspectiveTransformaAndWarpedImage(self, train_qrcode_image):

        train_qrcode_position = self.QRcode_Position(train_qrcode_image)
        # print("Incoming train image qr code position: ", train_qrcode_position)

        if train_qrcode_position is None:
            return None, None

        train_image = self.drawPoints(train_qrcode_image.copy(), train_qrcode_position)
        # print("train_qrcode_position: \n", train_qrcode_position)

        if train_image is not None:
            cv2.imwrite("train_detected.png", train_image)

        # train_qrcode_position: current incoming image
        # query_image_position: pre-set target qrcode image
        #H = cv2.getPerspectiveTransform(train_qrcode_position, self.query_image_position)
        H = cv2.getPerspectiveTransform(train_qrcode_position, self.query_image_position)

        self.train_qrcode_position = train_qrcode_position

        (h, w, c) = train_qrcode_image.shape
        warped_image = cv2.warpPerspective(train_qrcode_image, H, (w, h))
        cv2.imwrite("warped_image.png", warped_image)

        return H, warped_image

    # from train image to query image
    def recoverRTfromHomographyMat(self, H):
        _, Rotations, translations, normals = cv2.decomposeHomographyMat(H, self.K)

        return Rotations, translations, normals


    # def process_image(self, image):
    #
    #     H, warped_image = self.getPerspectiveTransformaAndWarpedImage(image)
    #
    #     if H is None:
    #         self.found_QRcode = False
    #         return None, None, None
    #
    #     Rs, ts, ns = self.recoverRTfromHomographyMat(H)
    #     self.found_QRcode = True
    #
    #     train_image_camera_points = self.pixel2cameraWithoutScale(self.train_qrcode_position)
    #
    #     # return Rs, ts, ns
    #     new_Rs = []
    #     new_ts = []
    #     new_normals = []
    #     idxs = []
    #     for index, normal in enumerate(ns):
    #         for pt in train_image_camera_points:
    #             result = np.dot(normal.T, np.array(pt))
    #             if result < 0:
    #                 continue
    #             else:
    #                 idxs.append(index)
    #
    #     idxs = set(idxs)
    #     for idx in idxs:
    #         new_Rs.append(Rs[idx])
    #         new_ts.append(ts[idx])
    #         new_normals.append(ns[idx])
    #
    #     return new_Rs, new_ts, new_normals

    def process_image(self, image):

        H, warped_image = self.getPerspectiveTransformaAndWarpedImage(image)

        if H is None:
            self.found_QRcode = False
            return None, None, None

        Rs, ts, ns = self.recoverRTfromHomographyMat(H)
        self.found_QRcode = True

        return Rs, ts, ns


    def pixel2cameraWithoutScale(self, pixel_points):

        camera_points = []
        camera_point = [0, 0, 0]
        for pixel in pixel_points:

            # assume depth is 1
            depth = 1
            camera_point[0] = (pixel[0] - 376.0) / 376.0
            camera_point[1] = (pixel[1] - 376.0) / 240.0
            camera_point[2] = depth

            camera_points.append(camera_point)

        return camera_points



if __name__ == '__main__':

    train_image = cv2.imread('target.png')
    query_image = cv2.imread('target.png')

    cv2.imwrite("train_image.png", train_image)

    qr = QRdetect(query_image)
    R, t = qr.process_image(train_image)

    print("R: ", R)
    print("t: ", t)



