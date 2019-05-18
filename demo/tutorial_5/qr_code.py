from __future__ import print_function
import pyzbar.pyzbar as pyzbar
import numpy as np
import cv2


class QRdetect:

    def __init__(self, QRcode_image, k=np.array([[376.0, 0, 376], [0, 376.0, 0], [0, 0, 1]])):

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
        print("default QRcode position: \n", self.query_image_position)

        self.found_QRcode = False


    def decode(self, image):

        decodedObjects = pyzbar.decode(image)

        print(decodedObjects)

        if not decodedObjects:
            return None

        # There could be multiple qr codes in the same image
        # each detected qr code has a type and corresponding polygon
        for obj in decodedObjects:
            print('Type : ', obj.type)
            print('Polygon : ', obj.polygon)

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
        image = cv2.resize(image, (np.shape(image)[1] / 3, np.shape(image)[0] / 3), interpolation=cv2.INTER_CUBIC)
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
            qrcode_position = np.array([[polygon[0][0], polygon[0][1]],
                                        [polygon[1][0], polygon[1][1]],
                                        [polygon[2][0], polygon[2][1]],
                                        [polygon[3][0], polygon[3][1]]], dtype="float32")

            # qrcode_position = np.array([[polygon[3][0], polygon[3][1]],
            #                             [polygon[2][0], polygon[2][1]],
            #                             [polygon[1][0], polygon[1][1]],
            #                             [polygon[0][0], polygon[0][1]]], dtype="float32")

            print(qrcode_position)
            return qrcode_position

        else:

            return None


    def drawPoints(self, image, target_qrcode_position):
        for p in target_qrcode_position:
            image = cv2.circle(image, (p[0], p[1]), 25, (0, 255, 0))
            image = cv2.putText(image, str(p[0]) + ", " + str(p[1]), (p[0], p[1]), cv2.FONT_HERSHEY_SIMPLEX, 1,
                                (0, 0, 255), 2, cv2.LINE_AA)
        return image


    def getPerspectiveTransformaAndWarpedImage(self, train_qrcode_image):

        target_qrcode_position = self.QRcode_Position(train_qrcode_image)
        print("target_qrcode_position: \n", target_qrcode_position)

        train_image = self.drawPoints(train_qrcode_image.copy(), target_qrcode_position)
        cv2.imwrite("train_qr_image_with_points.png", train_image)

        query_image = cv2.resize(self.query_image.copy(), (600, 600))
        query_image = self.drawPoints(query_image, self.query_image_position)
        cv2.imwrite("query_qr_image_with_points.png", query_image)

        H = cv2.getPerspectiveTransform(target_qrcode_position, self.query_image_position)

        (h, w, c) = train_qrcode_image.shape
        warped_image = cv2.warpPerspective(train_qrcode_image, H, (w, h))

        return H, warped_image


    def recoverRTfromHomographyMat(self, H):
        _, Rotation, translation, _ = cv2.decomposeHomographyMat(H, self.K)

        return Rotation, translation


    def process_image(self, image):

        if image is None:
            return None, None

        H, warped_image = self.getPerspectiveTransformaAndWarpedImage(image)

        R, t = self.recoverRTfromHomographyMat(H)

        return R, t


# Main 
if __name__ == '__main__':

    # Read image
    train_image = cv2.imread('1.png')
    query_image = cv2.imread('1.png')

    cv2.imwrite("train_image.png", train_image)

    qr = QRdetect(query_image)
    R, t = qr.process_image(train_image)

    print("R: ", R)
    print("t: ", t)



