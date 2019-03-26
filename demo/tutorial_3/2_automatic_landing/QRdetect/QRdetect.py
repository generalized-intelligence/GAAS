from __future__ import print_function
import pyzbar.pyzbar as pyzbar
import numpy as np
import cv2



class QRdetect:

  def __init__(self, QRcode_w, QRcode_h, QRcode_image):

    self.QRcode_w = QRcode_w
    self.QRcode_h = QRcode_h
    self.QRcode_image = QRcode_image

    # points order is:
    # A-------D
    # |       |
    # |       |
    # B-------C
    self.Default_QRcode_Position = self.QRcode_Position(self.QRcode_image)

    print("default QRcode position: \n", self.Default_QRcode_Position)

    self.found_QRcode = False


  def decode(self, image) :
    decodedObjects = pyzbar.decode(image)

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
      if len(points) > 4 :
        hull = cv2.convexHull(np.array([point for point in points], dtype=np.float32))
        hull = list(map(tuple, np.squeeze(hull)))
      else :
        hull = points

      # Number of points in the convex hull
      n = len(hull)

      # Draw the convext hull
      for j in range(0,n):
        cv2.line(image, hull[j], hull[ (j+1) % n], (255,0,0), 3)

    # Display results
    image = cv2.resize(image, (np.shape(image)[1]/3, np.shape(image)[0]/3), interpolation=cv2.INTER_CUBIC)
    # cv2.imshow("Results", image)
    # cv2.waitKey(5000)


  def QRcode_Position(self, image):

    qrcode_results = self.decode(image)
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

    return qrcode_position


  def QRcode_Position2(self, image):

    qrcode_results = self.decode(image)
    qrcode_result = qrcode_results[0]
    polygon = qrcode_result.polygon
    qrcode_position = np.array([[polygon[3][0], polygon[3][1]],
                                [polygon[0][0], polygon[0][1]],
                                [polygon[1][0], polygon[1][1]],
                                [polygon[2][0], polygon[2][1]]], dtype="float32")


  def changeOrder(self, points):
    temp = points.copy()
    temp[0] = points[3]
    temp[1] = points[0]
    temp[2] = points[1]
    temp[3] = points[2]

    print("cccc", points)
    print("aaad", temp)

    return temp


  def drawPoints(self, image, target_qrcode_position):
    for p in target_qrcode_position:
      print("p: ", p)
      image = cv2.circle(image, (p[0], p[1]), 25, (0, 255, 0))
      image = cv2.putText(image, str(p[0])+", "+str(p[1]), (p[0], p[1]), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)
    return image

  # Given QRcode width and height, we can infer perspective transformation
  def getPerspectiveTransformaAndWarpedImage(self, target_qrcode_image):

    target_qrcode_position = self.QRcode_Position(target_qrcode_image)
    print("target_qrcode_position: \n", target_qrcode_position)

    temp_image = self.drawPoints(target_qrcode_image.copy(), target_qrcode_position)
    cv2.imwrite("circles.png", temp_image)


    temp_image = cv2.resize(self.QRcode_image.copy(), (600, 600))
    temp_image = self.drawPoints(temp_image, self.Default_QRcode_Position)
    cv2.imwrite("circles2.png", temp_image)


    x=[]
    y=[]
    for p in target_qrcode_position:
      x.append(p[0])
      y.append(p[1])
    min_x = int(min(x))
    max_x = int(max(x))
    min_y = int(min(y))
    max_y = int(max(y))

    print("min x: ", min_x)
    print("max x: ", max_x)
    print("min y: ", min_y)
    print("max y: ", max_y)

    cropped_image = target_qrcode_image[min_y:max_y, min_x:max_x]
    cv2.imwrite("cropped_image.png", cropped_image)
    cropped_image_position = self.QRcode_Position(cropped_image)
    print("cropped_image_position", cropped_image_position)
    temp_image = self.drawPoints(cropped_image, cropped_image_position)
    cv2.imwrite("cropped_with_points.png", temp_image)



    cropped_image_position = self.changeOrder(cropped_image_position)



    M = cv2.getPerspectiveTransform(cropped_image_position, self.Default_QRcode_Position)
    # M = cv2.getPerspectiveTransform(self.Default_QRcode_Position, cropped_image_position)

    (h, w, c) = cropped_image.shape
    #warped_image = cv2.warpPerspective(target_qrcode_image, M, (w, h))
    warped_image = cv2.warpPerspective(cropped_image, M, (w, h))
    # warped_image = cv2.warpPerspective(target_qrcode_image, M, (target_qrcode_image.shape[1], target_qrcode_image.shape[0]))
    print("(target_qrcode_image.shape[1], target_qrcode_image.shape[0])", (target_qrcode_image.shape[1], target_qrcode_image.shape[0]))

    return M, warped_image

   
# Main 
if __name__ == '__main__':
 
  # Read image
  im = cv2.imread('1.jpg')
  print("input image size: ", im.shape)
  (h, w, c) = im.shape

  qrdetect = QRdetect(w, h, im)
  results = qrdetect.decode(im)
  # qrdetect.display(im, results)

  target_image = cv2.imread("3.jpeg")
  qrdetect.display(target_image)
  M, warped_image = qrdetect.getPerspectiveTransformaAndWarpedImage(target_image)

  print("calculated perspective matrix: \n", M)

  K = np.array([[360, 0, 385],
                [0, 359, 213],
                [0, 0, 1]])

  _, Rs, ts, normals = cv2.decomposeHomographyMat(M, K)


  for R in Rs:
    print("decomposed R: \n", R)

  for t in ts:
    print("decomposed t: \n", t)

  for n in normals:
    print("decomposed n: \n", n)


  cv2.imwrite("original_image.png", im)
  cv2.imwrite("warped_image.png", warped_image)
  #cv2.waitKey(20000)
