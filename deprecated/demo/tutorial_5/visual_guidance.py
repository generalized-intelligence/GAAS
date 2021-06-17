import numpy as np
import cv2 as cv
import os



'''
for more information regarding homography and how to recover translation and rotation, refer to 
this link: 
https://docs.opencv.org/3.4.1/d9/dab/tutorial_homography.html
'''
class visual_guidance:

    '''
    read target image,
    initialize feature detector,
    initialize feature matcher,
    pre-compute target keypoints and descriptors.
    '''
    def __init__(self, query_image_path, k=np.array([[376.0, 0, 376], [0, 376.0, 240], [0, 0, 1]]) ):

        if os.path.exists(query_image_path):
            self.query_image = cv.imread(query_image_path, cv.IMREAD_GRAYSCALE)
        else:
            print("target image path doesn't exit.")
            return

        self.K = k

        self.orb = cv.ORB.create()

        self.index_params = dict(algorithm=0, trees=5)
        self.search_params = dict(checks=50)
        self.flann_matcher = cv.FlannBasedMatcher(self.index_params, self.search_params)
        self.bf_matcher = cv.BFMatcher(cv.NORM_HAMMING)


        self.query_image_features, self.query_descriptors = self.orb.detectAndCompute(self.query_image, None)
        self.query_good_features = []
        self.query_image_kp = None

        self.query_image_kp = cv.drawKeypoints(self.query_image, self.query_image_features, self.query_image_kp, color=(255, 0, 0), flags=0)

        self.train_image = None
        self.train_image_kp = None
        self.train_features = None
        self.train_good_features = []
        self.train_descriptors = None

        self.matches = []

        cv.imwrite("target_image.png", self.query_image)

    def match_image(self, candidate_image):

        if len(np.shape(candidate_image)) is 3:
            candidate_image = cv.cvtColor(candidate_image, cv.COLOR_BGR2GRAY)

        print("match_image")

        self.train_image = candidate_image
        self.train_features, self.train_descriptors = self.orb.detectAndCompute(candidate_image, None)

        self.train_image_kp = cv.drawKeypoints(self.train_image, self.train_features, self.train_image_kp, color=(255, 0, 0))

        cv.imshow("train_image", self.train_image_kp)
        cv.waitKey(20)

        raw_matches = self.bf_matcher.match(self.query_descriptors, self.train_descriptors)
        # raw_matches = sorted(raw_matches, key=lambda x: x.distance)

        good_matches = []
        for match in raw_matches:
            if match.distance < 20:
                good_matches.append(match)

        print("raw matches: ", len(raw_matches))
        print("matched feature number: ", len(good_matches))
        if len(good_matches) < 5:
            return None, None

        self.query_good_features, self.train_good_features = self.filter_features(good_matches)

        matched_image = cv.drawMatches(self.query_image,
                                       self.query_image_features,
                                       self.train_image,
                                       self.train_features,
                                       good_matches,
                                       None)

        cv.imwrite("candidate_image.png", candidate_image)
        cv.imwrite("matched_image.png", matched_image)

        # conduct perspective transform
        H, mask = self.perspectiveTransform(self.query_good_features, self.train_good_features)
        print("calculated matrix: ",  H)

        # find rotation and translation from homography matrix
        _, Rotation, translation, _ = cv.decomposeHomographyMat(H, self.K)

        print("Rotation: ", Rotation)
        print("translation: ", translation)

        return Rotation, translation


    def filter_features(self, matches):
        for match in matches:
            self.query_good_features.append(self.query_image_features[match.queryIdx])
            self.train_good_features.append(self.train_features[match.trainIdx])

        return self.query_good_features, self.train_good_features

    def perspectiveTransform(self, query_pts, train_pts):
        query_array = np.float32([m.pt for m in query_pts]).reshape(-1, 1, 2)
        train_array = np.float32([m.pt for m in train_pts]).reshape(-1, 1, 2)

        matrix, mask = cv.findHomography(query_array, train_array, cv.RANSAC, 5.0)

        return matrix, mask


if __name__ == '__main__':

    # this is the simulation camera instrinstic
    camera_intrinstics = np.array([[376.0, 0.0, 376],
                                   [0.0, 376.0, 240.0],
                                   [0.0, 0.0, 1.0]])

    vg = visual_guidance("calibration_board.png", k=camera_intrinstics)
    image = cv.imread("calibration_board.png")

    vg.match_image(image)
