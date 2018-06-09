import math
import cv2 as cv
import numpy as np

DEBUG = False

def loadImage(path):
    imgList = open(path)
    imagePath = '../stereo/left/'
    images = []
    for imgName in imgList:
        img = cv.imread(str(imagePath + imgName[:-1]))
        images.append(img)
    return images

def getAllImageCorners(images, patternSize):
    allConers = []
    if DEBUG:
        cv.namedWindow('Corners')
    for image in images:
        image = cv.cvtColor(image,cv.COLOR_BGR2GRAY)
        retval, outCorners = cv.findChessboardCorners(image, patternSize)
        criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        outCorners = cv.cornerSubPix(image, outCorners, (5, 5), (-1, -1), criteria)
        allConers.append(outCorners)
        if DEBUG:
            image = cv.drawChessboardCorners(image, patternSize, outCorners, retval)
            cv.imshow('Corners', image)
            cv.waitKey(500)
    if DEBUG:
        cv.destroyWindow('Corners')
    return allConers

def getObjectCorner(patternSize):
    objectCorner = []
    for y in range(patternSize[1]):
        for x in range(patternSize[0]):
            objectCorner.append((x + 1, y + 1, 0))
    objectCorner = np.array(objectCorner, np.float32).reshape(-1, 3)
    return objectCorner

def calibrate(allImageCorners, patternSize, imageSize):
    objectCorner = getObjectCorner(patternSize)
    objectCorners = []
    for i in range(len(allImageCorners)):
        objectCorners.append(objectCorner)
    ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objectCorners, allImageCorners, imageSize, None, None)
    return mtx, dist, rvecs, tvecs

def undistortAll(images, mtx, dist, path):
    cv.namedWindow('undistorted images')
    cnt = 1
    for image in images:
        dst = cv.undistort(image, mtx, dist)
        cv.imshow('undistorted images', dst)
        cv.imwrite(path + 'left' + str(cnt) + '.jpg', dst )
        cv.waitKey(500)
        cnt = cnt + 1
    cv.destroyWindow('undistorted images')

######################        张正友的标定法           #############################
def transformData(allImageCorners, objectPoints):
    features = []
    results = []
    for Points in allImageCorners:
        X = []
        Y = []
        for imgPoint, objPoint in zip(Points, objectPoints):
            X.append(np.array([-objPoint[0], -objPoint[1], -1.0, 0, 0, 0, imgPoint[
                     0] * objPoint[0], imgPoint[0] * objPoint[1]]).reshape((8, 1)))
            Y.append(-imgPoint[0])
            X.append(np.array([0, 0, 0, -objPoint[0], -objPoint[1], -1.0, imgPoint[
                1] * objPoint[0], imgPoint[1] * objPoint[1]]).reshape((8, 1)))
            Y.append(-imgPoint[1])
        X = np.array(X).reshape((len(X), 8)).T
        features.append(X)
        Y = np.array(Y).reshape((len(Y), 1)).T
        results.append(Y)
    return features, results

def calculateH(X, Y):
    H = Y @ X.T @ np.linalg.inv(X @ X.T)
    H = np.concatenate((H, [[1]]), axis = 1).reshape((3, 3))
    return H

def getAllH(allImageCorners, objectConers):
    features, results = transformData(allImageCorners, objectConers)
    H = []
    for X, Y in zip(features, results):
        H.append(calculateH(X, Y))
    return H

def calculateV(h, i, j):  # 这里的下表和论文中的是专置关系，因为h1 h2 h3都是3 x 1的矩阵
    return np.array([h[0][i] * h[0][j], h[0][i] * h[1][j] + h[1][i] * h[0][j],
                     h[1][i] * h[1][j], h[2][i] * h[0][j] + h[0][i] * h[2][j],
                     h[2][i] * h[1][j] + h[1][i] * h[2][j], h[2][i] * h[2][j]]).reshape((1, 6))

def getAllV(H):
    V = np.zeros((1, 6))
    for h in H:
        V = np.concatenate([V, calculateV(h, 0, 1)])
        V = np.concatenate([V, (calculateV(h, 0, 0) - calculateV(h, 1, 1))])
    V = np.delete(V, 0, 0)
    return V

def getK(H):
    V = getAllV(H)
    w, B = np.linalg.eig(V.T @ V)
    B = B[:, -1]

    # B[1] = 0 #设置gama = 0

    v = (B[1] * B[3] - B[0] * B[4]) / (B[0] * B[2] - B[1] * B[1])
    mLambda = B[5] - (B[3] * B[3] + v * (B[1] * B[3] - B[0] * B[4])) / B[0]
    alpha = math.sqrt(mLambda / B[0])
    beta = math.sqrt(mLambda * B[0] / (B[0] * B[2] - B[1] * B[1]))
    gama = -B[1] * alpha * alpha * beta / mLambda
    u = gama * v / alpha - B[3] * alpha * alpha / mLambda
    K = np.array([alpha, gama, u, 0, beta, v, 0, 0, 1]).reshape(3, 3)
    return mLambda, K

def getD(u0, v0, undisortImgCorners, objectCorners):
    D = []
    for imgCorners in undisortImgCorners:
        for imgPoint, objPoint in zip(imgCorners, objectCorners):
            D.append(( (imgPoint[0] - u0) * (objPoint[0] * objPoint[0] + objPoint[1] * objPoint[1]),
                       (imgPoint[0] - u0) * (objPoint[0] * objPoint[0] + objPoint[1] * objPoint[1]) *
                                            (objPoint[0] * objPoint[0] + objPoint[1] * objPoint[1]) ) )
            D.append(( (imgPoint[1] - v0) * (objPoint[0] * objPoint[0] + objPoint[1] * objPoint[1]),
                       (imgPoint[1] - v0) * (objPoint[0] * objPoint[0] + objPoint[1] * objPoint[1]) *
                                            (objPoint[0] * objPoint[0] + objPoint[1] * objPoint[1]) ) )
    D = np.array(D)
    return D

def  getUndistortImgCorners(objectCorners, H):
    undisortImgCorners = []
    for h in H:
        undisortImg = (h @ objectCorners.T).T
        undisortImg = undisortImg / undisortImg[:, 2:3]
        undisortImg = np.delete(undisortImg, 2, 1)
        undisortImgCorners.append(undisortImg)
    return undisortImgCorners

def getDisCoef(allImageCorners, objectCorners, H, K):
    undisortImgCorners = getUndistortImgCorners(objectCorners, H)
    d = list(map(lambda undist, dist: dist - undist, undisortImgCorners, allImageCorners))
    d = np.array(d).reshape(-1)
    D = getD(K[0, 2], K[1, 2], undisortImgCorners, objectCorners)
    k = np.linalg.inv(D.T @ D) @ D.T @ d
    dist = np.zeros((1, 5))
    dist[0, 0] = k[0]
    dist[0, 1] = k[1]
    return dist

def getRt(H, K, mLambda):
    Rt = []
    K_inv = np.linalg.inv(K)
    mLambda = (1 / np.power((K_inv @ K[:, 0] ), 2).sum() +   1 / np.power((K_inv @ K[:, 1] ), 2).sum()) / 2
    for h in H:
        Rt.append(mLambda * K_inv @ h)
    return Rt

def ZhangCalibrate(allImageCorners, patternSize):
    allImageCorners = list(map(lambda imageCorners: imageCorners.reshape((54, 2)), allImageCorners))
    objectCorners = getObjectCorner(patternSize)
    objectCorners[:,2] += 1
    H = getAllH(allImageCorners, objectCorners)
    mLambda, K = getK(H)
    Rt = getRt(H, K, mLambda)
    dist = getDisCoef(allImageCorners, objectCorners, H, K)

    return K, dist, Rt, mLambda

def loss(originCorners, undistortCorners, filename):
    f = open(filename, 'w')
    l = 0
    f.write("Every Image's loss:\n")
    for o, u in zip(originCorners, undistortCorners):
        tl = np.mean(np.power((o - u), 2))
        f.write(str(tl) + '\n')
        l += tl
    f.close()
    return l / 13

def myUndistort(tmp, k1, k2, u0, v0, obj):
    obj = np.delete(obj, 2, 1)
    g = np.sum(np.power(obj, 2), axis = 1).reshape((54,1))
    g = k1 * g + k2 * np.power(g, 2)
    tmp = tmp  + (tmp - np.array([u0, v0])) * g
    return tmp

def main():
    originImages = loadImage('../stereo/imageList.txt')
    imageSize = originImages[0].shape[:-1]
    patternSize = (6, 9)
    allImageCorners = getAllImageCorners(originImages, patternSize)

    #calibrate using opencv
    mtx1, dist1, rvecs1, tvecs1 = calibrate(allImageCorners, patternSize, imageSize)
    dist1[0, 2] = 0
    dist1[0, 3] = 0
    dist1[0, 4] = 0
    objCorners = getObjectCorner(patternSize)
    undistortCorners = []
    for rvecs, tvecs in zip(rvecs1, tvecs1):
        undistortCorners.append(cv.projectPoints(objCorners, rvecs, tvecs, mtx1, dist1)[0])
    l = loss(allImageCorners, undistortCorners, '../Result/CameraBasics/opencvResult/result.txt',)
    # write result
    with open('../Result/CameraBasics/opencvResult/result.txt', 'a') as f:
        f.write("opencv loss: " + str(l) + '\n')
        f.write('opencv intrinsics:\n' +  str(mtx1) + '\n')
        f.write('opencv distortion coeﬃcients:\n' + str(dist1) + '\n')
    undistortAll(originImages, mtx1, dist1,
                 '../Result/CameraBasics/opencvResult/')  # undistort images using parameter from Zhang's method

    #calibrate using Zhang's method implemented by me
    mtx2, dist2, Rt, mLambda = ZhangCalibrate(allImageCorners, patternSize)
    undistortCorners = []
    objCorners[:, -1] += 1

    for rt in Rt:
        tmpCoeners = (mtx2 @ rt @ objCorners.T).T
        tmpCoeners /= tmpCoeners[:,2:3]
        tmpCoeners = np.delete(tmpCoeners, 2, 1)
        tmpCoeners = myUndistort(tmpCoeners, dist2[0, 0], dist2[0, 1], mtx2[0, 2], mtx2[1, 2], objCorners)
        undistortCorners.append(tmpCoeners)
    allImageCorners = list(map(lambda imageCorners: imageCorners.reshape((54, 2)), allImageCorners))
    l = loss(allImageCorners, undistortCorners, '../Result/CameraBasics/myImplementResult/result.txt')

    with open('../Result/CameraBasics/myImplementResult/result.txt', 'a') as f:
        f.write("My Implement:\n" + str(l))
        f.write('my implement intrinsics:\n' +  str(mtx2) + '\n')
        f.write('my implement distortion coeﬃcients:\n' + str(dist2) + '\n')
    undistortAll(originImages, mtx2, dist1,
                 '../Result/CameraBasics/myImplementResult/')  # undistort images using parameter from Zhang's method

if __name__ == '__main__':
    main()

