#遍历访问图片每个像素点，并修改相应的RGB
import cv2 as cv

def access_pixels(image):
    print(image.shape)
    height = image.shape[0]
    width = image.shape[1]
    channels = image.shape[2]
    print("width: %s  height: %s  channels: %s"%(width, height, channels))

    img_data = image.reshape((height*width*3))     
    cv.imshow("ROI",img_data)
    cv.waitKey(0)
    cv.destroyAllWindows()

    for row in range(height):
        for col in range(width):
            for c in range(channels):
                pv = image[row , col, c]        #获取每个像素点的每个通道的数值
                image[row, col, c]=255 - pv     #灰度值是0-255   这里是修改每个像素点每个通道灰度值
    cv.imshow("second_image",image)

src=cv.imread('/home/young/下载/sjtu_logo.png')   #blue, green, red
cv.namedWindow('first_image', cv.WINDOW_AUTOSIZE)
cv.imshow('first_image', src)

access_pixels(src)

cv.waitKey(0)
cv.destroyAllWindows()
