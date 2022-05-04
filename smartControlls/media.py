import cv2
import mediapipe as mp 
import time

cap =cv2.VideoCapture(0)

mpHands=mp.solutions.mediapipe.python.solutions.hands
hands =mpHands.Hands()

while True:
    success , img = cap.read()
    imgGray=cv2.cvtColor(img,cv2.COLOR_BAYER_BG2RGB)
    results = hands.process(imgGray)
    cv2.imshow("Image", img)
    cv2.waitkey(1)