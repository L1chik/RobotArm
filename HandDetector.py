import cv2
import mediapipe as mp
import pylab as p
import serial
import time
import logging
import serial.tools.list_ports


class HandDetector:
    def __init__(self, mode=False, maxHands=2, detectionCon=0.5, minTrackCon=0.5):
        self.mode = mode
        self.maxHands = maxHands
        self.detectionCon = detectionCon
        self.minTrackCon = minTrackCon

        self.mpHands = mp.solutions.hands
        self.hands = self.mpHands.Hands(self.mode, self.maxHands,
                                        self.detectionCon, self.minTrackCon)

        self.mpDraw = mp.solutions.drawing_utils
        self.tipIds = [4, 8, 12, 16, 20]
        self.fingers = []
        self.lmList = []


    def findHands(self, img, draw=True):
        imgRGB = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        self.results = self.hands.process(imgRGB)

        if self.results.multi_hand_landmarks:
            for handLms in self.results.multi_hand_landmarks:
                if draw:
                    self.mpDraw.draw_landmarks(img, handLms, self.mpHands.HAND_CONNECTIONS)

        return img


    def findPosition(self, img, handNo=0, draw=True):
        xList = []
        yList = []
        bbox = []
        bboxInfo = []
        self.lmList = []

        if self.results.multi_hand_landmarks:
            myHand = self.results.multi_hand_landmarks[handNo]

            for id, lm in enumerate(myHand.landmark):
                h, w, c = img.shape
                px, py = int(lm.x * w), int(lm.y * h)
                xList.append(px)
                yList.append(py)
                self.lmList.append([px, py])

                if draw:
                    cv2.circle(img, (px, py), 5, (255, 0, 255), cv2.FILLED)

            xmin, xmax = min(xList), max(xList)
            ymin, ymax = min(yList), max(yList)
            boxW, boxH = xmax - xmin, ymax - ymin
            bbox = xmin, ymin, boxW, boxH
            cx, cy = bbox[0] + (bbox[2] // 2), \
                     bbox[1] + (bbox[3] // 2)
            bboxInfo = {"id": id, "bbox": bbox, "center": (cx, cy)}

            if draw:
                cv2.rectangle(img, (bbox[0] - 20, bbox[1] - 20),
                              (bbox[0] + bbox[2] + 20, bbox[1] + bbox[3] + 20), (0, 255, 0), 2)

        return self.lmList, bboxInfo


    def fingersUp(self):
        if self.results.multi_hand_landmarks:
            myHandType = self.handType()
            fingers = []

            if myHandType == "Right":
                if self.lmList[self.tipIds[0]][0] > self.lmList[self.tipIds[0] - 1][0]:
                    fingers.append(1)
                else:
                    fingers.append(0)

            else:
                if self.lmList[self.tipIds[0]][0] < self.lmList[self.tipIds[0] - 1][0]:
                    fingers.append(1)
                else:
                    fingers.append(0)

            for id in range(1, 5):
                if self.lmList[self.tipIds[id]][1] < self.lmList[self.tipIds[id] - 2][1]:
                    fingers.append(1)
                else:
                    fingers.append(0)

        return fingers


    def handType(self):
        if self.results.multi_hand_landmarks:
            if self.lmList[17][0] < self.lmList[5][0]:
                return "Right"
            else:
                return "Left"


class SerialObject:

    def __init__(self, portNo=None, baudRate=9600, digits=1):
        self.portNo = portNo
        self.baudRate = baudRate
        self.digits = digits
        connected = False
        if self.portNo is None:
            ports = list(serial.tools.list_ports.comports())

            for p in ports:
                if "Arduino" in p.description:
                    print(f'{p.description} Connected')
                    self.ser = serial.Serial(p.device)
                    self.ser.baudrate = baudRate
                    connected = True

            if not connected:
                logging.warning("Arduino Not Found. Please enter COM Port Number instead.")

        else:
            try:
                self.ser = serial.Serial(self.portNo, self.baudRate)
                print("Serial Device Connected")
            except:
                logging.warning("Serial Device Not Connected")


    def sendData(self, data):
        myString = "$"
        for d in data:
            myString += str(int(d)).zfill(self.digits)
        try:
            self.ser.write(myString.encode())
            return True
        except:
            return False

def main():
    cap = cv2.VideoCapture(0)
    detector = HandDetector(maxHands=1, detectionCon=0.7)
    mySerial = SerialObject("COM4", 9600, 1)

    while True:
        success, img = cap.read()
        img = cv2.flip(img, 1)
        img = cv2.resize(img, (1280, 720), interpolation=cv2.INTER_LINEAR)
        img = detector.findHands(img)
        lmList, bboxInfo = detector.findPosition(img)
        if lmList:
            fingers = detector.fingersUp()
            print(fingers)
            if (fingers == [0, 0, 1, 0, 0]) or (fingers == [1, 0, 1, 0, 0]):
                fingers = [0, 1, 1, 0, 0]
            mySerial.sendData(fingers)
        cv2.imshow("Image", img)
        cv2.waitKey(1)

if __name__ == "__main__":
    main()