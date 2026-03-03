from flask import Flask, Response
import cv2

app = Flask(__name__)
cam = cv2.VideoCapture(0)

def gen():
    while True:
        ret, frame = cam.read()
        _, jpg = cv2.imencode('.jpg', frame)
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' +
               jpg.tobytes() + b'\r\n')

@app.route('/mjpeg')
def mjpeg():
    return Response(gen(),
        mimetype='multipart/x-mixed-replace; boundary=frame')

app.run(host='0.0.0.0', port=8080)
