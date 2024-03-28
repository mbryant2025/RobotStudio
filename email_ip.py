import smtplib, ssl
import socket


def get_local_ip():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.connect(('8.8.8.8', 80))
    r = s.getsockname()[0]
    s.close()
    return r


def get_password():
    with open('/home/robot/password.txt', 'r') as file:
        return file.read().strip()


port = 465
smtp_server = 'smtp.gmail.com'
sender_email = 'mbryant2025@gmail.com'
receiver_email = 'mbryant2025@gmail.com'
password = get_password()
message = f'''\
Subject: Robot IP Update

{get_local_ip()}'''

context = ssl.create_default_context()
with smtplib.SMTP_SSL(smtp_server, port, context=context) as server:
    server.login(sender_email, password)
    server.sendmail(sender_email, receiver_email, message)

