import qrcode
from PIL import Image
import subprocess
import sys

url = sys.argv[1]

qr = qrcode.QRCode(
    version=1,
    error_correction=qrcode.constants.ERROR_CORRECT_H,
    box_size=10,
    border=4,
)
qr.add_data(url)
qr.make(fit=True)

img = qr.make_image(fill="black", back_color="white")

img.show()

# Add these lines
print("Running checking_order.py...")
subprocess.run(["python3", "checking_order.py"])