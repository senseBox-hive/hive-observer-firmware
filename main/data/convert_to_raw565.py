from PIL import Image
import numpy as np

img = Image.open("example_frame.png").convert("RGB")  # QVGA
a = np.array(img, dtype=np.uint8)
r = (a[...,0] >> 3).astype(np.uint16)
g = (a[...,1] >> 2).astype(np.uint16)
b = (a[...,2] >> 3).astype(np.uint16)
rgb565 = (r << 11) | (g << 5) | b                      # packed 16-bit

##a = np.array(Image.open("example_frame.png").convert("RGB").resize((32,32)))
##row, col = 15, 15   # or wherever a strongly-colored pixel is
##print("PNG:", a[col, row])   # [R, G, B] ground truth

# CRITICAL: write in the SAME byte order your device unpacks.
# Your code does px = (second<<8)|first, i.e. buf[0]=low, buf[1]=high  → little-endian.
rgb565.astype('<u2').tofile("example_frame.rgb565")          # '<u2' = little-endian uint16
