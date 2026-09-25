import numpy as np
px = np.frombuffer(data[:2], dtype='<u2')[0]
r = ((px>>11)&0x1F); g=((px>>5)&0x3F); b=(px&0x1F)
r=(r<<3)|(r>>2); g=(g<<2)|(g>>4); b=(b<<3)|(b>>2)
print(f"px0 r={r} g={g} b={b}")
