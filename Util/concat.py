import shutil
import glob

with open('Tides.lua','wb') as wfd:
    for folder in ['Low', 'Medium', 'High']:
        for f in glob.glob(folder + '/*.lua'):
            with open(f, 'rb') as fd:
                shutil.copyfileobj(fd, wfd)
                wfd.write('\n'.encode())