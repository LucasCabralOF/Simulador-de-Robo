from PIL import Image

im = Image.open('artifacts/lecture-dh-reference.png')
im.crop((960, 370, 1440, 740)).save('artifacts/crop_dh_p6.png')
im.crop((480, 740, 960, 1110)).save('artifacts/crop_dh_p8.png')
im.crop((0, 1110, 480, 1480)).save('artifacts/crop_dh_p10.png')

im_axes = Image.open('artifacts/lecture-axes-reference.png')
im_axes.crop((0, 0, 480, 370)).save('artifacts/crop_axes_p1.png')
im_axes.crop((0, 740, 480, 1110)).save('artifacts/crop_axes_p7.png')
print('Cropped successfully')
