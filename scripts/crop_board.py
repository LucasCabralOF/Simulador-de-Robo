from PIL import Image

im = Image.open('C:/Users/lucas/.gemini/antigravity-ide/brain/3bfc7159-0155-46ef-98b3-da3e28aac7dc/.user_uploaded/media_1789688931511.jpg')
w, h = im.size
print("Size:", w, h)
# Crop top part (elbow, wrist, claw)
im.crop((int(w * 0.4), int(h * 0.0), int(w * 0.9), int(h * 0.55))).save('artifacts/crop_blackboard_top.png')
# Crop middle part (shoulder, prismatic)
im.crop((int(w * 0.2), int(h * 0.25), int(w * 0.7), int(h * 0.75))).save('artifacts/crop_blackboard_mid.png')
print("Cropped successfully")
