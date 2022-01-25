import cv2
import numpy as np
import open3d as o3d


# ---------- input parameters---------------------------------------------------
numDisparities=64 # Disparity range for stereo matching
blockSize=7 # Block size for stereo
scale=4 # Factor to shrink the image
maxDist=70 #  Maximal distance for point (on Z cordinates to remove outliers)
#----------Camera parameters---------------------------------------
Baseline=120
focal=1048.62
cx = 1081.38
cy = 607.91
#----------------Open video-----------------------------

vidcap = cv2.VideoCapture('stereo-video.mp4')

#---------Start reading frames-------------------------
while True:
  success,im = vidcap.read()
  if success==False:
      break
# Resize
  h,w,d = im.shape
  scale=4
  h =  int(h/scale)
  w = int(w/scale)
  im = cv2.resize(im,(w,h)).mean(2).astype(np.uint8)


  imOrig= im[:,:int(w / 2)].copy() # save original image


# filter irrelavant regions
  im[int(h*0.8):,]=255 # filter floor
  im[im>im.mean()/3]=255 # filter bright background region


# split image to left and right
  im1 = im[:,:int(w / 2)]
  im2 = im[:, int(w / 2) : w]#.mean(2).astype(np.uint8)

# Filter corner
  im1[int(h*0.7):, :int(w / 6)]=255
  im2[int(h*0.7):, :int(w / 6)]=255

# apply streo and find disparsity
  stereo = cv2.StereoBM_create(numDisparities=numDisparities, blockSize=blockSize)#cv2.StereoSGBM_create(numDisparities=32, blockSize=31)
  disp = stereo.compute(im1, im2)
  disp *= scale
  disp[im1==255]=0 # ignore background
  h,w=disp.shape

#----------Find XYZ map and Depth map-------------------------------------------------------------------------------------
# Find Z map
  zmap= Baseline*focal/(disp+0.000000001)

  zmap[disp<=0] = 0 # filter irrelevant regions
  zmap[zmap>maxDist] = 0
# Find Y map
  GridY = (np.array(range(h))*scale- cy)
  GridY = np.transpose(np.tile(GridY, (w, 1)))
  ymap = GridY * zmap / focal

# Find X map
  GridX = (np.array(range(w))*scale-cx)
  GridX = np.tile(GridX, (h, 1))
  xmap=GridX *zmap/focal

# Find depth map
  depthMap = (xmap ** 2 + ymap ** 2 + zmap ** 2 )**0.5
# -----------------Display depth map-------------------------------------------------------------------

  cv2.imshow("press SPACE to display point cloud",np.hstack([imOrig,((depthMap-depthMap.min())/(depthMap.max()-depthMap.min())*255).astype(np.uint8)]))
  ch=cv2.waitKey(100)

 #------Display point cloud------------------------------------------------------------------------
  if ch==32:
     points=np.zeros([0,3])
     colors = np.zeros([0, 3],np.float32)
     i=0
     for x in range(w):
       for y in range(h):
         if zmap[y,x]>0:
              points=np.concatenate([points,[[xmap[y,x],ymap[y,x],zmap[y,x]]]],0) # zmap[y,x]
              colors = np.concatenate([colors, [[imOrig[y,x], imOrig[y,x],imOrig[y,x]]]], 0)  # zmap[y,x]]
     points = np.concatenate([points, [[0, 0, 0]]], 0)  # zmap[y,x]
     colors = np.concatenate([colors, [[255, 0, 0]]], 0)  # zmap[y,x]]
     pcd = o3d.geometry.PointCloud()
     pcd.points = o3d.utility.Vector3dVector(points)
     pcd.colors = o3d.utility.Vector3dVector(colors/100)
     o3d.visualization.draw_geometries([pcd],"Red point is camera position")