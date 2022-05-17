import pybullet as p
import pybullet_data as pd
import math
import time
import numpy as np
# import pybullet_robots.panda.panda_sim_grasp as panda_sim
import panda_sim_grasp as panda_sim
from PIL import Image
import cv2

#video requires ffmpeg available in path
createVideo=False
fps=240.
timeStep = 1./fps

if createVideo:
	p.connect(p.GUI, options="--minGraphicsUpdateTimeMs=0 --mp4=\"pybullet_grasp.mp4\" --mp4fps="+str(fps) )
else:
	p.connect(p.GUI)

# p.configureDebugVisualizer(p.COV_ENABLE_Y_AXIS_UP,1)
p.configureDebugVisualizer(p.COV_ENABLE_GUI,1)
p.setPhysicsEngineParameter(maxNumCmdPer1ms=1000)
p.resetDebugVisualizerCamera(cameraDistance=2.3, cameraYaw=50, cameraPitch=-35, cameraTargetPosition=[0.0,0.0,0])
p.setAdditionalSearchPath(pd.getDataPath())

p.setTimeStep(timeStep)
p.setGravity(0,0,-10)

panda = panda_sim.PandaSimAuto(p,[0,0,0])
panda.control_dt = timeStep


# logId = panda.bullet_client.startStateLogging(panda.bullet_client.STATE_LOGGING_PROFILE_TIMINGS, "log.json")
panda.bullet_client.submitProfileTiming("start")
for i in range (20000):
	# panda.bullet_client.submitProfileTiming("full_step")
	panda.step()
	p.stepSimulation()
	# if createVideo:
	# 	p.configureDebugVisualizer(p.COV_ENABLE_SINGLE_STEP_RENDERING,1)
	# if not createVideo:
	# 	time.sleep(timeStep)
	# panda.bullet_client.submitProfileTiming()
	
	
	
	
	if i == 100:
		img_width, img_height, img_rgbPixels, img_depthPixels, img_segmentationMaskBuffer = p.getCameraImage(640,512)
panda.bullet_client.submitProfileTiming()




# panda.bullet_client.stopStateLogging(logId)
arr = img_rgbPixels[:,:,[2,1,0,3]]
# img = Image.fromarray(arr.astype(int), 'RGB')
cv2.imwrite('2.png',arr)
print(i)








#	state
#	0:	initial
#	1:	Ready pose, open grisp
#	2:	move to top of first plate
#	3:	move end-effector to lego
#	4:	close grips
#	5:	move to top of first plate
#	6:	move to top of second plate
#	7:	open grisp

#	to 1 or 2