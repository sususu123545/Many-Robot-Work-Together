"""Circle geometry with red-interior and edge support; pure perception only."""
import cv2
import numpy as np

def refine_red_boundary(red,cx,cy,radius):
    # The lower arc can merge with the floor reflection. Fit the other 270
    # degrees using observed red-to-background transitions, then reject outliers.
    points=[];height,width=red.shape
    for angle in np.deg2rad(np.linspace(135,405,91)):
        cosine,sine=np.cos(angle),np.sin(angle);distances=np.arange(radius*.60,radius*1.45,.5)
        xs=np.rint(cx+distances*cosine).astype(int);ys=np.rint(cy+distances*sine).astype(int)
        valid=(xs>=0)&(xs<width)&(ys>=0)&(ys<height)
        if not np.all(valid):
            first=int(np.flatnonzero(~valid)[0]);xs=xs[:first];ys=ys[:first];distances=distances[:first]
        outside=np.flatnonzero(red[ys,xs]==0)
        if len(outside):
            distance=distances[int(outside[0])]-.25;points.append([cx+distance*cosine,cy+distance*sine])
    if len(points)<50:return None
    points=np.asarray(points)
    for _ in range(4):
        a=np.column_stack((2*points[:,0],2*points[:,1],np.ones(len(points))))
        x,y,c=np.linalg.lstsq(a,np.sum(points**2,axis=1),rcond=None)[0]
        rr=np.sqrt(max(0,c+x*x+y*y));residual=np.abs(np.linalg.norm(points-[x,y],axis=1)-rr)
        keep=residual<=1.5
        if np.sum(keep)<45:return None
        points=points[keep]
    if not .7*radius<=rr<=1.3*radius or np.hypot(x-cx,y-cy)>radius*.25:return None
    return float(x),float(y),float(rr)

def find_hough_ball(image_bgr):
    height,width=image_bgr.shape[:2]
    gray=cv2.GaussianBlur(cv2.cvtColor(image_bgr,cv2.COLOR_BGR2GRAY),(5,5),0)
    hsv=cv2.cvtColor(cv2.GaussianBlur(image_bgr,(3,3),0),cv2.COLOR_BGR2HSV)
    red=cv2.inRange(hsv,np.array([0,75,45],np.uint8),np.array([12,255,255],np.uint8)) | cv2.inRange(hsv,np.array([168,75,45],np.uint8),np.array([180,255,255],np.uint8))
    count,labels,stats,_=cv2.connectedComponentsWithStats(red)
    boxes=[s for s in stats[1:] if s[cv2.CC_STAT_AREA]>=80]
    if not boxes:return None,0
    x0=max(0,min(s[0] for s in boxes)-20);y0=max(0,min(s[1] for s in boxes)-20)
    x1=min(width,max(s[0]+s[2] for s in boxes)+20);y1=min(height,max(s[1]+s[3] for s in boxes)+20)
    circles=cv2.HoughCircles(gray[y0:y1,x0:x1],cv2.HOUGH_GRADIENT,dp=1,minDist=35,param1=100,param2=28,minRadius=12,maxRadius=180)
    if circles is None:return None,0
    circles[0,:,0]+=x0;circles[0,:,1]+=y0
    edges=cv2.Canny(gray,45,100);candidates=[]
    yy,xx=np.ogrid[:height,:width]
    for cx,cy,radius in circles[0]:
        cx,cy,radius=map(float,(cx,cy,radius))
        initial_core=(xx-cx)**2+(yy-cy)**2<=(radius*.60)**2
        if not np.any(initial_core) or float(np.mean(red[initial_core]>0))<.70:continue
        refined=refine_red_boundary(red,cx,cy,radius)
        if refined is None:continue
        cx,cy,radius=refined
        if cx-radius<4 or cy-radius<4 or cx+radius>width-4 or cy+radius>height-4:continue
        disk=(xx-cx)**2+(yy-cy)**2<=(radius*.72)**2
        fraction=float(np.mean(red[disk]>0))
        if fraction<.70:continue
        supported=0
        for angle in np.linspace(0,2*np.pi,72,endpoint=False):
            x=round(cx+radius*np.cos(angle));y=round(cy+radius*np.sin(angle))
            supported+=bool(np.any(edges[max(0,y-3):min(height,y+4),max(0,x-3):min(width,x+4)]))
        edge=supported/72
        if edge<.55:continue
        candidates.append({'center_x':cx,'center_y':cy,'radius_px':radius,'bbox':[round(cx-radius),round(cy-radius),round(2*radius),round(2*radius)],'red_fraction':fraction,'edge_support':edge,'confidence':fraction*edge,'method':'hough_gradient_red_core_edge_support'})
    candidates.sort(key=lambda c:c['confidence'],reverse=True)
    if not candidates:return None,0
    if len(candidates)>1 and candidates[1]['confidence']>.85*candidates[0]['confidence']:return None,len(candidates)
    return candidates[0],len(candidates)
