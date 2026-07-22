#!/usr/bin/env python3
"""Framework overview — JointKD UAV KD.  Style matches 总览图.jpeg."""
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch

W, H = 28, 19
fig = plt.figure(figsize=(W, H), dpi=150)
ax = fig.add_axes([0, 0, 1, 1])
ax.set_xlim(0, W); ax.set_ylim(0, H); ax.axis('off')
fig.patch.set_facecolor('#FFFFFF')

# ── Colors ─────────────────────────────────────────────────────────────────
BD='#154360'; BM='#2E86C1'; BL='#D6EAF8'; BT='#EAF4FB'
OD='#784212'; OM='#CA6F1E'; OL='#FDEBD0'; OT='#FEF5E7'
PD='#4A235A'; PM='#7D3C98'; PL='#E8DAEF'
ND='#1B2631'; NL='#D5D8DC'; NT='#EAECEE'
GD='#145A32'; GM='#1E8449'; GL='#D5F5E3'
RD='#7B241C'; RM='#C0392B'; RL='#FADBD8'
TD='#0E6655'; TM='#148F77'; TL='#D0ECE7'
GRD='#2C3E50'; GRL='#ECF0F1'
WHT='#FFFFFF'; BLK='#17202A'

# ── Helpers ─────────────────────────────────────────────────────────────────
def rp(x,y,w,h,fc,ec,lw=1.8,r=0.06,z=1,a=1.0):
    ax.add_patch(FancyBboxPatch((x,y),w,h,
        boxstyle=f'round,pad={r}',facecolor=fc,edgecolor=ec,
        linewidth=lw,zorder=z,alpha=a))

def tb(x,y,w,h,c,text,fs=10,fc=WHT,z=3):
    rp(x,y,w,h,c,c,lw=0,r=0.03,z=z)
    ax.text(x+w/2,y+h/2,text,ha='center',va='center',
            fontsize=fs,fontweight='bold',color=fc,zorder=z+1)

def bx(x,y,w,h,fc,ec,text='',fs=8.5,tc=BLK,bold=False,lw=1.5,ha='center',z=3,r=0.05):
    rp(x,y,w,h,fc,ec,lw=lw,r=r,z=z)
    if text:
        ax.text(x+w/2 if ha=='center' else x+0.1,y+h/2,text,
                ha=ha,va='center',fontsize=fs,color=tc,
                fontweight='bold' if bold else 'normal',
                zorder=z+1,multialignment='center',linespacing=1.3)

def tx(x,y,s,fs=9,c=BLK,bold=False,ha='center',va='center',z=5,it=False):
    ax.text(x,y,s,ha=ha,va=va,fontsize=fs,color=c,
            fontweight='bold' if bold else 'normal',
            fontstyle='italic' if it else 'normal',zorder=z)

def ar(x1,y1,x2,y2,c=BLK,lw=1.5,ls='-',hw=0.10,hl=0.12,z=4):
    ax.annotate('',xy=(x2,y2),xytext=(x1,y1),
        arrowprops=dict(arrowstyle=f'->,head_width={hw},head_length={hl}',
                       color=c,lw=lw,linestyle=ls),zorder=z)

def da(x1,y1,x2,y2,c=BM,lw=1.2,z=4):
    ax.annotate('',xy=(x2,y2),xytext=(x1,y1),
        arrowprops=dict(arrowstyle='->,head_width=0.09,head_length=0.11',
                       color=c,lw=lw,linestyle='--'),zorder=z)

def cnn(x,y,n=4,bw=0.26,bh=1.0,gap=0.07,fc=OM,ec=OD,z=4):
    for i in range(n):
        ax.add_patch(FancyBboxPatch((x+i*gap,y+i*gap),bw,bh,
            boxstyle='square,pad=0.0',facecolor=fc,edgecolor=ec,
            linewidth=1.0,zorder=z+i*0.1,alpha=0.85))

# ════════════════════════════════════════════════════════════════════════════
# LAYOUT CONSTANTS
# ════════════════════════════════════════════════════════════════════════════
# Row 1 (top): Teacher | JointKD
R1Y=9.0; R1H=9.0; R1T=R1Y+R1H   # bottom=9.0, height=9.0, top=18.0
TBAR_H=0.6                        # panel title bar height
MAX_OFS=R1H-TBAR_H-0.05          # max safe y-offset inside any R1 panel = 8.35

# Row 2 (bottom)
R2Y=0.2; R2H=8.5; R2T=R2Y+R2H

# Teacher panel
TX=0.15; TW=9.0

# JointKD panel
FX=TX+TW+0.2; FW=W-FX-0.15  # FX=9.35, FW=18.5

# ════════════════════════════════════════════════════════════════════════════
# MAIN TITLE
# ════════════════════════════════════════════════════════════════════════════
rp(0.15,18.1,W-0.3,0.72,ND,ND,lw=0,r=0.06,z=1)
tx(W/2,18.46,
   'Overall Framework: Knowledge Distillation from a Classical Autonomous Flight System'
   '  →  Lightweight Vision-Based UAV Navigation',
   fs=12,c=WHT,bold=True)

# ════════════════════════════════════════════════════════════════════════════
# 1. TEACHER SYSTEM (blue)
# ════════════════════════════════════════════════════════════════════════════
rp(TX,R1Y,TW,R1H,BT,BD,lw=2.5,r=0.1,z=1)
tb(TX,R1Y+MAX_OFS,TW,TBAR_H,BD,
   '1. Teacher System: CERLAB Autonomous Flight  [Existing Work]',fs=10)

# Sim environment (top)
bx(TX+0.15,R1Y+6.15,TW-0.3,2.1,BL,BM,
   'Simulation Environment\nGazebo 11  ·  corridor_dynamic_9\n\n'
   'Teacher Sensors:\n'
   '• LiDAR / Point Cloud     • RGB-D Camera\n'
   '• IMU / Odometry            • Global Map',
   fs=8,tc=BD,z=2)

ar(TX+TW/2,R1Y+6.15,TX+TW/2,R1Y+5.82,c=BD,hw=0.08,hl=0.10)

# Pipeline
PY=R1Y+4.05
rp(TX+0.15,PY,TW-0.3,1.65,WHT,BD,lw=2,r=0.05,z=2)
tx(TX+TW/2,PY+1.48,'CERLAB Autonomous Flight Pipeline',fs=8.5,c=BD,bold=True)
pw=(TW-0.8)/3
for j,(nm,sub) in enumerate([('Perception','Mapping +\nObstacle\ntracking'),
                               ('B-spline\nPlanner','Trajectory\noptimization'),
                               ('Tracking\nController','Velocity\ncontrol')]):
    bx(TX+0.25+j*(pw+0.05),PY+0.1,pw,1.05,BM,BD,f'{nm}\n({sub})',
       fs=7.5,tc=WHT,bold=True,z=3)
    if j<2:
        ar(TX+0.25+(j+1)*(pw+0.05)-0.05,PY+0.62,
           TX+0.25+(j+1)*(pw+0.05),PY+0.62,c=WHT,lw=1.0,hw=0.07,hl=0.09)

ar(TX+TW/2,PY,TX+TW/2,R1Y+3.8,c=BD,hw=0.08,hl=0.10)

# Outputs row
OY=R1Y+0.3
bx(TX+0.15,OY,4.0,3.3,WHT,BM,z=2)
tx(TX+2.15,OY+3.05,'Trajectory Output  (Planner)',fs=8.5,c=BD,bold=True)
ax.text(TX+0.3,OY+2.7,r'$\tau_t = \{p_{t+1},\ldots,p_{t+K}\}$',
        fontsize=9,color=BD,fontweight='bold',zorder=5)
tx(TX+2.15,OY+2.35,'K=10 body-frame waypoints',fs=7.5,c=GRD)
tx(TX+2.15,OY+2.0,'(B-spline trajectory optimizer)',fs=7.5,c=GRD)
tx(TX+2.15,OY+1.55,'Planning-level knowledge:',fs=7.5,c=BD,bold=True)
tx(TX+2.15,OY+1.2,'where to go, path geometry,',fs=7.5,c=GRD,it=True)
tx(TX+2.15,OY+0.85,'obstacle avoidance structure',fs=7.5,c=GRD,it=True)

bx(TX+4.3,OY,TW-4.45,3.3,WHT,BM,z=2)
tx(TX+6.7,OY+3.05,'Control Output  (Controller)',fs=8.5,c=BD,bold=True)
ax.text(TX+4.45,OY+2.7,r'$U_t=[u_t,u_{t+1},\ldots,u_{t+H}]$',
        fontsize=9,color=BD,fontweight='bold',zorder=5)
tx(TX+6.7,OY+2.35,'H-step ctrl seq (vx,vy,vz,yaw_rate)',fs=7.5,c=GRD)
tx(TX+6.7,OY+2.0,'(CERLAB tracking controller)',fs=7.5,c=GRD)
tx(TX+6.7,OY+1.55,'Execution-level knowledge:',fs=7.5,c=BD,bold=True)
tx(TX+6.7,OY+1.2,'how to move, temporal smoothing,',fs=7.5,c=GRD,it=True)
tx(TX+6.7,OY+0.85,'dynamic constraints + feedback',fs=7.5,c=GRD,it=True)

# ════════════════════════════════════════════════════════════════════════════
# 2. JointKD FRAMEWORK (orange)
# ════════════════════════════════════════════════════════════════════════════
rp(FX,R1Y,FW,R1H,OT,OD,lw=2.5,r=0.1,z=1)
tb(FX,R1Y+MAX_OFS,FW,TBAR_H,OD,
   '2. JointKD — Proposed Method  [Core Contribution]',fs=10.5)

# ── Column widths (absolute x) ─────────────────────────────────────────────
# Input labels: FX+0.15 to FX+2.25
IC=FX+0.15; ICW=2.1
# Backbone: FX+2.45 to FX+5.55
BBX=FX+2.45; BBW=3.1
# Heads+Loss: FX+5.75 to FX+9.85
HX=FX+5.75; HW=4.1
# Deployment: FX+10.1 to FX+FW-0.15
DX=FX+10.1; DW=FW-10.25

# ── y-offsets (absolute) for each row ──────────────────────────────────────
# Traj row: offset 5.8 to 8.33  (abs: 14.8 to 17.33)
TRY=R1Y+5.8; TRH=2.53
# Ctrl row: offset 3.3 to 5.6   (abs: 12.3 to 14.6)
CRY=R1Y+3.3; CRH=2.3
# Loss row: offset 0.3 to 3.1   (abs: 9.3 to 12.1)
LRY=R1Y+0.3; LRH=2.8

# Input column
bx(IC,TRY,ICW,TRH,BL,BM,'Image  $I_t$\n\nFront-camera\nRGB\n480 × 640 × 3',
   fs=8,tc=BD,z=2)
bx(IC,CRY,ICW,CRH,OL,OM,'Trajectory\nLabel  $\\tau_t$\n\n'
   'K=10 body-frame\nwaypoints\n(from planner)',fs=7.8,tc=OD,z=2)
bx(IC,LRY,ICW,LRH,OL,OM,'Ctrl Seq\nLabel  $U_t$\n\n'
   '$[u_t,\\ldots,u_{t+H}]$\n\nH-step ctrl seq\n(from controller)',
   fs=7.8,tc=OD,z=2)

# Dashed arrows: teacher traj/ctrl outputs → JointKD training labels
# OY+LRH/2 centres on the traj/ctrl output boxes in the teacher panel
da(TX+4.0,OY+LRH/2+0.55,IC,CRY+CRH/2,c=BM)   # traj out → traj label
da(TX+4.3,OY+LRH/2,    IC,LRY+LRH/2,c=BM)    # ctrl out → ctrl label

# Arrow: image → backbone
ar(IC+ICW,TRY+TRH/2,BBX,TRY+TRH/2,c=OD)

# Backbone
BB_TOP=TRY+TRH-0.15   # abs top of backbone panel (inside JointKD content area)
BB_BOT=CRY-0.15       # abs bottom of backbone panel
rp(BBX,BB_BOT,BBW,BB_TOP-BB_BOT,OL,OD,lw=2.2,r=0.06,z=2)
# CNN blocks: safe height = top_limit - gap_stack - start_y
CNN_START=BB_BOT+0.55; CNN_TOP=BB_TOP-0.12
CNN_BH=min(CNN_TOP-CNN_START-4*0.08, 4.2)   # cap to avoid overflow
cnn(BBX+0.12,CNN_START,n=5,bw=0.3,bh=CNN_BH,gap=0.08,fc=OM,ec=OD,z=3)
tx(BBX+BBW/2,BB_BOT+0.28,'MobileNetV2 Backbone',fs=8.5,c=OD,bold=True)
tx(BBX+BBW/2,BB_BOT+0.06,'(1280-d shared features)',fs=8,c=OD)

# Arrows backbone → heads
ar(BBX+BBW,TRY+TRH/2,HX,TRY+TRH/2,c=OD)    # → traj head
ar(BBX+BBW,CRY+CRH/2,HX,CRY+CRH/2,c=OD)    # → ctrl head

# ── TRAJECTORY HEAD (red, train-only) ──────────────────────────────────────
rp(HX,TRY,HW,TRH,RL,RD,lw=2.0,r=0.05,z=2)
tb(HX,TRY+TRH-0.35,HW,0.35,RD,'⚠  TRAIN ONLY  —  Auxiliary Supervision',fs=8,z=4)
tx(HX+HW/2,TRY+TRH-0.62,'Trajectory Head',fs=9.5,c=RD,bold=True)
ax.text(HX+0.12,TRY+TRH-0.97,
        r'$\hat{\tau}_{student}$  (K × 3 body-frame)',fontsize=8.5,color=RD,zorder=5)
ax.text(HX+0.12,TRY+TRH-1.32,
        r'$\mathcal{L}_{traj}=\mathrm{MSE}(\hat{\tau}_{student},\tau_{teacher})$',
        fontsize=8.5,color=RD,zorder=5)
tx(HX+HW/2,TRY+0.72,'Forces backbone to encode planning intent',fs=8,c=GRD)
rp(HX+0.4,TRY+0.12,HW-0.8,0.38,RD,RD,lw=0,r=0.03,z=5)
tx(HX+HW/2,TRY+0.31,'Discarded at deployment',fs=8.5,c=WHT,bold=True,z=6)

# Dashed arrow: traj label → traj head
da(IC+ICW,CRY+CRH/2,HX,TRY+0.8,c=OM)

# ── CTRL SEQUENCE HEAD (teal, deployed) ────────────────────────────────────
rp(HX,CRY,HW,CRH,TL,TD,lw=2.0,r=0.05,z=2)
tb(HX,CRY+CRH-0.33,HW,0.33,TD,'✓  TRAIN + DEPLOY  (Main Output)',fs=8,z=4)
tx(HX+HW/2,CRY+CRH-0.58,'Ctrl Sequence Head',fs=9.5,c=TD,bold=True)
ax.text(HX+0.12,CRY+CRH-0.93,
        r'$\hat{U}_{student}=[u_t,u_{t+1},\ldots,u_{t+H}]$',
        fontsize=8.5,color=TD,zorder=5)
tx(HX+HW/2,CRY+1.25,'H = 4 steps  ·  (vx, vy, vz, yaw_rate)',fs=8,c=TD)
ax.text(HX+0.12,CRY+0.92,
        r'$\mathcal{L}_{ctrl}=\sum_k w_k\cdot\mathrm{MSE}(u^k_{pred},u^k_{teacher})$',
        fontsize=8,color=TD,zorder=5)
tx(HX+HW/2,CRY+0.55,'Receding horizon execution',fs=8,c=GRD)
tx(HX+HW/2,CRY+0.25,'(execute $u_t$, re-predict at $t+1$)',fs=8,c=GRD,it=True)

# Dashed arrow: ctrl label → ctrl head
da(IC+ICW,LRY+LRH/2,HX,CRY+CRH/2,c=OM)

# ── JOINT LOSS (orange) ─────────────────────────────────────────────────────
rp(HX,LRY,HW,LRH,OL,OD,lw=2.0,r=0.05,z=2)
tx(HX+HW/2,LRY+LRH-0.28,'Joint Loss  (training only)',fs=9,c=OD,bold=True)
ax.text(HX+0.15,LRY+LRH-0.62,
        r'$\mathcal{L}_{traj}=\mathrm{MSE}(\hat{\tau}_{student},\tau_{teacher})$',
        fontsize=8.5,color=RD,zorder=5)
ax.text(HX+0.15,LRY+LRH-0.97,
        r'$\mathcal{L}_{ctrl}=\sum_k w_k\cdot\mathrm{MSE}(u^k_{pred},u^k_{teacher})$',
        fontsize=8.5,color=TD,zorder=5)
ax.text(HX+0.15,LRY+LRH-1.52,
        r'$\mathcal{L}=\lambda_{traj}\cdot\mathcal{L}_{traj}+\lambda_{ctrl}\cdot\mathcal{L}_{ctrl}$',
        fontsize=11,color=OD,fontweight='bold',zorder=5)
tx(HX+HW/2,LRY+0.82,
   r'($\lambda_{traj}=\lambda_{ctrl}=1.0$  initial)',fs=8,c=GRD)
tx(HX+HW/2,LRY+0.5,
   r'ablation: $\lambda\in\{0.1,\,1.0,\,5.0\}$',fs=8,c=GRD)

# Arrows: heads → loss (dashed)
da(HX+HW/2,TRY,HX+HW/2,LRY+LRH,c=RD)
da(HX+HW/2,CRY,HX+HW/2,LRY+LRH,c=TD)

# ── DEPLOYMENT (dark panel) ─────────────────────────────────────────────────
rp(DX,R1Y,DW,R1H,'#1C2833','#17202A',lw=2.0,r=0.08,z=1)
tb(DX,R1Y+MAX_OFS,DW,TBAR_H,'#17202A','Deployment\n(No Teacher)',fs=9,z=3)

dcy=R1Y+7.2
tx(DX+DW/2,dcy,'Front-camera  $I_t$',fs=9,c='#85C1E9',bold=True)
bx(DX+0.25,dcy-0.65,DW-0.5,0.55,'#1A3A4A','#5DADE2',
   'front-camera image only',fs=8,tc='#85C1E9',z=3)
ar(DX+DW/2,dcy-0.65,DX+DW/2,dcy-1.15,c='#5DADE2',hw=0.08)
bx(DX+0.25,dcy-2.05,DW-0.5,0.82,'#1B4332','#52BE80',
   'MobileNetV2\nBackbone',fs=8.5,tc='#A9DFBF',bold=True,z=3)
ar(DX+DW/2,dcy-2.05,DX+DW/2,dcy-2.52,c='#52BE80',hw=0.08)
bx(DX+0.25,dcy-3.35,DW-0.5,0.75,'#1B4332','#52BE80',
   'Ctrl Sequence Head\n(traj head stripped)',fs=8.5,tc='#A9DFBF',bold=True,z=3)
ar(DX+DW/2,dcy-3.35,DX+DW/2,dcy-3.78,c='#52BE80',hw=0.08)
tx(DX+DW/2,dcy-4.05,'→  Execute  $u_t$  →  cmd_vel',fs=9,c='#A9DFBF',bold=True)
ar(DX+DW/2,dcy-4.3,DX+DW/2,dcy-4.72,c='#52BE80',hw=0.08)
rp(DX+0.25,dcy-5.6,DW-0.5,0.85,GD,GD,lw=0,r=0.04,z=3)
tx(DX+DW/2,dcy-5.17,'→  UAV Navigation  ✓',fs=10.5,c=WHT,bold=True,z=4)

# Arrow: ctrl head → deployment
ar(HX+HW,CRY+CRH/2,DX,R1Y+4.8,c=TM,lw=2.0)

# ════════════════════════════════════════════════════════════════════════════
# ROW 2 SECTION LABEL
# ════════════════════════════════════════════════════════════════════════════
rp(0.15,R2T+0.05,W-0.3,0.38,'#F0F3F4','#BDC3C7',lw=1,r=0.04,z=1)
tx(W/2,R2T+0.24,
   '2.  Ablation Study, Evaluation Metrics and Deployment Goal',
   fs=11.5,c=ND,bold=True)

# ════════════════════════════════════════════════════════════════════════════
# 3. ABLATION STUDY (purple)
# ════════════════════════════════════════════════════════════════════════════
AX=0.15; AW=16.5
rp(AX,R2Y,AW,R2H,PL,PD,lw=2.5,r=0.1,z=1)
tb(AX,R2Y+R2H-TBAR_H,AW,TBAR_H,PD,
   '3.  Ablation Study  —  Isolating JointKD Components',fs=10.5)

MDATA=[
    ('BC','No-KD\nBaseline',GRD,GRL,GRD,
     '$u_t$  (single step)',
     r'$\mathcal{L}_{BC}=\mathrm{MSE}(u_t,u_{GT})$',
     ['No traj supervision','No temporal context','Direct action imitation'],
     'c1'),
    ('TrajKD','Ablation A\nPlanning Only',PD,PL,PD,
     r'$\tau_t$ (aux) + $u_t$ (ctrl)',
     r'$\mathcal{L}=\lambda\mathcal{L}_{traj}+\mathcal{L}_{ctrl}$',
     ['Traj head: planning aux','Ctrl head: single-step','Traj head discarded at deploy'],
     'tc1'),
    ('CtrlKD','Ablation B\nExecution Only',TD,TL,TD,
     r'$U_t$  (H-step ctrl seq)',
     r'$\mathcal{L}_{ctrl}=\sum_k w_k\mathrm{MSE}$',
     ['No traj supervision','H-step temporal ctrl','Receding horizon execution'],
     'cH'),
    ('JointKD ★','Proposed\nMethod',OD,OL,OD,
     r'$\tau_t$ (aux) + $U_t$ (ctrl seq)',
     r'$\mathcal{L}=\lambda_{traj}\mathcal{L}_{traj}+\lambda_{ctrl}\mathcal{L}_{ctrl}$',
     ['Traj head: planning KD (aux)','H-step temporal ctrl seq','Backbone encodes BOTH'],
     'tcH'),
]
CW=(AW-0.5)/4
for i,(nm,sub,tc,fc,ec,sig,loss,notes,ht) in enumerate(MDATA):
    cx=AX+0.25+i*CW; cy=R2Y+0.3; cw=CW-0.15; ch=R2H-TBAR_H-0.45
    rp(cx,cy,cw,ch,fc,ec,lw=1.8,r=0.06,z=2)
    tb(cx,cy+ch-0.52,cw,0.52,ec,nm,fs=9.5 if nm!='JointKD ★' else 10.5,z=4)
    tx(cx+cw/2,cy+ch-0.88,sub,fs=8,c=tc,bold=True)
    # Signal box
    bx(cx+0.1,cy+ch-1.8,cw-0.2,0.68,WHT,ec,z=3)
    tx(cx+0.18,cy+ch-1.55,'Teacher signal:',fs=7.5,c=tc,bold=True,ha='left')
    ax.text(cx+0.18,cy+ch-1.92,sig,fontsize=7.5,color=tc,zorder=5)
    # Mini CNN
    nnx=cx+0.12; nny=cy+ch-3.4
    cnn(nnx,nny,n=3,bw=0.22,bh=0.9,gap=0.07,fc=OM,ec=OD,z=3)
    tx(nnx+0.35,nny-0.22,'CNN\nBackbone',fs=7,c=OD,bold=True)
    ar(nnx+0.7,nny+0.45,nnx+1.05,nny+0.45,c=ec,lw=1.1,hw=0.07,hl=0.09)
    hx=nnx+1.05; hy=nny
    if ht=='c1':
        bx(hx,hy+0.05,1.3,0.85,GRL,GRD,'Ctrl Head\n(deploy)\n1-step',fs=7.5,tc=GRD,z=4)
    elif ht=='tc1':
        bx(hx,hy+0.52,1.3,0.7,RL,RD,'Traj Head\n(train only)',fs=7,tc=RD,z=4)
        bx(hx,hy-0.05,1.3,0.52,TL,TD,'Ctrl Head\n(deploy) 1-step',fs=7,tc=TD,z=4)
    elif ht=='cH':
        bx(hx,hy+0.05,1.3,0.85,TL,TD,'Ctrl Seq\nHead (deploy)\nH-step',fs=7.5,tc=TD,z=4)
    else:
        bx(hx,hy+0.55,1.3,0.7,RL,RD,'Traj Head\n(train only)',fs=7,tc=RD,z=4)
        bx(hx,hy-0.05,1.3,0.6,TL,TD,'Ctrl Seq\nHead (deploy)\nH-step',fs=7,tc=TD,z=4)
    # Loss box
    bx(cx+0.1,cy+1.5,cw-0.2,0.72,WHT,ec,z=3)
    tx(cx+0.18,cy+1.88,'Loss',fs=8,c=tc,bold=True,ha='left')
    ax.text(cx+0.18,cy+1.52,loss,fontsize=7.8,color=tc,zorder=5)
    # Notes
    for j,n in enumerate(notes):
        tx(cx+0.18,cy+1.18-j*0.28,f'• {n}',fs=7.5,c=GRD,ha='left')

# ════════════════════════════════════════════════════════════════════════════
# 4. EVALUATION (navy)
# ════════════════════════════════════════════════════════════════════════════
EX=AX+AW+0.2; EW=5.45
rp(EX,R2Y,EW,R2H,NT,ND,lw=2.5,r=0.1,z=1)
tb(EX,R2Y+R2H-TBAR_H,EW,TBAR_H,ND,'4. Evaluation Metrics',fs=10.5)

tx(EX+EW/2,R2Y+R2H-1.05,'Gazebo Deployment',fs=9,c=ND,bold=True)
for j,(k,nm,desc) in enumerate([
        ('SR','Success Rate','% missions completed'),
        ('CR','Collision Rate','% missions with collision'),
        ('ATE','Trajectory Error','Avg displacement from ref'),
        ('S','Smoothness','Ctrl jerk magnitude'),
        ('FPS','Inference Speed','Frames / sec (real-time)'),]):
    yp=R2Y+R2H-1.55-j*1.0
    bx(EX+0.2,yp-0.32,0.68,0.68,ND,ND,k,fs=10,tc=WHT,bold=True,z=3)
    tx(EX+1.05,yp+0.02,nm,fs=9,c=ND,bold=True,ha='left')
    tx(EX+1.05,yp-0.26,desc,fs=8,c=GRD,ha='left')

tx(EX+EW/2,R2Y+1.95,'Offline Metrics',fs=9,c=ND,bold=True)
for j,item in enumerate([
        r'• Control MSE  (vx / vy / vz / yaw_rate)',
        r'• Trajectory ATE  (avg waypoint error)',
        r'• Val Loss  ($\mathcal{L}$/$\mathcal{L}_{traj}$/$\mathcal{L}_{ctrl}$)',
        r'• λ sweep:  $\lambda\in\{0.1,\,1.0,\,5.0\}$',
        r'• Model size (MB)  ·  Inference FPS',]):
    ax.text(EX+0.25,R2Y+1.62-j*0.3,item,fontsize=8,color=GRD,zorder=5)

# ════════════════════════════════════════════════════════════════════════════
# 5. DEPLOYMENT GOAL (green)
# ════════════════════════════════════════════════════════════════════════════
GX=EX+EW+0.2; GW=W-GX-0.15
rp(GX,R2Y,GW,R2H,GL,GD,lw=2.5,r=0.1,z=1)
tb(GX,R2Y+R2H-TBAR_H,GW,TBAR_H,GD,'5. Deployment Goal',fs=10.5)
tx(GX+GW/2,R2Y+R2H-1.1,'Lightweight Vision-Based\nUAV Navigation',fs=10,c=GD,bold=True)
tx(GX+GW/2,R2Y+R2H-2.02,'Front Camera Only',fs=9,c=GM,bold=True)
for j,g in enumerate([
        'Camera only  (no LiDAR / map)',
        'Lightweight model\n  (MobileNetV2)',
        'Real-time inference\n  (target: 30+ FPS)',
        'Obstacle-aware navigation\n  (from planning-level KD)',
        'Temporally smooth control\n  (from H-step execution KD)',
        'No teacher at runtime\n  (fully autonomous)',]):
    yp=R2Y+R2H-2.65-j*0.97
    tx(GX+0.35,yp,'✓',fs=14,c=GD,bold=True,ha='left')
    tx(GX+0.9,yp,g,fs=8,c=GRD,ha='left')

# ════════════════════════════════════════════════════════════════════════════
# LEGEND
# ════════════════════════════════════════════════════════════════════════════
LEG=[(BL,BD,'Existing Work (Teacher)'),(OL,OD,'Core Contribution (JointKD)'),
     (RL,RD,'Train-Only'),(TL,TD,'Deployed'),(PL,PD,'Ablation Study'),
     (NL,ND,'Evaluation'),(GL,GD,'Deployment Goal')]
lx=0.2
for j,(fc,ec,lab) in enumerate(LEG):
    xx=lx+j*3.95
    rp(xx,0.04,0.4,0.13,fc,ec,lw=1.2,r=0.02,z=5)
    tx(xx+0.5,0.105,lab,fs=7.5,c=BLK,ha='left',z=5)
arx=lx+len(LEG)*3.95
ar(arx,0.105,arx+0.5,0.105,c=BLK,lw=1.5,hw=0.07,hl=0.09)
tx(arx+0.6,0.105,'Data Flow',fs=7.5,c=BLK,ha='left')
ax.annotate('',xy=(arx+1.75,0.105),xytext=(arx+1.2,0.105),
    arrowprops=dict(arrowstyle='->,head_width=0.07,head_length=0.09',
                   color=BM,lw=1.2,linestyle='--'),zorder=5)
tx(arx+1.85,0.105,'Teacher Signal',fs=7.5,c=BLK,ha='left')

plt.savefig('/home/l/uav_kd_project/docs/framework_overview.png',
            dpi=150,bbox_inches='tight',facecolor='white')
plt.close()
print("Saved.")
