#!/usr/bin/env python3
"""Generate publication-quality mechanism figures for the SPPT manuscript."""

from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Arc, Circle, FancyArrowPatch, FancyBboxPatch, Polygon, Rectangle

OUT = Path(__file__).resolve().parents[1] / "docs" / "latex" / "figures"
C = {"ink": "#24323D", "muted": "#667782", "line": "#D9E2E6", "pale": "#F5F7F8",
     "white": "#FFFFFF", "ac": "#2673A8", "acp": "#DCEAF4", "dc": "#2E9270",
     "dcp": "#DCEFE7", "sem": "#7558A6", "semp": "#EAE4F3", "guard": "#D4881F",
     "guardp": "#F8EBD5", "bad": "#C84B42", "badp": "#F7E0DD"}


def configure():
    plt.rcParams.update({"font.family": "sans-serif", "font.sans-serif": ["Arial", "Helvetica", "DejaVu Sans"],
                         "font.size": 7.5, "pdf.fonttype": 42, "ps.fonttype": 42,
                         "savefig.dpi": 420, "savefig.bbox": "tight", "savefig.pad_inches": .03})


def save(fig, name):
    for ext in ("pdf", "png"):
        fig.savefig(OUT / f"{name}.{ext}", facecolor="white")
    plt.close(fig)


def clean(ax, xlim=(0, 10), ylim=(0, 10), equal=True):
    ax.set_xlim(*xlim); ax.set_ylim(*ylim)
    if equal: ax.set_aspect("equal", adjustable="box")
    ax.axis("off")


def panel(ax, letter, title):
    ax.text(0, 1, letter, transform=ax.transAxes, va="top", fontsize=8.5, weight="bold", color=C["ink"])
    ax.text(.075, 1, title, transform=ax.transAxes, va="top", fontsize=7.7, weight="bold", color=C["ink"])


def arrow(ax, start, end, color=None, lw=1.1, connection="arc3", style="-|>", z=5):
    ax.add_patch(FancyArrowPatch(start, end, arrowstyle=style, mutation_scale=8, linewidth=lw,
                                color=color or C["ink"], connectionstyle=connection,
                                shrinkA=1.5, shrinkB=1.5, zorder=z))


def box(ax, xy, w, h, face, edge, radius=.12, lw=.9, z=1):
    patch = FancyBboxPatch(xy, w, h, boxstyle=f"round,pad=.03,rounding_size={radius}",
                           facecolor=face, edgecolor=edge, linewidth=lw, zorder=z)
    ax.add_patch(patch); return patch


def chip(ax, x, y, text, color=None, w=None):
    color = color or C["sem"]; w = w or max(.72, .105*len(text))
    box(ax, (x-w/2, y-.19), w, .38, C["white"], color, .08, .75, 8)
    ax.text(x, y, text, ha="center", va="center", fontsize=5.7, color=C["ink"], zorder=9)


def ac_bus(ax, x, y, r=.12, label=None):
    ax.add_patch(Circle((x, y), r, facecolor="white", edgecolor=C["ac"], lw=1.3, zorder=5))
    for start in (0, 180):
        ax.add_patch(Arc((x, y), r*1.18, r*.70, theta1=start, theta2=start+180,
                         edgecolor=C["ac"], lw=.65, zorder=6))
    if label: ax.text(x, y-r-.10, label, ha="center", va="top", fontsize=5.6, color=C["muted"])


def dc_bus(ax, x, y, r=.12, label=None):
    ax.add_patch(Circle((x, y), r, facecolor="white", edgecolor=C["dc"], lw=1.3, zorder=5))
    ax.plot([x-r*.55, x+r*.55], [y+.025, y+.025], color=C["dc"], lw=.7, zorder=6)
    ax.plot([x-r*.36, x+r*.36], [y-.035, y-.035], color=C["dc"], lw=.7, zorder=6)
    if label: ax.text(x, y-r-.10, label, ha="center", va="top", fontsize=5.6, color=C["muted"])


def converter(ax, x, y, scale=1, label=None, face="white", edge=None):
    edge = edge or C["sem"]; w, h = .56*scale, .40*scale
    box(ax, (x-w/2, y-h/2), w, h, face, edge, .055*scale, 1.05, 5)
    ax.plot([x-w*.29, x+w*.29], [y+h*.25, y-h*.25], color=edge, lw=.95, zorder=6)
    if label: ax.text(x, y-h/2-.11*scale, label, ha="center", va="top", fontsize=5.6, color=C["ink"])


def solar(ax, x, y, scale=1):
    p = [[x-.18*scale, y-.07*scale], [x+.18*scale, y-.07*scale],
         [x+.12*scale, y+.14*scale], [x-.13*scale, y+.14*scale]]
    ax.add_patch(Polygon(p, closed=True, facecolor=C["acp"], edgecolor=C["ac"], lw=.9, zorder=5))
    ax.plot([x, x], [y-.07*scale, y-.20*scale], color=C["ink"], lw=.7, zorder=4)


def battery(ax, x, y, scale=1):
    w, h = .25*scale, .36*scale
    box(ax, (x-w/2, y-h/2), w, h, C["dcp"], C["dc"], .025*scale, .9, 5)
    ax.add_patch(Rectangle((x-.045*scale, y+h/2), .09*scale, .035*scale, facecolor=C["dc"], edgecolor="none"))
    ax.text(x, y, "+\n-", ha="center", va="center", fontsize=5.2, linespacing=.65, color=C["dc"], zorder=6)


def load(ax, x, y, scale=1, color=None):
    color = color or C["ac"]
    ax.add_patch(Polygon([[x-.11*scale, y+.12*scale], [x+.11*scale, y+.12*scale],
                          [x, y-.11*scale]], closed=True, facecolor=color, edgecolor=color, zorder=5))


def feeder(ax, x, y, scale=1, highlight=None):
    ac = [x+scale*v for v in (0, .82, 1.64, 2.46)]
    ax.plot([ac[0], ac[-1]], [y, y], color=C["ac"], lw=1.7, zorder=3)
    for value in ac: ac_bus(ax, value, y, .085*scale)
    solar(ax, ac[1], y+.52*scale, .72*scale); ax.plot([ac[1]]*2, [y+.08*scale, y+.37*scale], color=C["ac"], lw=.9)
    load(ax, ac[2], y+.40*scale, .75*scale); ax.plot([ac[2]]*2, [y+.08*scale, y+.29*scale], color=C["ac"], lw=.9)
    cx = ac[-1]+.48*scale; ax.plot([ac[-1], cx-.27*scale], [y, y], color=C["ac"], lw=1.4)
    converter(ax, cx, y, .88*scale)
    dc = [cx+scale*v for v in (.50, 1.25, 2.0)]
    ax.plot([cx+.27*scale, dc[-1]], [y, y], color=C["dc"], lw=1.7)
    for value in dc: dc_bus(ax, value, y, .085*scale)
    battery(ax, dc[1], y+.47*scale, .72*scale); ax.plot([dc[1]]*2, [y+.08*scale, y+.31*scale], color=C["dc"], lw=.9)
    ax.add_patch(Rectangle((dc[2]-.11*scale, y+.28*scale), .22*scale, .20*scale,
                           facecolor=C["dcp"], edgecolor=C["dc"], lw=.8))
    ax.plot([dc[2]]*2, [y+.08*scale, y+.28*scale], color=C["dc"], lw=.9)
    loc = {"pv": (ac[1], y+.52*scale), "load": (ac[2], y+.40*scale), "vsc": (cx, y),
           "battery": (dc[1], y+.47*scale), "dc_load": (dc[2], y+.38*scale)}
    if highlight:
        hx, hy = loc[highlight]
        ax.add_patch(Circle((hx, hy), .26*scale, fill=False, edgecolor=C["sem"], lw=1.1,
                            linestyle=(0, (2, 1.4)), zorder=7))
    return loc


def shield(ax, x, y, scale=.7, face=None, edge=None):
    face, edge = face or C["guardp"], edge or C["guard"]
    pts = np.array([[0, .62], [.50, .40], [.43, -.28], [0, -.70], [-.43, -.28], [-.50, .40]])
    ax.add_patch(Polygon(pts*scale+np.array([x, y]), closed=True, facecolor=face, edgecolor=edge, lw=1.05, zorder=4))


def overview():
    fig = plt.figure(figsize=(7.15, 3.35)); ax = fig.add_axes([.015, .06, .97, .90]); clean(ax, (0, 16), (0, 7.1))
    ax.add_patch(Rectangle((0, .55), 5.25, 5.95, facecolor="#F8FAFB", edgecolor="none"))
    ax.text(.25, 6.12, "Rich engineering model", fontsize=7.8, weight="bold", color=C["ink"])
    loc = feeder(ax, .35, 3.65, .76, "load")
    ax.text(.55, 2.72, "AC feeder", fontsize=6, weight="bold", color=C["ac"])
    ax.text(3.82, 2.72, "DC subsystem", fontsize=6, weight="bold", color=C["dc"])
    labels = [(loc["pv"], (1.00, 5.68), "PV-14 | kW | ABC"), (loc["load"], (2.62, 5.27), "load L-09 | +10%"),
              (loc["vsc"], (3.18, 2.18), r"VSC-17 | $V_{dc}$-$Q$"), (loc["battery"], (4.52, 5.70), "BESS-03 | SOC")]
    for target, origin, text in labels:
        chip(ax, *origin, text, C["sem"]); arrow(ax, (origin[0], origin[1]-.22), target, C["sem"], .65, style="-", z=2)
    ax.text(.30, 1.12, "heterogeneous files", fontsize=5.8, color=C["muted"])
    for i, text in enumerate(("GLM", "DSS", "JSON")):
        x=.35+i*.58; box(ax, (x, .58), .46, .38, "white", C["line"], .035, .7); ax.text(x+.23, .77, text, ha="center", va="center", fontsize=5.1, color=C["muted"])
    ax.text(2.40, .79, "+", fontsize=7, color=C["muted"], ha="center"); chip(ax, 3.45, .78, "AI modification", C["sem"], 1.55)
    ax.add_patch(Rectangle((5.25, .55), 3.20, 5.95, facecolor=C["semp"], edgecolor="none", alpha=.42))
    ax.text(6.85, 6.12, "Semantic projection", ha="center", fontsize=7.8, weight="bold", color=C["sem"])
    for i, (symbol, caption) in enumerate((("U","units"),("E","terminals"),("M","topology"),("D","scope"))):
        x=5.72+i*.73; ax.add_patch(Circle((x,4.35),.22,facecolor="white",edgecolor=C["sem"],lw=1,zorder=5))
        ax.text(x,4.35,rf"$\mathcal{{{symbol}}}$",ha="center",va="center",fontsize=7.2,color=C["sem"]); ax.text(x,3.93,caption,ha="center",va="top",fontsize=5.1,color=C["muted"])
        if i<3: arrow(ax,(x+.25,4.35),(x+.46,4.35),C["sem"],.8)
    for i,x in enumerate(np.linspace(5.7,7.95,8)): ax.add_patch(Circle((x,2.55),.055+i*.004,facecolor=C["guard"],edgecolor="none",alpha=.55+.05*i))
    ax.text(6.85,2.20,"correspondence record",ha="center",fontsize=5.8,color=C["guard"],weight="bold")
    ax.add_patch(Circle((6.05,1.20),.12,facecolor=C["bad"],edgecolor="none")); arrow(ax,(5.15,1.20),(5.88,1.20),C["bad"],1)
    ax.plot([6.25,6.47],[1,1.40],color=C["bad"],lw=1.25); ax.text(6.70,1.19,"invalid terminal / role",va="center",fontsize=5.4,color=C["bad"])
    ax.add_patch(Rectangle((8.45,.55),7.55,5.95,facecolor="#FAFBFB",edgecolor="none")); ax.text(8.78,6.12,"Canonical hybrid AC/DC substrate",fontsize=7.8,weight="bold",color=C["ink"])
    nodes=[(9.15,4.45,"ac"),(10.15,4.85,"ac"),(11.15,4.25,"ac"),(12.30,4.65,"dc"),(13.35,4.15,"dc")]
    for p,q in zip(nodes[:-1],nodes[1:]): ax.plot([p[0],q[0]],[p[1],q[1]],color=C[p[2]],lw=1.45)
    for x,y,d in nodes: (ac_bus if d=="ac" else dc_bus)(ax,x,y,.14)
    converter(ax,11.72,4.46,.68); ax.text(9.02,3.52,r"$F_{ac}(V,\theta)=0$",fontsize=6.4,color=C["ac"])
    ax.text(12.35,3.52,r"$F_{dc}(v)=0$",fontsize=6.4,color=C["dc"]); ax.text(10.62,3.02,r"$F_{vsc}(V,\theta,v)=0$",fontsize=6.4,color=C["sem"])
    center=(11.35,4.05)
    for diameter,color in ((4.70,C["guard"]),(5.05,C["sem"]),(5.40,C["dc"])):
        ax.add_patch(Arc(center,diameter,diameter*.62,theta1=20,theta2=150,edgecolor=color,lw=1.05))
    for x,y,label,color in ((13.92,5.44,"valid",C["guard"]),(14.12,5.03,"well-posed",C["sem"]),(14.30,4.62,"attributable",C["dc"])):
        ax.text(x,y,label,fontsize=5.2,color=color,ha="left",va="center")
    ax.text(8.78,2.02,"Device-level evidence",fontsize=6.6,weight="bold",color=C["ink"])
    items=[(9.35,r"$V_{B-204}$",C["ac"]),(10.75,r"$P_{C-31}$",C["sem"]),(12.15,r"$v_{D-08}$",C["dc"]),(13.65,r"$P_{VSC-17}$",C["guard"])]
    for x,text,color in items:
        ax.add_patch(Circle((x,1.45),.25,facecolor="white",edgecolor=color,lw=1.05)); ax.text(x,1.45,text,ha="center",va="center",fontsize=5.6)
    arrow(ax,(13.90,1.45),(15.35,1.45),C["dc"],1.25); ax.text(14.65,1.78,"report",ha="center",fontsize=5.4,color=C["dc"])
    arrow(ax,(9.08,1.12),(4.82,1.02),C["sem"],.9,"arc3,rad=-.03"); ax.text(7.05,.66,"reconstruction to stable device identities",ha="center",fontsize=5.5,color=C["sem"])
    save(fig,"sppt_mechanism_overview")


def projection():
    fig,axs=plt.subplots(1,3,figsize=(7.15,3.10),gridspec_kw={"wspace":.08})
    for ax in axs: clean(ax)
    panel(axs[0],"a","Terminal expansion"); panel(axs[1],"b","Topology and scope"); panel(axs[2],"c","Reverse attribution")
    ax=axs[0]; ax.text(1.65,7.95,"rich device",ha="center",fontsize=6,color=C["muted"])
    ac_bus(ax,.9,6.45,.19); converter(ax,2.1,6.45,1.35,"VSC-17",C["semp"]); dc_bus(ax,3.3,6.45,.19)
    ax.plot([1.09,1.72],[6.45]*2,color=C["ac"],lw=1.5); ax.plot([2.48,3.11],[6.45]*2,color=C["dc"],lw=1.5)
    for x,y,text,color,target in [(.78,4.92,"AC terminal",C["ac"],(1.02,6.22)),(2.15,3.88,r"$V_{dc}$-$Q$ role",C["sem"],(2.18,6.15)),(3.62,4.92,"DC terminal",C["dc"],(3.38,6.25))]:
        ax.text(x,y,text,ha="center",va="center",fontsize=5.7,color=color)
        arrow(ax,(x,y+.18),target,color,.7,style="-",z=2)
    arrow(ax,(4.35,6.42),(5.5,6.42),C["sem"],1.25); ax.text(4.92,6.76,r"$\mathcal{E}$",ha="center",fontsize=9,color=C["sem"])
    ax.text(7.55,7.95,"canonical coupling",ha="center",fontsize=6,color=C["muted"])
    ac_bus(ax,5.50,6.45,.18); converter(ax,7.50,6.45,1.2); dc_bus(ax,9.50,6.45,.18)
    ax.plot([5.68,7.16],[6.45]*2,color=C["ac"],lw=1.4); ax.plot([7.84,9.32],[6.45]*2,color=C["dc"],lw=1.4)
    ax.text(5.50,5.70,r"$(V,\theta)$",ha="center",fontsize=6.1,color=C["ac"]); ax.text(7.50,4.92,r"$P_{ac}+P_{dc}+P_{loss}=0$",ha="center",fontsize=5.7,color=C["sem"]); ax.text(9.50,5.70,r"$v$",ha="center",fontsize=6.1,color=C["dc"])
    for x,color,name in [(5.50,C["ac"],"B-204"),(7.50,C["sem"],"VSC-17"),(9.50,C["dc"],"D-08")]: ax.plot([x,x],[5.35,2.55],color=color,lw=.9,alpha=.7); chip(ax,x,2.2,name,color,1.08)
    ax.text(7.45,1.15,"device origin retained",ha="center",fontsize=5.7,color=C["guard"],weight="bold")
    ax=axs[1]; ax.text(2.2,8.2,"engineering topology",ha="center",fontsize=6,color=C["muted"])
    rich=[(1,6.8),(2,7.3),(3,6.5),(4,7.1)]
    for p,q in zip(rich[:-1],rich[1:]): ax.plot([p[0],q[0]],[p[1],q[1]],color=C["ac"],lw=1.25)
    for x,y in rich: ac_bus(ax,x,y,.15)
    ax.plot([3,3.45],[6.5,5.4],color=C["ac"],lw=1); ac_bus(ax,3.45,5.4,.14); chip(ax,2.47,5.25,r"$z=0$ link",C["guard"],1.3); arrow(ax,(2.55,5.47),(2.48,6.75),C["guard"],.7,style="-",z=2)
    ax.plot([.95,2.15],[3.45,3.1],color=C["muted"],lw=1,alpha=.35)
    for x,y in ((.95,3.45),(2.15,3.1)): ax.add_patch(Circle((x,y),.14,facecolor="white",edgecolor=C["muted"],lw=1,alpha=.45))
    ax.add_patch(Arc((1.55,3.27),1.8,1.25,theta1=0,theta2=360,edgecolor=C["bad"],lw=1,linestyle=(0,(3,2)))); ax.text(1.55,2.42,"de-energized",ha="center",fontsize=5.4,color=C["bad"])
    arrow(ax,(4.6,6.35),(5.55,6.35),C["sem"],1.25); ax.text(5.05,6.72,r"$\mathcal{D}\circ\mathcal{M}$",ha="center",fontsize=8.2,color=C["sem"]); ax.text(7.65,8.2,"canonical graph",ha="center",fontsize=6,color=C["muted"])
    can=[(6.15,6.75),(7.45,7.15),(8.65,6.45)]
    for p,q in zip(can[:-1],can[1:]): ax.plot([p[0],q[0]],[p[1],q[1]],color=C["ac"],lw=1.4)
    for x,y in can: ac_bus(ax,x,y,.17)
    ax.add_patch(Circle((7.45,7.15),.32,fill=False,edgecolor=C["guard"],lw=1,linestyle=(0,(2,1.5)))); ax.text(7.45,5.6,"merge class",ha="center",fontsize=5.6,color=C["guard"],weight="bold"); ax.text(7.45,4.95,r"$\{B_2,B_3\}\mapsto n_2$",ha="center",fontsize=6.1)
    box(ax,(5.45,2.18),4.05,1.42,C["guardp"],C["guard"],.12,.8); ax.text(7.475,3.08,"transformation record",ha="center",va="center",fontsize=5.8,color=C["guard"],weight="bold"); ax.text(7.475,2.55,"merge + energized scope",ha="center",va="center",fontsize=5.3)
    ax=axs[2]; ax.text(1.35,8.25,"canonical observables",ha="center",fontsize=6,color=C["muted"]); ax.text(7.70,8.25,"engineering evidence",ha="center",fontsize=6,color=C["muted"])
    mapping=[(7.20,r"$V_2$","bus B-204",C["ac"]),(6.00,r"$P_{23}$","cable C-31",C["sem"]),
             (4.80,r"$v_4$","DC bus D-08",C["dc"]),(3.60,r"$P_c$","converter VSC-17",C["guard"])]
    for y,observable,asset,color in mapping:
        ax.add_patch(Circle((1.35,y),.25,facecolor="white",edgecolor=color,lw=1.1)); ax.text(1.35,y,observable,ha="center",va="center",fontsize=6)
        box(ax,(6.48,y-.32),2.77,.64,"white",color,.1,.9); ax.text(7.865,y,asset,ha="center",va="center",fontsize=5.7)
        arrow(ax,(1.64,y),(6.34,y),color,.85,"arc3")
    ax.text(4.15,7.53,r"$\mathcal{R}_S$",ha="center",fontsize=8.2,color=C["sem"])
    box(ax,(.75,1.25),8.55,1.15,C["pale"],C["line"],.11,.7); ax.text(5.03,1.95,r"$\mathcal{R}_S(A_C(\Pi(S)))=A_S(S)$",ha="center",va="center",fontsize=6.9); ax.text(5.03,1.48,"observable, origin and scope commute",ha="center",va="center",fontsize=5.4,color=C["muted"])
    save(fig,"sppt_mechanism_projection")


def roles():
    fig=plt.figure(figsize=(7.15,3.75)); grid=fig.add_gridspec(2,2,left=.03,right=.985,bottom=.08,top=.98,wspace=.12,hspace=.16); axs=[fig.add_subplot(grid[i,j]) for i in range(2) for j in range(2)]
    for ax in axs: clean(ax,(0,10),(0,5.1))
    for ax,l,t in zip(axs,"abcd",["Closed control-role assignment","Missing DC balancing role","Structural rank signature","Device-level diagnosis"]): panel(ax,l,t)
    ax=axs[0]; y=2.65; ac_bus(ax,.85,y,.19,"AC reference"); converter(ax,2.55,y,1.15,r"$P$-$Q$"); dc_bus(ax,4.2,y,.19); dc_bus(ax,5.8,y,.19); converter(ax,7.45,y,1.15,r"$V_{dc}$-$Q$",C["dcp"]); ac_bus(ax,9.15,y,.19)
    ax.plot([1.04,2.22],[y]*2,color=C["ac"],lw=1.35); ax.plot([2.88,7.12],[y]*2,color=C["dc"],lw=1.35); ax.plot([7.78,8.96],[y]*2,color=C["ac"],lw=1.35)
    arrow(ax,(.85,4.25),(.85,2.9),C["ac"],1); ax.text(.85,4.48,r"fix $\theta_{ref}$",ha="center",fontsize=6.1,color=C["ac"]); arrow(ax,(7.45,4.25),(7.45,2.9),C["dc"],1); ax.text(7.45,4.48,r"fix $v_{dc}$",ha="center",fontsize=6.1,color=C["dc"])
    ax.plot([4.2,5.8],[1.18]*2,color=C["dc"],lw=1.1); ax.scatter([4.2,5.8],[1.18]*2,s=17,color=C["dc"],zorder=5); ax.text(5,.72,"unique DC potential",ha="center",fontsize=5.7,color=C["dc"],weight="bold")
    ax=axs[1]; y=2.55; ac_bus(ax,.85,y,.19,"AC reference"); converter(ax,2.55,y,1.15,r"$P$-$Q$"); dc_bus(ax,4.2,y,.19); dc_bus(ax,5.8,y,.19); converter(ax,7.45,y,1.15,r"$P$-$Q$",C["badp"],C["bad"]); ac_bus(ax,9.15,y,.19)
    ax.plot([1.04,2.22],[y]*2,color=C["ac"],lw=1.35); ax.plot([2.88,7.12],[y]*2,color=C["dc"],lw=1.35); ax.plot([7.78,8.96],[y]*2,color=C["ac"],lw=1.35)
    ax.add_patch(Arc((5.0,y),3.1,1.45,theta1=0,theta2=360,edgecolor=C["bad"],lw=1.0,linestyle=(0,(3,2))))
    ax.text(5.0,4.30,r"no equation assigns $v_{dc}^{\star}$ or droop",ha="center",fontsize=6.4,color=C["bad"])
    ax.text(5.0,1.15,"DC voltage and balancing power lack compatible roles",ha="center",fontsize=5.6,color=C["muted"])
    ax=axs[2]; labels=[r"$\theta_{ref}$",r"$P_c$",r"$Q_c$",r"$v_{dc}$",r"$P_{bal}$"]
    matrix=np.array([[1,.22,0,0,0],[.18,1,.20,.12,0],[0,.18,1,.10,0],[0,.08,.08,1,.18],[0,0,0,0,0]])
    for i in range(5):
        for j in range(5):
            value=matrix[i,j]; face=C["badp"] if i==4 else (C["semp"] if value>.5 else C["pale"])
            ax.add_patch(Rectangle((1.80+j*.82,3.90-i*.60),.69,.48,facecolor=face,edgecolor=C["bad"] if i==4 else C["line"],lw=.65))
            if value>0: ax.add_patch(Circle((2.145+j*.82,4.14-i*.60),.05+.065*value,facecolor=C["sem"],edgecolor="none",alpha=.5+.4*value))
    for j,text in enumerate(labels): ax.text(2.145+j*.82,4.60,text,ha="center",fontsize=5.1,color=C["muted"])
    for i,text in enumerate(("AC ref","AC bal.","Q bal.","DC bal.","role eq.")): ax.text(1.58,4.14-i*.60,text,ha="right",va="center",fontsize=5.0,color=C["muted"])
    ax.text(3.85,.75,"missing role equation leaves an unmatched variable",ha="center",fontsize=5.5,color=C["bad"],weight="bold")
    arrow(ax,(6.20,2.70),(6.85,2.70),C["bad"],1.0); ax.text(8.10,3.20,r"$\mathrm{rank}(J_r)<n_r$",ha="center",fontsize=7.0,color=C["sem"]); ax.text(8.10,2.35,"structurally singular",ha="center",fontsize=5.8,color=C["bad"])
    ax=axs[3]; ax.add_patch(Arc((2,2.85),2.35,2.35,theta1=0,theta2=360,edgecolor=C["bad"],lw=1.1,linestyle=(0,(3,2)))); dc_bus(ax,1.45,2.85,.17); dc_bus(ax,2.55,2.85,.17); converter(ax,2,1.7,1,"VSC-17",C["badp"],C["bad"]); ax.plot([1.45,2.55],[2.85]*2,color=C["dc"],lw=1.2); ax.plot([2,2],[2.68,1.92],color=C["dc"],lw=1); ax.text(2,4.35,"DC island D-2",ha="center",fontsize=6.1,weight="bold")
    arrow(ax,(3.4,2.75),(4.45,2.75),C["guard"],1.1); shield(ax,5.35,2.75,.78); ax.text(5.35,2.91,"role",ha="center",fontsize=5.5,color=C["guard"],weight="bold"); ax.text(5.35,2.55,"check",ha="center",fontsize=5.5,color=C["guard"],weight="bold"); arrow(ax,(6.22,2.75),(7.05,2.75),C["bad"],1.1)
    for y,text,color,weight in [(3.77,"missing support",C["bad"],"bold"),(3.2,"device: VSC-17",C["ink"],"normal"),(2.73,r"role: $P$-$Q$",C["ink"],"normal"),(2.26,r"repair: $V_{dc}$ or droop",C["sem"],"normal"),(1.44,"reject before solve",C["bad"],"bold")]: ax.text(7.35,y,text,fontsize=5.9 if weight=="bold" else 5.6,color=color,weight=weight)
    save(fig,"sppt_mechanism_converter_roles")


def agent():
    fig=plt.figure(figsize=(7.15,3.30)); grid=fig.add_gridspec(1,3,left=.025,right=.985,bottom=.06,top=.98,width_ratios=[.95,1.12,1.10],wspace=.1); axs=[fig.add_subplot(grid[0,i]) for i in range(3)]
    for ax in axs: clean(ax)
    panel(axs[0],"a","Structured engineering request"); panel(axs[1],"b","Isolated model assessment"); panel(axs[2],"c","Acceptance requires physical evidence")
    ax=axs[0]; box(ax,(.65,6.35),3.5,1.35,C["semp"],C["sem"],.18,.9); ax.text(2.4,7.02,'"Scale all loads',ha="center",fontsize=6); ax.text(2.4,6.61,'by 1.10"',ha="center",fontsize=6); ax.text(2.4,5.82,"untrusted intent",ha="center",fontsize=5.4,color=C["muted"]); arrow(ax,(4.28,7.02),(5.25,7.02),C["sem"],1.15)
    box(ax,(5.38,4.55),4,4.55,"white",C["sem"],.16,1); ax.text(7.38,8.62,"typed modification",ha="center",fontsize=6.2,color=C["sem"],weight="bold")
    for i,(key,val) in enumerate((("operation","scale"),("asset","all loads"),("magnitude","1.10"),("use","model change"),("evidence","user request"))):
        y=8-i*.64; ax.text(6.88,y,key,fontsize=4.9,color=C["muted"],va="center",ha="right"); ax.text(7.08,y,val,fontsize=5.4,va="center",ha="left");
        if i<4: ax.plot([5.78,8.95],[y-.32]*2,color=C["line"],lw=.55)
    shield(ax,7.35,2.5,.88,C["semp"],C["sem"]); ax.text(7.35,2.52,"allowed",ha="center",va="center",fontsize=5.2,color=C["sem"],weight="bold",zorder=6); ax.text(7.35,1.22,"AI proposes; admissible set constrains",ha="center",fontsize=5.4,color=C["muted"])
    ax=axs[1]; ax.add_patch(Rectangle((.25,5.05),9.4,3.95,facecolor=C["dcp"],edgecolor="none",alpha=.8)); ax.add_patch(Rectangle((.25,.65),9.4,3.95,facecolor=C["semp"],edgecolor="none",alpha=.8)); ax.text(.55,8.55,r"accepted model  $S$",fontsize=6.6,weight="bold",color=C["dc"]); ax.text(.55,4.15,r"proposed model  $S'$",fontsize=6.6,weight="bold",color=C["sem"]); feeder(ax,.75,6.75,.65); feeder(ax,.75,2.35,.65,"load"); ax.plot([.25,9.65],[4.82]*2,color=C["sem"],lw=1.2,linestyle=(0,(4,2))); ax.text(6.8,4.18,"assessment boundary",fontsize=5.2,color=C["sem"],ha="center"); arrow(ax,(8.6,6.25),(8.6,3.05),C["sem"],.9,"arc3,rad=.35"); ax.text(9.25,4.64,"derive",fontsize=5.3,color=C["sem"],rotation=-90,va="center"); chip(ax,7.05,1.25,"load factor = 1.10",C["sem"],2.2); ax.text(4.95,.18,"modification is evaluated only in the proposed model",ha="center",fontsize=5.2,color=C["muted"])
    ax=axs[2]; ax.text(1.25,8.42,"evidence",ha="center",fontsize=5.7,color=C["muted"],weight="bold"); evidence=[(7.75,"model integrity",C["ac"]),(6.80,"role closure",C["guard"]),(5.85,"device relation",C["sem"])]
    for y,text,color in evidence: ax.add_patch(Circle((.62,y),.16,facecolor=color,edgecolor="none")); ax.text(.98,y,text,va="center",fontsize=5.5)
    ax.plot([3.35]*2,[4.35,8.08],color=C["guard"],lw=2.2,alpha=.75)
    for y in np.linspace(4.75,7.78,6): ax.add_patch(Circle((3.35,y),.11,facecolor=C["guardp"],edgecolor=C["guard"],lw=.7))
    ax.text(3.35,8.42,"gate",ha="center",fontsize=5.7,color=C["guard"],weight="bold")
    for y,_,color in evidence: arrow(ax,(2.65,y),(3.12,y),color,.9)
    arrow(ax,(3.56,7.2),(5.1,7.2),C["dc"],1.4); ax.add_patch(Circle((5.5,7.2),.32,facecolor=C["dcp"],edgecolor=C["dc"],lw=1.1)); ax.text(5.5,7.2,r"$S'$",ha="center",va="center",fontsize=6.1,color=C["dc"]); arrow(ax,(5.85,7.2),(7.1,7.2),C["dc"],1.2); ax.text(7.3,7.2,r"$S\leftarrow S'$",va="center",fontsize=7.1,color=C["dc"],weight="bold"); ax.text(6.65,6.48,"adopt model",ha="center",fontsize=5.5,color=C["dc"])
    arrow(ax,(3.15,5.2),(2.15,3.85),C["bad"],1.1,"arc3,rad=.15"); ax.add_patch(Circle((1.9,3.42),.28,facecolor=C["badp"],edgecolor=C["bad"],lw=1)); ax.text(1.9,3.42,r"$S'$",ha="center",va="center",fontsize=6,color=C["bad"]); ax.plot([1.62,2.18],[3.14,3.7],color=C["bad"],lw=1); ax.plot([1.62,2.18],[3.7,3.14],color=C["bad"],lw=1); ax.text(1.9,2.78,"discard",ha="center",fontsize=5.4,color=C["bad"])
    ax.plot([4,8.85],[3.35]*2,color=C["dc"],lw=1.3); ax.scatter([4,8.85],[3.35]*2,s=20,color=C["dc"],zorder=5); ax.text(4,3.78,r"$S_{before}$",ha="center",fontsize=5.7,color=C["dc"]); ax.text(8.85,3.78,r"$S_{after}$",ha="center",fontsize=5.7,color=C["dc"]); ax.text(6.43,2.78,r"refusal: $S_{after}=S_{before}$",ha="center",fontsize=6.3,weight="bold"); ax.text(6.43,2.1,"the accepted model remains available for subsequent analysis",ha="center",fontsize=5.2,color=C["muted"])
    save(fig,"sppt_mechanism_agent")


def main():
    configure(); OUT.mkdir(parents=True,exist_ok=True); overview(); projection(); roles(); agent()
    print(f"Generated four vector mechanism figures in {OUT}")


if __name__ == "__main__": main()
