"""Plot the actual rim vs the rounded collision hull at the nearest flight-pose corner."""
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle,Ellipse
from pathlib import Path

fig,axes=plt.subplots(1,2,figsize=(10,4),sharey=True)
for ax,offset,title in zip(axes,[.035,.075],['Current: 35 mm rotor offset','Physical-envelope candidate: 75 mm']):
    ax.add_patch(Rectangle((-.06,-.04),.06,.08,facecolor='#aeb7c2',edgecolor='#344054',label='Chassis diagonal section'))
    ax.add_patch(Ellipse((.12,offset),.27,.03,facecolor='#36a9c6',alpha=.25,edgecolor='#16758b',label='Rounded collision envelope'))
    ax.add_patch(Rectangle((-.015,offset-.015),.015,.03,facecolor='#f3ac56',edgecolor='#b65c00',label='Full 30 mm-wide rim'))
    ax.plot([0,0],[-.05,.105],color='#777',linestyle=':',linewidth=1)
    if offset<.055:
        ax.add_patch(Rectangle((-.015,offset-.015),.015,.04-(offset-.015),facecolor='#c93737',alpha=.65,label='Physical interference'))
    else:
        ax.annotate('',xy=(-.008,offset-.015),xytext=(-.008,.04),arrowprops={'arrowstyle':'<->','color':'#333'})
        ax.text(-.006,.048,'20 mm',fontsize=9)
    ax.set_title(title,fontsize=11);ax.set_xlim(-.045,.035);ax.set_ylim(-.045,.105)
    ax.set_xlabel('Distance outward from chassis corner (m)');ax.grid(alpha=.2)
axes[0].set_ylabel('Height relative to chassis centre (m)')
handles,labels=axes[0].get_legend_handles_labels();fig.legend(handles,labels,loc='lower center',ncol=2,fontsize=8)
fig.suptitle('Flight pose: diagonal cut through the nearest rim segment',fontsize=13)
fig.subplots_adjust(bottom=.25,top=.82,wspace=.25)
Path('hardware-results').mkdir(exist_ok=True)
fig.savefig('hardware-results/clearance-comparison.png',dpi=180)
fig.savefig('hardware-results/clearance-comparison.svg')
