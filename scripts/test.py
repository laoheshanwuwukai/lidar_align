
import matplotlib.pyplot as plt
import matplotlib.patches as patches

# Set up figure
fig, ax = plt.subplots(figsize=(6, 10))
ax.set_xlim(0, 10)
ax.set_ylim(0, 16)
ax.axis('off')

# Step positions and labels
steps = [
    ("加载配置参数（如降采样分辨率）、历史地图数据、历史关键帧和新采集关键帧", "初始化"),
    ("加载历史会话和新会话数据，将关键帧分类为近端和远端关键帧", "会话加载"),
    ("基于八叉树加载新会话点云数据，支持高效空间查询", "八叉树地图加载"),
    ("通过K近邻搜索更新局部短暂性，移除动态对象点云", "动态对象移除"),
    ("通过KD树搜索和八叉树射线投射，删除过时点，更新全局短暂性", "长期地图更新"),
    ("分析关键帧冗余性，删除冗余关键帧并优化姿态图", "姿态图更新"),
    ("保存更新后的点云数据和姿态图", "保存更新结果")
]

# Drawing boxes and arrows
for i, (desc, title) in enumerate(steps):
    y = 15 - i * 2
    # Draw rectangle
    box = patches.Rectangle((2, y), 6, 1, edgecolor='black', facecolor='white')
    ax.add_patch(box)
    # Add text
    ax.text(5, y + 0.5, title + "\n" + desc, ha='center',
            va='center', fontsize=8, family='SimSun')
    # Draw arrow to next box
    if i < len(steps) - 1:
        ax.annotate('', xy=(5, y), xytext=(5, y - 1),
                    arrowprops=dict(arrowstyle='->', lw=1, color='black'))

# Add title
ax.text(5, -1, "图1：地图更新方法的整体流程图", ha='center', fontsize=10, family='SimSun')

# Save the image
file_path = "/mnt/data/图1_地图更新方法的整体流程图.png"
plt.savefig(file_path, bbox_inches='tight', dpi=300)
file_path
