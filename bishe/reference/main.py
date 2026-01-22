import matplotlib.pyplot as plt
import numpy as np

# 读取 grid 数据
def read_grid_from_file(filename):
    with open(filename, 'r') as f:
        grid = [list(map(int, line.strip().split())) for line in f]
    return np.array(grid)

# 绘制网格图
def plot_grid(grid):
    fig, ax = plt.subplots(figsize=(12, 8))
    ax.set_aspect('equal')
    rows, cols = grid.shape

    cnt=0
    # 绘制每个单元格
    for i in range(rows):
        for j in range(cols):
            val = grid[i, j]

            # ✅ 特殊处理：如果是最后一行并且值为 0，显示为红色
            if i == rows - 1 and val == 0:
                #color = '#e74c3c'  # 红色
                color = '#2ecc71'  # 线段节点：绿色
                cnt+=1
            elif val == -1:
                color = '#eeeeee'  # 空白节点：浅灰色
            elif val == 0:
                color = '#2ecc71'  # 线段节点：绿色
            else:
                color = '#3498db'  # 阀门节点：蓝色

            rect = plt.Rectangle((j, rows - i - 1), 1, 1, facecolor=color, edgecolor='white', linewidth=0.5)
            ax.add_patch(rect)

            # 控制阀门编号
            if val > 0:
                ax.text(j + 0.5, rows - i - 1 + 0.5, str(val),
                        color='white', ha='center', va='center', fontsize=8, fontweight='bold')

    ax.set_xlim(0, cols)
    ax.set_ylim(0, rows)
    ax.set_xticks([])
    ax.set_yticks([])
    ax.set_title("Routing Grid Visualization", fontsize=14)
    plt.tight_layout()
    plt.show()

    # 输出统计结果
    print(f"底行值为0的格子数量: {cnt}")

# 主函数
if __name__ == "__main__":
    # 请替换为你实际的路径
    grid = read_grid_from_file("D:\\bishe\\bishe\\benchmarks\\CPA.txt")
    plot_grid(grid)
