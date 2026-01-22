# reference/visualize_cpa.py
import argparse
import matplotlib.pyplot as plt
from matplotlib import cm
import os

def parse_cpa_file(path):
    with open(path, 'r', encoding='utf-8') as f:
        lines = [line.rstrip('\n') for line in f]

    # 读取二维数组直到首个空行
    grid = []
    i = 0
    while i < len(lines) and lines[i].strip() != "":
        row = [int(tok) for tok in lines[i].split()]
        grid.append(row)
        i += 1

    # 跳过空行
    while i < len(lines) and lines[i].strip() == "":
        i += 1

    # 后续为线网序列（每行一组），允许出现负数（将被忽略）
    sequences = []
    while i < len(lines):
        line = lines[i].strip()
        if line:
            seq = [int(tok) for tok in line.split()]
            sequences.append(seq)
        i += 1

    return grid, sequences

def build_id_index(grid):
    id_to_pos = {}  # 仅记录正数阀门的位置
    nonneg_cells = []  # 所有非负单元格坐标及值
    rows, cols = len(grid), len(grid[0]) if grid else 0
    for r in range(rows):
        for c in range(cols):
            val = grid[r][c]
            if val >= 0:
                nonneg_cells.append((r, c, val))
                if val > 0:
                    id_to_pos[val] = (r, c)
    return id_to_pos, nonneg_cells

def visualize(grid, sequences, output_path=None, dpi=180):
    rows = len(grid)
    cols = len(grid[0]) if grid else 0

    id_to_pos, nonneg_cells = build_id_index(grid)

    fig, ax = plt.subplots(figsize=(max(6, cols * 0.25), max(6, rows * 0.25)))

    # 画网格线
    for x in range(cols + 1):
        ax.axvline(x, color='#eeeeee', linewidth=0.8)
    for y in range(rows + 1):
        ax.axhline(y, color='#eeeeee', linewidth=0.8)

    # 坐标轴与比例
    ax.set_xlim(0, cols)
    ax.set_ylim(0, rows)
    ax.set_aspect('equal', adjustable='box')
    ax.invert_yaxis()  # 让 r=0 在顶部，更符合二维数组视觉

    # 标出非负点（方块）
    if nonneg_cells:
        xs = [c + 0.5 for (_, c, _) in nonneg_cells]
        ys = [r + 0.5 for (r, _, _) in nonneg_cells]
        ax.scatter(xs, ys, s=60, marker='s', facecolor='#d9d9d9', edgecolor='#666666', linewidth=0.5, label='非负单元')

        # 标注正数阀门编号（>0）
        for r, c, val in nonneg_cells:
            if val > 0:
                ax.text(c + 0.5, r + 0.5, str(val), ha='center', va='center', fontsize=7, color='#1a1a1a')

    # 为每组序列绘制正数阀门连线
    num_groups = len(sequences)
    cmap = cm.get_cmap('hsv', max(1, num_groups))  # 每组一个颜色

    for idx, seq in enumerate(sequences):
        positives = [v for v in seq if v > 0]
        negatives = [v for v in seq if v < 0]  # 提取负数
        coords = []
        for v in positives:
            if v in id_to_pos:
                r, c = id_to_pos[v]
                coords.append((c + 0.5, r + 0.5))  # 转为单元格中心坐标
        
        line_color = cmap(idx)
        
        if len(coords) >= 2:
            xs = [p[0] for p in coords]
            ys = [p[1] for p in coords]
            ax.plot(xs, ys, color=line_color, linewidth=2.0, marker='o', markersize=4, label=f'组 {idx+1}')
            
            # 在第一条线段旁边标注负数编号
            if negatives:
                # 计算第一条线段的中点
                seg_mid_x = (xs[0] + xs[1]) / 2
                seg_mid_y = (ys[0] + ys[1]) / 2
                neg_text = ', '.join(str(n) for n in negatives)
                ax.text(seg_mid_x + 0.5, seg_mid_y, neg_text, ha='left', va='center', 
                       fontsize=6, color=line_color, fontweight='bold',
                       bbox=dict(boxstyle='round,pad=0.3', facecolor='white', edgecolor=line_color, alpha=0.8))
        elif len(coords) == 1:
            # 单点也标示出来（不连线）
            ax.scatter(coords[0][0], coords[0][1], color=line_color, s=28, marker='o', label=f'组 {idx+1}')
            
            # 单点情况也标注负数
            if negatives:
                neg_text = ', '.join(str(n) for n in negatives)
                ax.text(coords[0][0] + 0.5, coords[0][1], neg_text, ha='left', va='center',
                       fontsize=6, color=line_color, fontweight='bold',
                       bbox=dict(boxstyle='round,pad=0.3', facecolor='white', edgecolor=line_color, alpha=0.8))

    ax.set_title('CPA 网格：非负点与分组彩色连线（仅连接序列中的正数阀门）', fontsize=10)
    # 图例移到外侧，避免遮挡
    if num_groups > 0:
        ax.legend(loc='center left', bbox_to_anchor=(1.02, 0.5), fontsize=8, frameon=False)

    plt.tight_layout()
    if output_path:
        plt.savefig(output_path, dpi=dpi)
        print(f'Saved visualization to: {output_path}')
    else:
        plt.show()

def main():
    # 获取脚本所在目录的父目录（项目根目录）
    script_dir = os.path.dirname(os.path.abspath(__file__))
    project_root = os.path.dirname(script_dir)
    
    # 默认路径
    default_input = os.path.join(project_root, 'benchmarks', 'CPA.txt')
    default_output = os.path.join(project_root, 'testCPA_visualization.png')
    
    parser = argparse.ArgumentParser(description='Visualize CPA grid and grouped valve connections.')
    parser.add_argument('input', nargs='?', default=default_input, help=f'Path to CPA input file (默认: {default_input})')
    parser.add_argument('--output', help=f'Output image path (默认: {default_output})', default=default_output)
    parser.add_argument('--dpi', type=int, default=180, help='输出图片 DPI')
    args = parser.parse_args()

    print(f'读取文件: {args.input}')
    print(f'输出路径: {args.output}')
    
    grid, sequences = parse_cpa_file(args.input)
    visualize(grid, sequences, output_path=args.output, dpi=args.dpi)

if __name__ == '__main__':
    main()