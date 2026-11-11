#!/usr/bin/env python3.8
"""View the XYZ cloud already saved by read_sensors.py; no vehicle commands."""
import argparse
import json
from pathlib import Path
import time
from zipfile import BadZipFile

import numpy as np


def load_cloud(path):
    with np.load(path, allow_pickle=False) as scan:
        xyz = np.array(scan['xyz'], dtype=float)
        meta = json.loads(str(scan['metadata'].item()))
    if xyz.ndim != 2 or xyz.shape[1] != 3:
        raise ValueError('Esperava uma nuvem XYZ com formato N x 3')
    return xyz[np.isfinite(xyz).all(axis=1)], meta


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--file', type=Path,
                        default=Path(__file__).resolve().parents[2] / 'gazebo_simple/run/sensors/lidar.npz')
    parser.add_argument('--interval', type=int, default=1000,
                        help='Atualizacao da janela em milissegundos (padrao 1000)')
    parser.add_argument('--save', type=Path, help='Salvar uma imagem PNG e sair, sem janela')
    parser.add_argument('--z-scale', type=float, default=5,
                        help='Ampliacao visual da altura (padrao 5; use 1 para escala fisica)')
    args = parser.parse_args()
    if args.interval < 100:
        parser.error('--interval deve ser pelo menos 100 ms')
    if not np.isfinite(args.z_scale) or args.z_scale <= 0:
        parser.error('--z-scale deve ser positivo e finito')
    if args.save:
        import matplotlib
        matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation
    from matplotlib.ticker import MaxNLocator

    fig = plt.figure(figsize=(10, 7))
    fig.canvas.manager.set_window_title('LiDAR 3D - Rover')
    ax = fig.add_subplot(111, projection='3d')
    fig.subplots_adjust(left=0.02, right=0.82, bottom=0.08, top=0.85)
    ax.view_init(elev=25, azim=-65)
    last_signature = None
    last_update = time.monotonic()
    caption = ''

    def draw(xyz, meta):
        elevation, azimuth = ax.elev, ax.azim
        ax.clear()
        if len(xyz):
            ax.scatter(xyz[:, 0], xyz[:, 1], xyz[:, 2], c=xyz[:, 2],
                       cmap='viridis', s=3, vmin=-0.5, vmax=2)
            bounds = np.vstack((xyz.min(axis=0), xyz.max(axis=0)))
            bounds[0] = np.minimum(bounds[0], 0)
            bounds[1] = np.maximum(bounds[1], 0)
            span = np.maximum(bounds[1] - bounds[0], 1)
            center = bounds.mean(axis=0)
            for setter, mid, extent in zip((ax.set_xlim, ax.set_ylim, ax.set_zlim), center, span):
                setter(mid - 0.55 * extent, mid + 0.55 * extent)
            ax.set_box_aspect(span * [1, 1, args.z_scale])
        ax.scatter([0], [0], [0], c='red', marker='x', s=70, label='LiDAR')
        ax.set(xlabel='X: frente (m)', ylabel='Y: esquerda (m)', zlabel='Z: cima (m)')
        ax.zaxis.set_major_locator(MaxNLocator(3))
        ax.view_init(elev=elevation, azim=azimuth)
        ax.legend(loc='upper right')
        stamp = meta['sec'] + meta['nsec'] * 1e-9
        return ('LiDAR 3D | %d pontos | t_sim=%.3f s\n'
                'Cor = altura em metros | escala visual Z x%g') % (len(xyz), stamp, args.z_scale)

    def update(_):
        nonlocal last_signature, last_update, caption
        try:
            stat = args.file.stat()
            signature = (stat.st_mtime_ns, stat.st_size)
            if signature != last_signature:
                xyz, meta = load_cloud(args.file)
                caption = draw(xyz, meta)
                last_signature, last_update = signature, time.monotonic()
            age = time.monotonic() - last_update
            ax.set_title(caption + (' | sem nova varredura ha %.0f s' % age if age > 3 else ''))
        except (OSError, EOFError, ValueError, KeyError, BadZipFile):
            # The collector can be in the middle of replacing the compressed file.
            ax.set_title('Aguardando varredura completa: ' + args.file.name)

    if args.save:
        try:
            xyz, meta = load_cloud(args.file)
        except (OSError, EOFError, ValueError, KeyError, BadZipFile) as exc:
            parser.error('Nao foi possivel ler %s: %s' % (args.file, exc))
        ax.set_title(draw(xyz, meta))
        args.save.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(args.save, dpi=150)
        plt.close(fig)
        print('Imagem salva:', args.save)
        return
    update(0)
    animation = FuncAnimation(fig, update, interval=args.interval, cache_frame_data=False)
    print('Lendo:', args.file)
    print('Arraste com o mouse para girar a nuvem. Feche a janela para sair.')
    try:
        plt.show()
    except KeyboardInterrupt:
        plt.close(fig)
    # Keep the animation alive until the GUI event loop has ended.
    del animation


if __name__ == '__main__':
    main()
