"""Plot wheel speed and current from the rover's CAN step logs."""

import argparse
from collections import defaultdict
from pathlib import Path
import struct

FORMATS = {
    b"CSR1": struct.Struct("<BHhhh"),
    b"CSR2": struct.Struct("<BIhhh"),
    b"CRP1": struct.Struct("<BHhhhh"),
    b"CRP2": struct.Struct("<BIhhhh"),
}


def read_log(path):
    data = path.read_bytes()
    header = data[:4]
    record = FORMATS.get(header)
    if record is None:
        raise ValueError(f"Cabeçalho desconhecido: {header!r}")
    payload = memoryview(data)[4:]
    if len(payload) % record.size:
        raise ValueError("Arquivo truncado: o último registro está incompleto")
    if not payload:
        raise ValueError("O arquivo contém apenas o cabeçalho, sem amostras")

    samples = defaultdict(list)
    for esc, time_ms, rpm10, cmd_ma, meas_ma, *target in record.iter_unpack(payload):
        samples[esc].append((time_ms, rpm10 / 10, cmd_ma / 1000, meas_ma / 1000,
                             target[0] / 10 if target else None))
    return samples


def plot_esc(plt, esc, samples, output):
    time_ms, rpm, commanded, measured, target = zip(*samples)
    time = [(sample - time_ms[0]) / 1000 for sample in time_ms]
    rpm_test = target[0] is not None
    reference = target if rpm_test else commanded
    changes = [t for t, before, after in zip(time[1:], reference, reference[1:]) if before != after]
    marker = "." if len(time) == 1 else None

    fig, (speed_ax, current_ax) = plt.subplots(2, 1, figsize=(8.4, 5.3), sharex=True,
                                               layout="constrained")
    speed_ax.set_title(f"ESC {esc} · Resposta ao degrau", loc="left", fontweight="bold")
    speed_ax.plot(time, rpm, color="#12678a", lw=1.8, marker=marker, label="Velocidade medida")
    speed_ax.set_ylabel("Velocidade (RPM)")
    if rpm_test:
        speed_ax.step(time, target, where="post", color="#ae4f19", marker=marker, label="Referência")
        speed_ax.legend(frameon=False)

    current_ax.plot(time, measured, color="#16826c", lw=1.35, marker=marker, label="Corrente medida")
    current_ax.step(time, commanded, where="post", color="#ae4f19", lw=2,
                    marker=marker, label="Corrente enviada")
    current_ax.set(xlabel="Tempo desde a primeira amostra do ESC (s)", ylabel="Corrente (A)")
    current_ax.legend(frameon=False)
    for ax in (speed_ax, current_ax):
        for transition in changes:
            ax.axvline(transition, color="#697786", ls="--", lw=1)
        ax.grid(alpha=0.23)
        ax.spines[["top", "right"]].set_visible(False)
    try:
        fig.savefig(output, dpi=150)
    finally:
        plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description="Gera um gráfico PNG por ESC em um log de degrau CAN.")
    parser.add_argument("log", type=Path, help="Arquivo CANSTEP.BIN ou CANRPM.BIN")
    parser.add_argument("-o", "--output-dir", type=Path, help="Pasta de saída (padrão: ao lado do log)")
    args = parser.parse_args()

    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        parser.exit(1, "Instale o matplotlib com: python -m pip install matplotlib\n")

    output_dir = args.output_dir or args.log.parent
    try:
        samples = read_log(args.log)
        output_dir.mkdir(parents=True, exist_ok=True)
        for esc, rows in sorted(samples.items()):
            output = output_dir / f"{args.log.name}-esc{esc}.png"
            plot_esc(plt, esc, rows, output)
            print(output)
    except (OSError, ValueError) as exc:
        parser.exit(1, f"Erro: {exc}\n")


if __name__ == "__main__":
    main()
