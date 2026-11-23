"""Format the flight controller's SD card through the HereLink.

Usage (close Mission Planner and esc_monitor.py first: they use UDP 14550):
  python sd_format.py          # asks for confirmation
  python sd_format.py --yes    # no question

Steps:
  1. Waits for the vehicle heartbeat and stops if the rover is ARMED.
  2. Sends MAV_CMD_STORAGE_FORMAT (526), storage 1, format = 1.
  3. Prints the COMMAND_ACK and the "Formatting SDCard" / "Format: OK" messages.

Formatting erases APM/scripts, the logs and the terrain data. Parameters and
the mission stay in the controller. Copy the scripts back afterwards.
"""
import socket
import struct
import sys
import time

from mav_param import KICK, frames, pack

MAV_CMD_STORAGE_FORMAT = 526
MAV_TYPE_GCS = 6
MAV_MODE_FLAG_SAFETY_ARMED = 128
HEARTBEAT_TIMEOUT_S = 20
FORMAT_TIMEOUT_S = 60
ACK_TEXT = {
    0: "aceito",
    1: "temporariamente recusado",
    2: "negado",
    3: "nao suportado",
    4: "falhou",
    5: "em andamento",
}


def main(args):
    if "--yes" not in args:
        print("Isto APAGA o cartao SD do rover (scripts, logs, terreno).")
        if input("Digite FORMATAR para continuar: ").strip() != "FORMATAR":
            print("Cancelado.")
            return 1

    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.bind(("0.0.0.0", 14550))
    except OSError:
        print("Porta UDP 14550 ocupada: feche o Mission Planner e o esc_monitor.py.")
        return 1
    s.settimeout(0.2)

    hb = pack(0, struct.pack("<IBBBBB", 0, MAV_TYPE_GCS, 8, 0, 0, 3), 50)
    cmd = pack(76, struct.pack("<7fHBBB", 1, 1, 0, 0, 0, 0, 0,
                               MAV_CMD_STORAGE_FORMAT, 1, 1, 0), 152)

    peer = None
    armed = None
    ack = None
    done = False
    tries = 0
    seen = set()
    t0 = time.time()
    t_ack = None
    last = 0.0
    while True:
        now = time.time()
        if armed is None and now - t0 > HEARTBEAT_TIMEOUT_S:
            print("Sem heartbeat do rover. Verifique o AndroidAP e se o rover esta ligado.")
            return 2
        if now - t0 > FORMAT_TIMEOUT_S:
            break
        if done or (t_ack is not None and now - t_ack > 20):
            break
        if now - last > 1.0:
            last = now
            s.sendto(hb, KICK)
            if armed is False and ack is None:
                if tries >= 3:
                    print("O rover nao respondeu ao comando de formatar.")
                    return 3
                s.sendto(cmd, peer or KICK)
                tries += 1
                print("Comando de formatar enviado", flush=True)
        try:
            d, a = s.recvfrom(4096)
        except socket.timeout:
            continue
        peer = peer or a
        for mid, pl in frames(d):
            if mid == 0 and armed is None and len(pl) >= 7:
                if pl[4] == MAV_TYPE_GCS:
                    continue  # another ground station, not the rover
                armed = bool(pl[6] & MAV_MODE_FLAG_SAFETY_ARMED)
                if armed:
                    print("Rover ARMADO: desarme antes de formatar. Nada foi enviado.")
                    return 1
                print("Rover desarmado", flush=True)
            elif mid == 77:
                c, r = struct.unpack("<HB", (pl + bytes(3))[:3])
                if c == MAV_CMD_STORAGE_FORMAT and ack is None:
                    ack = r
                    t_ack = time.time()
                    print(f"Resposta ao comando: {r} ({ACK_TEXT.get(r, '?')})", flush=True)
                    if r not in (0, 5):
                        return 4
            elif mid == 253:
                txt = (pl + bytes(51))[1:51].split(b"\0")[0].decode(errors="replace")
                if "ormat" in txt and txt not in seen:
                    seen.add(txt)
                    print("Rover:", txt, flush=True)
                    if "OK" in txt:
                        done = True
                    elif "fail" in txt.lower():
                        return 4

    if done:
        print("Cartao formatado. Reinicie a controladora e copie os scripts para APM/scripts.")
        return 0
    if ack in (0, 5):
        print("Comando aceito, mas sem a mensagem 'Format: OK'. Confira as mensagens no GCS.")
        return 0
    print("Sem confirmacao do rover.")
    return 3


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
