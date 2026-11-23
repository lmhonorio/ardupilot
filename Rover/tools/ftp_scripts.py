#!/usr/bin/env python3
"""
Envia os scripts Lua para o SD do rover pela HereLink (MAVFTP), conferindo cada
arquivo pelo CRC da propria placa.

  python ftp_scripts.py              # envia o que mudou em Rover/scripts
  python ftp_scripts.py --check      # so compara, nao envia
  python ftp_scripts.py --all        # envia tudo, mesmo o que ja esta igual
  python ftp_scripts.py rover-controlallocation.lua   # so estes arquivos

Feche o esc_monitor.py, o mission_editor.py e o mav_param.py antes (mesma porta
UDP 14550). O Mission Planner pode ficar aberto.

Por que nao pelo Mission Planner: a HereLink duplica e perde pacotes, e o MAVFtp
do MP falha no meio ("kErrFailErrno") ou grava o arquivo corrompido (BadCrc).
Aqui cada pedido e repetido ate a resposta certa chegar (a placa reenvia a
resposta de um pedido repetido sem refazer a operacao), respostas duplicadas
sao ignoradas, e o arquivo vai primeiro para um nome .tmp. So depois que o CRC
do .tmp na placa bate com o do PC ele troca de nome para o .lua; o .lua antigo
fica intacto se algo falhar. No fim o CRC do .lua final e conferido de novo.

O script que esta rodando continua na memoria: os novos so valem depois de
reiniciar a placa ou de "Restart Scripting" (MP > Config > MAVFtp / mensagens).
"""

import argparse
import glob
import os
import struct
import sys
import time
import zlib

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import esc_monitor as em  # noqa: E402  (MAVLink parser, link and encoder)

MSG_FILE_TRANSFER_PROTOCOL = 110
em.CRC_EXTRA[MSG_FILE_TRANSFER_PROTOCOL] = 84
em.PAYLOAD_LEN[MSG_FILE_TRANSFER_PROTOCOL] = 254

# MAVFTP opcodes and errors (libraries/GCS_MAVLink/GCS_FTP.cpp)
OP_TERMINATE, OP_RESET, OP_LIST, OP_OPEN_RO, OP_READ, OP_CREATE, OP_WRITE, OP_REMOVE, OP_MKDIR = 1, 2, 3, 4, 5, 6, 7, 8, 9
OP_RENAME, OP_CRC32 = 13, 14
OP_ACK, OP_NAK = 128, 129
ERR = {0: "nenhum", 1: "falha", 2: "erro do SD (errno)", 3: "tamanho invalido", 4: "sessao invalida",
       5: "sem sessao livre", 6: "fim do arquivo", 7: "comando desconhecido", 8: "ja existe",
       9: "protegido", 10: "nao encontrado"}
CHUNK = 239        # data bytes per FILE_TRANSFER_PROTOCOL packet
SESSION = 7

SCRIPTS_DIR = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "scripts"))


class FtpError(Exception):
    def __init__(self, code, errno_=None):
        self.code = code
        text = ERR.get(code, str(code))
        if errno_ is not None:
            text += " %d" % errno_
        super().__init__(text)


def board_crc32(data):
    """AP_Filesystem::crc32: crc_crc32 table CRC started at 0, no final xor."""
    return zlib.crc32(data, 0xFFFFFFFF) ^ 0xFFFFFFFF


class Ftp:
    def __init__(self, conn, kick, timeout, retries):
        self.link = em.Link(conn, kick)
        self.parser = em.MavlinkParser()
        self.enc = em.Encoder()
        self.sysid = None
        self.compid = 1
        self.seq = 0
        self.timeout = timeout
        self.retries = retries
        self.last_hb = 0.0
        self.resent = 0

    def _pump(self):
        now = time.monotonic()
        if now - self.last_hb >= 1.0:
            self.last_hb = now
            self.link.send(self.enc.heartbeat())
        out = []
        try:
            data = self.link.recv()
        except ConnectionResetError:
            data = b""   # Windows: ICMP "port unreachable" from an earlier send; keep going
        if not data:
            return out
        for msgid, sysid, compid, payload in self.parser.feed(data):
            if sysid == em.GCS_SYSID:
                continue
            if msgid == em.MSG_HEARTBEAT:
                hb = em.decode(msgid, payload)
                if hb["autopilot"] != em.MAV_AUTOPILOT_INVALID and hb["type"] != em.MAV_TYPE_GCS and self.sysid is None:
                    self.sysid, self.compid = sysid, compid
                continue
            if msgid == MSG_FILE_TRANSFER_PROTOCOL and sysid == self.sysid:
                out.append(payload)
        return out

    def wait_vehicle(self, timeout=15):
        end = time.monotonic() + timeout
        while self.sysid is None:
            if time.monotonic() > end:
                raise TimeoutError("nenhum rover respondeu em %ds (HereLink ligada? porta livre?)" % timeout)
            self._pump()
        print("Rover encontrado (sysid %d)" % self.sysid)

    def request(self, opcode, data=b"", offset=0, size=None, allow_nak=()):
        """Send one request and wait for its own reply; resend the SAME request
        (same seq) on timeout, so the board answers from its reply cache."""
        seq = self.seq
        self.seq = (self.seq + 1) & 0xFFFF
        size = len(data) if size is None else size
        body = struct.pack("<HBBBBBBI", seq, SESSION, opcode, size, 0, 0, 0, offset) + data
        frame = self.enc.pack(MSG_FILE_TRANSFER_PROTOCOL,
                              struct.pack("<BBB", 0, self.sysid, self.compid) + body.ljust(251, b"\0"))
        for attempt in range(self.retries):
            if attempt:
                self.resent += 1
            self.link.send(frame)
            end = time.monotonic() + self.timeout
            while time.monotonic() < end:
                for p in self._pump():
                    r = p[3:]
                    rseq, _sess, rop, rsize, rreq = struct.unpack("<HBBBB", r[:6])
                    roffset = struct.unpack("<I", r[8:12])[0]
                    if rseq != ((seq + 1) & 0xFFFF) or rreq != opcode:
                        continue   # duplicate or late reply to an older request
                    rdata = r[12:12 + rsize]
                    if rop == OP_NAK:
                        code = rdata[0] if rdata else 1
                        if code in allow_nak:
                            return None
                        raise FtpError(code, rdata[1] if code == 2 and len(rdata) > 1 else None)
                    return roffset, rdata
        raise TimeoutError("sem resposta da placa (opcode %d) depois de %d tentativas" % (opcode, self.retries))

    # ---- operations
    def reset(self):
        self.request(OP_RESET)

    def crc(self, path):
        r = self.request(OP_CRC32, path.encode(), allow_nak=(10, 2))
        return None if r is None else struct.unpack("<I", r[1][:4])[0]

    def remove(self, path):
        self.request(OP_REMOVE, path.encode(), allow_nak=(10, 2))

    def mkdir(self, path):
        self.request(OP_MKDIR, path.encode(), allow_nak=(8, 2))

    def rename(self, src, dst):
        self.request(OP_RENAME, src.encode() + b"\0" + dst.encode())

    def write_file(self, path, data, show):
        self.request(OP_CREATE, path.encode())
        try:
            for off in range(0, len(data), CHUNK):
                self.request(OP_WRITE, data[off:off + CHUNK], offset=off)
                show(min(off + CHUNK, len(data)))
        finally:
            self.request(OP_TERMINATE)


def local_files(names):
    if names:
        out = []
        for n in names:
            p = n if os.path.isabs(n) or os.path.exists(n) else os.path.join(SCRIPTS_DIR, n)
            if not os.path.isfile(p):
                raise FileNotFoundError(p)
            out.append(os.path.abspath(p))
        return out
    return sorted(glob.glob(os.path.join(SCRIPTS_DIR, "**", "*.lua"), recursive=True))


def upload_one(ftp, local, remote, tries):
    data = open(local, "rb").read().replace(b"\r\n", b"\n")
    want = board_crc32(data)
    tmp = remote + ".tmp"
    for attempt in range(1, tries + 1):
        t0 = time.monotonic()

        def show(done):
            sys.stdout.write("\r  enviando %s: %5.1f%% (%d/%d bytes)   " % (os.path.basename(local),
                                                                          100.0 * done / len(data), done, len(data)))
            sys.stdout.flush()
        try:
            ftp.reset()
            ftp.remove(tmp)
            ftp.write_file(tmp, data, show)
            sys.stdout.write("\n")
            got = ftp.crc(tmp)
            if got != want:
                print("  CRC do .tmp nao bate (placa %s, PC %08X); tentativa %d/%d" %
                      ("sem arquivo" if got is None else "%08X" % got, want, attempt, tries))
                continue
            ftp.remove(remote)
            time.sleep(1.0)   # a late duplicate RemoveFile must not hit the renamed file
            ftp.rename(tmp, remote)
            got = ftp.crc(remote)
            if got != want:
                print("  CRC final nao bate; tentativa %d/%d" % (attempt, tries))
                continue
            print("  OK %s (%d bytes, %.1fs, CRC %08X)" % (remote, len(data), time.monotonic() - t0, want))
            return True
        except (FtpError, TimeoutError) as exc:
            sys.stdout.write("\n")
            print("  falhou: %s; tentativa %d/%d" % (exc, attempt, tries))
            time.sleep(1.0)
    return False


def main():
    ap = argparse.ArgumentParser(description="Envia os .lua de Rover/scripts para o SD do rover (MAVFTP)")
    ap.add_argument("files", nargs="*", help="arquivos (padrao: todos os .lua de Rover/scripts e modules/)")
    ap.add_argument("--conn", default="udp:0.0.0.0:14550")
    ap.add_argument("--kick", default=em.HERELINK_KICK, help="porta MAVLink da HereLink ('none' desliga)")
    ap.add_argument("--dest", default="/APM/scripts", help="pasta na placa")
    ap.add_argument("--check", action="store_true", help="so compara PC x placa")
    ap.add_argument("--all", action="store_true", help="envia mesmo os iguais")
    ap.add_argument("--tries", type=int, default=4, help="tentativas por arquivo")
    ap.add_argument("--timeout", type=float, default=0.6, help="espera por resposta (s) antes de repetir")
    args = ap.parse_args()

    files = local_files(args.files)
    if not files:
        print("Nenhum .lua em", SCRIPTS_DIR)
        return 1
    ftp = Ftp(args.conn, args.kick, args.timeout, retries=25)
    ftp.wait_vehicle()
    ftp.reset()

    plan = []
    print("%-34s %10s  %s" % ("arquivo", "bytes", "situacao"))
    for local in files:
        rel = os.path.relpath(local, SCRIPTS_DIR).replace("\\", "/") if local.startswith(SCRIPTS_DIR) \
            else os.path.basename(local)
        remote = args.dest.rstrip("/") + "/" + rel
        data = open(local, "rb").read().replace(b"\r\n", b"\n")
        got = ftp.crc(remote)
        if got is None:
            state = "nao existe na placa"
        elif got == board_crc32(data):
            state = "igual"
        else:
            state = "diferente"
        print("%-34s %10d  %s" % (rel, len(data), state))
        if args.all or state != "igual":
            plan.append((local, remote))

    if args.check or not plan:
        if not plan:
            print("Tudo igual na placa; nada a enviar.")
        return 0

    dirs = sorted({os.path.dirname(r) for _l, r in plan} - {args.dest.rstrip("/")})
    for d in dirs:
        ftp.mkdir(d)
    failed = []
    for local, remote in plan:
        print("%s -> %s" % (os.path.relpath(local, SCRIPTS_DIR), remote))
        if not upload_one(ftp, local, remote, args.tries):
            failed.append(remote)
    print("Pedidos repetidos pela perda de pacotes: %d" % ftp.resent)
    if failed:
        print("FALHARAM (o arquivo antigo da placa continua la): " + ", ".join(failed))
        return 2
    print("Tudo enviado e conferido. Reinicie a placa ou o scripting para carregar.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
