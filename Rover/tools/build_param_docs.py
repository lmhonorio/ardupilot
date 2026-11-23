#!/usr/bin/env python3
"""Gera Rover/tools/hmi/param_docs.json com descrição de cada parâmetro.

Fonte principal: apm.pdef.json do Rover (autotest.ardupilot.org).
Complementa com @Param dos scripts Lua e com uma lista local dos
parâmetros deste rover (ESC_, CANT_, CTUN_, legado SR*, etc.).
"""

from __future__ import print_function

import json
import re
import sys
import urllib.request
from pathlib import Path

ROOT = Path(__file__).resolve().parent
HMI = ROOT / "hmi"
SCRIPTS = ROOT.parent / "scripts"
OUT = HMI / "param_docs.json"
PDEF_URL = "https://autotest.ardupilot.org/Parameters/Rover/apm.pdef.json"

# Descrições em português para parâmetros deste rover / legado da placa.
CUSTOM = {
    "ARMING_CHECK": {"d": "Máscara dos testes de armamento. Bit a bit; 0 desliga todos.", "s": "1"},
    "CAM_RC_TYPE": {"d": "Tipo de controle de câmera pelo rádio.", "s": "1"},
    "CTUN_RAD_ERR": {
        "d": "Erro radial máximo, em metros, aceito no ensaio CircleTune antes de abortar.",
        "u": "m", "s": "0.1",
    },
    "CTUN_RELAY": {
        "d": "Amplitude do relé do CircleTune, em porcentagem do comando de direção.",
        "u": "%", "s": "1",
    },
    "EK3_MAX_FLOW": {"d": "Limite de fluxo óptico no EKF3.", "s": "0.1"},
    "GPS_MIN_DGPS": {"d": "Qualidade mínima de DGPS exigida.", "s": "1"},
    "MIXER_FREQ": {
        "d": "Frequência pedida do mixer Lua em AUTO, CIRCLE, ACRO, RTL e SIMPLE (25–200 Hz).",
        "u": "Hz", "s": "1",
    },
    "RST_SWITCH_CH": {"d": "Canal do rádio que reinicia a placa.", "s": "1"},
    "SYSID_ENFORCE": {"d": "1 só aceita comandos do SYSID_MYGCS.", "s": "1"},
    "SYSID_MYGCS": {"d": "System ID da estação de solo autorizada.", "s": "1"},
    "SYSID_THISMAV": {"d": "System ID desta placa na rede MAVLink.", "s": "1"},
    "TELEM_DELAY": {"d": "Atraso ao iniciar a telemetria depois do boot.", "u": "s", "s": "1"},
    "THRO_INV": {
        "d": "1 inverte o acelerador que vem dos modos do ArduPilot antes das rodas. O stick manual não muda.",
        "s": "1",
    },
    "THRO_MAX_LUA": {
        "d": "Teto do PWM dos motores em torno do trim 1500 µs, em porcentagem da faixa 1000–2000.",
        "u": "%", "s": "1",
    },
    "THRO_MIN_LUA": {
        "d": "PWM mínimo acima do neutro para começar alinhamento de yaw parado. 1500 desliga.",
        "u": "µs", "s": "1",
    },
    "TRQD_TYPE": {"d": "Tipo de torque/driver associado.", "s": "1"},
    "CANT_ACTIVE": {"d": "Banco de ganhos carregado: 0 PWM, 1 CAN.", "s": "1"},
    "CANT_C_VALID": {"d": "1 indica que o banco de ganhos do CAN já foi gravado.", "s": "1"},
    "CANT_C_CRUISE": {"d": "Cópia CAN de CRUISE_THROTTLE.", "u": "%", "s": "1"},
    "CANT_C_SPDI": {"d": "Cópia CAN de ATC_SPEED_I.", "s": "0.001"},
    "CANT_C_SPDP": {"d": "Cópia CAN de ATC_SPEED_P.", "s": "0.001"},
    "CANT_C_STRFF": {"d": "Cópia CAN de ATC_STR_RAT_FF.", "s": "0.001"},
    "CANT_C_STRI": {"d": "Cópia CAN de ATC_STR_RAT_I.", "s": "0.001"},
    "CANT_C_STRP": {"d": "Cópia CAN de ATC_STR_RAT_P.", "s": "0.001"},
    "CANT_P_CRUISE": {"d": "Cópia PWM de CRUISE_THROTTLE.", "u": "%", "s": "1"},
    "CANT_P_SPDI": {"d": "Cópia PWM de ATC_SPEED_I.", "s": "0.001"},
    "CANT_P_SPDP": {"d": "Cópia PWM de ATC_SPEED_P.", "s": "0.001"},
    "CANT_P_STRFF": {"d": "Cópia PWM de ATC_STR_RAT_FF.", "s": "0.001"},
    "CANT_P_STRI": {"d": "Cópia PWM de ATC_STR_RAT_I.", "s": "0.001"},
    "CANT_P_STRP": {"d": "Cópia PWM de ATC_STR_RAT_P.", "s": "0.001"},
    "ESC_CAN": {"d": "1 liga o controle dos motores pela CAN.", "s": "1"},
    "ESC_MODE": {"d": "0: cada roda fecha o próprio PI. Ensaios 4 e 5 exigem 0.", "s": "1"},
    "ESC_RPM": {"d": "Teto de rotação pedido às rodas.", "u": "rpm", "s": "1"},
    "ESC_GEAR": {"d": "Relação de redução motor→roda usada no cálculo de rpm.", "s": "0.001"},
    "ESC_CUR": {"d": "Teto de corrente de cada motor.", "u": "A", "s": "0.5"},
    "ESC_P": {"d": "Ganho P médio das rodas. Pn = 0 usa este valor.", "s": "0.001"},
    "ESC_I": {"d": "Ganho I médio das rodas.", "s": "0.001"},
    "ESC_D": {"d": "Ganho D médio das rodas.", "s": "0.001"},
    "ESC_FF": {"d": "Feedforward médio, em corrente por rpm.", "s": "0.0001"},
    "ESC_IMAX": {"d": "Teto do integral de cada roda.", "s": "0.5"},
    "ESC_REV": {"d": "Bits que invertem o sentido: 1, 2, 4 e 8 são as rodas 1 a 4.", "s": "1"},
    "ESC_OPT": {"d": "Opções do driver CAN dos motores.", "s": "1"},
    "ESC_BRK": {"d": "1 habilita freio/recuperação no driver.", "s": "1"},
    "ESC_ACC": {"d": "Rampa da rotação pedida. 0 não limita.", "u": "rpm/s", "s": "10"},
    "ESC_BRKCUR": {"d": "Corrente de freio pedida ao ESC.", "u": "A", "s": "1"},
    "ESC_LOC_P": {"d": "P do laço local auxiliar de rpm.", "s": "0.001"},
    "ESC_LOC_I": {"d": "I do laço local auxiliar de rpm.", "s": "0.001"},
    "ESC_LOC_MAX": {"d": "Teto do laço local auxiliar.", "s": "0.1"},
    "ESC_SLIP": {"d": "Limite de escorregamento entre lados.", "s": "1"},
    "ESC_LMASK": {"d": "Bits dos motores do lado esquerdo. 6 = 2+3; 9 = 1+4.", "s": "1"},
    "ESC_WT_RELAY": {"d": "Fração de ESC_CUR usada no pulso do ensaio das rodas.", "u": "%", "s": "1"},
    "ESC_WT_RPM": {"d": "Patamar alto de rotação do ensaio das rodas.", "u": "rpm", "s": "1"},
    "ESC_WT_BW": {"d": "Largura de banda alvo do WheelTune.", "u": "Hz", "s": "0.1"},
    "ESC_CMD_L": {"d": "Comando misturado do lado esquerdo (−1…1), escrito pelo mixer.", "s": "0.001"},
    "ESC_CMD_R": {"d": "Comando misturado do lado direito (−1…1), escrito pelo mixer.", "s": "0.001"},
    "ESC_CMD_SEQ": {"d": "Contador de sequência dos comandos ESC_CMD_*.", "s": "1"},
    "ESC_STAT": {"d": "Caminho ativo reportado pelo esc_can (PWM ou CAN).", "s": "1"},
    "ESC_FR1": {"d": "Fator de escala da roda 1.", "s": "0.001"},
    "ESC_FR2": {"d": "Fator de escala da roda 2.", "s": "0.001"},
    "ESC_FR3": {"d": "Fator de escala da roda 3.", "s": "0.001"},
    "ESC_FR4": {"d": "Fator de escala da roda 4.", "s": "0.001"},
    "ESC_P1": {"d": "P da roda 1, gravado pela calibração.", "s": "0.001"},
    "ESC_I1": {"d": "I da roda 1.", "s": "0.001"},
    "ESC_FF1": {"d": "Feedforward da roda 1.", "s": "0.0001"},
    "ESC_P2": {"d": "P da roda 2.", "s": "0.001"},
    "ESC_I2": {"d": "I da roda 2.", "s": "0.001"},
    "ESC_FF2": {"d": "Feedforward da roda 2.", "s": "0.0001"},
    "ESC_P3": {"d": "P da roda 3.", "s": "0.001"},
    "ESC_I3": {"d": "I da roda 3.", "s": "0.001"},
    "ESC_FF3": {"d": "Feedforward da roda 3.", "s": "0.0001"},
    "ESC_P4": {"d": "P da roda 4.", "s": "0.001"},
    "ESC_I4": {"d": "I da roda 4.", "s": "0.001"},
    "ESC_FF4": {"d": "Feedforward da roda 4.", "s": "0.0001"},
    "ESC_GCS_HZ": {"d": "Taxa com que corrente, rotação e temperatura chegam nos gráficos.", "u": "Hz", "s": "1"},
    "ESC_PREF": {"d": "Prefiltro da rotação pedida (fração do tempo integral).", "s": "0.01"},
    "ESC_STALL": {"d": "Boost de corrente em stall, em ampères por segundo.", "u": "A/s", "s": "1"},
    "ESC_RPM_AUX": {
        "d": "1 liga rampa, prefiltro, restrição de corrente oposta, stall boost e dessaturação. 0 desliga esses auxiliares.",
        "s": "1",
    },
    "YAWT_MAX_ANG_ERR": {"d": "Erro de proa que aborta o ensaio de yaw.", "u": "graus", "s": "1"},
    "YAWT_MAX_OUT": {"d": "Maior comando que o relé de yaw pode usar.", "u": "%", "s": "1"},
    "YAWT_GAIN_MULT": {"d": "Multiplicador do P e do I na hora de alinhar a proa.", "s": "0.1"},
    "YAWT_MAX_ANG": {"d": "Erro de proa em que o relé de yaw inverte o comando.", "u": "graus", "s": "1"},
    "YAWT_MIN_OUT": {"d": "Comando com que o relé de yaw começa.", "u": "%", "s": "1"},
    "YAWT_MAX_CYCLE": {"d": "Ciclos do relé, contando os dois primeiros descartados.", "s": "1"},
    "YAWT_TIMEOUT": {"d": "Tempo sem girar até a verificação de yaw parar.", "u": "s", "s": "1"},
}

SR_FIELDS = {
    "ADSB": "Taxa de mensagens ADS-B neste fluxo MAVLink.",
    "EXTRA1": "Taxa de EXTRA1 (atitude) neste fluxo.",
    "EXTRA2": "Taxa de EXTRA2 (VFR_HUD) neste fluxo.",
    "EXTRA3": "Taxa de EXTRA3 (AHRS, baterias…) neste fluxo.",
    "EXT_STAT": "Taxa de estado estendido neste fluxo.",
    "PARAMS": "Taxa de streaming de parâmetros neste fluxo.",
    "POSITION": "Taxa de posição neste fluxo.",
    "RAW_CTRL": "Taxa de RAW_CONTROLLER neste fluxo.",
    "RAW_SENS": "Taxa de sensores brutos neste fluxo.",
    "RC_CHAN": "Taxa de canais RC neste fluxo.",
}

PARAM_BLOCK = re.compile(
    r"//\s*@Param:\s*(\w+)\s*((?:(?:\r?\n)\s*//\s*@\w+:[^\r\n]*)+)",
    re.MULTILINE,
)
FIELD = re.compile(r"//\s*@(\w+):\s*([^\r\n]*)")


def fetch_pdef(cache: Path) -> dict:
    if cache.is_file():
        print("usando cache", cache)
        return json.loads(cache.read_text(encoding="utf-8"))
    print("baixando", PDEF_URL)
    with urllib.request.urlopen(PDEF_URL, timeout=60) as resp:
        raw = resp.read()
    cache.write_bytes(raw)
    return json.loads(raw.decode("utf-8"))


def compact_from_pdef(pdef: dict) -> dict:
    out = {}
    for group, params in pdef.items():
        if group in ("json",) or not isinstance(params, dict):
            continue
        for name, meta in params.items():
            if not isinstance(meta, dict) or name.startswith("SIM_"):
                continue
            desc = (meta.get("Description") or meta.get("DisplayName") or "").strip()
            if not desc:
                continue
            item = {"d": desc}
            units = (meta.get("Units") or "").strip()
            if units:
                item["u"] = units
            inc = (meta.get("Increment") or "").strip()
            if inc:
                item["s"] = inc
            values = meta.get("Values")
            if isinstance(values, dict) and values:
                bits = ", ".join("%s=%s" % (k, v) for k, v in list(values.items())[:12])
                if bits and bits not in item["d"]:
                    item["d"] = item["d"].rstrip(".") + ". " + bits + "."
            out[name] = item
    return out


def parse_lua_params() -> dict:
    out = {}
    if not SCRIPTS.is_dir():
        return out
    for path in SCRIPTS.rglob("*.lua"):
        text = path.read_text(encoding="utf-8", errors="replace")
        for match in PARAM_BLOCK.finditer(text):
            name = match.group(1)
            fields = {k: v.strip() for k, v in FIELD.findall(match.group(2))}
            desc = fields.get("Description") or fields.get("DisplayName") or ""
            if not desc:
                continue
            item = {"d": desc}
            if fields.get("Units"):
                item["u"] = fields["Units"]
            if fields.get("Increment"):
                item["s"] = fields["Increment"]
            out[name] = item
    return out


def add_stream_rates(docs: dict) -> None:
    for n in range(0, 5):
        for field, desc in SR_FIELDS.items():
            name = "SR%d_%s" % (n, field)
            docs.setdefault(name, {"d": desc + " (SR%d)." % n, "u": "Hz", "s": "1"})


def main() -> int:
    cache = HMI / "apm.pdef.rover.json"
    try:
        pdef = fetch_pdef(cache)
    except Exception as exc:
        print("falha ao obter pdef:", exc, file=sys.stderr)
        if not cache.is_file():
            return 1
        pdef = json.loads(cache.read_text(encoding="utf-8"))

    docs = compact_from_pdef(pdef)
    for name, item in parse_lua_params().items():
        docs[name] = item
    add_stream_rates(docs)
    for name, item in CUSTOM.items():
        docs[name] = item

    OUT.write_text(json.dumps(docs, ensure_ascii=False, separators=(",", ":")), encoding="utf-8")
    print("escreveu", OUT, "(%d parâmetros, %.1f KB)" % (len(docs), OUT.stat().st_size / 1024.0))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
