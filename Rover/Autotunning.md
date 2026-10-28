# Autotuning do Rover (CAN)

Três camadas em cascata. Afinar **de dentro para fora**:

| Camada | Controlador | Feedback | Ferramenta | `SCR_USER6` | Modo |
|---|---|---|---|---|---|
| 1. Roda | `C620_P` / `C620_I` | encoder CAN (RPM) | relé Åström–Hägglund + Tyreus–Luyben | **3** | **HOLD** |
| 2. Chassi | `ATC_STR_RAT_*`, `ATC_SPEED_*`, cruise | RPM CAN (com fallback AHRS) | Rover QuickTune | **1** | **CIRCLE** |
| 3. Yaw no lugar | `steering_steady_pid` (`SCR_USER2/3`) | AHRS heading | relé Åström–Hägglund + Tyreus–Luyben | **2** | **HOLD** |

Faça **1 → 2 → 3**. A camada de yaw inclui o PID de roda no caminho; se a alocação ou o C620 mudar, refaça 2 e 3.

---

## 1. Seletor: `SCR_USER6`

| `SCR_USER6` | Ação | Inicia ao entrar em |
|---|---|---|
| **0** | Operação normal | — |
| **1** | QuickTune dos PIDs da Pixhawk | **CIRCLE** |
| **2** | Autotune do PI de yaw | **HOLD** |
| **3** | Autotune do PI de velocidade de roda (CAN) | **HOLD** |

- Ao **concluir com sucesso**, os ganhos são gravados e `SCR_USER6` volta a 0.
- **Abortar:** sair do modo (CIRCLE restaura ganhos do QuickTune; HOLD cancela yaw/roda sem gravar).
- Chaves RC: `RCx_OPTION = 300` (QuickTune) e `301` (yaw) continuam como alternativa.

> Com `SCR_USER6` em 2 ou 3, **qualquer** HOLD dispara o teste, inclusive o HOLD automático após alinhar um waypoint. Só deixe nesses valores durante a sessão de tuning.

---

## 2. Arquivos no cartão (`APM/scripts/`)

```
c620_can.lua
rover-controlallocation.lua
rover-quicktune.lua
system-start.lua
modules/autotune.lua
modules/functions.lua
modules/pid.lua
```

| Parâmetro | Valor | Motivo |
|---|---|---|
| `SCR_ENABLE` | 1 | Scripts Lua |
| `SCR_HEAP_SIZE` | ≥ 200000 | Mixer + C620 + QuickTune |
| `SCR_USER5` | 2 | Este mixer é o rover |
| `C620_CAN` | 1 | Driver CAN ligado |
| `RTUN_ENABLE` | 1 | QuickTune (após 1º boot) |
| `RTUN_AUTO_SAVE` | 5 | Salva 5 s após concluir |
| `WENC_RADIUS` | raio da roda (m) | Converte RPM → m/s (senão 0,10 m) |
| `WENC_POS_Y` / `WENC2_POS_Y` | meia-bitola (m) | Taxa de giro pelas rodas (senão 0,40 m) |
| `SCR_USER6` | 0 | Seletor |

---

## 3. Camada 1 — roda (CAN)

O rover oscila para frente e para trás em torno de 0 rpm. O relé aplica a mesma corrente nas 4 rodas (bypassa o PID de velocidade) e mede o RPM médio do CAN.

1. `C620_CAN = 1`, piso uniforme, alguns metros livres à frente e atrás.
2. `SCR_USER6 = 3`.
3. Arme parado e entre em **HOLD**.
4. Mensagens:
   ```
   WheelTune: started relay=…A
   WheelTune: Ku=… Pu=…s a=…rpm
   WheelTune: saved C620_P=… C620_I=…
   WheelTune: SCR_USER6 set to 0 (normal)
   ```
5. Relé = 25% de `C620_CUR` (entre 0,5 A e 5 A). `C620_MODE` é forçado para 0 (malha de velocidade).

**Fórmulas** (iguais ao yaw, unidades A e rpm):

```
Ku = 4·d / (π·√(a² − ε²))
Kp = Ku / 3,2
Ti = 2,2 · Pu   →  Ki = Kp / Ti
```

| Falha | Ação |
|---|---|
| `CAN is off` | `C620_CAN = 1`, desarmado |
| `missing ESC feedback` | conferir barramento / `CAN_Dx_PROTOCOL = 10` |
| `amplitude below hysteresis` | aumentar `C620_CUR` ou o relé |
| `oscillation too large` | reduzir `C620_CUR` |
| `inconsistent cycles` | piso uniforme, pneus no chão |
| `timeout` | o rover precisa realmente girar as rodas |

---

## 4. Camada 2 — chassi (QuickTune)

Igual ao applet oficial, mas com CAN:

- velocidade = RPM médio × 2πR/60
- taxa de giro = (v_esq − v_dir) / bitola

Se o CAN cair, usa AHRS/gyro. As mensagens de progresso mostram `(CAN)` ou `(AHRS)`/`(gyro)`.

1. Ajuste `CIRC_RADIUS` (~4 m) e `CIRC_SPEED`.
2. `SCR_USER6 = 1`.
3. Arme em MANUAL no centro da área, mude para **CIRCLE**.
4. Duração típica ~30 s. Sair de CIRCLE **restaura** os ganhos antigos.

Ajustados: `ATC_STR_RAT_{FF,P,I,FLTT,FLTD}`, `ATC_SPEED_{P,I}`, `CRUISE_SPEED`, `CRUISE_THROTTLE`.

---

## 5. Camada 3 — yaw (relé)

Gira no lugar em torno do heading atual. O comando passa pelo mixer CAN (`C620.set_norms`), então o PID de roda já precisa estar afinado.

1. `SCR_USER6 = 2`.
2. Arme parado, **HOLD**.
3. Oscilação típica 5–15°. Ganhos em `SCR_USER2` (P) e `SCR_USER3` (I).

Validação: missão com alinhamentos de 20° e 90°; deve entrar em HOLD a ±2° antes de 15 s.

---

## 6. Espaço

| Teste | Área |
|---|---|
| Roda | alguns metros à frente e atrás (oscila em translação) |
| QuickTune | ~2×`CIRC_RADIUS` + margem de 3–5 m |
| Yaw | ~4×4 m, giro no lugar, aborta se passar de ±45° |

---

## 7. Arquivos

| Arquivo | Papel |
|---|---|
| `scripts/c620_can.lua` | Driver CAN, RPM, override de corrente para o WheelTune |
| `scripts/rover-controlallocation.lua` | Mixer + seletor + yaw + WheelTune |
| `scripts/rover-quicktune.lua` | QuickTune com feedback CAN |
| `scripts/modules/autotune.lua` | Relé Åström–Hägglund + Tyreus–Luyben |
| `scripts/modules/pid.lua` | PI de alinhamento de yaw |
