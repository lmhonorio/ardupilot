# Manual passo a passo — parâmetros e autotuning (CAN + PWM)

Guia operacional. Ordem obrigatória: primeiro a malha de roda, depois os PIDs de cima. Cada autotune volta `SCR_USER1` para 0 sozinho ao terminar com sucesso.

---

## 0. Arquivos no cartão

Em `APM/scripts/`:

| Arquivo | Obrigatório |
|---|---|
| `esc_can.lua` | sim (CAN) |
| `rover-controlallocation.lua` | sim |
| `rover-quicktune.lua` | sim (QuickTune) |
| `modules/functions.lua` | sim |
| `modules/autotune.lua` | sim |
| `modules/pid.lua` | sim |
| `modules/circle-tune.lua` | sim (Circle relay) |

**Apague** qualquer `c620_can.lua` antigo. Reinicie a placa depois de copiar.

---

## 1. Parâmetros de base (antes de qualquer tuning)

Confira **desarmado**. Reinicie depois de mudar CAN.

### Barramento CAN

| Parâmetro | Valor | Nota |
|---|---|---|
| `CAN_D1_PROTOCOL` | **10** | Scripting. Se for 1 (DroneCAN), o Lua não vê o barramento |
| `CAN_P1_BITRATE` | 1000000 | |
| `CAN_P1_DRIVER` | 1 | ESCs no conector **CAN1** |
| `CAN_P2_DRIVER` | **0** | Não deixe P1 e P2 no mesmo driver |
| `CAN_P2_BITRATE` | 1000000 | Só se os ESCs estiverem no CAN2: aí `P1_DRIVER=0`, `P2_DRIVER=1` |

Se os ESCs estão no **CAN2**: `CAN_P1_DRIVER=0`, `CAN_P2_DRIVER=1`, `CAN_D1_PROTOCOL=10`.

### Scripting e mixer

| Parâmetro | Valor | Nota |
|---|---|---|
| `SCR_ENABLE` | 1 | |
| `SCR_HEAP_SIZE` | **400000** (ou 500000) | Reinicie depois de mudar |
| `SCR_VM_I_COUNT` | **100000** | Obrigatório; com 10000 o `esc_can.lua` estoura e é removido |
| `SCR_USER1` | **0** | Seletor de tuning (sempre voltar a 0) |
| `SCR_USER2` | (ganhos PWM de yaw P) | Não mexer à mão no CAN |
| `SCR_USER3` | (ganhos PWM de yaw I) | Não mexer à mão no CAN |
| `SCR_USER4` | 4 | (uso do Vinicius; não é seletor) |
| `SCR_USER5` | **2** | Mixer Lua ativo |
| `SCR_USER6` | 0 | |

### Servos / motores

| Parâmetro | Valor típico | Nota |
|---|---|---|
| `SERVO1_FUNCTION` | 73 | ThrottleLeft (ESC1) |
| `SERVO2_FUNCTION` | 74 | ThrottleRight (ESC2) |
| `SERVO3_FUNCTION` | 74 | ThrottleRight (ESC3) |
| `SERVO4_FUNCTION` | 73 | ThrottleLeft (ESC4) |
| `MOT_SAFE_DISARM` | 0 | |

O mapeamento 73/74/74/73 é o do Vinicius. Se o seu hardware for outro, ajuste `ESC_LMASK` e `ESC_REV` no passo 3.

### Driver ESC (valores iniciais)

| Parâmetro | Valor inicial | O que é |
|---|---|---|
| `ESC_CAN` | **1** | 1 = CAN, 0 = PWM. **Só mude desarmado** |
| `ESC_MODE` | **0** | 0 = PI de velocidade (obrigatório para tuning) |
| `ESC_RPM` | 400 | RPM da roda com comando 1.0 |
| `ESC_GEAR` | 19.2032 | Redução M3508 P19 |
| `ESC_CUR` | 10 | Limite de corrente (A) |
| `ESC_P` | 0.03 | Será sobrescrito pelo tuning 4 |
| `ESC_I` | 0.3 | Será sobrescrito pelo tuning 4 |
| `ESC_D` | 0 | Não usado |
| `ESC_FF` | 0 | Será sobrescrito pelo tuning 4 |
| `ESC_IMAX` | 10 | Limite do integral do lado |
| `ESC_REV` | 0 | Ajuste no passo 3 |
| `ESC_LMASK` | **6** | ESCs da esquerda: bits (2+4=6 → ESC2 e ESC3) |
| `ESC_OPT` | **5** | bit0=log DJIC, bit2=dessaturação (1+4=5). Sem hold desarmado |
| `ESC_BRK` | 1 | Freia até `ESC_BRKCUR` |
| `ESC_BRKCUR` | 30 | % de `ESC_CUR` na frenagem |
| `ESC_ACC` | 800 | Limite de aceleração da referência (rpm/s) |
| `ESC_LOC_P` | 0 | Tuning 6 ajusta |
| `ESC_LOC_I` | 0.5 | Tuning 6 ajusta (0.5 já ajuda em baixa velocidade) |
| `ESC_LOC_MAX` | 15 | % de `ESC_CUR`; tuning 5 ajusta |
| `ESC_SLIP` | 0 | 0 = aviso de slip desligado nos testes; depois 60 |
| `ESC_WT_RELAY` | 25 | Amplitude do relé (% de `ESC_CUR`) |
| `ESC_WT_RPM` | 15 | Ponto de operação (% de `ESC_RPM`) |
| `ESC_FR1`…`ESC_FR4` | 0 | Tuning 5 mede |

**Não edite / não grave:** `ESC_CMD_L`, `ESC_CMD_R`, `ESC_CMD_SEQ`, `ESC_STAT` (só RAM).

### Yaw e Circle (parâmetros do ensaio)

| Parâmetro | Valor inicial | Nota |
|---|---|---|
| `YAWT_MAX_ANG` | 45 | Abortar se o erro de yaw passar disso (°) |
| `YAWT_MAX_OUT` | 20 | Amplitude do relé de yaw em **PWM** (%) |
| `YAWT_GAIN_MULT` | 1 | Multiplica P/I de yaw em uso |
| `CANT_YAW_OUT` | **10** | Amplitude do relé de yaw em **CAN** (%) |
| `CANT_YAW_P` / `CANT_YAW_I` | 0 | Tuning 2 (CAN) grava |
| `RTUN_ENABLE` | 1 | QuickTune ligado |
| `RTUN_AXES` | 3 | 1=só direção, 2=só velocidade, 3=ambos |
| `RTUN_AUTO_SAVE` | 5 | Segundos para gravar ao terminar |
| `CIRC_RADIUS` | 4…20 | m; ajuste à área |
| `CIRC_SPEED` | ~metade da máxima | m/s; 0 = usa `WP_SPEED` |
| `THRO_MAX_LUA` | 90 | Limite % do throttle no mixer |

---

## 2. Seletor `SCR_USER1`

| Valor | Autotune | Onde | Modo | Grava |
|---|---|---|---|---|
| **0** | normal | — | — | — |
| **1** | QuickTune | chão | CIRCLE (recomendado) | `ATC_STR_RAT_*`, `ATC_SPEED_*`, `CRUISE_*` |
| **2** | relé de yaw | chão, área livre | HOLD (recomendado) | PWM: `SCR_USER2/3`; CAN: `CANT_YAW_P/I` |
| **3** | relé Circle | chão | **CIRCLE** | `ATC_STR_RAT_P/I` |
| **4** | relé de roda | chão, ~2 m livres | **HOLD** | `ESC_FF`, `ESC_P`, `ESC_I` |
| **5** | arranque por roda | **rodas no ar** | **HOLD** | `ESC_FR1..4`, `ESC_LOC_MAX` |
| **6** | relé diferencial | ar ou chão | **HOLD** | `ESC_LOC_P`, `ESC_LOC_I` |

Com `ESC_CAN = 0`, os valores 4, 5 e 6 só avisam e não fazem nada.

**Abortar qualquer ensaio:** `SCR_USER1 = 0` ou desarmar. Nos 4/5/6, sair de HOLD ou mexer no stick também aborta.

---

## 3. Conferir sentido e lados (antes do tuning)

1. Rover nos cavaletes / rodas no ar.
2. `ESC_CAN = 1`, `SCR_USER1 = 0`, armar em **MANUAL**.
3. GCS: mensagens `ESC: found ESC 1..4`. Se não, confira a seção 1 (CAN).
4. Stick **só para frente** (sem esterçar):

| Sintoma | Ação |
|---|---|
| As 4 rodas giram para frente | OK |
| As 4 giram para trás | `ESC_REV` = valor atual XOR **15** |
| Só um lado gira para trás | Inverta só aquele lado: XOR **6** (ESC2+3) ou XOR **9** (ESC1+4) |
| Uma roda gira ao contrário | Inverta só o bit dela (`1`, `2`, `4` ou `8`) |

5. Stick só para a **direita**:

| Sintoma | Ação |
|---|---|
| Direita mais lenta / gira para trás, esquerda mais rápida | OK |
| O contrário (vira para a esquerda) | Troque os lados: `ESC_LMASK` de 6 → **9** (ou de 9 → 6) |
| Acelerador gira no lugar e esterço anda reto | Um lado ainda está invertido — volte ao passo 4 |

6. Desarme. Anote o `ESC_REV` e o `ESC_LMASK` finais.

---

## 4. Ordem dos autotunes

Faça nesta ordem. A malha de roda muda a planta das outras; se você retocar `ESC_P/I`, refaça 1 → 2 → 3 → 4 de cima.

```text
A. Roda (lado)          SCR_USER1 = 4     → ESC_P / ESC_I / ESC_FF
B. Arranque (no ar)     SCR_USER1 = 5     → ESC_FR1..4 / ESC_LOC_MAX
C. Local (entre rodas)  SCR_USER1 = 6     → ESC_LOC_P / ESC_LOC_I
D. QuickTune            SCR_USER1 = 1     → ATC_* / CRUISE_*
E. Circle relay         SCR_USER1 = 3     → ATC_STR_RAT_P/I (refino)
F. Yaw relay            SCR_USER1 = 2     → CANT_YAW_P/I  (ou SCR_USER2/3 em PWM)
```

---

### A. Relé de roda — `SCR_USER1 = 4`

**Onde:** chão, ~2 m livres à frente e atrás.  
**Modo:** HOLD. Sticks no centro.  
**Pré-requisito:** passo 3 OK, `ESC_MODE = 0`, `ESC_CAN = 1`.

1. Arme em HOLD, sticks centrados.
2. Coloque `SCR_USER1 = 4`.
3. Espere as mensagens `WheelTune:`:
   - `started; … rpm forward then reverse`
   - `forward: Ku … Pu …`
   - `reverse: Ku … Pu …`
   - `saved ESC_FF=… ESC_P=… ESC_I=…`
   - `SCR_USER1 set to 0`
4. O rover anda ~1–2 m para frente oscilando a velocidade e volta em ré.
5. Confira em MANUAL: acelerador leve → as quatro rodas respondem juntas.

**Se falhar:** mensagem `WheelTune: failed - …`; ganhos antigos ficam. Corrija (stick, HOLD, ESC perdido) e repita.

---

### B. Arranque por roda — `SCR_USER1 = 5`

**Onde:** rover **levantado**, rodas no ar. **Nunca no chão.**  
**Modo:** HOLD. Sticks no centro.

1. Arme em HOLD.
2. `SCR_USER1 = 5`.
3. Mensagens:
   - `breakaway started; wheels must be in the air`
   - `forward breakaway a b c d A`
   - `reverse breakaway …`
   - `saved ESC_FR … ESC_LOC_MAX=…`
4. Confira `ESC_FR1..4`: valores típicos ~0,3–2 A. Uma roda bem acima das outras → checar mecânica.
5. Desarme.

Com isso, em operação cada roda já recebe a diferença de atrito em relação à média do lado.

---

### C. Relé diferencial — `SCR_USER1 = 6`

**Onde:** preferência **ainda no ar** (resultado não depende do piso); no chão também funciona.  
**Modo:** HOLD. Sticks no centro.  
**Pré-requisito:** A feito; idealmente B feito. Duas rodas por lado (`ESC_LMASK` correto).

1. Arme em HOLD.
2. `SCR_USER1 = 6`.
3. Mensagens:
   - `local relay started; forward then reverse`
   - `local forward: Ku … Pu …`
   - `local reverse: …`
   - `saved ESC_LOC_P=… ESC_LOC_I=…`
4. Teste em MANUAL com pouquíssima velocidade: as quatro rodas devem sair juntas.

---

### D. QuickTune — `SCR_USER1 = 1`

**Onde:** chão, área para Circle.  
**Modo recomendado:** CIRCLE.  
**Atenção:** o código inicia o QuickTune ao armar com `SCR_USER1 = 1` em **qualquer** modo. Só coloque 1 quando já estiver pronto.

1. Configure `CIRC_RADIUS` e `CIRC_SPEED` **antes** de entrar em CIRCLE.
2. `RTUN_ENABLE = 1`, `RTUN_AXES = 3`, `RTUN_AUTO_SAVE = 5`.
3. Entre em CIRCLE, arme, então `SCR_USER1 = 1`.
4. Deixe o rover fazer o Circle. Mensagens `RTun: …`.
5. Ao terminar, espera `RTUN_AUTO_SAVE` s e grava. `SCR_USER1` volta a 0.
6. Desarme. Em CAN, o mixer copia os `ATC_*` para `CANT_C_*` no desarme.

---

### E. Relé em Circle — `SCR_USER1 = 3`

**Onde:** mesmo Circle, chão.  
**Modo:** **CIRCLE** (obrigatório). Sticks em neutro.

1. Arme em CIRCLE.
2. `SCR_USER1 = 3`.
3. O rover oscila a taxa de giro em torno do Circle.
4. Sucesso grava `ATC_STR_RAT_P/I` e volta `SCR_USER1` a 0 após `RTUN_AUTO_SAVE`.
5. Se saturar corrente de roda (`ESC_STAT` bit 1), o ensaio cancela com aviso de saturação.

---

### F. Relé de yaw — `SCR_USER1 = 2`

**Onde:** chão, área livre para girar no lugar.  
**Modo recomendado:** HOLD.  
**Atenção:** inicia ao armar com `SCR_USER1 = 2` em qualquer modo. O rover **gira no lugar**.

1. Com `ESC_CAN = 1`, confira `CANT_YAW_OUT = 10` (em PWM usa `YAWT_MAX_OUT = 20`).
2. HOLD, arme, sticks centrados.
3. `SCR_USER1 = 2`.
4. Mensagens `YawTune:` (Ku, Pu, depois sequência 0° / 90° / 180° / 270° / 360°).
5. Em CAN grava `CANT_YAW_P` e `CANT_YAW_I`. Em PWM grava `SCR_USER2` e `SCR_USER3`.
6. `SCR_USER1` volta a 0; rover em HOLD.

---

## 5. Depois de tudo — operação normal

| Parâmetro | Valor |
|---|---|
| `SCR_USER1` | **0** |
| `ESC_CAN` | 1 (CAN) ou 0 (PWM do Vinicius) |
| `ESC_SLIP` | 60 (ligar de novo o aviso de patinagem) |
| `ESC_MODE` | 0 |

Trocar CAN ↔ PWM: **só desarmado**. O mixer guarda/restaura os `ATC_*` e usa `CANT_YAW_*` ou `SCR_USER2/3` conforme o caminho.

---

## 6. Checklist rápido no campo

```text
[ ] c620_can.lua apagado; scripts novos no cartão; reboot
[ ] CAN_D1_PROTOCOL = 10; só uma porta com DRIVER ≠ 0
[ ] SCR_HEAP_SIZE ≥ 400000; SCR_USER5 = 2; SCR_USER1 = 0
[ ] ESC_CAN = 1; ESC_MODE = 0
[ ] Passo 3: sentido e lados OK (ESC_REV / ESC_LMASK)
[ ] A  SCR_USER1=4  HOLD no chão          → ESC_P/I/FF
[ ] B  SCR_USER1=5  HOLD no ar            → ESC_FR* / LOC_MAX
[ ] C  SCR_USER1=6  HOLD (ar ou chão)     → ESC_LOC_P/I
[ ] D  SCR_USER1=1  CIRCLE                → ATC_* / CRUISE
[ ] E  SCR_USER1=3  CIRCLE                → ATC_STR_RAT_P/I
[ ] F  SCR_USER1=2  HOLD (gira no lugar)  → CANT_YAW_P/I
[ ] SCR_USER1 = 0; ESC_SLIP = 60
```

---

## 7. Problemas comuns

| Sintoma | O que olhar |
|---|---|
| `ESC 1 not found` / `ESC rx 0 0 0 0` | `CAN_D1_PROTOCOL=10`, porta CAN certa, bateria dos ESCs religada, sem `c620_can.lua` |
| `SCR_USER1=4 needs ESC_CAN=1` | `ESC_CAN` está 0 |
| `center the sticks to start` | Stick fora do trim |
| Uma roda só anda em baixa velocidade | Rode B e C; ou suba `ESC_LOC_MAX` |
| Acelerador e esterço “trocados” | Um lado invertido → `ESC_REV` (passo 3) |
| Vira para o lado errado | `ESC_LMASK` |
| QuickTune “throttle/speed low” | Aumente `CIRC_SPEED` ou raio; confira `ESC_RPM` |
| Yaw aborta “oscillation too large” | Baixe `CANT_YAW_OUT` (ex.: 8) ou suba `YAWT_MAX_ANG` |
| `not enough mem` | `SCR_HEAP_SIZE` 500000 + reboot; tire scripts não usados |
