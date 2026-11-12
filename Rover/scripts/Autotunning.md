# Autotuning do Rover (CAN + PWM) — guia completo

Guia único para instalar, configurar, sintonizar e operar em **MANUAL** e **AUTO** o rover 4 motores em modelo diferencial (par esquerdo / par direito com o mesmo comando).

Ordem obrigatória: **primeiro a malha de roda (CAN), depois os PIDs de cima**. Cada autotune bem-sucedido volta `SCR_USER1` para **0** sozinho.

---

## 0. Arquitetura (o que cada malha faz)

```text
Missão / RC / Circle
        ↓
Mixer Lua (throttle + steering → norm L / norm R)
        ↓
esc_can.lua  (ESC_MODE=0): referência de RPM por lado
        ├─ PI de velocidade do lado          → ESC_P / ESC_I / ESC_FF
        ├─ Correção local entre as 2 rodas   → ESC_LOC_P / ESC_LOC_I / ESC_FR*
        └─ Split de tração se uma patina     → corta a aérea, empurra a do solo
        ↓
Corrente CAN → 4× C620 / M3508
```

| Malha | Script | Período | Ganhos |
|---|---|---|---|
| Velocidade do lado + local | `esc_can.lua` | ~10 ms | `ESC_*` |
| Taxa de giro / velocidade | ArduRover | ~50 Hz | `ATC_STR_RAT_*`, `ATC_SPEED_*`, `CRUISE_*` |
| Alinhamento de yaw (pós-WP) | `rover-controlallocation.lua` | 200 ms | CAN: `CANT_YAW_P/I` · PWM: `SCR_USER2/3` · × `YAWT_GAIN_MULT` |

O rover é **diferencial 4WD**: as 2 rodas do mesmo lado devem ter o **mesmo RPM alvo**. `ESC_LMASK` define quem é esquerda; `ESC_REV` inverte sentido por motor.

---

## 1. Arquivos no cartão SD

Em `APM/scripts/`:

| Arquivo | Obrigatório |
|---|---|
| `esc_can.lua` | sim (caminho CAN) |
| `rover-controlallocation.lua` | sim (mixer + yaw + Circle relay) |
| `rover-quicktune.lua` | sim (QuickTune) |
| `modules/functions.lua` | sim |
| `modules/autotune.lua` | sim |
| `modules/pid.lua` | sim |
| `modules/circle-tune.lua` | sim (relé Circle) |

**Apague** qualquer `c620_can.lua` antigo. Os `.md` **não** precisam ir para o cartão.

Depois de copiar: `SCR_ENABLE=1`, reboot da Pixhawk.

---

## 2. Parâmetros de base (antes de qualquer tuning)

Confira **desarmado**. Reinicie depois de mudar CAN / heap / VM.

### 2.1 Barramento CAN

| Parâmetro | Valor | Nota |
|---|---|---|
| `CAN_D1_PROTOCOL` | **10** | Scripting. Se for 1 (DroneCAN), o Lua não vê o barramento |
| `CAN_P1_BITRATE` | 1000000 | |
| `CAN_P1_DRIVER` | 1 | ESCs no **CAN1** |
| `CAN_P2_DRIVER` | **0** | Não deixe P1 e P2 no mesmo driver |

ESCs no **CAN2**: `CAN_P1_DRIVER=0`, `CAN_P2_DRIVER=1`, `CAN_D1_PROTOCOL=10`.

### 2.2 Scripting e mixer

| Parâmetro | Valor | Nota |
|---|---|---|
| `SCR_ENABLE` | 1 | |
| `SCR_HEAP_SIZE` | **400000** (ou 500000) | reboot após mudar |
| `SCR_VM_I_COUNT` | **100000** | com 10000 o `esc_can.lua` estoura e some |
| `SCR_USER1` | **0** | seletor de tuning |
| `SCR_USER2` / `SCR_USER3` | (PWM yaw P/I) | em CAN não mexer à mão |
| `SCR_USER5` | **2** | mixer Rover Lua ativo |
| `THRO_MAX_LUA` | 90 | limite % do throttle no mixer |

### 2.3 Servos / motores

| Parâmetro | Valor típico | Nota |
|---|---|---|
| `SERVO1_FUNCTION` | 73 | ThrottleLeft |
| `SERVO2_FUNCTION` | 74 | ThrottleRight |
| `SERVO3_FUNCTION` | 74 | ThrottleRight |
| `SERVO4_FUNCTION` | 73 | ThrottleLeft |
| `MOT_SAFE_DISARM` | 0 | |

### 2.4 Driver ESC (CAN) — valores iniciais

| Parâmetro | Inicial | Função |
|---|---|---|
| `ESC_CAN` | **1** | 1=CAN, 0=PWM. **Só mude desarmado** |
| `ESC_MODE` | **0** | PI de velocidade (obrigatório para tuning e AUTO) |
| `ESC_RPM` | 400 | RPM da roda com comando 1.0 |
| `ESC_GEAR` | 19.2032 | redução M3508 P19 |
| `ESC_CUR` | 10 | limite de corrente (A) |
| `ESC_P` / `ESC_I` / `ESC_FF` | 0.03 / 0.3 / 0 | sobrescritos pelo tuning 4 |
| `ESC_D` | 0 | não usado |
| `ESC_IMAX` | 10 | limite do integral do lado |
| `ESC_REV` | ver §3 | bitmask de inversão |
| `ESC_LMASK` | ver §3 | bitmask do lado esquerdo |
| `ESC_OPT` | **5** | bit0=log DJIC, bit2=dessaturação |
| `ESC_BRK` | 1 | freio até `ESC_BRKCUR` |
| `ESC_BRKCUR` | 30 | % de `ESC_CUR` na frenagem |
| `ESC_ACC` | 800 | limite de aceleração da referência (rpm/s) |
| `ESC_LOC_P` | 0 | tuning 6 |
| `ESC_LOC_I` | 0.5 | tuning 6 (0.5 já ajuda em baixa velocidade) |
| `ESC_LOC_MAX` | 15 | % de `ESC_CUR`; tuning 5 ajusta |
| `ESC_SLIP` | 0 nos testes; depois **60** | aviso + limiar de divergência do split |
| `ESC_WT_RELAY` | 25 | amplitude do relé de roda (% de `ESC_CUR`) |
| `ESC_WT_RPM` | 15 | ponto de operação (% de `ESC_RPM`) |
| `ESC_FR1`…`ESC_FR4` | 0 | tuning 5 |

**Não edite / não grave:** `ESC_CMD_L`, `ESC_CMD_R`, `ESC_CMD_SEQ`, `ESC_STAT` (só RAM).

### 2.5 Hardware de referência (4 motores diferencial)

Configuração validada neste projeto:

| Parâmetro | Valor | Significado |
|---|---|---|
| `ESC_LMASK` | **9** | esquerda = ESC1 + ESC4 (bits 0+3); direita = ESC2 + ESC3 |
| `ESC_REV` | **6** | ESC2 e ESC3 invertidos (bits 1+2) |

Se o seu chassi for outro, ignore esses números e faça o passo 3 do zero.

### 2.6 Yaw / Circle / QuickTune

| Parâmetro | Inicial | Nota |
|---|---|---|
| `YAWT_MAX_ANG` | 45 | aborta yaw se erro > isso (°) |
| `YAWT_MAX_OUT` | 20 | amplitude do relé de yaw em **PWM** (%) |
| `YAWT_GAIN_MULT` | 1 | multiplica P/I de yaw em uso |
| `CANT_YAW_OUT` | **10** | amplitude do relé de yaw em **CAN** (%) |
| `CANT_YAW_P` / `CANT_YAW_I` | 0 | gravados pelo tuning 2 em CAN |
| `RTUN_ENABLE` | 1 | QuickTune ligado |
| `RTUN_AXES` | 3 | 1=direção, 2=velocidade, 3=ambos |
| `RTUN_AUTO_SAVE` | 5 | segundos para gravar ao terminar |
| `CIRC_RADIUS` | 4…20 | m; ajuste à área |
| `CIRC_SPEED` | ~½ da máxima | m/s; 0 = usa `WP_SPEED` |
| `CTUN_RELAY` | 5 | amplitude do relé Circle (pontos % de direção) |
| `CTUN_RAD_ERR` | 2 | m; desvio radial máximo no Circle relay |

---

## 3. Seletor `SCR_USER1`

| Valor | Autotune | Onde | Modo | Grava |
|---:|---|---|---|---|
| **0** | operação normal | — | — | — |
| **1** | QuickTune | chão | CIRCLE recomendado | `ATC_STR_RAT_*`, `ATC_SPEED_*`, `CRUISE_*` |
| **2** | relé de yaw | chão, área livre | HOLD recomendado | PWM: `SCR_USER2/3` · CAN: `CANT_YAW_P/I` |
| **3** | relé Circle | chão | **CIRCLE** | `ATC_STR_RAT_P/I` |
| **4** | relé de roda (lado) | chão, ~2 m livres | **HOLD** | `ESC_FF`, `ESC_P`, `ESC_I` |
| **5** | arranque por roda | **rodas no ar** | **HOLD** | `ESC_FR1..4`, `ESC_LOC_MAX` |
| **6** | relé diferencial (local) | ar ou chão | **HOLD** | `ESC_LOC_P`, `ESC_LOC_I` |

Com `ESC_CAN=0`, os valores **4 / 5 / 6** só avisam e não fazem nada.

**Abortar:** `SCR_USER1=0` ou desarmar. Nos 4/5/6, sair de HOLD ou mexer no stick também aborta.

> **Atenção:** `SCR_USER1=1` e `=2` iniciam assim que o rover está **armado**, em **qualquer** modo. Só coloque 1 ou 2 quando a área e o modo já estiverem prontos.

---

## 4. Conferir sentido e lados (obrigatório antes do tuning)

1. Rover nos cavaletes / rodas no ar.
2. `ESC_CAN=1`, `SCR_USER1=0`, armar em **MANUAL**.
3. GCS: mensagens `ESC: found ESC 1..4`. Se não → §2.1 e §12.
4. Stick **só para frente** (sem esterçar):

| Sintoma | Ação |
|---|---|
| As 4 rodas giram para frente | OK |
| As 4 giram para trás | `ESC_REV` = atual XOR **15** |
| Só um lado gira para trás | XOR **6** (ESC2+3) ou XOR **9** (ESC1+4) |
| Uma roda ao contrário | inverta só o bit (`1`, `2`, `4` ou `8`) |

5. Stick só para a **direita**:

| Sintoma | Ação |
|---|---|
| Direita mais lenta / ré, esquerda mais rápida | OK |
| O contrário | troque lados: `ESC_LMASK` 6 ↔ **9** |
| Acelerador gira no lugar e esterço anda reto | um lado ainda invertido → volte ao passo 4 |

6. Desarme. Anote `ESC_REV` e `ESC_LMASK` finais (neste hardware: **6** e **9**).

---

## 5. Ordem dos autotunes

A malha de roda muda a planta das de cima. Se retocar `ESC_P/I`, refaça QuickTune / Circle / Yaw.

```text
A. Roda (lado)          SCR_USER1 = 4   HOLD no chão     → ESC_P / ESC_I / ESC_FF
B. Arranque             SCR_USER1 = 5   HOLD no ar       → ESC_FR1..4 / ESC_LOC_MAX
C. Local (entre rodas)  SCR_USER1 = 6   HOLD ar ou chão  → ESC_LOC_P / ESC_LOC_I
D. QuickTune            SCR_USER1 = 1   CIRCLE           → ATC_* / CRUISE_*
E. Circle relay         SCR_USER1 = 3   CIRCLE           → ATC_STR_RAT_P/I
F. Yaw relay            SCR_USER1 = 2   HOLD (gira)      → CANT_YAW_P/I  (ou SCR_USER2/3)
```

Depois: `SCR_USER1=0`, `ESC_SLIP=60`, validar MANUAL → depois AUTO.

---

## 6. A — Relé de roda (`SCR_USER1=4`)

**Onde:** chão, ~2 m livres à frente e atrás.  
**Modo:** HOLD. Sticks no centro.  
**Pré:** passo 4 OK, `ESC_MODE=0`, `ESC_CAN=1`.

### Teoria

Relé Åström–Hägglund na malha de velocidade do **lado**. Estima `Ku`/`Pu` e grava PI + feed-forward (`ESC_FF`, `ESC_P`, `ESC_I`). O rover anda ~1–2 m oscilando a velocidade e volta em ré.

### Execução

1. Arme em HOLD, sticks centrados.
2. `SCR_USER1=4`.
3. Mensagens `WheelTune:`:
   - `started; … rpm forward then reverse`
   - `forward: Ku … Pu …` / `reverse: …`
   - `saved ESC_FF=… ESC_P=… ESC_I=…`
   - `SCR_USER1 set to 0`
4. Em MANUAL: acelerador leve → as quatro rodas respondem juntas.

**Falha:** `WheelTune: failed - …` (ganhos antigos ficam). Corrija stick/HOLD/ESC e repita.

---

## 7. B — Arranque por roda (`SCR_USER1=5`)

**Onde:** rover **levantado**, rodas no ar. **Nunca no chão.**  
**Modo:** HOLD. Sticks centrados.

### Teoria

Sobe a corrente em cada roda até ela girar (frente e ré). Grava a corrente de arranque `ESC_FR1..4` e um `ESC_LOC_MAX` suficiente para o pior desbalanceamento de atrito dentro do lado. Em operação, cada roda recebe a diferença de atrito em relação à média do lado.

### Execução

1. Arme em HOLD.
2. `SCR_USER1=5`.
3. Mensagens:
   - `breakaway started; wheels must be in the air`
   - `forward breakaway a b c d A` / `reverse …`
   - `saved ESC_FR … ESC_LOC_MAX=…`
4. Valores típicos `ESC_FR*` ~0,3–2 A. Uma roda bem acima das outras → mecânica (eixo, rolamento, freio).
5. Desarme.

---

## 8. C — Relé diferencial / local (`SCR_USER1=6`)

**Onde:** preferência ainda no ar; no chão também.  
**Modo:** HOLD.  
**Pré:** A feito; idealmente B feito. Duas rodas por lado (`ESC_LMASK` correto).

### Teoria

Relé na **diferença de RPM** entre as duas rodas do mesmo lado. Grava `ESC_LOC_P` e `ESC_LOC_I`, que redistribuem corrente dentro do par para equalizar velocidade.

### Execução

1. Arme em HOLD.
2. `SCR_USER1=6`.
3. Mensagens:
   - `local relay started; forward then reverse`
   - `local forward: Ku … Pu …` / `local reverse: …`
   - `saved ESC_LOC_P=… ESC_LOC_I=…`
4. MANUAL com pouquíssima velocidade: as quatro devem sair juntas.

### Split de tração (operação normal, sem ensaio)

Além do `ESC_LOC_*`, com `ESC_MODE=0` o driver faz:

1. Se as 2 rodas do lado **divergem** (limiar ≈ `ESC_SLIP`, ou 40 rpm se `ESC_SLIP=0`), o PI do lado passa a seguir a **roda atrasada** (solo), não a média.
2. Roda **rápida / no ar** → corrente de tração cortada (0; freio leve se passar da meta).
3. Roda **lenta / no solo** → fica com a corrente do lado; se ainda ~parada, empurra até `ESC_CUR`.

Isso evita o sintoma clássico do giro no eixo: aérea dispara, solo fica parado. Se ainda faltar torque no solo, suba `ESC_LOC_MAX` e/ou `ESC_CUR`.

---

## 9. D — QuickTune (`SCR_USER1=1`)

**Onde:** chão, área para Circle.  
**Modo recomendado:** CIRCLE.

### Teoria

Identificação feed-forward em campo: relaciona comando ↔ resposta (taxa de giro e velocidade) e deriva P/I pelas razões `RTUN_*_RATIO`.

Com `RTUN_AXES=3` ajusta:

- Direção: `ATC_STR_RAT_FF`, `ATC_STR_RAT_P`, `ATC_STR_RAT_I` (+ filtros se `RTUN_AUTO_FILTER>0`)
- Velocidade: `CRUISE_SPEED`, `CRUISE_THROTTLE`, `ATC_SPEED_P`, `ATC_SPEED_I`

Não calcula ganhos D.

### Preparação

1. Área plana e livre (ex.: raio 4 m + margem → ~16×16 m).
2. `CIRC_RADIUS` e `CIRC_SPEED` **antes** de entrar em CIRCLE (`CIRC_SPEED=0` → `WP_SPEED`).
3. `RTUN_ENABLE=1`, `RTUN_AXES=3`, `RTUN_AUTO_SAVE=5`.
4. GPS/AHRS OK, bateria OK, E-stop testado.

### Execução

1. Entre em CIRCLE, arme, então `SCR_USER1=1`.
2. Sticks neutros (movimento de stick pausa a coleta ~4 s).
3. Mensagens `RTun: …` até `Tuning DONE` / `tuning gains saved`.
4. `SCR_USER1` → 0. Desarme. Em CAN, o mixer copia `ATC_*` para `CANT_C_*` no desarme.

**Mínimos internos (aprox.):** direção ≥ 0,10 e yaw rate > 10°/s; throttle ≥ 0,20 e velocidade > `RTUN_SPEED_MIN` (0,5 m/s). Se avisar “low”, aumente `CIRC_SPEED` / raio.

### Chave RC (opcional, com `SCR_USER1=0`)

Ex.: `RC6_OPTION=300`, `RTUN_RC_FUNC=300`:

| Posição | Ação |
|---|---|
| Baixa | cancela e restaura ganhos pendentes |
| Meio | inicia / retoma |
| Alta | pausa e grava (útil com `RTUN_AUTO_SAVE=0`) |

### Parâmetros `RTUN_*`

| Parâmetro | Padrão | Função |
|---|---:|---|
| `RTUN_ENABLE` | 1 | habilita |
| `RTUN_AXES` | 3 | 1 dir, 2 vel, 3 ambos |
| `RTUN_STR_FFRATIO` | 0,9 | escala FF de taxa |
| `RTUN_STR_P_RATIO` | 0,5 | P a partir do FF; 0 preserva P |
| `RTUN_STR_I_RATIO` | 0,5 | I a partir do FF; 0 preserva I |
| `RTUN_SPD_FFRATIO` | 1,0 | escala cruzeiro/FF velocidade |
| `RTUN_SPD_P_RATIO` | 1,0 | P velocidade |
| `RTUN_SPD_I_RATIO` | 1,0 | I velocidade |
| `RTUN_AUTO_FILTER` | 1 | `FLTT/FLTD` = 0,5×`INS_GYRO_FILTER` |
| `RTUN_AUTO_SAVE` | 5 | s para gravar (QuickTune e Circle relay) |
| `RTUN_RC_FUNC` | 300 | função RC |
| `RTUN_SPEED_MIN` | 0,5 | m/s mínimo para amostras de velocidade |

---

## 10. E — Relé em Circle (`SCR_USER1=3`)

**Onde:** mesmo Circle. **Modo CIRCLE obrigatório.** Sticks neutros.

### Teoria

Relé em torno do comando médio de direção enquanto o Circle roda. Estima `Ku`/`Pu` da planta de taxa de giro e aplica Tyreus–Luyben em `ATC_STR_RAT_P/I` (refino após QuickTune).

### Execução

1. Arme em CIRCLE.
2. `SCR_USER1=3`.
3. `CircleTune: waiting for stable Circle` → depois `relay started`.
4. Sucesso: aplica P/I, espera `RTUN_AUTO_SAVE` s, grava, `SCR_USER1=0`.
5. Se saturar corrente de roda (`ESC_STAT` bit 1), cancela com aviso.

Estabilidade mínima aproximada: V ≥ 0,3 m/s, |yaw rate| ≥ 3°/s, desvio radial ≤ `CTUN_RAD_ERR`. Log `CRTU` no DataFlash.

Abortar: mudar seletor, sair de CIRCLE ou desarmar. Abort durante a espera de save restaura ganhos anteriores.

---

## 11. F — Relé de yaw (`SCR_USER1=2`)

**Onde:** chão, área livre para girar no lugar.  
**Modo recomendado:** HOLD. O rover **gira no lugar**.

### Teoria

Relé ±d no steering com histerese de 2° no erro de yaw. Descarta 2 ciclos, mede 4, Tyreus–Luyben:

```text
ε = 2°
Ku = 4·d / (π·√(A² − ε²))
Kp = Ku / 3,2
Ti = 2,2·Pu
Ki = Kp / Ti
```

Depois valida headings 0° / 90° / 180° / 270° / 360° e entra em HOLD.

| Caminho | Amplitude do relé | Ganhos gravados |
|---|---|---|
| CAN (`ESC_CAN=1`) | `CANT_YAW_OUT` (padrão 10%) | `CANT_YAW_P`, `CANT_YAW_I` |
| PWM (`ESC_CAN=0`) | `YAWT_MAX_OUT` (padrão 20%) | `SCR_USER2`, `SCR_USER3` |

Em ambos, o uso operacional multiplica por `YAWT_GAIN_MULT`.

### Execução

1. CAN: confira `CANT_YAW_OUT=10`. PWM: `YAWT_MAX_OUT=20`.
2. HOLD, arme, sticks centrados.
3. `SCR_USER1=2`.
4. Mensagens `YawTune:` (Ku, Pu, headings…).
5. Sucesso → `SCR_USER1=0`, HOLD.

Abortar: `SCR_USER1=0` ou desarmar. Se “oscillation too large”: baixe `CANT_YAW_OUT` (ex. 8) ou suba `YAWT_MAX_ANG`.

### Uso em missão (AUTO)

Após alcançar um waypoint com yaw no `z`, o mixer:

1. Lê o yaw desejado do WP.
2. Muda para **STEERING** e corre o PI de yaw (CAN ou PWM conforme `ESC_CAN`).
3. Critério de pronto: ~600 ms dentro de ±2° e |yaw rate| < 2°/s → **HOLD**.
4. Timeout por etapa: 15 s.

Mensagem GCS ~1 Hz: `YawAlign target=… current=… error=…`.  
Se o erro **aumentar** sob correção → polaridade / `ESC_REV` / `ESC_LMASK`, não só ganhos.

Esses ganhos **não** substituem `ATC_STR_RAT_*` da navegação normal.

---

## 12. Depois do tuning — operação normal e AUTO

| Parâmetro | Valor |
|---|---|
| `SCR_USER1` | **0** |
| `ESC_CAN` | **1** |
| `ESC_MODE` | **0** |
| `ESC_SLIP` | **60** |

Trocar CAN ↔ PWM: **só desarmado**. O mixer guarda/restaura `ATC_*` (`CANT_C_*` / `CANT_P_*`) e escolhe `CANT_YAW_*` ou `SCR_USER2/3`.

### Checklist pré-missão AUTO

```text
[ ] SCR_USER1 = 0; ESC_CAN = 1; ESC_MODE = 0
[ ] ESC_LMASK / ESC_REV conferidos (ex.: 9 / 6)
[ ] Malha de roda feita (4 → 5 → 6)
[ ] QuickTune + (opcional) Circle relay + yaw feitos
[ ] GPS 3D fix / AHRS saudável; bateria OK
[ ] Missão com WPs e yaw (z) onde o alinhamento for necessário
[ ] THRO_MAX_LUA / WP_SPEED / CIRC_* coerentes com a área
[ ] Teste curto em MANUAL: frente, ré, giro no eixo (4 rodas no solo)
[ ] E-stop / desarmar testados
```

### O que esperar em AUTO

| Fase | Comportamento |
|---|---|
| Entre waypoints | ArduRover manda throttle/steering → mixer → RPM L/R → PI de roda |
| Chegada com yaw | Lua alinha em STEERING com PI de yaw CAN |
| Patinagem 1 roda no ar | split de tração: corta aérea, empurra solo |
| Falha ESC / feedback | `esc_can` para motores e avisa no GCS |

---

## 13. Checklist rápido no campo

```text
[ ] c620_can.lua apagado; scripts novos; reboot
[ ] CAN_D1_PROTOCOL=10; só uma porta com DRIVER ≠ 0
[ ] SCR_HEAP_SIZE ≥ 400000; SCR_VM_I_COUNT=100000
[ ] SCR_USER5=2; SCR_USER1=0
[ ] ESC_CAN=1; ESC_MODE=0
[ ] Passo 4: sentido/lados OK (ESC_REV / ESC_LMASK)
[ ] A  SCR_USER1=4  HOLD no chão          → ESC_P/I/FF
[ ] B  SCR_USER1=5  HOLD no ar            → ESC_FR* / LOC_MAX
[ ] C  SCR_USER1=6  HOLD (ar ou chão)     → ESC_LOC_P/I
[ ] D  SCR_USER1=1  CIRCLE                → ATC_* / CRUISE
[ ] E  SCR_USER1=3  CIRCLE                → ATC_STR_RAT_P/I
[ ] F  SCR_USER1=2  HOLD (gira no lugar)  → CANT_YAW_P/I
[ ] SCR_USER1=0; ESC_SLIP=60
[ ] Validar MANUAL e depois missão AUTO curta
```

---

## 14. Problemas comuns

| Sintoma | O que olhar |
|---|---|
| `ESC 1 not found` / `ESC rx 0 0 0 0` | `CAN_D1_PROTOCOL=10`, porta CAN, bateria ESC, sem `c620_can.lua` |
| `SCR_USER1=4 needs ESC_CAN=1` | `ESC_CAN` está 0 |
| `center the sticks to start` | stick fora do trim |
| `not enough mem` | `SCR_HEAP_SIZE=500000` + reboot; tire scripts extras |
| Script some / não carrega | `SCR_VM_I_COUNT=100000` |
| Uma roda só anda / aérea gira e solo para | rode B+C; suba `ESC_LOC_MAX` e `ESC_CUR`; confira `ESC_SLIP` |
| Acelerador e esterço “trocados” | `ESC_REV` (§4) |
| Vira para o lado errado | `ESC_LMASK` (6 ↔ 9) |
| QuickTune “throttle/speed low” | `CIRC_SPEED` / raio / `ESC_RPM` |
| Yaw “oscillation too large” | baixe `CANT_YAW_OUT` ou suba `YAWT_MAX_ANG` |
| Circle relay cancela por saturação | baixe `CIRC_SPEED` / `CTUN_RELAY` ou suba `ESC_CUR` |
| YawAlign erro cresce | polaridade / LMASK / REV antes de mexer em P/I |
| Troca CAN↔PWM estranha nos ATC | desarme; deixe o mixer sincronizar `CANT_ACTIVE` |

---

## 15. Significado rápido dos parâmetros de yaw

| Parâmetro | Significado |
|---|---|
| `SCR_USER1` | 0 normal; 1 QuickTune; 2 yaw; 3 Circle; 4/5/6 roda CAN |
| `SCR_USER2/3` | P/I yaw no caminho **PWM** |
| `CANT_YAW_P/I` | P/I yaw no caminho **CAN** |
| `CANT_YAW_OUT` | amplitude do relé yaw em CAN (%) |
| `YAWT_MAX_ANG` | aborta se \|erro\| > isso |
| `YAWT_MAX_OUT` | amplitude do relé yaw em PWM (%) |
| `YAWT_GAIN_MULT` | multiplica P/I em alinhamentos (0,1–10000) |

---

## 16. Referências

- Manual operacional resumido: `Manual-Tuning-CAN.md`
- Arquitetura CAN / mixer: `Controle-CAN.md`
- [Rover QuickTune — ArduPilot](https://ardupilot.org/rover/docs/quiktune.html)
- [Rover Tuning Process](https://ardupilot.org/rover/docs/rover-tuning-process.html)
- [Tuning Turn Rate](https://ardupilot.org/rover/docs/rover-tuning-steering-rate.html)
