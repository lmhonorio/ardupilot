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
esc_can.lua  (ESC_MODE=0): a meta de RPM do lado vai para cada roda
        └─ PI próprio em cada roda, no encoder dela
              ESC_P1..4 / ESC_I1..4 / ESC_FF1..4
              (0 em qualquer um desses usa ESC_P / ESC_I / ESC_FF)
        ↓
Corrente CAN → 4× C620 / M3508
```

As duas rodas de um lado recebem a **mesma meta** (o comando daquele lado). O que é individual é o PI: erro, integral e feed-forward de cada encoder. Uma roda no ar chega na meta e pede pouca corrente; a do solo continua com erro e o integral sobe até `ESC_IMAX` / `ESC_CUR`.

| Malha | Script | Período | Ganhos |
|---|---|---|---|
| Velocidade de cada roda | `esc_can.lua` | ~10 ms | `ESC_Pn` / `ESC_In` / `ESC_FFn` |
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
| `ESC_P` / `ESC_I` / `ESC_FF` | 0.03 / 0.3 / 0 | fallback quando o ganho da roda está 0; a média do tuning também é gravada aqui |
| `ESC_P1..4` / `ESC_I1..4` / `ESC_FF1..4` | 0 | PI e feed-forward de cada roda. 0 = usa `ESC_P` / `ESC_I` / `ESC_FF`. Tuning 4 e 5 gravam. **Reboot** depois de atualizar o script, para a tabela criar esses parâmetros |
| `ESC_D` | 0 | termo D, compartilhado, em geral 0 |
| `ESC_IMAX` | 10 | limite do integral de cada roda |
| `ESC_REV` | ver §3 | bitmask de inversão |
| `ESC_LMASK` | ver §3 | bitmask do lado esquerdo |
| `ESC_OPT` | **5** | bit0=log DJIC, bit2=dessaturação |
| `ESC_BRK` | 1 | freio até `ESC_BRKCUR` |
| `ESC_BRKCUR` | 30 | % de `ESC_CUR` na frenagem |
| `ESC_ACC` | 800 | limite de aceleração da referência (rpm/s) |
| `ESC_LOC_P` / `ESC_LOC_I` / `ESC_LOC_MAX` | — | não entram mais no controle. O PI de cada roda substitui a correção local |
| `ESC_SLIP` | 0 nos testes; depois **60** | aviso se uma roda se afasta da média do lado (0 = desligado) |
| `ESC_WT_RELAY` | **100** | amplitude do relé, % de `ESC_CUR`. 100 usa o limite de corrente. Se a placa ainda tem 25 gravado, ponha 100 |
| `ESC_WT_RPM` | — | não escala mais o ensaio. Os patamares são **60 rpm** e **120 rpm** |
| `ESC_FR1`…`ESC_FR4` | 0 | não entram mais no controle |

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
| **4** | relé sincronizado, um PI para todas as rodas | chão, alguns metros livres | **HOLD** | `ESC_P1..4` e `ESC_I1..4` iguais, `ESC_FF1..4` por roda, média em `ESC_P/I/FF` |
| **5** | relé de uma roda por vez | **rodas no ar** | **HOLD** | os mesmos ganhos, medidos em cada motor |
| **6** | retirado | — | — | o PI de cada roda substitui o laço local |

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

A malha de roda muda a planta das de cima. Se retocar o PI das rodas, refaça QuickTune / Circle / Yaw.

```text
A. Roda no ar (opcional)   SCR_USER1 = 5   HOLD no ar       → PI de cada motor, sem o chassi
B. Roda no chão            SCR_USER1 = 4   HOLD no chão     → substitui A pelos ganhos com o rover carregado
D. QuickTune               SCR_USER1 = 1   CIRCLE           → ATC_* / CRUISE_*
E. Circle relay            SCR_USER1 = 3   CIRCLE           → ATC_STR_RAT_P/I
F. Yaw relay               SCR_USER1 = 2   HOLD (gira)      → CANT_YAW_P/I  (ou SCR_USER2/3)
```

`SCR_USER1=6` não sintoniza mais nada. O antigo laço local (`ESC_LOC_*`, `ESC_FR*`) saiu do controle.

O ensaio que vale para dirigir é o **B**. O **A** é seguro e mostra diferença entre motores; os ganhos dele são mais baixos (roda livre oscila mais) e ficam moles no chão. Se rodar os dois, faça **A e depois B**. Rodar o 5 depois do 4 apaga o ganho carregado.

Depois: `SCR_USER1=0`, `ESC_SLIP=60`, validar MANUAL → depois AUTO.

---

## 6. A — Relé de uma roda (`SCR_USER1=5`)

**Onde:** rover **levantado**, rodas no ar. **Nunca no chão.**  
**Modo:** HOLD. Sticks no centro.  
**Pré:** passo 4 OK, `ESC_MODE=0`, `ESC_CAN=1`. Opcional: pode ir direto ao ensaio B.

### Teoria

Relé de Åström–Hägglund **fechado em uma roda de cada vez**. As outras ficam em 0 A. No ar não há chassi no meio, então o que se mede é o motor e a redução.

A velocidade é inércia mais um atraso curto. A regra para essa planta é o sistema típico de tipo II (ótimo simétrico), com largura de meia frequência `h = 7` (margem de fase cerca de 46°):

```text
Ti = 7 · T
Kp = 8 / (14 · T · Kv)
Ki = Kp / Ti
```

`Kv` é a rampa de rpm por ampere, medida logo depois que a velocidade vira. `T` é o atraso entre o chaveamento da corrente e esse extremo. A amplitude entra pela fundamental de Fourier, não pelo pico. Descartam-se 3 ciclos e usam-se 6; a mediana tem de concordar dentro de 25%. A histerese é 3× o desvio do rpm no patamar alto, entre 2 rpm e 5% da meta.

Se a rampa não repetir, o ensaio cai no equivalente desse `h` no ponto de −180°: `Kp = 0.36·Ku`, `Ti = 1.75·Pu`. A mensagem diz `fallback`.

O feed-forward é a inclinação da corrente entre dois patamares (metade de `ESC_WT_RPM` e `ESC_WT_RPM`): `FF = (I₂ − I₁) / (rpm₂ − rpm₁)`. Cada motor guarda o seu. O yaw continua em Tyreus–Luyben; esta regra é só da roda.

Roda livre tem menos inércia, então o `Kv` sai maior e o PI fica mole no chão. Serve para comparar motores. O ensaio B substitui pelos ganhos da planta carregada.

### Execução

1. Arme em HOLD.
2. `SCR_USER1=5`.
3. Mensagens `WheelTune:`:
   - `in the air; one wheel at a time`
   - `ESC1 fwd type2 Kv … T …` / `ESC1 fwd P … I …`, ou `fallback Ku … Pu …` se a rampa não repetir
   - o mesmo em ré, depois ESC2, ESC3, ESC4
   - `P …` / `I …` / `FF …` (P e I daquele motor; FF da inclinação de corrente)
   - `SCR_USER1 set to 0`
4. Uma roda com `Ku` muito diferente → mecânica (eixo, rolamento) ou ESC.
5. Se abortar com `oscillation too large`, baixe `ESC_WT_RELAY`. Se abortar com `oscillation too small for hysteresis`, suba `ESC_WT_RELAY`.
6. Desarme.

---

## 7. B — Relé sincronizado no chão (`SCR_USER1=4`)

**Onde:** chão, alguns metros livres à frente e atrás. O ensaio segura dois patamares de velocidade e depois oscila, para a frente e para trás.  
**Modo:** HOLD. Sticks no centro.  
**Pré:** passo 4 OK, `ESC_MODE=0`, `ESC_CAN=1`.

### Teoria

A mesma regra de tipo II (`h = 7`) do ensaio A, agora com o rover em cima dos pneus. As quatro rodas recebem o **mesmo** acréscimo de corrente, chaveado pelo erro **médio** de rpm, para o veículo andar reto em vez de guinar. `Kv` e `T` são os da média: no chão as rodas estão presas ao mesmo chassi, e a diferença de amplitude entre elas é escorregamento, não quatro plantas. O PI gravado é **um só**, copiado para `ESC_P1..4` e `ESC_I1..4`.

O `FF` continua por roda, da inclinação de corrente entre 60 rpm e 120 rpm. O pulso do relé vai até `ESC_CUR` quando `ESC_WT_RELAY` está em 100. Se a mensagem sair `fallback`, a rampa não repetiu e o PI veio de `Kp = 0.36·Ku`, `Ti = 1.75·Pu` na fundamental.

Grava `ESC_P1..4`, `ESC_I1..4`, `ESC_FF1..4` e a média em `ESC_P` / `ESC_I` / `ESC_FF`. Enquanto o ganho de uma roda estiver 0, ela usa essa média.

### Execução

1. Arme em HOLD, sticks centrados.
2. `SCR_USER1=4`.
3. Mensagens `WheelTune:`:
   - `synced; 60/120 rpm fwd/rev relay …A` (os dois patamares e a corrente do relé, até `ESC_CUR`)
   - `fwd type2 Kv … T …` e `fwd P … I …` (ou `fallback Ku … Pu …`)
   - o mesmo em `rev`
   - `P …` / `I …` / `FF …` (P e I iguais nas quatro; FF de cada roda)
   - `SCR_USER1 set to 0`
4. O rover anda para a frente nos dois patamares, oscila a velocidade e volta em ré. São mais metros que um único patamar.
5. Em MANUAL: acelerador leve → as quatro rodas respondem juntas.

**Falha:** `WheelTune: failed - …` (ganhos antigos ficam). `oscillation too small for hysteresis` ou `inconsistent cycles`: suba `ESC_WT_RELAY` ou repita em piso mais liso. No chão o P das quatro rodas é o mesmo; diferença de P acima de 30% só aparece no ensaio no ar.

---

## 8. `SCR_USER1=6` está retirado

O PI de cada roda ocupa o lugar da correção local e do split de tração. Com `SCR_USER1=6` o script só avisa e não mexe em parâmetro.

Com `ESC_MODE=0`, em operação:

1. As duas rodas do lado têm a mesma meta de rpm (o comando daquele lado, com rampa `ESC_ACC` se ela for maior que 0).
2. Cada uma fecha o próprio PI no próprio rpm.
3. Roda no ar chega na meta e a corrente cai para perto de `FF·rpm`.
4. Roda no solo que não acompanha continua com erro; o integral sobe até `ESC_IMAX` e a saída até `ESC_CUR`.

`ESC_SLIP` só avisa. Não corta corrente.

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
[ ] Malha de roda feita (5 no ar, opcional, depois 4 no chão)
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
| Patinagem 1 roda no ar | a do ar fica na meta com pouca corrente; a do solo integra até `ESC_CUR` |
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
[ ] A  SCR_USER1=5  HOLD no ar           → ESC_Pn/In/FFn (opcional, antes do 4)
[ ] B  SCR_USER1=4  HOLD no chão         → substitui com a planta carregada
[ ] SCR_USER1=6 não é mais ensaio
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
| `reboot so ESC_P1..FF4 can be added` | o script novo pediu mais parâmetros; **reboot** da placa (reload do script não aumenta a tabela) |
| `SCR_USER1=4 needs ESC_CAN=1` | `ESC_CAN` está 0 |
| `center the sticks to start` | stick fora do trim |
| `not enough mem` | `SCR_HEAP_SIZE=500000` + reboot; tire scripts extras |
| Script some / não carrega | `SCR_VM_I_COUNT=100000` |
| Uma roda não acompanha / no ar gira e no solo para | confira `ESC_Pn/In/FFn` (0 usa `ESC_P/I/FF`); suba `ESC_IMAX` ou `ESC_CUR`; `ESC_SLIP` só avisa |
| `oscillation too small for hysteresis` | suba `ESC_WT_RELAY` para a oscilação passar da histerese |
| `inconsistent cycles` | piso irregular ou relé baixo demais; repita em chão mais liso ou suba `ESC_WT_RELAY` |
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
| `SCR_USER1` | 0 normal; 1 QuickTune; 2 yaw; 3 Circle; 4 relé no chão; 5 relé no ar; 6 retirado |
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
