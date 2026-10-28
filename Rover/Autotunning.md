# Autotuning do Rover

Este documento descreve os dois procedimentos de autotuning disponíveis no Rover:

| Controlador | Onde atua | Ferramenta |
|---|---|---|
| `ATC_STR_RAT_*` (taxa de giro) e `ATC_SPEED_*` (velocidade) — **PIDs da Pixhawk** | AUTO, GUIDED, CIRCLE, ACRO, RTL | Rover QuickTune (`scripts/rover-quicktune.lua`) |
| `steering_steady_pid` — **PI de alinhamento de yaw** (Lua) | Alinhamento no lugar ao chegar em waypoints (modo STEERING) | Autotune por relé (`scripts/modules/autotune.lua`) |

Faça **primeiro o tuning da Pixhawk** (é o que mais impacta a missão) e **depois o de yaw**.

---

## 1. Seletor de tuning: `SCR_USER6`

O tuning é escolhido por parâmetro, sem precisar de chave no rádio:

| `SCR_USER6` | Ação | Inicia ao entrar no modo |
|---|---|---|
| **0** | Operação normal | — |
| **1** | Autotuning dos PIDs da Pixhawk (QuickTune) | **CIRCLE** |
| **2** | Autotuning do PI de yaw (relé) | **HOLD** |

- O valor fica salvo na Pixhawk (persiste após reiniciar).
- Ao **concluir com sucesso**, o script grava os ganhos e coloca `SCR_USER6 = 0` automaticamente.
- **Para abortar**, basta sair do modo:
  - Sair de CIRCLE durante o QuickTune → os ganhos antigos são **restaurados**.
  - Sair de HOLD durante o tuning de yaw → cancela **sem gravar**.
  - Em ambos os casos, `SCR_USER6` mantém o valor para tentar de novo.
- As chaves RC continuam funcionando como alternativa (`RCx_OPTION = 300` para o QuickTune e `301` para o yaw).

> ⚠️ Com `SCR_USER6 = 2`, **qualquer** entrada em HOLD dispara o tuning, inclusive o HOLD automático após um alinhamento de waypoint. Só deixe em 2 durante a sessão de tuning.

---

## 2. Configuração inicial (uma vez)

Copie para o cartão SD da Pixhawk, em `APM/scripts/`:

```
rover-controlallocation.lua
rover-quicktune.lua
system-start.lua
modules/autotune.lua
modules/functions.lua
modules/pid.lua
```

Os arquivos `.bak`, `.orig` e `.md` **não** precisam ir para o cartão. A pasta `tests/` também não.

Parâmetros (Mission Planner → **CONFIG → Full Parameter List**, buscar pelo nome, alterar e clicar em **Write Params**):

| Parâmetro | Valor | Motivo |
|---|---|---|
| `SCR_ENABLE` | 1 | Habilita scripts Lua |
| `SCR_HEAP_SIZE` | ≥ 150000 | Três scripts rodando juntos |
| `RTUN_ENABLE` | 1 | Habilita o QuickTune (criado pelo script após o 1º boot) |
| `RTUN_AUTO_SAVE` | 5 | Salva os ganhos 5 s após concluir |
| `SCR_USER2` | 0 | P do yaw (0 = usar padrão até tunar) |
| `SCR_USER3` | 0 | I do yaw (0 = usar padrão até tunar) |
| `SCR_USER6` | 0 | Seletor de tuning (ver seção 1) |

Reinicie a Pixhawk após copiar os scripts.

---

## 3. Parâmetros do modo CIRCLE

Ficam em **CONFIG → Full Parameter List**, buscando por `CIRC`:

| Parâmetro | Atual | Recomendado para o tuning |
|---|---|---|
| `CIRC_RADIUS` | 20 m | **4 m** (ou ~2× o raio mínimo de curva do rover) |
| `CIRC_SPEED` | 0 (usa `CRUISE_SPEED`) | Velocidade da missão (ex.: **1.0 m/s**) |
| `CIRC_DIR` | 0 (horário) | 0 ou 1 |

> Com o padrão de 20 m, o círculo tem 40 m de diâmetro e você precisaria de uma área de ~50 × 50 m.
> Esses valores só passam a valer quando o rover **entra** em CIRCLE, então ajuste antes de trocar de modo.

---

## 4. Espaço necessário

### QuickTune (CIRCLE)

```
Lado da área ≈ 2 × CIRC_RADIUS + 2 × margem
```

- Margem de **3–5 m** de cada lado, para cobrir o erro do GPS (1–3 m sem RTK) e as oscilações do tuning.
- Exemplo: raio de 4 m e margem de 4 m → **~16 × 16 m** livres.
- Piso plano e do mesmo tipo da operação real.

### Yaw (HOLD)

O rover gira no lugar, oscilando no máximo ±45° (o script aborta se passar disso).

```
Raio livre ≈ meia diagonal do rover + 1 m
```

- Exemplo: rover de 1,0 × 0,7 m → ~**4 × 4 m** livres.
- Mais importante que o tamanho: **piso uniforme**, sem mistura de grama com concreto e sem desnível. Atrito variável causa o erro "inconsistent cycles".

Para fazer os dois na mesma sessão, use a área do QuickTune e faça o tuning de yaw no centro dela.

---

## 5. Antes de ir a campo

1. **Ajuste a base do controle de velocidade:** andando reto em MANUAL, anote throttle × velocidade e defina `CRUISE_SPEED` / `CRUISE_THROTTLE`. Confira também `ATC_STR_RAT_MAX` e `ATC_ACCEL_MAX` ([processo de tuning do Rover](https://ardupilot.org/rover/docs/rover-tuning-process.html)).
2. **Teste no SITL** com `simrover.sh` / `gzrover.sh`, usando os mesmos parâmetros.
3. Bateria **carregada**: a resposta dos motores muda com a tensão.

---

## 6. Teste 1 — PIDs da Pixhawk (QuickTune)

1. Defina `SCR_USER6 = 1`.
2. Ajuste `CIRC_RADIUS` e `CIRC_SPEED` (seção 3).
3. Arme em **MANUAL**, com o rover no centro da área.
4. Mude para **CIRCLE**. O tuning começa sozinho. Mensagens esperadas no GCS:
   ```
   RTun: starting ATC_STR_RAT tune
   RTun: ATC_STR_RAT_FF xx% complete
   RTun: ATC_STR_RAT_FF tuning done
   RTun: starting ATC_SPEED tune
   RTun: ATC_SPEED_FF tuning done
   RTun: Tuning DONE
   RTun: tuning gains saved
   RTun: done, SCR_USER6 set to 0 (normal)
   ```
   Duração típica: ~30 s.
5. **Não mexa nos sticks**: qualquer entrada do piloto pausa o tuning.
6. Se algo ficar perigoso, **mude para MANUAL ou HOLD**: os ganhos antigos são restaurados.

**O que é ajustado:** o QuickTune mede o feedforward (quanto de comando gera quanto de giro e de velocidade) e calcula P e I como frações do FF (`RTUN_STR_P_RATIO`, `RTUN_STR_I_RATIO`, `RTUN_SPD_P_RATIO`, `RTUN_SPD_I_RATIO`). Também ajusta `ATC_STR_RAT_FLTT`/`FLTD`, `CRUISE_SPEED` e `CRUISE_THROTTLE`. Os ganhos saem conservadores.

**Validação:** rode uma missão AUTO e compare nos logs `STER.DesTurnRate × STER.TurnRate` e `THR.DesSpeed × THR.Speed`.

**Mensagens de problema:**

| Mensagem | Ação |
|---|---|
| `must be armed and moving to tune` | Rover parado: confira armamento e `CIRC_SPEED`/`CRUISE_SPEED` |
| `increase steering` | Diminua `CIRC_RADIUS` |
| `increase turn rate` | Diminua `CIRC_RADIUS` ou aumente `CIRC_SPEED` |

> **Atenção:** o `rover-controlallocation.lua` substitui o mixer do ArduPilot e é não linear (o ganho de giro cai quando há throttle). Os ganhos do QuickTune só valem **com esse script** e na velocidade usada no tuning. Se a alocação mudar, refaça o tuning.

---

## 7. Teste 2 — PI de alinhamento de yaw

1. Defina `SCR_USER6 = 2`.
2. Arme com o rover **parado** (em MANUAL, por exemplo).
3. Mude para **HOLD**. O rover passa a oscilar em torno do yaw atual: gira para um lado, depois para o outro, com amplitude típica de 5–15°. Mensagens:
   ```
   YawTune: started
   YawTune: Ku=… Pu=…s a=…deg
   YawTune: saved P=… I=…
   YawTune: SCR_USER6 set to 0 (normal)
   ```
   Duração típica: 10–30 s.
4. Para abortar, saia de HOLD.

**Como funciona:** um relé com histerese aplica steering ±0,45 e gera uma oscilação controlada. São descartados 2 ciclos e medidos 4. Com o período `Pu` e a amplitude `a`:

```
Ku = 4·d / (π·√(a² − ε²))       d = 0,45 (relé), ε = 2° (histerese)
Kp = Ku / 3,2                   (Tyreus-Luyben)
Ti = 2,2 · Pu      →  Ki = Kp / Ti
```

Os ganhos vão para `SCR_USER2` (P) e `SCR_USER3` (I) e passam a valer na hora e a cada boot.

**Validação:** rode uma missão com waypoints de yaw (`MissaoGiro.plan`), com alinhamentos de 20° e de 90°. O rover deve chegar em ±2° e entrar em HOLD antes do timeout de 15 s.

**Mensagens de falha:**

| Mensagem | Causa | Ação |
|---|---|---|
| `amplitude below hysteresis` / rover não gira | Relé abaixo da zona morta dos motores | Aumentar `relay` em `rover-controlallocation.lua` (`Autotune:new({ relay = 0.45 })`) |
| `oscillation too large` | Oscilação acima de 45° | Reduzir `relay` (ex.: 0.3) |
| `inconsistent cycles` | Piso irregular ou patinando | Repetir em piso uniforme |
| `timeout` | Não completou em 60 s | Verificar se o rover realmente gira |
| `Pu out of range` | Período fora de 0,4–20 s | Verificar AHRS/bússola |

---

## 8. Resultados da simulação

Validado com Lua 5.4.6 (a mesma versão do ArduPilot), rodando os scripts reais sobre um simulador das APIs:

| Planta | Ganhos (P / I) | Alinhamento 20° | Alinhamento 90° |
|---|---|---|---|
| Nominal | 2,43 / 0,94 | 2,0 s, overshoot 11,6° | 2,6 s, overshoot 16,3° |
| Motor fraco | 3,45 / 1,01 | 1,2 s, overshoot 1,1° | 2,6 s, sem overshoot |
| Mais inércia | 1,98 / 0,50 | 1,2 s, overshoot 0,8° | 5,0 s, overshoot 24,6° |

O alinhamento só conclui após permanecer por 600 ms dentro de ±2° do rumo e abaixo de 2°/s de velocidade angular. O controlador do modo STEERING ainda atualiza a cada 200 ms; valide o tempo de acomodação e o overshoot no SITL e no veículo.

Durante o QuickTune em CIRCLE (`SCR_USER6 = 1`), o alocador atualiza o PWM a 40 Hz para acompanhar a frequência de medição do QuickTune. Em outros modos, a frequência normal do alocador permanece inalterada.

### Rodando os testes

Na pasta `tests/` (requer Lua 5.4):

```
lua test.lua                # autotune + alinhamento + modos CIRCLE/desconhecido
K=1 TAU=0.6 lua test.lua    # outra planta (K = ganho de giro, TAU = inércia)
lua test_sel.lua            # seletor SCR_USER6 = 2
lua test_qt.lua             # seletor SCR_USER6 = 1 com QuickTune
```

---

## 9. Arquivos

| Arquivo | Descrição |
|---|---|
| `scripts/rover-controlallocation.lua` | Alocação + alinhamento de yaw + autotune de yaw + seletor (original em `.bak`) |
| `scripts/rover-quicktune.lua` | QuickTune oficial do ArduPilot, com o seletor `SCR_USER6` (original em `.orig`) |
| `scripts/modules/autotune.lua` | Autotune por relé (Åström-Hägglund + Tyreus-Luyben) |
| `scripts/modules/pid.lua` | PID com anti-windup e correção de limites nil |
| `tests/` | Simulador e testes |

### Referências

- [Rover QuickTune — ArduPilot](https://ardupilot.org/rover/docs/quiktune.html)
- [Rover Tuning Process](https://ardupilot.org/rover/docs/rover-tuning-process.html)
- [Tuning Turn Rate](https://ardupilot.org/rover/docs/rover-tuning-steering-rate.html)
