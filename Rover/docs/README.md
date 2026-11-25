# Documentação do Rover

Esta pasta é a documentação do rover. O GitHub mostra este arquivo ao abrir `Rover/docs`. Os `.md` ficam no repositório; não vão para o cartão SD. A cópia para leitura e impressão é [documentacao-rover.pdf](documentacao-rover.pdf).

| Documento | Conteúdo |
| --- | --- |
| [esc_can.md](esc_can.md) | O que o driver CAN (`modules/can_*.lua`) faz, cada variável, a lei de controle, o CAN do C620 e o autotune de roda |
| [Manual-Tuning-CAN.md](Manual-Tuning-CAN.md) | Ordem de campo: parâmetros de base e cada autotune |
| [Autotunning.md](Autotunning.md) | Guia de instalação, sintonia e operação em MANUAL e AUTO |
| [Controle-CAN.md](Controle-CAN.md) | Arquitetura do mixer e do caminho CAN. A parte final ainda descreve o desenho antigo de PI por lado |

Os scripts estão em `Rover/scripts/` (estrutura abaixo). A lista oficial de cada parâmetro do firmware, com unidade e faixa, está em [Parameters do Rover](https://ardupilot.org/rover/docs/parameters.html). Abaixo está o resumo do que este veículo usa e dos grupos que o firmware declara.

## Estrutura dos scripts

Só existe um script na pasta principal: `rover.lua`. Ele carrega os módulos de `modules/`, um por execução do script, e roda as tarefas (mixer, QuickTune e driver CAN) num agendador único.

**Carga sob demanda (memória):** o heap do scripting (`SCR_HEAP_SIZE` = 520000, com só ~52 kB de RAM livre na placa) não comporta tudo junto. Por isso `quicktune.lua` só carrega com `SCR_USER1 = 1` (ou a chave `RTUN_RC_FUNC`) e `can_wheeltune.lua` só com `SCR_USER1 = 4/5`; cada um sai quando o seletor muda e não há tune em andamento. Os parâmetros `RTUN_` ficam em `quicktune_params.lua`, carregado no boot. Ao carregar o tune de roda aparece `Rover: wheel tune loaded`; o QuickTune mostra `Rover QuickTune loaded`. No cartão SD fica igual: `APM/scripts/rover.lua` e `APM/scripts/modules/*.lua`.

| Módulo | Função |
| --- | --- |
| `rover_common.lua` | Constantes, utilitários e a tabela compartilhada onde os módulos se encontram |
| `mixer.lua` | Modos de condução e alocação esquerda/direita; parâmetros `THRO_` e `MIXER_` |
| `out_pwm.lua` | Saída **PWM** (MAIN 1..4), kick de yaw e `THRO_MIN_LUA` |
| `can_esc.lua` | Driver **CAN** do C620: E/S do barramento, saúde, pré-arm, `ESC_STAT` |
| `can_state.lua` | Estado e parâmetros `ESC_` do driver CAN |
| `can_speed.lua` | PI de velocidade de cada roda (rampa, pré-filtro, frenagem, stall boost) |
| `can_wheeltune.lua` | Autotune de roda por degrau, frente e ré (`SCR_USER1 = 4/5`), carregado sob demanda |
| `can_report.lua` | Log `DJIC`, telemetria de ESC e valores ao vivo para a GCS |
| `yaw_align.lua` | Alinhamento de proa no waypoint e sequência de verificação |
| `yaw_tune.lua` | Autotune de yaw por relé (`SCR_USER1 = 2`) |
| `quicktune.lua` | QuickTune do ArduPilot (`SCR_USER1 = 1`), carregado sob demanda |
| `quicktune_params.lua` | Parâmetros `RTUN_` (sempre carregado) |
| `circle-tune.lua` | Relé de curva em Circle (`SCR_USER1 = 3`) |
| `gain_banks.lua` | Bancos de ganho PWM/CAN (`CANT_`) |
| `autotune.lua`, `pid.lua`, `functions.lua` | Relé, PID e funções comuns |

**Tirar um caminho de saída:** apague do cartão `modules/out_pwm.lua` para rodar só em CAN, ou todos os `modules/can_*.lua` para rodar só em PWM. O `rover.lua` informa na inicialização quais caminhos carregou (`Rover r2611a: PWM yes, CAN yes`). `can_wheeltune.lua` e `can_report.lua` também podem sair sozinhos.

**Scripts antigos:** `esc_can.lua`, `rover-controlallocation.lua` e `rover-quicktune.lua` não podem ficar em `APM/scripts` junto com o `rover.lua` (dois scripts mexeriam nos mesmos motores). Se algum for encontrado, o `rover.lua` não inicia nada e recusa o arm. O `tools/ftp_scripts.py` apaga esses arquivos ao enviar os novos.

**Falhas:** um erro em uma tarefa não derruba as outras. Erro no mixer para as rodas e recusa o arm até reiniciar; o driver CAN, ao falhar, zera a corrente e recusa o arm; o QuickTune avisa e tenta de novo em 1 s.

**Teste sem o rover:** `tools/test/lua_sim.lua` roda os scripts num simulador da API do ArduPilot (4 rodas, CAN e PWM) e grava um traço para comparar versões.

## Seletor `SCR_USER1`

Um valor por vez. O autotune que termina bem grava 0 sozinho.

| Valor | Quem atende | Condição |
| --- | --- | --- |
| 0 | ninguém | Uso normal |
| 1 | `quicktune.lua` | QuickTune do Pixhawk, armado |
| 2 | `yaw_tune.lua` | Relé de yaw, armado em HOLD |
| 3 | `modules/circle-tune.lua` | Relé de curva, armado em Circle |
| 4 | `can_wheeltune.lua` | Sintonia de roda no chão, armado em HOLD, sticks no centro |
| 5 | `can_wheeltune.lua` | A mesma sintonia, uma roda por vez, rodas no ar |
| 6 | — | Aposentado. Não faz nada |

`SCR_USER5 = 2` faz o driver CAN seguir o comando do mixer Lua. Outro valor faz o driver de ESC ler as saídas de servo.

## Parâmetros criados pelos scripts

Cada tabela Lua tem uma chave fixa. Não reutilizar a chave com outro prefixo.

| Prefixo | Chave | Módulo |
| --- | --- | --- |
| `BATT_SOC` | 14 | `system-start.lua` |
| `RTUN_` | 15 | `quicktune_params.lua` |
| `YAWT_` | 16 | `yaw_align.lua` (`GAIN_MULT`, `TIMEOUT`) e `yaw_tune.lua` (o resto) |
| `THRO_` | 17 | `mixer.lua` |
| `CTUN_` | 18 | `modules/circle-tune.lua` |
| `CANT_` | 19 | `gain_banks.lua` |
| `MIXER_` | 20 | `mixer.lua` |
| `ESC_` | 74, ou 72 se a 74 já tiver outro prefixo | `can_state.lua` |

`ESC_STAT` mostra o estado do driver, só em RAM. Não salvar. `ESC_CMD_L`, `ESC_CMD_R` e `ESC_CMD_SEQ` não são mais usados (o mixer chama o driver direto); as posições ficam para a tabela não deslocar.

### `ESC_` — driver C620

O comportamento de cada um está em [esc_can.md](esc_can.md). Resumo:

| Parâmetro | Padrão | Função |
| --- | --- | --- |
| `ESC_CAN` | 0 | 0 = PWM, barramento calado. 1 = CAN. Só troca desarmado |
| `ESC_MODE` | 0 | 0 = PI de velocidade por roda. 1 = corrente proporcional ao comando |
| `ESC_RPM` | 400 | Rpm de roda no comando 1 |
| `ESC_GEAR` | 19,2032 | Redução rotor/roda do M3508 P19 |
| `ESC_CUR` | 10 | Limite de corrente, A. O protocolo corta em 20 A |
| `ESC_P` `ESC_I` `ESC_D` `ESC_FF` | 0,03 / 0,3 / 0 / 0 | PI+D e feed-forward compartilhados. D não tem cópia por roda |
| `ESC_IMAX` | 10 | Teto do integrador, A |
| `ESC_REV` | 0 | Máscara de inversão. Bit 0 = ESC 1 |
| `ESC_OPT` | 1 | Bit 0 log `DJIC`. Bit 1 segura 0 rpm desarmado. Bit 2 dessatura o outro lado |
| `ESC_BRK` | 1 | 0 = corrente sempre em ±`ESC_CUR`. 1 = corrente contrária limitada enquanto segura ou acelera |
| `ESC_ACC` | 0 | Rampa da referência, rpm/s. 0 = sem rampa |
| `ESC_BRKCUR` | 30 | Corrente contrária com `ESC_BRK = 1`, % de `ESC_CUR` |
| `ESC_LOC_P` `ESC_LOC_I` `ESC_LOC_MAX` | 0 / 0,5 / 15 | Sem uso. A correção de lado virou o PI por roda |
| `ESC_SLIP` | 60 | Diferença de rpm que conta como patinação. 0 desliga o aviso |
| `ESC_LMASK` | 6 | Bits do lado esquerdo. 6 = ESC 2 e ESC 3 |
| `ESC_WT_RELAY` | 100 | Degrau do autotune de roda, % de `ESC_CUR` |
| `ESC_WT_RPM` | 120 | Patamar alto do autotune, rpm. Abaixo de 60 o script usa 120 |
| `ESC_WT_BW` | 8 | Banda desejada do autotune, rad/s |
| `ESC_CMD_L` `ESC_CMD_R` | 0 | Sem uso desde o `rover.lua` modular. Mantidos para a tabela não deslocar |
| `ESC_CMD_SEQ` | 0 | Sem uso desde o `rover.lua` modular |
| `ESC_STAT` | 0 | Bits: 1 CAN, 2 saturado, 4 tune rodando, 8 tune ok, 16 tune falhou, 32 patinação |
| `ESC_FR1` … `ESC_FR4` | 0 | Sem uso. Mantidos para a tabela não deslocar |
| `ESC_P1` `ESC_I1` `ESC_FF1` … `P4` `I4` `FF4` | 0 | Ganhos da roda. 0 usa o compartilhado |
| `ESC_GCS_HZ` | 4 | `NAMED_VALUE_FLOAT` para a estação de solo, Hz. 0 desliga |
| `ESC_PREF` | 0,5 | Pré-filtro da referência, fração do tempo integral. 0 desliga |
| `ESC_RPM_AUX` | 1 | Liga os auxiliares configurados do controle de RPM. 0 contorna rampa, pré-filtro, restrição adicional de frenagem, boost de destravamento e dessaturação entre lados, preservando PID e limites absolutos de corrente. |

### `THRO_` — curso do mixer

| Parâmetro | Padrão | Função |
| --- | --- | --- |
| `THRO_MAX_LUA` | 90 | Curso do PWM em torno de 1500 µs, em %. 90 dá ±450 µs |
| `THRO_MIN_LUA` | 1500 | PWM mínimo acima do neutro para começar um alinhamento de proa parado. Vale também para baixo. 1500 desliga |

### `MIXER_` — frequência de atuação

| Parâmetro | Padrão | Função |
| --- | --- | --- |
| `MIXER_FREQ` | 50 Hz | Frequência solicitada ao mixer em AUTO, CIRCLE (incluindo CircleTune), ACRO, RTL e SIMPLE, nos caminhos CAN e PWM. Faixa efetiva: 25–200 Hz. |

O script registra `MIXER_FREQ` como parâmetro dinâmico do ArduPilot, persistente quando gravado pela estação de solo. Para aparecer no **Full Parameter List** do Mission Planner, instale o `rover.lua` e o `modules/mixer.lua` atualizados em `APM/scripts`, deixe `SCR_ENABLE = 1`, reinicie a controladora ou os scripts e use **Refresh Params** depois que o script carregar. Não é necessário recompilar o firmware. Se o script não carregar, consulte as mensagens de erro de scripting.

A alteração do valor é lida a cada ciclo e muda o próximo reagendamento. O período é `1000 / MIXER_FREQ`, arredondado para milissegundos inteiros; 50 Hz solicita 20 ms e 100 Hz solicita 10 ms. Valores positivos fora da faixa são limitados a 25–200 Hz; zero, negativos e NaN usam 50 Hz. A frequência real depende da carga do scripting.

O mínimo de 25 Hz mantém o intervalo nominal abaixo do limite de 50 ms do CircleTune e do timeout de 60 ms das suas saídas PWM. O parâmetro não altera o PI de alinhamento de proa (5 Hz), o controle de RPM do driver CAN (200 Hz), o período de MANUAL/HOLD nem `SCHED_LOOP_RATE`.

### `YAWT_` — autotune de yaw

| Parâmetro | Padrão | Função |
| --- | --- | --- |
| `YAWT_MAX_ANG_ERR` | 90 | Abortar se o erro de proa passar disto, graus |
| `YAWT_MAX_OUT` | 100 | Maior comando do relé, % do curso de `THRO_MAX_LUA` |
| `YAWT_GAIN_MULT` | 1 | Multiplica o P e o I gravados na hora de alinhar a proa |
| `YAWT_MAX_ANG` | 45 | Erro de proa em que o relé inverte, graus |
| `YAWT_MIN_OUT` | 5 | Comando em que o relé começa, % |
| `YAWT_MAX_CYCLE` | 6 | Ciclos do relé, contando os dois primeiros que são descartados |
| `YAWT_TIMEOUT` | 15 | Segundos sem 1° de mudança de proa para encerrar a verificação |

O P e o I de yaw do caminho PWM ficam em `SCR_USER2` e `SCR_USER3`.

### `CANT_` — bancos de ganho CAN e PWM

O mixer, desarmado, deixa `ATC_*` e `CRUISE_THROTTLE` no conjunto do caminho que está ativo. O outro conjunto fica aqui.

| Parâmetro | Função |
| --- | --- |
| `CANT_ACTIVE` | 1 = o conjunto CAN está nos parâmetros do ArduPilot. 0 = o conjunto PWM |
| `CANT_C_VALID` | 1 = o banco CAN já foi gravado alguma vez |
| `CANT_C_STRFF` `CANT_C_STRP` `CANT_C_STRI` | Cópia CAN de `ATC_STR_RAT_FF`, `ATC_STR_RAT_P`, `ATC_STR_RAT_I` |
| `CANT_C_SPDP` `CANT_C_SPDI` | Cópia CAN de `ATC_SPEED_P` e `ATC_SPEED_I` |
| `CANT_C_CRUISE` | Cópia CAN de `CRUISE_THROTTLE` |
| `CANT_P_STRFF` `CANT_P_STRP` `CANT_P_STRI` | As mesmas três de curva, cópia do caminho PWM |
| `CANT_P_SPDP` `CANT_P_SPDI` `CANT_P_CRUISE` | Velocidade e cruise, cópia do caminho PWM |

### `RTUN_` — QuickTune

Roda em Circle e pode gravar `ATC_STR_RAT_P/I/D/FF`, `ATC_SPEED_P/I/D`, `CRUISE_SPEED` e `CRUISE_THROTTLE`.

| Parâmetro | Padrão | Função |
| --- | --- | --- |
| `RTUN_ENABLE` | 1 | 0 desliga o QuickTune |
| `RTUN_AXES` | 3 | Bit 0 curva, bit 1 velocidade. 3 sintoniza os dois |
| `RTUN_STR_FFRATIO` | 0,9 | Resposta medida em relação ao feed-forward de curva |
| `RTUN_STR_P_RATIO` | 0,5 | P de curva em relação ao feed-forward. 0 não mexe no P |
| `RTUN_STR_I_RATIO` | 0,5 | I de curva em relação ao feed-forward. 0 não mexe no I |
| `RTUN_SPD_FFRATIO` | 1 | Resposta medida em relação a `CRUISE_THROTTLE` |
| `RTUN_SPD_P_RATIO` | 1 | P de velocidade em relação ao feed-forward |
| `RTUN_SPD_I_RATIO` | 1 | I de velocidade em relação ao feed-forward |
| `RTUN_AUTO_FILTER` | 1 | Ajusta o filtro do PID a partir de `INS_GYRO_FILTER` |
| `RTUN_AUTO_SAVE` | 5 | Segundos depois do fim para gravar sozinho |
| `RTUN_RC_FUNC` | 300 | `RCn_OPTION` que inicia, para e grava. 300 = Scripting1 |
| `RTUN_SPEED_MIN` | 0,5 | Velocidade mínima para começar, m/s |

### `CTUN_` — relé de Circle

| Parâmetro | Padrão | Função |
| --- | --- | --- |
| `CTUN_RELAY` | 5 | Amplitude do relé, % do curso. Vale de 1 a 20 |
| `CTUN_RAD_ERR` | 2 | Erro radial máximo, em metros, para a medição valer. Acima disso o tune aborta |

Grava `ATC_STR_RAT_P` e `ATC_STR_RAT_I`.

### `BATT_SOC_` — partida do sistema

A tabela tem 32 vagas e o prefixo do monitor de bateria. Estes quatro são dos scripts.

| Parâmetro | Padrão | Função |
| --- | --- | --- |
| `BATT_SOC_ACTOBSAVD` | 0 | Liga o desvio de obstáculo |
| `BATT_SOC_OBSYAW` | 0 | Yaw usado nesse desvio |
| `BATT_SOC_OBSTHR` | 0 | Acelerador usado nesse desvio |
| `BATT_SOC_SYSSTART` | 1 | 1 = a partida já rodou. Os outros scripts esperam este valor |

## Parâmetros do firmware que este rover precisa

Valores de barramento e de scripting usados com os C620 no CAN1. Mudou CAN, heap ou a cota da VM, reinicie a placa.

| Parâmetro | Valor neste rover | Função |
| --- | --- | --- |
| `CAN_D1_PROTOCOL` | 10 | Scripting. 1 é DroneCAN e o Lua não vê o barramento |
| `CAN_P1_DRIVER` | 1 | ESCs no conector CAN1 |
| `CAN_P1_BITRATE` | 1000000 | 1 Mbit/s do C620 |
| `CAN_P2_DRIVER` | 0 | Não pode ser o mesmo número de `CAN_P1_DRIVER` |
| `SCR_ENABLE` | 1 | Liga os scripts |
| `SCR_HEAP_SIZE` | 400000 | Memória dos scripts |
| `SCR_VM_I_COUNT` | 100000 | Instruções por ciclo. Com 10000 o `rover.lua` estoura e é removido |
| `SCR_USER4` | 4 | Uso antigo do mixer. Não é seletor |
| `SCR_USER6` | 0 | Livre |
| `SERVO1_FUNCTION` | 73 | ThrottleLeft, ESC 1 |
| `SERVO2_FUNCTION` | 74 | ThrottleRight, ESC 2 |
| `SERVO3_FUNCTION` | 74 | ThrottleRight, ESC 3 |
| `SERVO4_FUNCTION` | 73 | ThrottleLeft, ESC 4 |
| `MOT_SAFE_DISARM` | 0 | Sem PWM de segurança no desarmar |

No CAN2 a troca é `CAN_P1_DRIVER = 0`, `CAN_P2_DRIVER = 1`, `CAN_D1_PROTOCOL = 10`. Se o chicote não for 73/74/74/73, ajustam-se `ESC_LMASK` e `ESC_REV` juntos.

Malhas de cima, gravadas pelo QuickTune ou trocadas de banco pelo mixer:

| Parâmetro | Função |
| --- | --- |
| `ATC_STR_RAT_P` `ATC_STR_RAT_I` `ATC_STR_RAT_D` `ATC_STR_RAT_FF` | PID de taxa de curva |
| `ATC_SPEED_P` `ATC_SPEED_I` `ATC_SPEED_D` | PI de velocidade |
| `CRUISE_SPEED` | Velocidade de cruzeiro, m/s |
| `CRUISE_THROTTLE` | Acelerador que produz essa velocidade |
| `INS_GYRO_FILTER` | Filtro do giroscópio. O QuickTune copia para o filtro do PID |
| `GCS_PID_MASK` | Quais PIDs a estação de solo recebe ao vivo |
| `RCMAP_ROLL` `RCMAP_THROTTLE` | Canais de direção e acelerador |
| `RC1_TRIM` `RC3_TRIM` `RC1_DZ` `RC3_DZ` | Trim e zona morta. O autotune de roda exige os sticks a menos de 100 µs do trim |
| `RC3_REVERSED` | Sentido do acelerador no mixer |

## Grupos do firmware Rover

Tudo o que não está nas tabelas de cima pertence a um destes grupos, declarados em `Rover/Parameters.cpp`. O nome completo é o prefixo mais o sufixo do grupo. `ATC_STR_RAT_P` está no grupo `ATC`. A descrição de cada item está no fonte indicado e na [lista oficial](https://ardupilot.org/rover/docs/parameters.html).

Parâmetros soltos do veículo, sem prefixo de biblioteca: `FORMAT_VERSION`, `LOG_BITMASK`, `RST_SWITCH_CH`, `INITIAL_MODE`, `SYSID_THISMAV`, `SYSID_MYGCS`, `SYSID_ENFORCE`, `TELEM_DELAY`, `GCS_PID_MASK`, `AUTO_TRIGGER_PIN`, `AUTO_KICKSTART`, `CRUISE_SPEED`, `CRUISE_THROTTLE`, `PILOT_STEER_TYPE`, `FS_ACTION`, `FS_TIMEOUT`, `FS_THR_ENABLE`, `FS_THR_VALUE`, `FS_GCS_ENABLE`, `FS_GCS_TIMEOUT`, `FS_CRASH_CHECK`, `FS_EKF_ACTION`, `FS_EKF_THRESH`, `FS_OPTIONS`, `MODE_CH`, `MODE1` … `MODE6`, `TURN_RADIUS`, `ACRO_TURN_RATE`, `RTL_SPEED`, `FRAME_CLASS`, `FRAME_TYPE`, `BAL_PITCH_MAX`, `BAL_PITCH_TRIM`, `CRASH_ANGLE`, `LOIT_TYPE`, `LOIT_RADIUS`, `LOIT_SPEED_GAIN`, `SIMPLE_TYPE`, `MIS_DONE_BEHAVE`, `STICK_MIXING`, `SPEED_MAX`, `GUID_OPTIONS`, `MANUAL_OPTIONS`, `MANUAL_STR_EXPO`, `PIVOT_TURN_ANGLE`, `PIVOT_TURN_RATE`.

| Grupo | O que reúne |
| --- | --- |
| `COMPASS_` | Bússola |
| `SCHED_` | Escalonador do firmware |
| `BARO` | Barômetro |
| `RELAY` | Relés |
| `RCMAP_` | Qual canal de rádio é roll, pitch, throttle e yaw |
| `SR0_` … `SR6_` | Taxa de cada mensagem MAVLink, por porta |
| `RNGFND` | Rangefinder |
| `INS` | IMU, giroscópio, acelerômetro e filtros |
| `SIM_` | Simulador. Não existe na placa |
| `AHRS_` | Atitude e referência de proa |
| `CAM` | Câmera |
| `PLND_` | Pouso de precisão |
| `MNT` | Gimbal |
| `ARMING_` | Checagens de armar |
| `BATT` | Bateria. A tabela Lua `BATT_SOC_*` divide o mesmo prefixo |
| `BRD_` | Placa, pinos e segurança de hardware |
| `CAN_` | Portas, bitrate e protocolo CAN |
| `GPS` | Receptores GNSS |
| `EK2_` | EKF2 |
| `EK3_` | EKF3, o estimador usado no rover |
| `RPM` | Sensores de rotação |
| `MIS_` | Opções de missão |
| `RSSI_` | Intensidade do rádio |
| `NTF_` | LED, buzzer e aviso |
| `BTN_` | Botões da placa |
| `OSD` | OSD analógico |
| `FLOW` | Fluxo óptico |
| `SERVO` | Função, faixa e trim de cada saída |
| `RC` | Canais do rádio, falha de sinal e opções |
| `AFS_` | Failsafe avançado, se o firmware foi compilado com ele |
| `BCN` | Beacon de posição |
| `MOT_` | Misturador nativo de motor, safety disarm e limites |
| `WENC` | Encoder de roda do firmware. Os M3508 usam o encoder do C620, lido pelo driver CAN (`can_esc.lua`) |
| `ATC` | PIDs de curva e de velocidade |
| `SRTL_` | Smart RTL |
| `PRX` | Proximidade |
| `AVOID_` | Desvio de obstáculo |
| `FOLL` | Seguir outro veículo |
| `SPRAY_` | Pulverização |
| `WRC` | Guincho |
| `RALLY_` | Pontos de rally |
| `WNDVN_` | Cata-vento |
| `WP_` | Navegação de waypoint, raio e velocidade |
| `SAIL_` | Veleiro |
| `OA_` | Obstacle avoidance por caminho |
| `TRQD_` | Torque diferencial |
| `PSC` | Controle de posição |
| `DOCK` | Atracação |
| `CIRC` | Modo Circle |
