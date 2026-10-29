# Autotuning do Rover

Este guia reúne a explicação teórica e prática dos dois autotunings implementados nesta branch. As instruções refletem o código atual em `Rover/scripts`; a seleção de tuning é feita por `SCR_USER1`, não por `SCR_USER6`.

| Método | Controlador ajustado | Resultado salvo em |
|---|---|---|
| Rover QuickTune | Controladores nativos de taxa de giro e velocidade do ArduPilot | `ATC_*`, `CRUISE_SPEED`, `CRUISE_THROTTLE` |
| Autotune de yaw por relé | PI Lua que alinha o rover parado depois de um waypoint | `SCR_USER2` (P), `SCR_USER3` (I) |

Os dois métodos são independentes. QuickTune não ajusta o PI de alinhamento de yaw; o autotune de yaw não ajusta `ATC_STR_RAT_*` nem `ATC_SPEED_*`.

## 1. Antes de instalar ou testar

Copie para o cartão SD da Pixhawk, em `APM/scripts/`:

```text
rover-controlallocation.lua
rover-quicktune.lua
system-start.lua
modules/autotune.lua
modules/functions.lua
modules/pid.lua
```

Habilite Lua com `SCR_ENABLE=1` e reinicie a Pixhawk após copiar ou alterar scripts. Garanta que `SCR_HEAP_SIZE` seja suficiente para todos os scripts instalados. Os `.md`, arquivos de teste e backups não precisam ir para o cartão.

Confira que os arquivos instalados são desta branch. A documentação antiga usava `SCR_USER6`, mas o código atual usa `SCR_USER1` como seletor. O dump enviado anteriormente tinha `SCR_USER1=1550`, que não corresponde aos valores de seleção descritos abaixo; confira o valor atual na Pixhawk antes de armar.

> **Atenção:** o autotune de yaw atual inicia sempre que `SCR_USER1=2` e o rover está armado, sem verificar o modo de condução. Ele assume os motores e aplica comandos alternados de direção. Use `SCR_USER1=2` somente quando o rover estiver em uma área de ensaio isolada e pronto para girar no lugar. `HOLD` é o modo pretendido para esse ensaio, mas o código não o impõe.

## 2. Seletor e parâmetros de yaw

`SCR_USER1` é compartilhado entre os dois scripts:

| `SCR_USER1` | `rover-controlallocation.lua` | `rover-quicktune.lua` |
|---:|---|---|
| 0 | Operação normal; yaw autotune inativo | Lê a posição da função de scripting RC definida por `RTUN_RC_FUNC` |
| 1 | Apenas informa que QuickTune foi selecionado; segue controle normal | Inicia QuickTune automaticamente enquanto armado, em qualquer modo |
| 2 | Executa o autotune de yaw enquanto armado, em qualquer modo | Deixa de usar seleção automática; consulta a chave RC |

O script de yaw cria `YAWT_MAX_ANG` (padrão 45°), `YAWT_MAX_OUT` (padrão 20%) e `YAWT_GAIN_MULT` (padrão 1). A amplitude efetiva do relé é `YAWT_MAX_OUT / 100`, limitada a 5–45%. `YAWT_GAIN_MULT` multiplica P e I usados pelo PI de orientação; por exemplo, fator 10 aplica P=10 e I=5 se os ganhos base forem P=1 e I=0,5. Os ganhos base, sem multiplicador, continuam salvos em `SCR_USER2/3`.

`SCR_USER2` e `SCR_USER3` guardam P e I base do PI de yaw. Se ambos forem maiores que zero quando `rover-controlallocation.lua` inicia, o script aplica `YAWT_GAIN_MULT` a esses valores, sem divisão por 1000. Se algum for zero ou negativo, o script mantém os defaults P=3,5 e I=8. Ao terminar com sucesso, o autotune grava os ganhos base e usa no PID em memória os ganhos multiplicados.

## 3. QuickTune dos controladores nativos

### Teoria: identificação feed-forward

QuickTune é uma identificação aproximada em campo. Em vez de testar todas as combinações de ganhos, observa pares de comando e resposta do rover para estimar o feed-forward (FF), isto é, quanto comando é necessário para produzir certa resposta. Em seguida, define P e I como proporções dessa estimativa, configuradas pelos parâmetros `RTUN_*_RATIO`.

Para taxa de giro, relaciona a saída de direção do controlador com a taxa angular medida pelo giroscópio Z. Para velocidade, relaciona throttle e velocidade longitudinal estimada pelo AHRS. Pneus, piso, bateria, carga, velocidade e o alocador PWM Lua fazem parte da resposta medida; os ganhos obtidos são mais representativos quando essas condições se parecem com a operação real.

### O que ele ajusta

Com `RTUN_AXES=3` (padrão), o script tenta ajustar direção e velocidade:

- **Direção/taxa de giro:** `ATC_STR_RAT_FF`, `ATC_STR_RAT_P`, `ATC_STR_RAT_I`.
- **Filtros de direção:** se `RTUN_AUTO_FILTER>0`, ajusta `ATC_STR_RAT_FLTT` e `ATC_STR_RAT_FLTD` para metade de `INS_GYRO_FILTER`.
- **Velocidade:** `CRUISE_SPEED`, `CRUISE_THROTTLE`, `ATC_SPEED_P` e `ATC_SPEED_I`.

O script não calcula `ATC_STR_RAT_D` nem `ATC_SPEED_D`. Os parâmetros `RTUN_STR_*_RATIO` e `RTUN_SPD_*_RATIO` definem as proporções de P/I derivadas das estimativas feed-forward. `RTUN_AXES=1` ajusta só direção; `RTUN_AXES=2` só velocidade.

### Significado dos parâmetros alterados

| Parâmetro | O que controla |
|---|---|
| `ATC_STR_RAT_FF` | Ação feed-forward de direção: antecipa a saída necessária para a taxa de giro desejada. |
| `ATC_STR_RAT_P` | Corrige o erro entre taxa de giro desejada e medida; ganho maior torna a correção mais forte. |
| `ATC_STR_RAT_I` | Corrige erro persistente de taxa de giro acumulando a correção ao longo do tempo. |
| `ATC_STR_RAT_FLTT` | Filtra o alvo de taxa de giro; suaviza mudanças rápidas do comando desejado. |
| `ATC_STR_RAT_FLTD` | Filtra o termo derivativo da malha; afeta ruído/suavidade, embora este QuickTune não ajuste o ganho D. |
| `CRUISE_SPEED` | Velocidade de cruzeiro de referência para navegação quando outra velocidade não é especificada. |
| `CRUISE_THROTTLE` | Estimativa base de throttle necessária para manter a velocidade de cruzeiro. |
| `ATC_SPEED_P` | Corrige erro entre velocidade desejada e medida com resposta proporcional. |
| `ATC_SPEED_I` | Compensa erro de velocidade persistente com correção integral. |

Os filtros `FLTT` e `FLTD` são frequências em Hz e o QuickTune os define como `0,5 × INS_GYRO_FILTER` quando `RTUN_AUTO_FILTER` está habilitado. Os ganhos P/I resultantes são gravados nos parâmetros nativos ArduPilot. Eles influenciam os comandos internos que o alocador Lua transforma em PWM em `AUTO`/`CIRCLE`; não substituem a mistura final dos motores feita pelo Lua.

### Preparação do teste

1. Faça o ensaio em área plana, ampla e livre de pessoas e obstáculos. O modo CIRCLE precisa de espaço para o círculo e para o erro de navegação; por exemplo, raio 4 m com margem de 4 m pede aproximadamente 16 × 16 m livres.
2. Configure `CIRC_RADIUS` para um valor compatível com o rover e com a área disponível. O padrão do firmware é 20 m; um raio menor, como 4 m, só deve ser usado se o rover conseguir fazer essa curva sem patinar ou sair do controle.
3. Configure `CIRC_SPEED` para a velocidade desejada do ensaio. Zero usa a velocidade padrão de navegação (`CRUISE_SPEED`/configuração do Rover).
4. Verifique previamente a base de velocidade, bateria, sentido dos motores, GPS/AHRS e parada de emergência. A mistura PWM customizada do `rover-controlallocation.lua` faz parte da planta que o QuickTune observa.

### Execução conforme o código atual

1. Deixe `RTUN_ENABLE=1` e escolha `RTUN_AXES`.
2. Ajuste `CIRC_RADIUS` e `CIRC_SPEED` antes de entrar em CIRCLE.
3. Arme e selecione `CIRCLE` para o ensaio previsto. **O código não exige CIRCLE:** `SCR_USER1=1` inicia QuickTune assim que armado, qualquer que seja o modo. Não selecione `SCR_USER1=1` em campo sem já estar no modo e na área de teste pretendidos.
4. Mantenha sticks de direção e throttle neutros. Uma entrada detectada pausa a coleta por quatro segundos após o último movimento.
5. Monitore mensagens no GCS. QuickTune não gera o movimento; o rover deve estar efetivamente se movendo e virando para haver amostras válidas.
6. Para abortar e restaurar os ganhos temporários, desarme ou coloque o seletor RC do QuickTune na posição baixa. **Mudar de CIRCLE para outro modo não é, por si só, uma condição de abort implementada.**
7. Ao terminar as etapas selecionadas, os resultados são salvos após `RTUN_AUTO_SAVE` segundos (padrão 5). Se `RTUN_AUTO_SAVE=0`, use uma chave RC configurada para a função `RTUN_RC_FUNC`: posição 1 executa e posição 2 salva. Ao usar `SCR_USER1=1` automático, o script não apresenta uma forma RC de salvar; mantenha o auto save habilitado.

O script QuickTune roda a 40 Hz para medir e atualizar os parâmetros. O alocador de motores continua escrevendo saídas a 5 Hz nos modos normais, então os ganhos obtidos precisam ser validados com o rover e sua mistura reais. Não assuma que a frequência do QuickTune seja a frequência de atualização PWM.

### Mensagens e validação

Mensagens esperadas incluem:

```text
RTun: starting ATC_STR_RAT tune
RTun: adjusted ATC_STR_RAT_FF ...
RTun: ATC_STR_RAT_FF tuning done
RTun: starting ATC_SPEED tune
RTun: ATC_SPEED_FF tuning done
RTun: Tuning DONE
RTun: tuning gains saved
```

O QuickTune pode avisar que direção, taxa de giro, throttle ou velocidade estão baixos. Para taxa de giro, o código exige comando de direção ≥ 0,10 e taxa de giro > 10°/s, e coleta pelo menos dez segundos e o equivalente a duas voltas. Para velocidade, exige throttle ≥ 0,20, velocidade válida maior que `RTUN_SPEED_MIN` (padrão 0,5 m/s) e dez segundos de dados.

Após salvar, compare os parâmetros antes/depois e valide uma missão em `AUTO`. Examine nos logs a taxa de giro desejada e medida, e velocidade desejada e medida. Refaça o tuning se mudar a alocação de motores, pneus, piso, carga ou condições que mudem significativamente a resposta.

## 4. Autotune do PI de alinhamento de yaw

### Teoria: realimentação por relé

O autotune usa o método de realimentação por relé, associado a Åström–Hägglund. Em lugar de um PI provisório, o script aplica uma saída discreta `+d` ou `−d` conforme o erro angular cruza os limites de uma histerese. Isso gera uma oscilação limitada em torno do rumo inicial. O período e a amplitude dessa oscilação aproximam o ganho último `Ku` e o período último `Pu` da planta composta por rover, motores, piso e estimativa de yaw. A regra PI Tyreus–Luyben converte essa estimativa em ganhos conservadores.

### O que o rover faz

O procedimento aplica aceleração zero e alterna a direção entre `+d` e `−d`, enquanto mede o erro entre o yaw capturado no início e o yaw atual. O rover deve girar no lugar, alternando o sentido, dentro do limite configurado. O script descarta dois ciclos iniciais, mede quatro ciclos e estima P/I pela regra Tyreus–Luyben. O ensaio roda no mesmo período de 200 ms usado pelo PI de alinhamento.

As fórmulas usadas são:

```text
ε = 2° (histerese)
Ku = 4·d / (π·√(A² − ε²))
Kp = Ku / 3,2
Ti = 2,2·Pu
Ki = Kp / Ti
```

`A` é a amplitude média do erro e `Pu` é o período médio da oscilação, após descartar os ciclos iniciais. O algoritmo rejeita ciclos com variação acima de 25%, período fora de 0,4–20 s, erro maior que `YAWT_MAX_ANG`, ou experimento acima de 60 s.

Os valores calculados não são escalados por 1000: `Kp` vai diretamente para `SCR_USER2` e `Ki` para `SCR_USER3`. O controlador é PI (`D=0`) e a saída fica limitada pelo PID Lua. Depois de um ajuste bem-sucedido, o script aplica os ganhos multiplicados por `YAWT_GAIN_MULT` e tenta orientar o rover sucessivamente para 0°, 90°, 180°, 270° e 360°/0°. Cada tentativa e conclusão é informada no GCS. Ao terminar, o rover entra em `HOLD`; se uma etapa exceder o timeout, a sequência é interrompida e o rover também entra em `HOLD`. Ganhos só são apropriados se o sentido do comando fizer o erro diminuir e a estimativa de yaw for estável; o autotune não verifica sozinho a polaridade física de cada motor.

### Execução

1. Escolha uma área plana, livre ao redor do rover, com margem para girar. Faça primeiro no SITL e depois em ambiente de teste controlado.
2. Confirme que `SCR_USER1` não está em 1 e que o rover não está seguindo uma missão.
3. Com o rover parado, selecione `HOLD`, arme e então coloque `SCR_USER1=2`. Como a implementação não exige HOLD, faça isso somente quando puder aceitar que o script tome os motores.
4. Observe o rover: ele deve alternar giros ao redor do yaw inicial. Se o erro crescer continuamente, o sentido de controle ou a estimativa de yaw pode estar incorreto; interrompa o ensaio.
5. Sucesso salva P base em `SCR_USER2`, I base em `SCR_USER3`, aplica o multiplicador e inicia a sequência automática de headings. O seletor volta a `SCR_USER1=0`. Falha zera a saída, mas não salva ganhos nem retorna automaticamente o seletor a zero.
6. Para abortar durante execução, altere `SCR_USER1` para 0/1 ou desarme. O código não detecta saída de HOLD como cancelamento.

Mensagens incluem `YawTune: started`, progresso de erro/taxa/saída, `Ku`, `Pu`, amplitude, ganhos base/efetivos e o heading de cada etapa. Em falha, podem aparecer `inconsistent cycles`, `Pu out of range`, `amplitude below hysteresis`, `oscillation too large` ou `timeout`.

### Uso dos ganhos em missão

O PI ajustado é usado pelo `rover-controlallocation.lua` durante a sequência de verificação após o autotune e durante o alinhamento customizado após a chegada a um waypoint. Para o waypoint, a missão avança, o Lua lê o `z` do waypoint recém-alcançado como yaw desejado, muda para `STEERING` e corrige o rumo. Cada alinhamento termina depois de permanecer 600 ms dentro de ±2° e com velocidade angular abaixo de 2°/s; então o script entra em `HOLD`. Cada etapa tem timeout de 15 s. Os ganhos não substituem `ATC_STR_RAT_*` usados pelo controlador nativo na navegação.

Durante o alinhamento, o GCS recebe uma mensagem por segundo com `YawAlign target=... current=... error=...`, em graus. Se o erro aumentar em vez de diminuir sob correção, interrompa e investigue sinal da realimentação/mistura antes de ajustar ganhos.

### Significado dos parâmetros do autotune de yaw

| Parâmetro | Significado |
|---|---|
| `SCR_USER1` | Seletor compartilhado nesta branch: 0 = operação normal; 1 = QuickTune automático enquanto armado; 2 = autotune de yaw enquanto armado. |
| `SCR_USER2` | P do PI Lua de alinhamento; o script usa o valor diretamente se positivo. |
| `SCR_USER3` | I do PI Lua de alinhamento; o script usa o valor diretamente se positivo. |
| `YAWT_MAX_ANG` | Limite do erro angular do experimento; acima dele o autotune aborta. Padrão: 45°. |
| `YAWT_MAX_OUT` | Amplitude desejada do relé em porcentagem. Padrão: 20%, limitada internamente a 5–45%. |
| `YAWT_GAIN_MULT` | Multiplicador dos ganhos P/I durante alinhamentos. Padrão: 1; faixa aceita: 0,1–10000. Os valores base permanecem em `SCR_USER2/3`. |

`SCR_USER2/3` não são parâmetros `ATC_*`. Eles só alimentam o `steering_steady_pid` usado para alinhamento após waypoint; não ajustam a navegação normal em `AUTO`.

## 5. Parâmetros `RTUN_*` do QuickTune

O script cria estes parâmetros no primeiro boot em que carrega:

| Parâmetro | Padrão | Função |
|---|---:|---|
| `RTUN_ENABLE` | 1 | Habilita QuickTune |
| `RTUN_AXES` | 3 | Bitmask: 1 direção, 2 velocidade, 3 ambas |
| `RTUN_STR_FFRATIO` | 0,9 | Escala FF de taxa de giro |
| `RTUN_STR_P_RATIO` | 0,5 | Proporção do FF usada para P de direção; 0 preserva o P atual |
| `RTUN_STR_I_RATIO` | 0,5 | Proporção do FF usada para I de direção; 0 preserva o I atual |
| `RTUN_SPD_FFRATIO` | 1,0 | Escala estimativa de cruzeiro/FF de velocidade |
| `RTUN_SPD_P_RATIO` | 1,0 | Proporção usada para P de velocidade; 0 preserva P atual |
| `RTUN_SPD_I_RATIO` | 1,0 | Proporção usada para I de velocidade; 0 preserva I atual |
| `RTUN_AUTO_FILTER` | 1 | Ajusta `ATC_STR_RAT_FLTT/FLTD` a partir de `INS_GYRO_FILTER` |
| `RTUN_AUTO_SAVE` | 5 s | Autosave após conclusão; 0 exige posição de salvar da chave RC |
| `RTUN_RC_FUNC` | 300 | Função de scripting da chave RC para executar/salvar quando `SCR_USER1` não é 1 |
| `RTUN_SPEED_MIN` | 0,5 m/s | Velocidade mínima para amostras de speed tuning |

## 6. Arquivos e testes

Arquivos necessários no cartão para estes recursos:

- `scripts/rover-controlallocation.lua`
- `scripts/rover-quicktune.lua`
- `scripts/modules/autotune.lua`
- `scripts/modules/functions.lua`
- `scripts/modules/pid.lua`

O QuickTune cria os parâmetros `RTUN_*` descritos na seção 5. `system-start.lua` só é necessário para os parâmetros customizados da aplicação de barco; o alocador Rover desta branch não depende de `BATT_SOC_SYSSTART`.

Os testes auxiliares em `Rover/tests/` simulam APIs Lua e podem ajudar a verificar a lógica local, mas não substituem SITL nem validação física. O alocador e a mistura PWM devem ser verificados com o arranjo de motores real.

### Referências

- [Rover QuickTune — ArduPilot](https://ardupilot.org/rover/docs/quiktune.html)
- [Rover Tuning Process](https://ardupilot.org/rover/docs/rover-tuning-process.html)
- [Tuning Turn Rate](https://ardupilot.org/rover/docs/rover-tuning-steering-rate.html)
