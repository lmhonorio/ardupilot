# Relatório de testes — Mission Manager

Data: 10/11/2026. Ambiente: ROS 2 Jazzy, Python 3.12.3, Gazebo Harmonic 8.15.0,
ArduRover SITL compilado desta árvore, MAVProxy, FAST-LIO, MID360 e câmera Samsung.
Cenário `gravel_plane`, estação `container_gravel_plane`, vão de 1 × 1 × 1 m.
O transporte DDS usado foi UDPv4 (`FASTDDS_BUILTIN_TRANSPORTS=UDPv4`).
As observações abaixo separam teste de software, movimento simulado e limitações.
Não são ensaios em hardware real.

## Critérios e método

O controlador recebeu posição relativa estimada pelo LiDAR e auxílio visual
opcional. A posição verdadeira do modelo Gazebo foi lida **somente pelo programa
de validação**, sem ser enviada ao estimador/controlador.

O ensaio de movimento exigiu:

- posição verdadeira até 8 cm do alvo por eixo e rumo até 6°;
- confirmação interna mais restrita: até 7 cm / 5°, envelope dentro, medições
  estáveis, parada, HOLD e desarmamento;
- SOC operacional simulado em 100%, mantendo a bateria real separada;
- aproximadamente 15 segundos de recarga observados no tópico; o relógio interno
  da recarga é monotônico e o tópico acrescenta atraso de amostragem/transporte;
- saída confirmada antes de AUTO e início da missão em seq 1;
- GUIDED manual sem pedido de missão não pode causar saída;
- pedidos duplicados durante operação são recusados.

A missão curta contém HOME e um waypoint P4 externo, a 4 m da abertura. Ela
exercita a saída, a navegação AUTO, o evento de chegada e o retorno ao docking.
Não equivale a validar uma missão longa, múltiplas estações ou planejamento
em qualquer configuração de obstáculos.

## Testes de software

**154 testes passaram**, sendo 108 regressões existentes e 46 testes novos do
Mission Manager. Comando executado:

```bash
cd ~/ardupilot/Rover
source ros2/setup.bash
OPENBLAS_NUM_THREADS=1 python -m unittest discover -s python/tests -p 'test_*.py'
```

Evidências: [log da suíte](../../python/run/mission_manager_validation/unit_tests.log)
e [testes do manager](../../python/tests/test_mission_manager.py).

Os testes novos cobrem fusão com/sem QR, perda/retorno e divergência visual,
identidade e timestamps, envelope e obstáculos, reposicionamento externo,
prioridade da varredura atual e fallback registrado, vínculo/hash de missão,
recarga em 15 segundos, bateria insuficiente/desatualizada, transições repetidas,
GUIDED manual, pedidos duplicados, falta de confirmação de modo, falha de saída,
missão alterada durante autorização e HEARTBEAT de outro componente MAVLink.

As confirmações nos testes de orquestração usam um link simulado. Esses testes
verificam a sequência de estados/comandos, mas não demonstram movimento físico.

## Compilação e intertravamento do firmware

Compilação concluída com Python 3.8.10 (compatibilidade com este Waf):

```bash
cd ~/ardupilot
/home/grin/.pyenv/versions/3.8.10/bin/python3.8 ./waf rover -j4
```

[Log da compilação](../../python/run/mission_manager_validation/firmware_build.log).
SHA-256 do binário usado:
`d172bbde3679cc1f11d3c3f45f472fc0921e90cd239b26183012b38858913dcb`.

Com `DOCK_LOCK=1`, o comando real de mudança para AUTO foi recusado pelo SITL e
o HEARTBEAT permaneceu em HOLD. Isso foi repetido nas séries de integração,
inclusive com o rover estacionado dentro do container. Não foi apenas uma
verificação do comando transmitido.

## Resultados com movimento

**A última série concluiu três entradas consecutivas**, incluindo dois ciclos completos de saída → AUTO → retorno → recarga, sem reiniciar o manager entre os ciclos. Na série final, o maior erro absoluto por eixo foi **2,90 cm** e o maior erro de rumo foi **1,24°**.

| Série / ciclo | Longitudinal (cm) | Lateral (cm) | Rumo (°) | Recarga observada (s) | Resultado |
| --- | ---: | ---: | ---: | ---: | --- |
| [092701 / 1](../../python/run/mission_manager_validation/series_20261110_092701/results.json) | -2.82 | -0.03 | -0.01 | 14.982 | Aprovado |
| [095016 / 1](../../python/run/mission_manager_validation/series_20261110_095016/results.json) | -2.82 | 0.19 | 0.10 | 15.210 | Aprovado |
| [095016 / 2](../../python/run/mission_manager_validation/series_20261110_095016/results.json) | -0.08 | 3.45 | -0.20 | 15.118 | Aprovado |
| [095744 / 1](../../python/run/mission_manager_validation/series_20261110_095744/results.json) | -2.75 | 2.60 | 0.87 | 15.115 | Aprovado |
| [095744 / 2](../../python/run/mission_manager_validation/series_20261110_095744/results.json) | -2.62 | 0.17 | -0.12 | erro de pareamento | Falha no cálculo; ver reanálise |
| [100805 / 1](../../python/run/mission_manager_validation/series_20261110_100805/results.json) | -0.63 | -2.98 | -1.07 | 14.954 | Aprovado |
| [100805 / 2](../../python/run/mission_manager_validation/series_20261110_100805/results.json) | -2.98 | 1.74 | 1.30 | 15.114 | Aprovado |
| [102512 / 1](../../python/run/mission_manager_validation/series_20261110_102512/results.json) | -2.90 | -1.59 | -0.38 | 15.201 | Aprovado |
| [102512 / 2](../../python/run/mission_manager_validation/series_20261110_102512/results.json) | -2.20 | -2.80 | -1.24 | 15.017 | Aprovado |
| [102512 / 3](../../python/run/mission_manager_validation/series_20261110_102512/results.json) | -0.83 | 1.95 | 0.16 | 15.067 | Aprovado |

A recarga interna da série final durou 15,016 s, 15,004 s e 15,014 s. As diferenças na coluna acima vêm da observação pelo tópico ROS. Cada chegada terminou em HOLD, desarmado, com SOC operacional 100%.

As duas saídas da série final confirmaram separação longitudinal do envelope de 1,260 m e 1,272 m em relação ao centro da estação, acima do limite de 0,560 m. O recuo foi de aproximadamente 2 m antes do giro; o giro deslocou a base em 0,354 m e 0,306 m, resultando em recuo líquido final de 1,636 m e 1,655 m. Não se assumiu rotação perfeita sobre um ponto. Veja os eventos `exit_check` e `exit_confirmed` no [registro da série final](../../python/run/mission_manager_validation/trial20/events.jsonl).

Os hashes exatos de cada execução ficam no respectivo `manifest.json`. Depois da série final foram acrescentadas guardas de seleção de destino/ocupação de outras estações, cobertas pelos testes de software; o cenário físico ensaiado usa a única estação padrão.


Os arquivos `results.json` preservam os resultados do programa de validação;
`status_history.json` contém as amostras recebidas. Os diretórios `trial*` guardam
`events.jsonl`, telemetria e manifestos dos argumentos e hashes de código.

A série de 09:57 revelou um defeito **na medição do teste**: no segundo ciclo,
o cálculo escolheu o status “recarga concluída” do ciclo anterior e produziu
uma duração negativa. O resultado original foi preservado como falha. A
[reanálise dos timestamps](../../python/run/mission_manager_validation/series_20261110_095744/charging_reanalysis.json)
mostra 15,115 s e 15,011 s observados. O pareamento foi corrigido para usar apenas
conclusões posteriores ao início da recarga daquele ciclo, e a série foi repetida.

Na série com câmera, foram registradas 95 estimativas com correção QR aceita,
419 com QR não confiável para fusão naquele instante e 175 usando apenas LiDAR.
Isso demonstra o uso seletivo da câmera. Não significa que houve uma campanha
física de oclusões e códigos deliberadamente errados; esses casos são cobertos
pelos testes de software.

## Matriz dos 16 cenários solicitados

| # | Cenário | Evidência e alcance |
| --- | --- | --- |
| 1 | LiDAR + QR | Fusão testada em software e correções aceitas registradas no Gazebo |
| 2 | Somente LiDAR | Série real no Gazebo com `--no-qr` |
| 3 | Perda/retorno do QR | Teste determinístico; disponibilidade visual variou durante a série, sem exigir QR para avançar |
| 4 | Divergência LiDAR/QR | Teste determinístico rejeita inovação inconsistente; sem injeção física de erro na câmera |
| 5 | Rover desalinhado | Giro de aquisição e alinhamento executados no retorno, incluindo rover voltado para longe |
| 6 | AUTO → GUIDED por solicitação de docking | Evento real de chegada ao waypoint vinculado e modo confirmado no SITL |
| 7 | Entrada + recarga | Geometria medida no Gazebo, HOLD, desarmamento e SOC 100% |
| 8 | Início fora da estação | Teste de orquestração e inícios AUTO nas séries de desenvolvimento |
| 9 | Pedido dentro, bateria suficiente | Saídas e ciclos completos no SITL |
| 10 | Bateria insuficiente | Testes de software e recusa real no SITL com SOC 10%, mínimo 30%, sem saída |
| 11 | AUTO direto dentro | Recusa real de `DO_SET_MODE` pelo firmware com intertravamento ativo |
| 12 | GUIDED manual sem pedido | Sem saída e sem deslocamento maior que 2 cm nos ensaios |
| 13 | Saída completa antes de AUTO | Confirmação de envelope/odometria registrada antes da transição e início seq 1 |
| 14 | Saída falha não libera AUTO | Testes de confirmação/erro e parada real por geometria desatualizada no desenvolvimento |
| 15 | Ciclos consecutivos | Séries de dois ciclos sem reiniciar o manager entre eles |
| 16 | Duplicação/reconciliação | Testes de estados, rejeição real de `start` repetido e reconciliação após reinício |

## Ensaio de bateria insuficiente

Com o rover estacionado no centro, iniciou-se o manager com SOC operacional
simulado de 10% e mínimo de 30%, mantendo uma missão válida carregada. O pedido
`start` foi processado e recusado por bateria insuficiente. O rover permaneceu
em HOLD, desarmado, sem missão pendente; o deslocamento medido no Gazebo foi
0,000 m. A bateria reportada pelo autopiloto continuou separada, em 44%.

[Resultado e leitura de bateria](../../python/run/mission_manager_validation/series_20261110_104048/results.json).

## Diagnóstico de renderização do LiDAR

Um mundo mínimo com chão plano e sensor imóvel separou falhas de renderização
de erros do controlador. Contagem de pontos em oito setores iguais de azimute:

| Renderização | Pontos finitos | Pontos por setor em uma amostra |
| --- | ---: | --- |
| normal, servidor sem GUI | 2.384 | 298, 298, 298, 298, 298, 298, 298, 298 |
| EGL `--headless-rendering` | 948 | 297, 298, 55, 0, 0, 0, 0, 298 |

Os valores EGL variaram entre amostras e mantiveram setores ausentes. No cenário
com rover, a varredura normal retornou pontos nos quatro quadrantes, ao contrário
da execução EGL investigada. A correção foi separar `gui:=false` de
`headless_rendering:=true`: não abrir a GUI passou a manter o renderizador normal.
A máscara experimental do próprio rover foi retirada; não se apagaram paredes
nem colisões para fazer o docking passar.

Evidências: [mundo mínimo](../../python/run/mission_manager_validation/lidar_render_probe.sdf),
[leitor](../../python/run/mission_manager_validation/lidar_render_read.py),
[normal](../../python/run/mission_manager_validation/lidar_render_x11.txt),
[EGL](../../python/run/mission_manager_validation/lidar_render_egl.txt).
Essa conclusão vale para esta máquina/configuração, não para toda instalação EGL.

## Falhas encontradas durante o desenvolvimento

As tentativas que falharam não foram contadas como ciclos aprovados:

| Execuções | Resultado observado | Correção/decisão |
| --- | --- | --- |
| `trial2`, `trial3` | Telemetria ou velocidade/atitude antigas | Fluxos MAVLink a 10 Hz, mantendo verificação temporal |
| `trial4`, `trial6`, `trial7`, `trial8`, `trial10` | Poucos/nenhum ponto do container; entrada interrompida | Diagnóstico de setores do LiDAR e correção do caminho de renderização |
| `trial5` | Cancelamento de teste e estação atrás tratada como ultrapassagem | Permitir referência atrás apenas no controlador LiDAR externo, com giro antes do avanço |
| `trial11` | Inicialização antes do GPS 3D e nuvem registrada atrasada | Espera por GPS e prioridade da varredura atual |
| `trial14` | Entrada e recarga aprovadas; saída recusada antes de mover por estimativa atrasada | Ajuste com amostra limitada e menor concorrência de processamento visual |
| `trial15` | Saída/AUTO confirmados; retorno sem geometria suficiente quando voltado para longe | Giro externo de aquisição sem avanço, seguido de confirmação LiDAR |
| série 10:08, terceiro ciclo | Saída interrompida na confirmação após giro; dois estacionamentos anteriores aprovados | Verificação contínua do envelope externo e deslocamento limitado durante o giro, em lugar de exigir base quase fixa |
| série 10:05 | Parada por salto de pose durante giro, antes de entrar | Evitar retrocesso temporal na troca de fontes e comparar deslocamentos nos eixos da estação |
| série 09:57, ciclo 2 | Movimento concluído, mas duração negativa calculada pelo ensaio | Corrigido o pareamento dos estados; original preservado e repetição realizada |

Também houve tentativas de inicialização/ensaio sem status disponível ou durante
reconciliação e um pedido recusado por operação ainda ocupada. Os registros brutos
permanecem em `python/run/mission_manager_validation/series_*`. Mensagens
`Cancelado` ao encerrar um manager estacionado são encerramentos solicitados,
não falhas de um ciclo em movimento.

## Encerramento e estado preservado

Depois da campanha, uma saída adicional para encerrar a simulação foi
interrompida por uma varredura de proteção desatualizada. O manager confirmou
HOLD e **não liberou AUTO**. Após os dados voltarem, `reconcile` confirmou pelo
LiDAR que o rover já estava totalmente fora e liberou o intertravamento.
Esse comportamento foi registrado em
[trial22_finish/events.jsonl](../../python/run/mission_manager_validation/trial22_finish/events.jsonl).
Não foi contado como ciclo completo aprovado.

O rover foi desarmado, a missão original de **11 itens** foi restaurada e
conferida por download/hash, e os processos desta simulação de teste foram
encerrados. `DOCK_LOCK` ficou em 0 somente depois da confirmação externa.
[Confirmação da restauração](../../python/run/mission_manager_validation/restore/result.json).
A missão antiga mantém seus comandos originais; para usar o manager, prepare
uma missão compatível ou carregue o exemplo do tutorial.

## Limites da validação

- Sem hardware real, carga elétrica, contato de encaixe ou sensor de corrente do
  carregador. A recarga de 15 s altera apenas o SOC operacional simulado.
- A implantação atual do manager exige SITL. Não está liberada para uso direto
  no rover físico nem em firmware ArduPilot padrão sem `DOCK_LOCK`.
- O caminho externo calculado em torno da estação foi testado geometricamente;
  não foi realizada campanha de manobras a partir de todos os lados/obstáculos.
- Não foram forçadas falhas físicas de IMU/GPS, travamento completo do computador,
  ruído extremo ou todos os casos de perda de rede. Failsafes nativos continuam necessários.
- O aviso legado `FAST-LIO: Lua pose stale` apareceu intermitentemente. O manager
  verifica a odometria ROS usada na saída; a estabilidade temporal da ponte Lua
  ainda precisa de avaliação própria antes de depender dela em controle físico.
- Não foi automatizada a interface do QGroundControl. A missão foi carregada e
  confirmada pelo protocolo MAVLink via MAVProxy, o mesmo caminho do veículo.

Para repetir, siga o [tutorial completo](TUTORIAL.md). Para entender os módulos,
consulte a [arquitetura e relação de códigos](ARQUITETURA.md).


## Atualização de 10/11/2026 — início por arquivo e ciclo automático

Esta seção valida a revisão posterior ao fluxo que exigia `bind` separado.
Os ensaios anteriores acima permanecem como histórico, não substituem os desta revisão.

**174 testes de software passaram** (154 anteriores + 20 novos), incluindo:

- Importação `.waypoints`/`.plan`, rejeição de Takeoff, sequência inválida,
  NaN, pausa explícita e itens complexos não suportados.
- Preservação dos quatro parâmetros MAVLink e das coordenadas no envio;
  comparação de leitura de volta com tolerâncias de representação do protocolo.
- Associação automática, backup antes do upload e início tanto por arquivo
  quanto pela chamada Trigger antiga; estação física de saída preservada.
- Recusa antes de enviar/armar para arquivo inválido, destino inválido e
  bateria insuficiente; divergência no readback impede movimento.
- Saída antes de AUTO quando dentro, nenhum movimento de saída quando fora;
  pedido durante a recarga aguarda os 15 segundos. Arquivo inválido não a cancela.
- Atraso transitório de sensores na saída zera os comandos até obter amostra
  válida; atraso persistente e covariância inválida continuam interrompendo.

Compilação da interface `rover_mission_interfaces` e transporte real do serviço
`/mission_manager/start_file` também foram verificados. O carregamento do ambiente
foi corrigido para usar `install/local_setup.bash` sobre Jazzy, evitando reintroduzir
Noetic/catkin capturados anteriormente por colcon. `Clock` e `StartMission` foram
importados como tipos ROS 2 e o novo processo executou com esse ambiente.

### Ensaio real Gazebo + SITL por arquivo

Arquivo usado: [`missao_atual_docking.waypoints`](missao_atual_docking.waypoints),
exportado da missão do usuário: HOME + seis waypoints. O hash executável da missão
foi preservado: `1bd6bf288531a3e4e02306434265cab9aacb16b5c38b3be183423fc1817e7e45`.
Não houve comando manual de `bind` ou `dock` durante o ciclo aprovado.

| Verificação | Resultado |
| --- | --- |
| Início dentro da estação, HOLD e desarmado | Confirmado |
| Upload, leitura de volta e vínculo automático à estação | Confirmados |
| Saída GUIDED antes de liberar AUTO | Confirmada |
| Execução dos seis waypoints | Concluída |
| Entrada automática ao atingir o último waypoint | Confirmada |
| Parada final e desarmamento | HOLD, desarmado |
| SOC operacional final | 100% |
| Recarga por relógio monotônico, eventos internos | 15,012 s |
| Duração total observada do ciclo | 377,874 s |
| Erro longitudinal, medido pelo Gazebo | −0,02969 m |
| Erro lateral, medido pelo Gazebo | +0,00398 m |
| Erro de rumo, medido pelo Gazebo | −0,31905° |

A bateria MAVLink permaneceu independente (60% no final); a recarga simulada
não altera `BATTERY_STATUS` nem faz o indicador do QGroundControl subir a 100%.
O gerenciador atualizado ficou ativo, estacionado e desarmado ao final.

### Falhas e preparação do ensaio — não contabilizadas como sucesso

1. A primeira inicialização após recompilar a interface encontrou mistura de
   tipos ROS 1/ROS 2. Corrigido o carregamento do overlay local.
2. A primeira tentativa de saída parou em HOLD com `Timestamp FAST-LIO invalido`.
   A medição independente encontrou atrasos de 0,392 a 0,788 s nas amostras do
   FAST-LIO, próximos do limite de validade. Foi adicionada espera limitada com
   comandos zerados; os limites de validade não foram aumentados.
3. A reconciliação seguinte recusou o rover cerca de 16 cm atrás do centro.
   Isso não foi convertido artificialmente em “estacionado”. Um comando manual
   de preparação do ensaio reposicionou o rover lentamente até x=4,97564 m,
   y=−2,01380 m, antes de começar a rodada final. Essa preparação usou ground truth,
   está registrada à parte e **não faz parte do resultado do controle automático**.
4. O ciclo final recuperou sete atrasos transitórios da varredura durante a saída,
   zerando o movimento nessas ocasiões, sem liberar AUTO antecipadamente.

Houve **um ciclo completo aprovado nesta revisão**, além das falhas registradas;
não é uma afirmação de confiabilidade estatística nem validação em hardware real.
A espera por recarga antes de sair foi testada por software; o ciclo físico partiu
com a recarga inicial já concluída.

Evidências:

- [Resumo estruturado](../../python/run/mission_manager_validation/file_start_20261110_143455/summary.json).
- [Resultado do ciclo e transições](../../python/run/mission_manager_validation/file_start_20261110_143455/file_cycle_results.json).
- [Eventos da rodada aprovada](../../python/run/mission_manager_validation/file_start_20261110_143455/manager_final/events.jsonl).
- [Log dos 174 testes](../../python/run/mission_manager_validation/file_start_20261110_143455/unit_tests.log).
- [Preparação manual, excluída do ciclo](../../python/run/mission_manager_validation/file_start_20261110_143455/manual_test_setup/trace.json).
- [Primeira tentativa interrompida](../../python/run/mission_manager_validation/file_start_20261110_143455/manager/events.jsonl).

O ensaio antigo de bateria baixa do validador foi atualizado para exigir o rover
**fora** da estação em `ESPERA`; dentro, a nova versão inicia recarga automaticamente.
Esse caso físico de bateria baixa não foi repetido nesta rodada; a recusa foi
coberta pelos testes de software.


## Atualização — recarga no estimador nativo do SITL

Substitui o contador separado descrito nos ensaios históricos acima.
**183 testes de software passaram** (174 anteriores + 9 da recarga nativa).
Cobrem SOC inicial vindo de telemetria, consumo nativo, ausência de dados,
comando e confirmações, monitor incompatível, movimento/armamento/posição/modo
inválidos, ACK recusado e ausência de confirmação de 100% em SYS_STATUS.

O teste ao vivo foi feito com o rover já estacionado e desarmado. O novo manager
confirmou sua geometria e fez a recarga no próprio monitor do ArduPilot:

| Medida | Resultado |
| --- | --- |
| Monitor/capacidade | BATT_MONITOR=4, 3300 mAh |
| SOC inicial em BATTERY_STATUS | 39% |
| SOC final em BATTERY_STATUS | 100% |
| SOC final em SYS_STATUS | 100% |
| Consumo acumulado final | 0 mAh e 0 hJ |
| Tensão final | 12,600 V |
| Corrente final | 0 A |
| Duração incluindo confirmação | 15.389 s |
| Atualizações nativas confirmadas | 16 |
| Estado final | HOLD, desarmado, RECARGA_CONCLUIDA |

As mensagens finais foram conferidas por uma conexão independente do MAVProxy
na porta 14557. O ArduPilot distribui a mesma telemetria ao QGroundControl via
14550. A interface gráfica do QGroundControl não foi inspecionada visualmente;
foi confirmada a origem da carga nas duas mensagens MAVLink, sem falsificá-las.
Não foi repetido um ciclo completo de navegação nesta revisão: foi validada a
fase de recarga real do SITL, e os testes anteriores de navegação ficam históricos.

- [Resumo nativo](../../python/run/mission_manager_validation/native_battery_20261110_151946/summary.json).
- [Confirmação MAVLink independente](../../python/run/mission_manager_validation/native_battery_20261110_151946/independent_check/result.json).
- [Eventos e percentuais intermediários](../../python/run/mission_manager_validation/native_battery_20261110_151946/manager/events.jsonl).
- [183 testes de software](../../python/run/mission_manager_validation/native_battery_20261110_151946/unit_tests.log).


## Atualização — saída em ré de 3 m, sem giro prévio

O Mission Manager agora recua 3 m da posição estacionada, confirma a parada e
a posição externa e entrega o controle à missão AUTO. A etapa Giro180 e o
limite de deslocamento de 40 cm durante esse giro foram removidos deste fluxo.
Permanecem a varredura de obstáculos, o controle de alinhamento, a confirmação
LiDAR/FAST-LIO e a liberação de AUTO somente após a confirmação de saída.

**191 testes de software passaram**, incluindo 8 novos testes que executam
`Motion.exit` com medições sintéticas: alvo de 3 m, desaceleração perto do alvo,
nenhum comando de giro isolado, tolerância de parada, distância geométrica
insuficiente, ultrapassagem do alvo, rumo fora do limite, obstáculo e ausência
de solicitação. Os testes existentes verificam a ordem saída confirmada → AUTO.

O gerenciador foi reiniciado com o rover parado e desarmado. **Não foi executado
um novo recuo físico de 3 m no Gazebo nesta revisão**; os ciclos anteriores com
giro permanecem apenas como histórico. Não foi enviada nem iniciada outra missão.

- [Log dos testes](../../python/run/mission_manager_validation/reverse_3m_20261110_163441/unit_tests.log).
- [Estado após atualização](../../python/run/mission_manager_validation/reverse_3m_20261110_163441/validation.json).


## Diagnóstico e correções — 11/11/2026

Pedido investigado: `Missions/new_argo.plan`, recebido por `start_file` com
aceitação assíncrona, seguido de FALHA. O arquivo era válido. A primeira causa
foi a comparação direta de `/clock` com `ATTITUDE.time_boot_ms`: 74 amostras
mostraram diferença persistente de aproximadamente 2,85 s, embora os dados
continuassem chegando. O sincronizador TIMESYNC agora converte os horários;
o limite de idade de 0,5 s permanece. O início espera dados recentes antes de
alterar a missão, e uma recusa nessa etapa preserva a recarga/estado anterior.

Resultados intermediários, incluindo falhas:

| Ensaio | Resultado observado |
| --- | --- |
| Início com comparação direta dos relógios | Falha por atitude considerada antiga; sem movimento |
| Início após TIMESYNC, auxílio QR ativo | Duas tentativas recusadas na confirmação das paredes, antes de armar; nuvens com idade de 0,628 e 0,816 s |
| Início com `--no-qr` | Paredes confirmadas; começou o recuo mantendo alinhamento |
| Recuo comandado pela distância FAST-LIO | LIO 2,996 m, geometria 3,750 m; abortou em HOLD pela divergência |
| Verificação independente no Gazebo após esse recuo | x=1,243 m, y=-1,900 m; confirma distância física próxima de 3,75 m desde o container em x=5 m |

A saída foi corrigida para controlar distância pelas paredes medidas durante
todo o recuo. FAST-LIO permanece como verificação de movimento e alinhamento.
Não foi ampliada a tolerância de parada para aceitar o recuo excessivo.
Os testes de software cobrem também deriva longitudinal do LIO, ausência de
movimento, perda de alinhamento geométrico e salto de pose.

**210 testes de software passaram**. Esses testes não substituem os ensaios
físicos no Gazebo; os resultados desses ensaios estão registrados abaixo.

Evidências desta revisão:

- [Testes de software](../../python/run/mission_manager_validation/preflight_20261111_105603/unit_tests.log).
- [Amostras dos relógios](../../python/run/mission_manager_validation/preflight_20261111_105603/time_probe/samples.json).
- [Falha por divergência da distância](../../python/run/mission_manager_validation/preflight_20261111_105603/lio_distance_file_cycle_results.json).
- [Eventos após as correções](../../python/run/mission_manager_validation/preflight_20261111_105603/manager_geometry/events.jsonl).

### Ensaio do arquivo do usuário, partindo de fora

`new_argo.plan` foi enviado pelo serviço público, baixado novamente e conferido.
Foram concluídos os seis waypoints e a sequência automática alinhamento →
entrada → HOLD → desarmamento → recarga. Tempo observado: **219,0 s**;
recarga observada: **15,4 s**; SOC final nativo **100%**. Sem reposicionamento
manual durante esse ciclo.

A posição de referência independente do Gazebo confirmou erros finais de
**−0,71 cm longitudinal, −1,65 cm lateral e +0,033° de rumo** em relação ao
centro da estação. Este primeiro ciclo começou fora e não testa a saída.

- [Resultado do ciclo iniciado fora](../../python/run/mission_manager_validation/preflight_20261111_105603/outside_file_cycle_results.json).
- [Amostras de estado do ciclo](../../python/run/mission_manager_validation/preflight_20261111_105603/outside_file_cycle_status.json).

### Saída corrigida, partindo estacionado

O mesmo arquivo foi solicitado novamente, com rover dentro, HOLD, desarmado
e SOC 100%. A ré terminou com **2,962 m reais**, deslocamento lateral de
**1,48 cm** e variação de rumo de **−0,522°**, medidos independentemente no
Gazebo. A geometria LiDAR indicou 2,961 m; FAST-LIO indicou 2,239 m.
A parada foi governada pela geometria, sem estender o recuo para compensar
FAST-LIO. A missão AUTO começou somente após a confirmação externa e a parada.
Não houve etapa de giro entre a ré e AUTO.


O ciclo iniciado dentro também **passou**: saída em ré → seis waypoints →
alinhamento → entrada → desarmamento → recarga nativa. Tempo total observado:
**252,2 s**, com **15,2 s** de recarga. Erros finais independentes no Gazebo:
**−2,69 cm longitudinal, −2,34 cm lateral e −1,400° de rumo**. Estado final:
`RECARGA_CONCLUIDA`, `inside=true`, HOLD, desarmado, SOC **100%**, sem erro.
Nenhuma intervenção manual de posição foi feita durante o ciclo.

Os critérios de aprovação dos ciclos foram erros finais inferiores a 8 cm
por eixo e 6° de rumo, HOLD/desarmado, confirmação da estação e SOC nativo 100%.
No segundo ciclo também se exigiu ré real de 3 m ± 15 cm, desvio lateral menor
que 12 cm e variação de rumo menor que 12°. Os dois ensaios usaram `--no-qr`;
não validam operação com auxílio QR ligado sob a mesma carga de processamento.
A divergência observada na translação acumulada do FAST-LIO continua sendo
uma limitação da odometria; o controle de distância de saída passou a usar
as paredes, sem afirmar que o SLAM foi recalibrado.

- [Resultado do ciclo com saída](../../python/run/mission_manager_validation/preflight_20261111_105603/file_cycle_results.json).
- [Estados do ciclo com saída](../../python/run/mission_manager_validation/preflight_20261111_105603/file_cycle_status.json).
- [Programa de validação independente](../../python/run/mission_manager_validation/preflight_20261111_105603/validate_exit_cycle.py).
