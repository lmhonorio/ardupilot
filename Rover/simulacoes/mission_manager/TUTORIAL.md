# Tutorial — missão, docking LiDAR/QR, recarga e saída no ROS 2

> Para executar os serviços de bordo na Raspberry e manter Gazebo/SITL/RViz
> neste computador, use o [tutorial do container remoto](../companion/RASPBERRY_DOCKER.md).
> Nesse fluxo, envie missões com `./companion/control.sh mission ARQUIVO`;
> os caminhos passados diretamente a `start_file` são caminhos dentro do container.


Este fluxo usa **Mission Manager**. Ele substitui o acionamento simultâneo das
rotinas independentes `docking-qr`, `docking-lidar`, `sair-qr` e `sair-lidar`.
Use um controlador por vez. Consulte o [relatório de testes](TESTES.md) para
saber quais casos foram medidos no Gazebo e quais foram testes de software.

## Início por arquivo (fluxo recomendado)

Com Gazebo, SITL, FAST-LIO e mapa já funcionando, mantenha o gerenciador aberto:

```bash
cd ~/ardupilot/Rover
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
./ros2/service.sh mission-manager
```

Em outro terminal, envie o arquivo e solicite o ciclo completo:

```bash
cd ~/ardupilot/Rover
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
./ros2/service.sh mission-start /caminho/absoluto/minha_missao.waypoints
```

Aceita também `.plan` do QGroundControl com itens simples. Substitua o caminho
pelo seu arquivo. O comando substitui a missão do veículo, salva um backup,
confere o readback e **associa automaticamente o último waypoint à estação**.
Não é mais necessário chamar `bind` antes de iniciar pelo gerenciador.
Se já estiver dentro, espera terminar a recarga, recua 3 m, confirma a saída
e inicia AUTO, sem giro de saída programado. Ao terminar a missão, entra, desarma e simula a recarga por 15 s.
O último waypoint deve estar diante da abertura (P2/P4), a menos de 8 m do centro;
o programa não inventa uma rota de retorno se o arquivo terminar em outro local.

A missão de 6 waypoints que estava no rover foi salva nesta atualização.
Para repetir essa mesma rota usando o novo fluxo:

```bash
cd ~/ardupilot/Rover
./ros2/service.sh mission-start "$PWD/simulacoes/mission_manager/missao_atual_docking.waypoints"
```

**Atualização de uma instalação existente:** compile a nova interface uma vez
(não precisa recompilar o firmware por causa desta interface):

```bash
cd ~/ardupilot/Rover/ros2
source setup.bash
export PATH="/usr/bin:/bin:$PATH"
colcon build --merge-install --base-paths src/rover_mission_interfaces \
  --packages-select rover_mission_interfaces \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3
```

Reinicie somente o processo antigo do Mission Manager, com o rover parado em
HOLD. Ele não recarrega o código Python automaticamente. Em cada terminal ROS
já aberto, execute novamente `source ~/ardupilot/Rover/ros2/setup.bash`.
A compilação completa `ros2/build.sh` também inclui essa interface.

## 1. Preparar o firmware desta revisão

O bloqueio de AUTO dentro da estação exige o novo parâmetro `DOCK_LOCK` desta
base. Não basta instalar os arquivos Python sobre um firmware antigo.
Com o SITL anterior encerrado:

```bash
cd ~/ardupilot
/home/grin/.pyenv/versions/3.8.10/bin/python3.8 ./waf rover -j4
```

O Python 3.8 é usado **apenas para o Waf desta árvore**, que depende de `imp`.
Os serviços ROS 2 Jazzy usam o Python do ambiente `ros2/setup.bash` (Python 3.12).
Não execute o manager com `/usr/bin/python` de outro ambiente ou o Python 3.8.
Não apague o EEPROM nem os arquivos de parâmetros para realizar a atualização.

## 2. Conferir o cadastro e o espaço físico

```bash
cd ~/ardupilot/Rover
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
source ros2/setup.bash
python python/common/docking_stations.py
```

O arquivo [`estacoes_recarga.txt`](../../estacoes_recarga.txt) define nome, dois
GPS externos a 4 m e 2 m da **abertura**, conteúdo do QR e dimensões internas.
A descrição está no [cadastro das estações](../docking/CADASTRO_ESTACOES.md).
A estação atual é `container_gravel_plane`: centro Gazebo `(5, -2)`, abertura
em `(4,5, -2)`, rumo de entrada leste. O rover precisa caber com margem.

O envelope conservador padrão é 0,60 × 0,54 × 0,65 m. Para outro rover, um JSON
pode configurar `length`, `width`, `height`, `margin`, `base_to_center_x` e
`base_to_center_y`, em metros. Passe-o com `--envelope /caminho/envelope.json`.
Não reduza o envelope apenas para fazer a checagem aceitar uma estação estreita.

## 3. Terminal 1 — Gazebo

Para o cenário conhecido, iniciado na região frontal:

```bash
cd ~/ardupilot/Rover
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
./ros2/service.sh gazebo \
  world:=gravel_plane N:=1 start_x:=2.5 start_y:=-2 \
  mid360_enabled:=true samsung_s23_ultra_cam_enabled:=true \
  insta360_enabled:=false hokuyo_enabled:=false
```

Para não abrir a janela gráfica, acrescente `gui:=false`. Essa opção mantém o
renderizador normal usando `DISPLAY`. Nesta máquina, o caminho EGL
`headless_rendering:=true` perdeu setores da varredura de 360°; por isso não é o
padrão. Para servidor sem monitor, prepare um servidor X/Xvfb e valide a nuvem.
Não confunda a câmera
com a vista livre do Gazebo. O docking usa a câmera embarcada, com calibração e
juntas medidas. LiDAR e IMU são obrigatórios; o QR é complementar.

## 4. Terminal 2 — ArduRover, Lua e MAVProxy

```bash
cd ~/ardupilot/Rover
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
./sim_1_gzrover.sh --foreground --add-param-file nrover_sem_desvio.param --no-rebuild
```

A versão compilada no passo 1 será usada. O arquivo de parâmetros selecionado
continua sendo aplicado normalmente. O Lua mantém a alocação aos motores.
O QGroundControl usa UDP 14550; o manager recebe MAVLink pelo MAVProxy em 14551.
A taxa inicial do MAVProxy é 10 Hz para a supervisão em simuladores lentos.

Aguarde GPS/EKF prontos. Não force armamento nem desative os pre-arm checks.
Para usar o perfil com desvio durante a navegação, selecione `nroverobst.param`
e inicie também a ponte de obstáculos do projeto, conforme o
[tutorial de serviços separados](../TUTORIAIS_SERVICOS_SEPARADOS.md).
A ponte precisa entregar dados válidos; `PRX1: No Data` não é resolvido ignorando
as verificações. Dentro do docking, o manager aplica temporariamente o perfil
adequado à estação estreita e verifica a faixa de movimento pela nuvem 3D.

## 5. Terminal 3 — FAST-LIO

```bash
cd ~/ardupilot/Rover
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
./ros2/service.sh fastlio
```

Mantenha o rover parado durante a inicialização. Espere `poses` aumentar.
Um tópico anunciado sem mensagens ou `poses=0` não confirma funcionamento.

## 6. Terminal 4 — mapa e nuvem local

```bash
cd ~/ardupilot/Rover
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
./ros2/service.sh mapa --wall-timeout 3
```

O programa fornece:

- `/rover_slam_3d/map`: mapa acumulado para visualizar;
- `/rover_slam_3d/local_cloud`: geometria recente de apoio ao docking.

O ajuste prioriza a varredura atual `/rover_argo_N1/Instance1/livox/lidar`,
reduzindo a latência; usa a nuvem local registrada quando uma varredura não
contém superfícies suficientes. FAST-LIO continua necessário para a saída.
Não substitua a nuvem local pelo mapa acumulado. Para visualizar, opcionalmente:

```bash
cd ~/ardupilot/Rover
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
./ros2/service.sh rviz
```

## 7. Terminal 5 — Mission Manager

```bash
cd ~/ardupilot/Rover
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
./ros2/service.sh mission-manager --simulate-charge
```

O manager conecta ao SITL, verifica `DOCK_LOCK`, reconcilia a posição e informa
`ESPERA` se estiver fora. Se confirmar que já está no alvo, desarmado,
passa por `ESTACIONADO` e inicia `RECARGA` automaticamente.
Não move o rover só por abrir o programa. A reconciliação faz uma parada em HOLD.
Inicie o manager com o rover parado.

`--simulate-charge` é o padrão e agora atualiza a **bateria do próprio SITL**
com `MAV_CMD_BATTERY_RESET`. O percentual inicial vem de `BATTERY_STATUS`,
e o mínimo para iniciar missão é 30%. Reiniciar o gerenciador não inventa
uma carga inicial. `--initial-soc` e `--discharge-rate` são argumentos legados:
não controlam o SOC no modo integrado; o consumo vem do ArduPilot.

```bash
./ros2/service.sh mission-manager --simulate-charge --minimum-soc 30
```

Use `--no-simulate-charge` para desativar a recarga automática. A configuração
validada é `BATT_MONITOR=4`, `BATT_CAPACITY=3300`, `SIM_BATT_CAP_AH=0` e
`SIM_BATT_VOLTAGE=12.6`. O gerenciador consulta e verifica esses parâmetros,
sem substituir configurações incompatíveis silenciosamente.

Em uma simulação muito lenta, aumente os prazos de movimento, mantendo os
watchdogs de entrega de sensores:

```bash
./ros2/service.sh mission-manager --simulate-charge \
  --docking-timeout 600 --phase-timeout 300 --exit-timeout 600
```

Para reduzir a carga de processamento, execute usando somente a localização LiDAR:

```bash
./ros2/service.sh mission-manager --no-qr
```

O QR é um auxílio opcional; LiDAR continua sendo obrigatório. No teste de
11/11/2026, desativar o auxílio do QR permitiu confirmar as paredes dentro do
limite de idade de 0,6 s, que foi mantido. Execute apenas um gerenciador por vez.
`--station container_gravel_plane` restringe o destino de docking (as demais
estações cadastradas continuam sendo consideradas para detectar ocupação); `--estacoes`
permite selecionar outro cadastro TXT. Veja todas as opções com `--help`.

## 8. Terminal 6 — supervisão e comandos

Prepare cada terminal de comandos:

```bash
cd ~/ardupilot/Rover
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
source ros2/setup.bash
```

Veja o estado:

```bash
ros2 topic echo /mission_manager/status --full-length
```

O JSON inclui estado, modo, armamento, estação, posição, waypoint, missão
pendente, hash da missão, situação do LiDAR e bateria. A bateria contém
`real_soc`, `simulated_soc`, `operational_soc` e `source`. No modo integrado,
`source` é `sitl_battery_status` e os três percentuais refletem a carga confirmada
pelo ArduPilot. `charge_target_soc` indica o alvo temporário durante a recarga.
`DOCKING`, `ESTACIONADO` e `RECARGA` são estados da aplicação, não novos flight
modes do ArduPilot. O QGroundControl mostra GUIDED durante o movimento e HOLD
quando parado; os detalhes do ciclo aparecem no tópico ROS.
O QGroundControl acompanha a recarga porque o **próprio ArduPilot** atualiza
`BATTERY_STATUS` e `SYS_STATUS`. O Python não publica pacotes falsos de bateria.
A recarga só é concluída após confirmação de 100% nessas duas mensagens.
`last_request` mostra o resultado do processamento do pedido e `last_error`
explica recusas/falhas. `concluido` em `last_request` significa que o pedido foi
processado; a conclusão do ciclo inteiro é o estado `RECARGA_CONCLUIDA`.

Para ver os serviços:

```bash
ros2 service list | grep /mission_manager/
```

## 9. Enviar o arquivo e executar missão + docking + recarga

1. Crie uma missão de Rover no QGroundControl e salve como `.plan`, ou exporte
   `QGC WPL 110` (`.waypoints`). O primeiro item executável deve ser **Waypoint**.
   Takeoff (22), paraquedas (208), NAV_GUIDED_ENABLE (92), relés, saltos e scripts
   não são aceitos. Itens complexos/survey precisam ser exportados como waypoints.
2. Termine com um **Waypoint** no P2 ou P4 da estação, do lado da abertura.
3. Deixe o rover parado em **HOLD**. Se estiver estacionado, o próprio gerenciador
   já mantém esse modo. Não selecione AUTO manualmente dentro do contêiner.
4. Solicite o ciclo com o arquivo:

```bash
cd ~/ardupilot/Rover
./ros2/service.sh mission-start /caminho/absoluto/minha_missao.plan
```

Exemplo curto fornecido (vai ao ponto P4 e retorna para docking):

```bash
./ros2/service.sh mission-start "$PWD/simulacoes/mission_manager/missao_exemplo.waypoints"
```

O gerenciador valida o arquivo e a estação antes do upload, salva a missão
anterior, envia pelo MAVProxy, compara o readback e vincula a versão devolvida
pelo ArduPilot. Preserva parâmetros dos itens, incluindo velocidade e aceitação;
metadados do plano como `cruiseSpeed`, geofence e rally não são enviados como
parâmetros do veículo. Use DO_CHANGE_SPEED nos itens para definir velocidade.
Campos numéricos nulos/NaN são recusados com diagnóstico.

O mesmo pedido diretamente como serviço ROS 2 é:

```bash
ros2 service call /mission_manager/start_file rover_mission_interfaces/srv/StartMission \
  "{mission_file: '/home/grin/ardupilot/Rover/simulacoes/mission_manager/missao_exemplo.waypoints'}"
```

O caminho é lido **no computador do gerenciador**. `success=True` significa
recebido na fila; confira `last_request`, `last_error`, `binding` e `state`:

```bash
ros2 topic echo /mission_manager/status --full-length
```

Arquivo inválido é rejeitado antes de alterar a missão e não cancela uma recarga
em curso. A verificação de parada aguarda telemetria recente por até 5 s;
se faltar informação, recusa o pedido antes do upload, sem colocar o gerenciador
em FALHA. O prazo acompanha `--mode-timeout` quando configurado. Ao consultar
`/mission_manager/status`, mantenha `--full-length` para visualizar o diagnóstico. Falha de upload, leitura de volta ou movimento entra em `FALHA`/HOLD.
Não envie outra missão pelo QGroundControl durante a operação: a alteração
invalida o vínculo e pode bloquear a execução.

Para a missão que já está carregada no veículo, a chamada antiga continua válida
**e também faz o vínculo automaticamente**:

```bash
ros2 service call /mission_manager/start std_srvs/srv/Trigger '{}'
```

`bind` permanece disponível para associação explícita, mas não é mais pré-requisito
para `start` ou `start_file`. Ao confirmar a chegada ao último waypoint vinculado,
o gerenciador seleciona GUIDED e inicia docking. Não adicione “Set flight mode”.

## 10. Acompanhar entrada e recarga

O fluxo esperado é:

```text
MISSAO → DOCKING_SOLICITADO → ALINHAMENTO → APROXIMACAO
       → ESTACIONADO → RECARGA → RECARGA_CONCLUIDA
```

O LiDAR ajusta as superfícies do contêiner. O controlador primeiro alcança o
eixo externo, ajusta o rumo e só então avança. Se estiver voltado para longe e
a geometria não for visível, pode fazer um giro de aquisição sem avançar,
somente na região frontal externa com margem e varredura recente. Depois do
giro, ainda exige a confirmação das paredes pelo LiDAR. QR ausente não impede continuar;
QR inconsistente não substitui o LiDAR. Obstáculo, perda do LiDAR ou geometria
insegura provocam HOLD e falha registrada.

A conclusão exige observações geométricas estáveis, envelope dentro da estação,
HOLD, parada e desarmamento confirmados. Depois, com `--simulate-charge`, o
estimador de carga da bateria 0 do SITL é atualizado aproximadamente a cada
segundo, até 100% em **15 segundos reais**, mais o tempo de confirmação MAVLink.
A indicação também sobe no QGroundControl. Não há modelo eletroquímico nem
confirmação de contato elétrico; tensão e corrente continuam calculadas pelo SITL.
As verificações exigem SITL identificado, geometria estacionada, HOLD, rover
parado e desarmado. Falta de ACK ou de confirmação da carga gera falha.

Para um teste manual de redefinição da carga, no **console MAVProxy** (não Bash):

```text
long BATTERY_RESET 1 100
```

`1` é a máscara da primeira bateria; `100` é o percentual solicitado. Esse comando
manual é imediato, não executa a rampa de recarga nem as verificações do gerenciador.
Não é necessário executá-lo para o ciclo automático.

**Após 100%, permanece estacionado.** Mudar manualmente para GUIDED também não
solicita saída.

## 11. Solicitar nova missão quando está dentro

Use o mesmo comando com o próximo arquivo, mantendo o gerenciador aberto:

```bash
./ros2/service.sh mission-start /caminho/absoluto/proxima_missao.waypoints
```

O programa detecta a ocupação da estação pelos sensores, não pelo nome do arquivo.

Com bateria e missão válidas, o fluxo é:

```text
PREPARAR_MISSAO → GUIDED confirmado → SAIDA
  → ré de 3 m → parada e confirmação de envelope fora
  → PREPARAR_MISSAO → AUTO confirmado → MISSAO
```

Os 3 m são medidos a partir da posição estacionada, diretamente pela geometria
das paredes no LiDAR. FAST-LIO verifica movimento e alinhamento; sua translação
acumulada não define o ponto de parada, pois pode apresentar deriva.
O controlador desacelera perto do alvo;
a verificação final admite 15 cm de tolerância e mantém os limites de
alinhamento e detecção de obstáculos. Não há giro programado de 180° ou 360°
entre a ré e a missão. Em AUTO, o navegador pode fazer curvas para alcançar
o primeiro waypoint.

Não use o botão de AUTO como pedido de saída: dentro do docking o firmware
rejeita AUTO com `Docking: AUTO locked; request guided exit`. Use `mission-start` ou o serviço
`start_file`. O modo GUIDED só autoriza a saída quando existe pedido válido pendente.
Se a saída falhar, AUTO continua bloqueado.

Uma solicitação recebida durante a recarga aguarda sua conclusão. Bateria
insuficiente fora dessa fase recusa o início; não altera os checks do veículo.
Cada novo início seleciona seq 1; não é uma função de retomar do waypoint anterior.

## 12. Entrada manual pelo gerenciador

Para testar docking sem executar uma missão, posicione o rover na frente da
estação, dentro dos 8 m, e arme normalmente pelo MAVProxy/QGroundControl. Com
o manager em `ESPERA`, peça:

```bash
ros2 service call /mission_manager/dock std_srvs/srv/Trigger '{}'
```

Esse pedido não cria missão. O manager confirma GUIDED, utiliza a percepção e
executa o mesmo fluxo de entrada. Não execute `entrar_container.py` em paralelo.

## 13. Cancelar, recuperar e encerrar

```bash
ros2 service call /mission_manager/cancel std_srvs/srv/Trigger '{}'
```

A resposta confirma o pedido de cancelamento, não a parada física. Confira HOLD,
velocidade zero e o registro do resultado. Corrija a causa antes de reconciliar:

```bash
ros2 service call /mission_manager/reconcile std_srvs/srv/Trigger '{}'
```

A reconciliação consulta sensores e autopiloto. Se o rover estiver parcialmente
encaixado ou a geometria não for confiável, ela mantém o bloqueio e falha; não
inventa um estado “fora”. O operador pode intervir em HOLD/GUIDED com supervisão.
Não zere `DOCK_LOCK` dentro da estação.

Para encerrar, cancele o movimento, confirme HOLD e pressione Ctrl+C no manager;
depois encerre mapa, FAST-LIO, SITL e Gazebo nos respectivos terminais. Não remova
arquivos `.lock`: o bloqueio pertence ao processo, não ao conteúdo do arquivo.

## 14. Diagnóstico

| Sintoma | Verificar |
| --- | --- |
| `DOCK_LOCK` não existe | Recompile e reinicie o SITL com o binário desta revisão |
| AUTO recusado dentro | Comportamento esperado; solicite `start` pelo manager |
| `LiDAR` ausente/sem paredes | MID360, FAST-LIO, nuvem local recente, abertura visível ao sensor e `poses` aumentando |
| Sem QR | Pode continuar pelo LiDAR; verifique a câmera se quiser auxílio visual |
| Missão alterada / hash diferente | Em HOLD, envie novamente o arquivo com `mission-start` |
| Sem recarga no QGroundControl | Confirme manager atualizado, `source=sitl_battery_status`, ACK de BATTERY_RESET e BATTERY_STATUS/SYS_STATUS; a versão anterior só atualizava um contador separado |
| `start_file` indisponível | Compile a interface e reinicie o Mission Manager atualizado |
| Serviço aceitou mas não moveu | Veja estado/eventos; aceite da solicitação não equivale à operação concluída |
| Serviço já em execução | Encerre o controlador antigo antes de abrir o manager |
| FAST-LIO sem nó/poses, apesar de IMU/nuvem | Verifique processos suspensos e DDS; não prossiga com geometria ausente |
| Telemetria de movimento desatualizada | Veja `clock_sync` e `last_error`. O manager sincroniza o relógio de boot do ArduPilot com ROS via TIMESYNC; dados recentes ainda precisam ter idade corrigida ≤ 0,5 s |
| FALHA mesmo em HOLD/desarmado | HOLD é modo do veículo; FALHA é estado do gerenciador. Corrija `last_error` e use `reconcile` antes de reenviar |

Nesta máquina foi observada uma inicialização presa no transporte compartilhado
Fast DDS após processos antigos suspensos. Os comandos deste tutorial já
selecionam o transporte usado nos ensaios. Em cada terminal novo, mantenha:

```bash
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
```

Isso foi usado nos ensaios para evitar o transporte SHM preso. Não substitui a
verificação de mensagens e geometria.

## 15. Registros e repetição dos testes

Cada execução grava `manifest.json` (argumentos e hashes do código/firmware),
`events.jsonl`, `telemetry.jsonl` e, após associação automática ou `bind`,
`mission_binding.json` em `python/run/mission_manager/<data_hora>/`. As alterações
temporárias também têm registro de recuperação em
`ros2/run/mission_manager_recovery.json`.

Testes de software, sem movimento:

```bash
cd ~/ardupilot/Rover
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
source ros2/setup.bash
python -m unittest discover -s python/tests -p 'test_*.py'
```

O ensaio explícito de integração altera a missão simulada e movimenta o rover;
salva a missão anterior em `mission_backup.json`. Execute somente com SITL,
manager e sensores preparados:

```bash
python python/tests/validate_mission_manager_sitl.py --execute --cycles 2
```

O [relatório de testes](TESTES.md) registra os resultados realmente obtidos.
A [arquitetura](ARQUITETURA.md) explica decisões, módulos e limitações de hardware.
