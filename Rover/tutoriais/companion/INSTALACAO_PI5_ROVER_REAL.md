# Raspberry Pi 5 — preparar a mesma imagem para o rover real

Este procedimento reutiliza a instalação Docker do
[tutorial da simulação](INSTALACAO_PI5_SIMULACAO.md), em `rover@192.168.0.167`.
O software está incluído na imagem; conexão, calibração e ensaios físicos
dependem do autopiloto, sensores e carregador instalados no veículo.
Não é uma declaração de validação do rover real.

## 1. Camada de abstração: o que muda e o que permanece

O gerenciador usa a mesma interface MAVLink pelo MAVProxy e os mesmos tópicos
ROS normalizados de odometria e nuvem. `companion/stack.py` seleciona os
adaptadores pelo perfil. Não é necessário reescrever missão ou docking ao
trocar o Gazebo por sensores reais.

| Interface | Simulação | Rover físico |
| --- | --- | --- |
| Autopiloto | SITL → MAVProxy → UDP 14600 | Autopiloto → USB/serial → MAVProxy |
| LiDAR/IMU | Gazebo e adaptador C++ | Driver Livox/SDK2 e calibração medida |
| Aquisição por ponto | Adaptação específica do sensor simulado | Timestamps reais do Livox preservados |
| Relógio | `/clock`, `use_sim_time=true` | Relógio real, `use_sim_time=false` |
| Saída comum | `/fast_lio/odometry`, `/rover_slam_3d/local_cloud`, MAVLink | As mesmas interfaces |
| Supervisão | `/mission_manager/status`, serviços `start_file`, `cancel`, `reconcile` | Os mesmos nomes |
| Recarga | `MAV_CMD_BATTERY_RESET`, somente SITL | Telemetria da bateria real; sem reset artificial |
| Domínio ROS | 0 | 20 |

Isso torna a lógica comum independente da origem dos dados, mas não elimina
diferenças físicas de calibração, latência e atuação. O adaptador real atual
não integra um carregador elétrico: após docking/desarmamento, permanece
`ESTACIONADO`. Confirmar contato, comandar carregador e acompanhar sua conclusão
exige a integração com o hardware de carga. Não habilite `--simulate-charge`.

## 2. Instalar a base

Siga as etapas 1 e 2 do tutorial da simulação: instalar Docker/Compose,
ajustar buffers e executar `deploy.sh`. Não abra Gazebo/SITL neste fluxo.
O driver Livox e ROS 2 já estão dentro da imagem.

Na Raspberry, localize o autopiloto:

```bash
ssh rover@192.168.0.167
ls -l /dev/serial/by-id/
cd ~/rover_companion
```

Use o caminho persistente do dispositivo real. O arquivo Compose mapeia-o
para `/dev/ttyACM0` dentro do container, mesmo que o nome no host seja outro.
Para UART da placa, use seu dispositivo e ajuste baudrate/fiação; estes não
foram definidos nesta instalação.

O autopiloto precisa de firmware compatível com este projeto (`DOCK_LOCK`,
scripting/Lua e parâmetros de atuação). Não grave o binário SITL no autopiloto.
O firmware físico deve ser compilado para a placa correspondente, que ainda
precisa ser identificada. Preserve os parâmetros reais; não carregue
`nrover_sem_desvio.param` do simulador como configuração do veículo físico.

Os scripts Lua ficam no autopiloto, não no Docker. Na placa com scripting e
cartão SD, instale os arquivos atuais de `Rover/scripts` em `APM/scripts`,
incluindo `lidar_odometry.lua`, `rover-controlallocation.lua`, `system-start.lua`
e a pasta `modules`. Não copie as pastas de backup como scripts ativos.
Confira habilitação de scripting, funções/polaridades dos servos e a atuação
real conforme o [manual Lua](../lua/README.pt-BR.md), antes de autorizar movimento.

## 3. Preparar configuração e calibração

Na Raspberry, crie os arquivos a partir dos modelos **somente se ainda não
existirem**, preservando calibrações anteriores:

```bash
cd ~/rover_companion
test -e config/hardware.yaml || cp config/hardware.example.yaml config/hardware.yaml
test -e config/livox.yaml || cp config/livox.example.yaml config/livox.yaml
test -e config/mid360.json || cp config/mid360.example.json config/mid360.json
nano config/real.yaml
```

Mantenha `hardware_configured: false` até terminar as verificações. Configure:

- `mavproxy_master: /dev/ttyACM0` e `mavproxy_baud` conforme o autopiloto;
- `stations: /config/estacoes_reais.txt`, separando as estações físicas das simuladas;
- `calibration_file`, `livox_params` e `envelope_file` apontando para `/config/...`;
- `no_qr: true` e `obstacles: false` para o perfil inicial.

Edite `config/hardware.yaml` com medições reais:

- `base_to_imu`: posição da IMU em relação à base, metros, eixos ROS FLU;
- `lidar_to_imu_rotation` e `lidar_to_imu_translation`: transformação LiDAR → IMU;
- `imu_axes_aligned_to_base`: só use `true` se os eixos estiverem alinhados;
- `validated`: só use `true` após validar as medições/transformações.

Os zeros do modelo não são uma calibração. A implementação atual exige os
eixos da IMU alinhados à base; outra montagem requer adaptar as transformações.
O driver não deve aplicar uma segunda vez os extrínsecos já usados pelo FAST-LIO.

Crie `config/envelope.json` com as dimensões reais, em metros. Estrutura:

```json
{
  "length": 0.60,
  "width": 0.54,
  "height": 0.65,
  "margin": 0.05,
  "base_to_center_x": 0.0,
  "base_to_center_y": 0.0
}
```

**Valores acima são ilustrativos, não uma medição do seu rover.** Ajuste-os
antes de usar. Cadastre as estações reais em `config/estacoes_reais.txt`:

```text
# NOME=lat_4m,lon_4m,lat_2m,lon_2m,QR,comprimento_m,largura_m,altura_m,alvo_ate_qr_m
```

P4 e P2 ficam a 4 m e 2 m da abertura, do lado de fora. Use coordenadas GPS
medidas e dimensões internas reais. Veja o [formato do cadastro](../docking/CADASTRO_ESTACOES.md).
Não copie as coordenadas `container_gravel_plane` para uma estação física.

## 4. MID360 e relógios

Edite `config/mid360.json`: IP do sensor e os IPs do host para receber nuvem,
IMU e controle devem corresponder à interface Ethernet ligada ao MID360.
Os endereços `192.168.1.x` do modelo são exemplos. A interface do sensor pode
ser distinta da interface usada para SSH/ROS com o computador.
Nenhuma configuração de IP foi aplicada automaticamente nesta instalação.

`config/livox.yaml` deve manter `xfer_format: 1` para `CustomMsg`, apontar para
`/config/mid360.json` e publicar `/livox/lidar` e `/livox/imu`.
Configure sincronização de horário na Pi e no computador; confirme timestamp
dos sensores coerente com ROS. Não publique `/clock` do Gazebo no domínio físico.

## 5. Conferir sem iniciar controle

Depois de preencher os arquivos, valide a calibração dentro da imagem:

```bash
cd ~/rover_companion
docker run --rm -v "$PWD/config:/config:ro" rover-companion:jazzy \
  python -c 'from common.hardware import calibration; calibration("/config/hardware.yaml"); print("Formato da calibracao OK")'
```

Essa validação confere formato, finitude, rotação e declaração de alinhamento;
não substitui medir o hardware. Para verificar apenas o driver, sem iniciar
o gerenciador, com o container principal parado:

```bash
docker run --rm --network host -e ROS_DOMAIN_ID=20 \
  -v "$PWD/config:/config:ro" rover-companion:jazzy \
  ros2 run livox_ros_driver2 livox_ros_driver2_node \
  --ros-args --params-file /config/livox.yaml
```

Em outro SSH na Pi, confirme mensagens:

```bash
docker run --rm --network host -e ROS_DOMAIN_ID=20 rover-companion:jazzy \
  ros2 topic hz /livox/lidar
```

Repita para `/livox/imu`. Encerre esses dois testes com Ctrl+C antes de abrir
o container principal. Depois dos testes de hardware/calibração, altere
`hardware_configured: true` em `config/real.yaml`.

## 6. Iniciar o perfil real

Na Raspberry, com veículo parado e após comissionamento:

```bash
cd ~/rover_companion
docker compose down
export ARDUPILOT_DEVICE=/dev/serial/by-id/SEU_AUTOPILOTO
docker compose -f compose.yaml -f compose.real.yaml up -d
bash source/companion/logs.sh
```

Substitua `SEU_AUTOPILOTO` pelo nome encontrado. O domínio físico padrão é 20.
Neste fluxo use o Compose com os **dois arquivos**; `companion/control.sh up`
do computador inicia o perfil de simulação e não deve ser usado para hardware.
O gerenciador só abre após odometria/nuvem recentes; abrir não solicita missão,
mas a reconciliação comunica-se com o autopiloto e confirma HOLD.

## 7. Operar pelo computador

Em cada terminal de operação:

```bash
cd ~/ardupilot/Rover
export ROVER_REMOTE=rover@192.168.0.167
export ROS_DOMAIN_ID=20
source companion/pc_env.bash
./companion/control.sh status
```

Para RViz, use relógio real:

```bash
rviz2 -d ros2/config/mid360_3d.rviz --ros-args -p use_sim_time:=false
```

Após validação dos sensores e do controle, envie uma missão **do local real**:

```bash
./companion/control.sh mission /caminho/da/missao_real.plan
./companion/control.sh watch
```

Não use `Missions/new_argo.plan`, que pertence ao cenário simulado. Os serviços
são iguais; as coordenadas, dimensões, margens e firmware precisam ser reais.
O perfil atual não inicia câmera/QR nem desvio. Habilitá-los exige drivers,
calibrações e validação próprios, mesmo com seu código incluído na imagem.

O perfil real não envia automaticamente MAVLink para o QGroundControl deste
computador: os `--out` do MAVProxy atendem os serviços internos. Para QGroundControl,
use o enlace GCS do autopiloto ou configure uma saída adicional para o IP/porta
do computador. Isso é independente dos comandos ROS de status/missão acima.

## 8. Encerrar e voltar à simulação

No computador, ainda no domínio 20:

```bash
./companion/control.sh cancel
./companion/control.sh status
```

Confirme HOLD, parada e desarmamento. Na Raspberry:

```bash
cd ~/rover_companion
docker compose -f compose.yaml -f compose.real.yaml down
```

Feche RViz e clientes. Para simular depois, use domínio 0 e o procedimento
do primeiro tutorial. A recuperação de parâmetros fica separada em
`data/real` e `data/simulation`; não transfira EEPROM ou recuperação do SITL
para o autopiloto físico.
