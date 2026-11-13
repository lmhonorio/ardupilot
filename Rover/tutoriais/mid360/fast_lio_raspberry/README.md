> Código deste componente: `Rover/mid360/fast_lio_raspberry/`. Execute os comandos a partir de `~/ardupilot/Rover`; caminhos `run/` e `ws/` deste documento se referem ao diretório do código.

# MID360 físico: FAST-LIO na Raspberry e RViz no PC

Teste independente da simulação, do autopiloto, do docking e do gerenciador de
missões. Os comandos abaixo são executados **no PC**, em `~/ardupilot/Rover`.

## Divisão entre as máquinas

```text
MID360 físico ── Ethernet ── Raspberry Pi 5
                             driver Livox + adaptador de tempo + FAST-LIO
                                             │
                             nuvem registrada + posição + trajetória
                                             │ SSH
                                             ▼
                                      PC: ponte + RViz
                                      mapa visual acumulado
```

Na Raspberry, os dados do lidar e da IMU vão diretamente ao FAST-LIO, sem passar
pelo PC. O adaptador preserva o tempo de cada ponto; não é uma conversão para
o RViz. A representação visual acumulada é montada no PC a partir dos pontos já
localizados pelo FAST-LIO. Não há outro estimador de posição no PC.

## Pré-requisitos desta instalação

- Raspberry `rover@192.168.0.167`, acesso SSH por chave, Docker e imagem
  `rover-companion:jazzy` já instalados.
- MID360 serial `47MCNAH0037261`, IP `192.168.1.161`, alimentado externamente
  e conectado à mesma rede Ethernet da Raspberry.
- A Raspberry usa também `192.168.1.50/24` na `eth0` para alcançar o sensor.
  `iniciar.sh` prepara esse endereço secundário sem mudar a rota padrão.
- PC com ROS 2 Jazzy, RViz e o ambiente `Rover/ros2/setup.bash` disponível.
  Não use o Python 3.8 neste teste Jazzy; os scripts selecionam o ambiente
  Python compatível com o ROS 2 instalado.

É reutilizada a imagem Docker já existente, mas os serviços de controle daquela
imagem **não são iniciados**. O teste usa comandos e containers próprios.

## Iniciar tudo

Mantenha o sensor parado durante a inicialização. No PC:

```bash
cd ~/ardupilot/Rover
./mid360/fast_lio_raspberry/iniciar.sh
```

O comando inicia o driver, o adaptador e o FAST-LIO na Raspberry e abre a
visualização no PC. Na primeira execução, compila somente o adaptador ARM64
se ele ainda não existir. O FAST-LIO utiliza o binário já instalado na imagem.

Para iniciar a Raspberry sem abrir o RViz:

```bash
./mid360/fast_lio_raspberry/iniciar.sh --sem-rviz
```

Para abrir/reabrir a visualização sem reiniciar o FAST-LIO:

```bash
./mid360/fast_lio_raspberry/visualizar.sh
```

Fechar o RViz ou pressionar `Ctrl+C` nesse terminal encerra somente a ponte e
a visualização no PC. O FAST-LIO continua na Raspberry. Se a conexão SSH cair,
abra novamente `visualizar.sh`; o mapa visual do PC recomeça vazio, mas a origem
do FAST-LIO permanece enquanto o estimador continuar rodando.

## Usar o RViz

- Fixed Frame: `camera_init`.
- Ferramenta **Move Camera**, vista **Orbit**.
- Botão esquerdo e arrastar: girar a vista; roda do mouse: aproximar/afastar;
  botão do meio e arrastar: deslocar a vista.
- `FAST-LIO - nuvem registrada`: pontos atuais, com 1 segundo de persistência.
- `Mapa acumulado no PC`: pontos acumulados com voxel de 10 cm, atualizado
  aproximadamente uma vez por segundo. Limite de 300 mil voxels; ao atingir o
  limite, descarta os menos recentemente observados.
- `Trajetoria FAST-LIO`: últimas 3 mil poses enviadas para visualização.

O mapa visual não é um mapa com fechamento de laços. O mapa interno usado pelo
FAST-LIO para localizar o sensor continua na Raspberry. Desabilitamos apenas
a publicação nativa do mapa crescente para evitar acumulação ilimitada de
memória e retransmissão de todo esse mapa pela rede.

## Parar o teste completo

No PC:

```bash
cd ~/ardupilot/Rover
./mid360/fast_lio_raspberry/parar.sh
```

Encerra a visualização deste teste e remove somente os containers
`rover-mid360-fastlio`, `rover-mid360-adapter` e `rover-mid360`.
Não encerra o container `rover-companion`, simulação ou outros serviços.
O endereço secundário temporário da Raspberry permanece. Para remover também
esse endereço, depois de parar:

```bash
./mid360/control.sh network-down
```

## Conferir funcionamento

```bash
ssh rover@192.168.0.167 \
  'cd ~/mid360/fast_lio_raspberry && docker compose ps'

ssh rover@192.168.0.167 \
  'docker logs --tail 30 rover-mid360-fastlio'

ssh rover@192.168.0.167 \
  'docker exec rover-mid360-fastlio /opt/rover/Rover/companion/entrypoint.sh python /test/verificar.py --seconds 30 --stationary'
```

O último comando pressupõe sensor parado. Mede entrada de nuvem e IMU, saída
de odometria, nuvem registrada e variação da posição. Remova `--stationary`
para medir apenas recepção/consistência durante movimento.

Para inspecionar os tópicos recebidos **no PC**:

```bash
source ~/ardupilot/Rover/ros2/setup.bash
export ROS_DOMAIN_ID=23 ROS_LOCALHOST_ONLY=1
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
ros2 topic list
ros2 topic hz /lio_pi/cloud_registered
```

O teste usa domínio ROS **21** local à Raspberry e **23** local ao PC. A ponte
SSH transporta somente saídas para visualização; ela pode descartar quadros
visuais antigos se a rede ficar lenta. Os dados de entrada do FAST-LIO não são
descartados por essa ponte, pois permanecem na Raspberry.

Logs da ponte e do RViz: `mid360/fast_lio_raspberry/run/ponte.log` e `rviz.log`.
Se não houver saída, confira primeiro o driver, o cabo e a alimentação do
MID360. Uma janela vazia não comprova que o estimador está recebendo dados.

## Atualizar os arquivos do teste na Raspberry

Depois de alterar os arquivos desta pasta ou o adaptador compartilhado:

```bash
./mid360/fast_lio_raspberry/parar.sh
./mid360/fast_lio_raspberry/preparar.sh
./mid360/fast_lio_raspberry/iniciar.sh
```

`preparar.sh` envia os arquivos para `~/mid360/fast_lio_raspberry` na Raspberry
e recompila incrementalmente o adaptador. O código dele é compartilhado com
o teste anterior em `mid360/fast_lio_local/adapter`; nenhuma rotina de controle
do rover é utilizada.
