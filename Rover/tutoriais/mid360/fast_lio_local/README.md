> Código deste componente: `Rover/mid360/fast_lio_local/`. Execute os comandos a partir de `~/ardupilot/Rover`; caminhos `run/` e `ws/` deste documento se referem ao diretório do código.

# FAST-LIO local — ensaio independente com MID360 físico

Este ensaio executa **FAST-LIO neste computador**, em ROS 2 Jazzy, domínio
**22** e relógio real. Não inicia nem utiliza missão, docking, MAVLink,
Gazebo ou SITL. Os arquivos e a compilação ficam nesta pasta.

A Raspberry continua apenas adquirindo o MID360 físico `192.168.1.161` pelo
container independente `rover-mid360`. Os dados chegam por SSH ao computador.
Nenhum FAST-LIO é iniciado na Raspberry por este procedimento.

## Iniciar

O driver físico precisa estar funcionando na Pi. Se necessário:

```bash
cd ~/ardupilot/Rover
./mid360/control.sh up
```

Mantenha o sensor parado sobre uma superfície firme nos primeiros segundos.
Em seguida, neste computador:

```bash
cd ~/ardupilot/Rover
./mid360/fast_lio_local/iniciar.sh
```

O script abre transporte, adaptador de tempos, FAST-LIO e RViz separados.
Antes disso, confere o endereço secundário `192.168.1.50/24` na Ethernet da Pi.
Se ele tiver desaparecido, restaura apenas esse endereço com a rotina do teste
e reinicia somente `rover-mid360`. Também verifica resposta do sensor ao ping.
O teste não deve ser iniciado duas vezes; há um lock para evitar duplicação.
Para rodar sem janela, use `iniciar.sh --no-rviz`.

Se estiver aberta a visualização bruta do teste anterior, feche seu RViz e
interrompa `mid360/ponte_pc.sh` com Ctrl+C para não duplicar tráfego. O novo
ensaio não depende dessa ponte de visualização.

Para encerrar este ensaio, pressione Ctrl+C no terminal que o iniciou ou:

```bash
./mid360/fast_lio_local/parar.sh
```

Esse encerramento deixa o driver físico disponível na Pi. Para encerrar
este driver também, execute `./mid360/control.sh down` depois.

## O que aparece

Fixed Frame do RViz: `camera_init`.

Para navegar com o mouse, selecione **Move Camera** na barra superior e use
a área 3D: arraste com o botão esquerdo para girar; com o botão central para
deslocar a vista; use a rodinha ou arraste com o botão direito para zoom.
As ferramentas de interação estão incluídas no arquivo `fastlio.rviz`.

| Tópico, somente no domínio 22 | Conteúdo |
| --- | --- |
| `/lio_test/points` | PointCloud2 físico original |
| `/lio_test/imu` | IMU física preservada |
| `/lio_test/livox` | Adaptação local para CustomMsg exigida pelo FAST-LIO |
| `/lio_test/odometry` | Posição e orientação estimadas localmente |
| `/lio_test/cloud_registered` | Varredura alinhada no referencial do estimador |
| `/lio_test/map` | Nuvem acumulada publicada por esta versão do FAST-LIO |
| `/lio_test/path` | Trajetória estimada |

O mapa é relativo à inicialização. Este teste não fornece posição GPS, não
inclui fechamento de laços e não envia pose ao autopiloto. Ao reiniciar,
o referencial e o mapa recomeçam.

## Conferir dados

Em outro terminal:

```bash
cd ~/ardupilot/Rover
source ros2/setup.bash
export ROS_DOMAIN_ID=22 ROS_LOCALHOST_ONLY=1 FASTDDS_BUILTIN_TRANSPORTS=UDPv4
unset FASTRTPS_DEFAULT_PROFILES_FILE
python mid360/fast_lio_local/verificar.py --seconds 30
```

Para conferir também estabilidade com o sensor **fisicamente parado**, acrescente
`--stationary`. Esse teste adicional exige variação de posição menor que 10 cm
em cada eixo durante a janela; não é uma certificação de precisão absoluta.
O teste sem essa opção verifica recepção, tempos crescentes e saídas finitas,
mas sozinho não detecta toda estimativa incorreta.

Os logs e resultados ficam em `run/`. `run/latest.txt` aponta para a execução
atual. Uma desconexão, fila excessivamente atrasada ou falha de processo encerra
somente os processos desse ensaio. A mensagem `IMU Initial Done` confirma a
inicialização da IMU; um `No point, skip this scan` inicial isolado pode preceder
a formação do mapa. Mensagens repetidas de falta de pontos exigem investigação.

Se aparecer `Fonte do teste encerrou`, consulte `transport.log`: a ausência
de dados por 8 s faz a fonte encerrar e o supervisor fecha o teste inteiro,
incluindo RViz. O endereço de aquisição é temporário e pode desaparecer após
reinicialização/reconfiguração de rede. O iniciador o recupera antes de cada
nova execução; não tenta continuar um mapa antigo após perda de sensores.
Estar parado não causa esse encerramento: o sensor deve publicar também parado.

## Como foi separado

- Compilação própria em `ws/install`, a partir de uma cópia de FAST-LIO.
- Domínio 22 restrito ao localhost; não participa do domínio da simulação.
- Configuração em `config.yaml` e todos os tópicos de dados em `/lio_test/...`.
- Logs internos de depuração do FAST-LIO apontam para `/dev/null` **na cópia**;
  não escrevem nos diretórios da instalação anterior.
- Reutiliza bibliotecas ROS e tipos de mensagens já instalados, sem executar
  os módulos de aplicação do projeto.

## Transporte e tempo por ponto

A ponte anterior de RViz mantinha só a última amostra pendente; ela não é
adequada para alimentar a integração inercial. `enviar_teste.py` usa uma
FIFO limitada, preserva a ordem e transmite contador de sequência por tópico.
Se a fila exceder 32 MiB/4.096 mensagens ou ficar atrasada por mais de 3 s,
a execução falha em vez de descartar dados silenciosamente. Uma fonte sem
progresso de envio por 8 s encerra para evitar processos órfãos.
Isso não elimina a possibilidade de perdas no sensor/driver; os intervalos
de timestamps também são medidos na validação.

Neste driver, o campo `timestamp` de PointCloud2 contém **nanosegundos absolutos**.
O adaptador C++ subtrai o início da nuvem para preencher `offset_time` de
CustomMsg. Não inventa tempos a partir da ordem dos pontos ou da direção
angular. O pequeno arredondamento de FLOAT64 em timestamps de época é tratado
no primeiro ponto. XYZ, intensidade, tag, linha e timestamps são preservados.

## Configuração inicial e limites

Os extrínsecos internos LiDAR/IMU vieram do arquivo MID360 da versão de FAST-LIO
usada: translação `[-0.011, -0.02329, 0.04412]` m, rotação identidade.
São valores iniciais do modelo, não uma calibração da montagem do seu rover.
A estimativa online de extrínsecos está desligada para este ensaio.
A aceleração é normalizada pelo próprio FAST-LIO durante a inicialização;
o adaptador não aplica um segundo fator de gravidade.

O cubo local tem lado 200 m e alcance de detecção configurado 40 m.
Nesta versão, o cubo precisa superar três vezes esse alcance para não remover
prematuramente pontos úteis; o iniciador verifica essa relação.
Filtros de superfície e mapa: 0,10 m. Gravação automática PCD desativada.
A publicação acumulada do fork não tem retenção limitada por este iniciador;
este é um teste interativo, não uma configuração de operação prolongada.

## Recompilar

```bash
cd ~/ardupilot/Rover
./mid360/fast_lio_local/build.sh
```

A cópia inicial usa `Rover/ros2/deps/FAST_LIO`, revisão local
`2fffc570a25d0df172720bac034fbdb6a13d2162`, incluindo os ajustes Jazzy já presentes
nessa fonte. `build.sh` não modifica essa origem nem instala por cima dela.
A compilação usa um processo por padrão para reduzir a pressão de memória.
